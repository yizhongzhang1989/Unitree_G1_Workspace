"""三路视频落盘。腕部与头部走两条完全不同的路径，原因是数据形态不同。

**腕部（IP 相机）**：相机吐的本来就是 H.264 码流，``-c copy`` 直接搬字节，实测
1.33% 单核/路；解码再转 720p 是 103.8%，**贵 78 倍而且画质更差**。720p 留到导出机上离线降。

**头部（D435i）**：USB 上是 YUYV 裸帧，躲不掉编码。走 ROS 订阅拿得到硬件时间戳
（实测 header.stamp 间隔 p50=p95=33.4 ms）和 camera_info，比 v4l2 直抓贵约 1.5 倍，
换来的是时间戳质量 —— 而时间戳是这套数据集里最难事后补救的东西。

时间戳：腕部用 ``-use_wallclock_as_timestamps`` + ``mkvtimestamp_v2`` 边录边出每帧收包
墙钟；抖动约 ±20 ms（实测），帧率是硬件恒定的，导出侧可以用鲁棒线性拟合抹掉。
头部直接落 ``header.stamp``，无需拟合。
"""

from __future__ import annotations

import signal
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np

#: 落盘编码档。ultrafast 默认码率虚高（720p 实测 14.99 Mbps），加 crf 后降到 6.18
#: 且 CPU 还少 15 个点 —— 编码器在盲目堆比特。
ENCODE = ('-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '26',
          '-tune', 'zerolatency', '-pix_fmt', 'yuv420p', '-g', '60')

#: ffmpeg 在脚本里必须加 -nostdin，否则它吞掉 stdin 直接卡死（表现为超时无输出）。
BASE = ('ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'warning')


@dataclass
class StreamHealth:
    name: str
    frames: int = 0
    started: float = 0.0
    alive: bool = False
    error: str = ''

    @property
    def fps(self) -> float:
        dt = time.time() - self.started
        return self.frames / dt if dt > 1.0 else 0.0


class _FfmpegProcess:
    """ffmpeg 子进程 + 进度读取线程。子类决定命令行与喂数据的方式。"""

    def __init__(self, name: str, out_path: Path) -> None:
        self.name = name
        self.out_path = Path(out_path)
        self.health = StreamHealth(name=name)
        self.proc: subprocess.Popen | None = None
        self.killed = False
        self._reader: threading.Thread | None = None

    def _spawn(self, args, stdin=None) -> None:
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        self.proc = subprocess.Popen(
            args, stdin=stdin, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.health.started = time.time()
        self.health.alive = True
        self._reader = threading.Thread(target=self._pump, daemon=True)
        self._reader.start()

    def _pump(self) -> None:
        """读 ``-progress`` 的输出抽帧计数。stderr 得同时排空，否则管道满了 ffmpeg 会阻住。"""
        assert self.proc is not None
        threading.Thread(target=self._pump_stderr, daemon=True).start()
        for line in self.proc.stdout:
            if line.startswith(b'frame='):
                try:
                    self.health.frames = int(line[6:])
                except ValueError:
                    pass
        self.health.alive = False

    def _pump_stderr(self) -> None:
        assert self.proc is not None
        for line in self.proc.stderr:
            low = line.lower()
            if b'error' in low or b'failed' in low:
                self.health.error = line.decode('utf-8', 'replace').strip()[:200]

    def stop(self, timeout: float = 10.0) -> None:
        """必须走 SIGINT：SIGKILL 会让 mkv 缺 Cues，文件虽能放但没法精确 seek。"""
        if self.proc is None or self.proc.poll() is not None:
            self.health.alive = False
            return
        self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            # 阻塞在 socket 读里的 ffmpeg 收不到 SIGINT，只能硬杀；调用方要据此告警
            self.killed = True
            self.proc.kill()
            self.proc.wait(timeout=5)
        self.health.alive = False


class WristRecorder(_FfmpegProcess):
    """IP 相机原码流直落，不解码。"""

    def __init__(self, name: str, url: str, out_dir: Path) -> None:
        super().__init__(name, Path(out_dir) / f'{name}.mkv')
        self.url = url
        self.ts_path = Path(out_dir) / f'{name}.ts.txt'
        self.pts_path = Path(out_dir) / f'{name}.pts.bin'

    def start(self) -> None:
        args = [
            *BASE,
            '-rtsp_transport', 'tcp',
            # low_delay 管编解码器重排序缓冲，要留；nobuffer 会丢掉入场 IDR
            # 导致开头几秒全是灰帧，绝不能加（实测 32 帧纯灰）。
            '-flags', 'low_delay',
            # 相机卡住时 ffmpeg 会一直阻塞在 socket 读里，连 SIGINT 都处理不了，
            # 最后被 stop() 硬杀。**必须是 -stimeout**：-timeout 是「等待入站连接」
            # 且隐含 listen 标志，会把客户端变成服务器。
            '-stimeout', '5000000',
            # 这两个一起才能让时间戳文件里是绝对 unix 时间：前者把收包墙钟写进
            # 输入包，后者阻止 ffmpeg 把时间轴平移到 0。缺 -copyts 就只有相对值。
            '-use_wallclock_as_timestamps', '1', '-copyts',
            '-i', self.url,
            '-c', 'copy', '-y', str(self.out_path),
            # 时间戳这一路**必须也写 -c copy**：不写就走默认编码器，ffmpeg 为此
            # 把 1080p 整个解码一遍，实测 1.0% -> 25.1% 单核，25 倍。
            #
            # flush_packets 让每帧的时间戳立刻落盘。不加的话进程被硬杀时整个文件
            # 是空的 —— 实测 SIGKILL 后 0 行 vs 加了之后 348 行，一次 82 秒的采集
            # 就是这样丢光了腕部时间戳。文本量约 420 B/s，代价可忽略。
            '-c', 'copy', '-f', 'mkvtimestamp_v2', '-vsync', 'passthrough',
            '-flush_packets', '1',
            '-y', str(self.ts_path),
            '-progress', 'pipe:1', '-nostats',
        ]
        self._spawn(args)

    def finalize(self) -> int:
        """把 ffmpeg 的毫秒时间戳文本转成 float64 秒的 .bin，然后删掉中间文件。"""
        if not self.ts_path.is_file():
            return 0
        vals = []
        for line in self.ts_path.read_text(encoding='utf-8', errors='ignore').splitlines():
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            try:
                vals.append(float(line) / 1000.0)
            except ValueError:
                continue
        np.asarray(vals, dtype=np.float64).tofile(self.pts_path)
        self.ts_path.unlink(missing_ok=True)
        return len(vals)


class HeadRecorder(_FfmpegProcess):
    """从 ROS Image 帧喂 ffmpeg 软编，并把 header.stamp 逐帧落表。

    写管道放在独立线程：ROS 回调里同步写会在管道满时阻塞订阅，直接开始丢帧。
    """

    #: ROS encoding -> ffmpeg 输入像素格式。YUYV 直出省掉一次色彩转换，
    #: 720p 实测编码从 98.0% 降到 60.3% 单核，且 RGB8 会丢帧而 YUYV 不会。
    PIX = {'rgb8': 'rgb24', 'bgr8': 'bgr24', 'yuv422_yuy2': 'yuyv422'}

    def __init__(self, name: str, out_dir: Path, queue_size: int = 90) -> None:
        super().__init__(name, Path(out_dir) / f'{name}.mkv')
        self.pts_path = Path(out_dir) / f'{name}.pts.bin'
        self._queue: deque = deque()
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._writer: threading.Thread | None = None
        self._max = queue_size
        self._stamps: list[float] = []
        self.dropped = 0
        self.width = self.height = 0
        self.encoding = ''

    def start_from(self, width: int, height: int, encoding: str, fps: int = 30) -> None:
        if encoding not in self.PIX:
            raise ValueError(f'头部相机不支持的编码 {encoding}')
        self.width, self.height, self.encoding = width, height, encoding
        args = [*BASE, '-f', 'rawvideo', '-pix_fmt', self.PIX[encoding],
                '-s', f'{width}x{height}', '-r', str(fps), '-i', 'pipe:0',
                *ENCODE, '-y', str(self.out_path),
                '-progress', 'pipe:1', '-nostats']
        self._spawn(args, stdin=subprocess.PIPE)
        self._writer = threading.Thread(target=self._drain, daemon=True)
        self._writer.start()

    def push(self, payload, stamp: float) -> None:
        """ROS 回调调它。只入队不写盘，回调必须立刻返回。"""
        with self._lock:
            if len(self._queue) >= self._max:
                self.dropped += 1
                return
            self._queue.append(payload)
            self._stamps.append(stamp)
        self._event.set()

    def _drain(self) -> None:
        assert self.proc is not None and self.proc.stdin is not None
        stdin = self.proc.stdin
        while True:
            self._event.wait(0.2)
            self._event.clear()
            while True:
                with self._lock:
                    if not self._queue:
                        break
                    buf = self._queue.popleft()
                if buf is None:                    # stop() 放进来的哨兵
                    return
                try:
                    stdin.write(buf)
                except (BrokenPipeError, ValueError):
                    return

    def stop(self, timeout: float = 15.0) -> None:
        with self._lock:
            self._queue.append(None)
        self._event.set()
        if self._writer:
            self._writer.join(timeout=5)
        if self.proc and self.proc.stdin and not self.proc.stdin.closed:
            try:
                self.proc.stdin.close()
            except BrokenPipeError:
                pass
        if self.proc:
            try:
                self.proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.health.alive = False

    def finalize(self) -> int:
        np.asarray(self._stamps, dtype=np.float64).tofile(self.pts_path)
        return len(self._stamps)


def preview_url(url: str) -> str:
    """主码流 URL -> 子码流 URL。认不出就原样返回（还能用，只是贵）。

    腕相机的 ``stream1`` 是 640x360 HEVC；同样「解码 + 2fps + 缩到 480」实测
    7% 单核，主码流 1080p 那路是 18%。预览是常驻的，这个差价不能不要。
    """
    return url[:-1] + '1' if url.endswith('stream0') else url


class PreviewPump:
    """一路 RTSP 低帧率预览：ffmpeg 常驻解码，最新一帧 JPEG 留在内存里。

    和 ``snapshot`` 那条路是两回事：快照抓的是 1080p 主码流、每次重新握手（腕部约
    1.3 s），只适合点一下看一眼；预览要连续出帧，所以走子码流并把帧率压到 2。

    **没人取帧就自己停**（``idle()`` + 调用方定期收）。页面一关就再没人来取，
    不停就是白烧 CPU，而这个面板经常整天开着。
    """

    #: 出帧节流。低帧率是有意的：这是「看一眼画面对不对」，不是监视器。
    FPS = 2.0
    #: 多久没人取帧就停。要大于前端的轮询周期，不然一边收一边起。
    IDLE_STOP = 6.0
    #: 起不来时的重试间隔。不退避的话相机不在线会变成每次轮询都 fork 一个 ffmpeg。
    RETRY = 5.0

    def __init__(self, name: str, url: str, width: int = 480) -> None:
        self.name, self.url, self.width = name, url, width
        self._lock = threading.Lock()
        self._frame = b''
        self._err = ''
        self._proc: subprocess.Popen | None = None
        self._retry_at = 0.0
        self._touched = time.monotonic()

    def frame(self) -> bytes:
        """最近一帧 JPEG。取帧这一下也是「有人在看」的唯一信号。"""
        self._touched = time.monotonic()
        self._ensure()
        with self._lock:
            if self._frame:
                return self._frame
            err = self._err
        raise RuntimeError(err or f'{self.name} 预览还在起（腕部要等 RTSP 握手）')

    def idle(self) -> bool:
        return time.monotonic() - self._touched > self.IDLE_STOP

    def _ensure(self) -> None:
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                return
            if time.monotonic() < self._retry_at:
                return
            self._retry_at = time.monotonic() + self.RETRY
            self._err = ''
            args = [*BASE, '-rtsp_transport', 'tcp', '-stimeout', '5000000',
                    '-i', self.url, '-an',
                    '-vf', f'fps={self.FPS},scale={self.width}:-2',
                    '-q:v', '7', '-f', 'mjpeg', 'pipe:1']
            self._proc = subprocess.Popen(
                args, stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            proc = self._proc
        threading.Thread(target=self._read, args=(proc,), daemon=True).start()
        threading.Thread(target=self._read_err, args=(proc,), daemon=True).start()

    def _read(self, proc: subprocess.Popen) -> None:
        """从 MJPEG 管道里切出整帧。

        按 EOI(``ff d9``) 切是安全的：JPEG 的熵编码段里 ``ff`` 后面一定跟 ``00``，
        而 ffmpeg 的 mjpeg 输出不带 EXIF 缩略图，图里不会再出现第二个 EOI。
        """
        buf = b''
        assert proc.stdout is not None
        while True:
            chunk = proc.stdout.read1(1 << 16)     # read() 会等满 n 字节
            if not chunk:
                break
            buf += chunk
            while True:
                i = buf.find(b'\xff\xd9')
                if i < 0:
                    break
                frame, buf = buf[:i + 2], buf[i + 2:]
                if frame.startswith(b'\xff\xd8'):
                    with self._lock:
                        self._frame = frame

    def _read_err(self, proc: subprocess.Popen) -> None:
        """排空 stderr（管道满了 ffmpeg 会阻住），顺便留下最后一句话当错误。"""
        assert proc.stderr is not None
        last = b''
        for line in proc.stderr:
            line = line.strip()
            if line:
                last = line
        proc.wait()
        with self._lock:
            if proc is self._proc:
                self._err = last.decode('utf-8', 'replace')[:200]
                self._frame = b''

    def stop(self) -> None:
        with self._lock:
            proc, self._proc, self._frame = self._proc, None, b''
        if proc is None or proc.poll() is not None:
            return
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()


def probe_stream(url: str, timeout: float = 20.0) -> dict:
    """探一路 RTSP 的实参。ffprobe **没有** -nostdin 选项，加了会当输入文件报错。"""

    args = ['ffprobe', '-v', 'error', '-rtsp_transport', 'tcp',
            '-select_streams', 'v:0', '-show_entries',
            'stream=codec_name,width,height,avg_frame_rate',
            '-of', 'default=nw=1:nk=1', url]
    try:
        out = subprocess.run(args, capture_output=True, timeout=timeout,
                             stdin=subprocess.DEVNULL, text=True)
    except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
        return {'ok': False, 'error': str(exc)}
    if out.returncode != 0:
        return {'ok': False, 'error': out.stderr.strip()[:200]}
    fields = out.stdout.split()
    if len(fields) < 4:
        return {'ok': False, 'error': f'ffprobe 输出异常: {out.stdout!r}'}
    num, _, den = fields[3].partition('/')
    try:
        fps = float(num) / float(den or 1)
    except (ValueError, ZeroDivisionError):
        fps = 0.0
    return {'ok': True, 'codec': fields[0], 'width': int(fields[1]),
            'height': int(fields[2]), 'fps': round(fps, 2)}
