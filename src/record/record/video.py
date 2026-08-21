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

import re
import shutil
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

#: 落盘编码档。ultrafast 默认码率虚高（720p 实测 14.99 Mbps），加 crf 后降到 6.18
#: 且 CPU 还少 15 个点 —— 编码器在盲目堆比特。
ENCODE = ('-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '26',
          '-tune', 'zerolatency', '-pix_fmt', 'yuv420p', '-g', '60')

#: ffmpeg 在脚本里必须加 -nostdin，否则它吞掉 stdin 直接卡死（表现为超时无输出）。
BASE = ('ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'warning')

_PROGRESS_FRAME = re.compile(rb'frame=(\d+)')


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

    def as_dict(self) -> dict:
        return {'name': self.name, 'frames': self.frames,
                'fps': round(self.fps, 2), 'alive': self.alive,
                'error': self.error}


class _FfmpegProcess:
    """ffmpeg 子进程 + 进度读取线程。子类决定命令行与喂数据的方式。"""

    def __init__(self, name: str, out_path: Path) -> None:
        self.name = name
        self.out_path = Path(out_path)
        self.health = StreamHealth(name=name)
        self.proc: subprocess.Popen | None = None
        self._reader: threading.Thread | None = None
        self._stderr: list[bytes] = []

    def _spawn(self, args, stdin=None) -> None:
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        self.proc = subprocess.Popen(
            args, stdin=stdin, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.health.started = time.time()
        self.health.alive = True
        self._reader = threading.Thread(target=self._pump, daemon=True)
        self._reader.start()

    def _pump(self) -> None:
        assert self.proc is not None
        err = threading.Thread(target=self._pump_stderr, daemon=True)
        err.start()
        for line in self.proc.stdout:
            m = _PROGRESS_FRAME.search(line)
            if m:
                self.health.frames = int(m.group(1))
        self.health.alive = False

    def _pump_stderr(self) -> None:
        assert self.proc is not None
        for line in self.proc.stderr:
            self._stderr.append(line)
            del self._stderr[:-40]
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
            # 这两个一起才能让时间戳文件里是绝对 unix 时间：前者把收包墙钟写进
            # 输入包，后者阻止 ffmpeg 把时间轴平移到 0。缺 -copyts 就只有相对值。
            '-use_wallclock_as_timestamps', '1', '-copyts',
            '-i', self.url,
            '-c', 'copy', '-y', str(self.out_path),
            # 时间戳这一路**必须也写 -c copy**：不写就走默认编码器，ffmpeg 为此
            # 把 1080p 整个解码一遍，实测 1.0% -> 25.1% 单核，25 倍。
            '-c', 'copy', '-f', 'mkvtimestamp_v2', '-vsync', 'passthrough',
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
        self._queue: list = []
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
        while True:
            self._event.wait(0.2)
            self._event.clear()
            while True:
                with self._lock:
                    if not self._queue:
                        break
                    buf = self._queue.pop(0)
                if buf is None:
                    return
                try:
                    self.proc.stdin.write(buf)
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


def ffmpeg_available() -> bool:
    return shutil.which('ffmpeg') is not None


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
