"""IP 相机 RTSP → sensor_msgs/Image。没人要图的时候不拉流。"""

import array
import json
import subprocess
import threading
import time
from typing import Any

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import Image

_DEFAULTS = {
    'rtsp_url': '',
    'image_topic': '~/image_raw',
    'frame_id': '',
    'image_width': 0,
    'image_height': 0,
    'fps': 0,
    'server_port': 0,
    'jpeg_quality': 60,
    'poll_period_s': 1.0,
    'stale_timeout_s': 5.0,
}
_SIZE_PARAMS = ('fps', 'image_width', 'image_height')
# 顺序要和 _open_reader 的解包一致
_STREAM_PARAMS = ('rtsp_url',) + _SIZE_PARAMS


def probe_size(url):
    """用 ffprobe 读原生分辨率，读不到返回 None"""
    command = [
        'ffprobe', '-v', 'error', '-rtsp_transport', 'tcp',
        '-select_streams', 'v:0', '-show_entries', 'stream=width,height',
        '-of', 'json', url,
    ]
    try:
        probe = subprocess.run(
            command, capture_output=True, text=True, timeout=8.0)
    except (OSError, subprocess.SubprocessError):
        return None
    if probe.returncode != 0:
        return None
    try:
        stream = json.loads(probe.stdout)['streams'][0]
        return int(stream['width']), int(stream['height'])
    except (LookupError, TypeError, ValueError):
        return None


def ffmpeg_command(url, fps=0, scale=None):
    """限帧和缩放都交给 ffmpeg，管道上只走目标分辨率的裸 BGR"""
    filters = []
    if fps > 0:
        filters.append(f'fps={fps}')
    if scale is not None:
        # area 是下采样的盒式平均：比默认 bicubic 省 12% CPU 且不产生 aliasing
        filters.append(f'scale={scale[0]}:{scale[1]}:flags=area')
    command = [
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin',
        # 别加 -fflags nobuffer：它会丢掉入场的 IDR，之后整整一个 GOP 都在拿
        # 凭空造的灰帧当参考帧解码。low_delay 才是管稳态延迟的那个
        '-rtsp_transport', 'tcp', '-flags', 'low_delay',
        '-an', '-i', url,
    ]
    if filters:
        command += ['-vf', ','.join(filters)]
    return command + ['-f', 'rawvideo', '-pix_fmt', 'bgr24', '-']


def fit_size(native, width, height):
    """只给一边就按原生宽高比补另一边，补出来的边取偶数"""
    if width > 0 and height > 0:
        return width, height
    if height > 0:
        return _even(native[0] * height / native[1]), height
    if width > 0:
        return width, _even(native[1] * width / native[0])
    return native


def _even(value):
    return max(2, round(value / 2) * 2)


class RtspReader:
    """一个 ffmpeg 子进程配一个读帧线程，逐帧调 on_frame(raw, width, height)"""

    def __init__(self, command, width, height, on_frame):
        self.width = width
        self.height = height
        self._frame_bytes = width * height * 3
        self._on_frame = on_frame
        self._stamp = time.monotonic()
        self._process = subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        self._thread = threading.Thread(
            target=self._pump, daemon=True, name='rtsp-reader')
        self._thread.start()

    def frame_age(self):
        return time.monotonic() - self._stamp

    def alive(self):
        return self._process.poll() is None

    def _pump(self):
        stdout = self._process.stdout
        assert stdout is not None
        while True:
            raw = stdout.read(self._frame_bytes)
            if len(raw) != self._frame_bytes:
                return
            self._stamp = time.monotonic()
            self._on_frame(raw, self.width, self.height)

    def stop(self):
        self._process.kill()  # 卡在 read() 上的线程只有等管道关掉才会醒
        try:
            self._process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            pass
        self._thread.join(timeout=3.0)


class CameraNode(Node):
    """有 ROS 订阅者或网页在看才开 ffmpeg，都断开就把流关掉"""

    def __init__(self):
        super().__init__('camera_node')
        for name, default in _DEFAULTS.items():
            self.declare_parameter(name, default)
        if not self._setting('rtsp_url'):
            raise ValueError('rtsp_url 不能为空')
        self.add_on_set_parameters_callback(self._check_parameters)

        self._poll_period = self._setting('poll_period_s')
        self._stale_timeout = self._setting('stale_timeout_s')
        self._frame_id = self._setting('frame_id') or self.get_name()

        self._publisher = self.create_publisher(
            Image, self._setting('image_topic'), QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST, depth=1))

        self._frames = threading.Condition()
        self._frame = None
        self._seq = 0
        self._viewers = 0
        self._publish_frames = False
        self._reader = None
        self._applied = None
        self._probed = {}
        self._error = None
        self._shutdown = threading.Event()

        self._preview = None
        port = self._setting('server_port')
        if port > 0:
            from camera_node.preview import PreviewServer
            self._preview = PreviewServer(self, port)
            self.get_logger().info(f'预览页 http://0.0.0.0:{port}')

        self._supervisor = threading.Thread(
            target=self._supervise, daemon=True, name='camera-supervisor')
        self._supervisor.start()
        self.get_logger().info(
            f'{self._setting("rtsp_url")} → {self._publisher.topic_name}'
            f'（按需拉流）')

    @property
    def stopping(self):
        return self._shutdown.is_set()

    @property
    def jpeg_quality(self):
        return self._setting('jpeg_quality')

    def add_viewer(self):
        with self._frames:
            self._viewers += 1

    def remove_viewer(self):
        with self._frames:
            self._viewers -= 1

    def wait_frame(self, seq, timeout):
        """等一帧比 seq 新的图，超时返回 None"""
        with self._frames:
            if self._frame is None or self._frame[0] <= seq:
                self._frames.wait(timeout)
            frame = self._frame
        if frame is None or frame[0] <= seq:
            return None
        return frame

    def status(self):
        reader = self._reader
        error = self._error
        if error is not None:
            return {'is_running': False, 'state': 'error', 'message': error}
        if reader is None:
            return {'is_running': True, 'state': 'idle',
                    'message': '空闲：没有订阅者，也没人在看'}
        return {
            'is_running': True, 'state': 'streaming',
            'message': (f'{reader.width}x{reader.height}，'
                        f'最近一帧 {reader.frame_age():.1f} s 前'),
        }

    def _on_frame(self, raw, width, height):
        if self._publish_frames:
            message = Image()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self._frame_id
            message.height = height
            message.width = width
            message.encoding = 'bgr8'
            message.step = width * 3
            # 必须给 array.array：赋 bytes 会走 rclpy 的逐元素断言，一帧上百毫秒
            message.data = array.array('B', raw)
            self._publisher.publish(message)
        if self._preview is not None:
            with self._frames:
                self._seq += 1
                self._frame = (self._seq, raw, width, height)
                self._frames.notify_all()
        self._error = None

    def _supervise(self):
        while not self._shutdown.wait(self._poll_period):
            try:
                self._step()
            except Exception as error:  # 守护线程不能死
                self._error = str(error)
                self.get_logger().error(f'拉流管理异常：{error}')

    def _step(self):
        subscribers = self._publisher.get_subscription_count()
        self._publish_frames = subscribers > 0
        with self._frames:
            viewers = self._viewers
        reader = self._reader
        if subscribers == 0 and viewers == 0:
            if reader is not None:
                self._close_reader('没人要图，停止拉流')
            return
        if reader is None:
            self._open_reader()
        elif not reader.alive():
            self._close_reader('ffmpeg 已退出，重连')
            self._open_reader()
        elif reader.frame_age() > self._stale_timeout:
            self._close_reader(f'{self._stale_timeout:.0f} s 没有新帧，重连')
            self._open_reader()
        elif self._stream_settings() != self._applied:
            self._close_reader('参数改了，重开流')
            self._open_reader()

    def _setting(self, name) -> Any:
        return self.get_parameter(name).value

    def _stream_settings(self):
        """改了就要重开 ffmpeg 的那几个参数"""
        return tuple(self._setting(name) for name in _STREAM_PARAMS)

    def _check_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == 'rtsp_url' and not parameter.value:
                return SetParametersResult(
                    successful=False, reason='rtsp_url 不能为空')
            if parameter.name in _SIZE_PARAMS and parameter.value < 0:
                return SetParametersResult(
                    successful=False, reason=f'{parameter.name} 不能为负')
        return SetParametersResult(successful=True)

    def _native_size(self, url):
        size = self._probed.get(url)
        if size is None:
            size = probe_size(url)
            if size is not None:
                self._probed[url] = size
        return size

    def _open_reader(self):
        settings = self._stream_settings()
        url, fps, width, height = settings
        if width > 0 and height > 0:
            size = scale = (width, height)
        else:
            native = self._native_size(url)
            if native is None:
                self._fail(f'ffprobe 读不到分辨率：{url}')
                return
            size = fit_size(native, width, height)
            scale = None if size == native else size
        try:
            self._reader = RtspReader(
                ffmpeg_command(url, fps, scale),
                size[0], size[1], self._on_frame)
        except OSError as error:
            self._fail(f'启动 ffmpeg 失败：{error}')
            return
        self._applied = settings
        self.get_logger().info(f'开始拉流 {size[0]}x{size[1]}')

    def _close_reader(self, reason):
        reader = self._reader
        self._reader = None
        if reader is not None:
            self.get_logger().info(reason)
            reader.stop()

    def _fail(self, message):
        self._error = message
        self.get_logger().warn(message)

    def destroy_node(self):
        self._shutdown.set()
        self._supervisor.join(timeout=3.0)
        if self._preview is not None:
            self._preview.stop()
        self._close_reader('节点退出')
        with self._frames:
            self._frames.notify_all()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
