"""ROS 侧的采集：三路图像、TF、关节状态，以及运行时切分辨率。

图像不走 cv_bridge —— 它内部对 uint8[] 逐元素处理，1080p 一帧要几百毫秒。
直接 np.frombuffer 按 step 切片，零拷贝。
"""

from __future__ import annotations

import threading
import time

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformListener

from camera_calibration import transforms

# 和 camera_node 的发布端一致，QoS 对不上会一帧都收不到
IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=1)


def image_to_bgr(message: Image) -> np.ndarray:
    """按 encoding 转 BGR。头部 RealSense 出 rgb8、腕相机出 bgr8，弄反了颜色是错的。"""
    encoding = message.encoding.lower()
    raw = np.frombuffer(message.data, np.uint8)
    height, width = message.height, message.width
    if encoding in ('bgr8', 'rgb8'):
        rows = raw.reshape(height, message.step)[:, :width * 3]
        image = rows.reshape(height, width, 3)
        return image if encoding == 'bgr8' else cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == 'mono8':
        rows = raw.reshape(height, message.step)[:, :width]
        return cv2.cvtColor(rows.reshape(height, width), cv2.COLOR_GRAY2BGR)
    if encoding in ('yuyv', 'yuv422_yuy2'):
        rows = raw.reshape(height, message.step)[:, :width * 2]
        return cv2.cvtColor(rows.reshape(height, width, 2), cv2.COLOR_YUV2BGR_YUYV)
    raise ValueError(f'不认识的图像编码：{message.encoding}')


class CameraSource:
    """一路相机：缓存最新一帧，并负责把它切到指定档位。"""

    def __init__(self, node, name: str, config: dict) -> None:
        self.node = node
        self.name = name
        self.config = config
        self.label = config.get('label', name)
        self.role = config.get('role', 'target')
        self.frame = config.get('frame', name)
        self.parent_frame = config.get('parent_frame')
        self.profiles = config.get('profiles', [])
        self._lock = threading.Lock()
        self._frame = None                   # (seq, bgr, stamp)
        self._seq = 0
        self._info = None
        self._client = None

        node.create_subscription(Image, config['topic'], self._on_image, IMAGE_QOS)
        if config.get('info_topic'):
            node.create_subscription(CameraInfo, config['info_topic'], self._on_info, 10)
        switch = config.get('switch', {})
        if switch.get('kind') in ('camera_node', 'realsense'):
            self._client = node.create_client(
                SetParameters, f"{switch['node'].rstrip('/')}/set_parameters",
                callback_group=node.callbacks)

    def _on_image(self, message: Image) -> None:
        try:
            image = image_to_bgr(message)
        except ValueError as error:
            self.node.get_logger().warn(f'{self.name}: {error}', once=True)
            return
        with self._lock:
            self._seq += 1
            self._frame = (self._seq, image.copy(), time.monotonic())

    def _on_info(self, message: CameraInfo) -> None:
        self._info = {
            'width': message.width, 'height': message.height,
            'camera_matrix': [float(v) for v in message.k],
            'distortion_model': message.distortion_model or 'plumb_bob',
            'distortion_coefficients': [float(v) for v in message.d][:5] or [0.0] * 5,
        }

    @property
    def factory_info(self) -> dict | None:
        """相机自己报的内参（只有 RealSense 有），用来和标定结果对照"""
        return self._info

    def latest(self):
        with self._lock:
            return self._frame

    def grab(self, timeout: float = 3.0):
        """等一帧比当前更新的图。切完档位后必须用它，否则会拿到旧分辨率的残帧。"""
        with self._lock:
            start = self._seq
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                frame = self._frame
            if frame is not None and frame[0] > start:
                return frame[1]
            time.sleep(0.02)
        return None

    def status(self) -> dict:
        base = {'name': self.name, 'label': self.label, 'role': self.role}
        frame = self.latest()
        if frame is None:
            return {**base, 'online': False, 'message': '还没收到图'}
        height, width = frame[1].shape[:2]
        age = time.monotonic() - frame[2]
        return {**base, 'online': age < 3.0, 'width': width, 'height': height,
                'age_s': round(age, 2)}

    def find_profile(self, width: int, height: int) -> dict | None:
        for profile in self.profiles:
            if (profile['width'], profile['height']) == (width, height):
                return profile
        return None

    def apply_profile(self, width: int, height: int, timeout: float = 25.0) -> dict:
        profile = self.find_profile(width, height)
        if profile is None:
            raise ValueError(f'{self.label} 没有 {width}x{height} 这个档位')
        if self._client is None:
            raise RuntimeError(f'{self.label} 不支持在线切档位，请改 launch 参数后重启')

        if self.config['switch']['kind'] == 'camera_node':
            # 只换拉哪一路流（stream0 1080p / stream1 640x360）。宽高给 0 = 用流的
            # 原生尺寸，不让 ffmpeg 多挂一个 scale 滤镜。
            self._call([_param('rtsp_url', profile['url']),
                        _param('image_width', 0), _param('image_height', 0)],
                       timeout=10.0)
        else:
            self._switch_realsense(profile['value'])

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.grab(timeout=1.0)
            if frame is not None and frame.shape[1] == width and frame.shape[0] == height:
                return {'width': width, 'height': height}
        raise RuntimeError(f'{self.label} 切到 {width}x{height} 后 {timeout:.0f} s 内没等到对应尺寸的图')

    def _switch_realsense(self, value: str) -> None:
        """先关掉彩色流再改档位。

        实测：直接改 ``rgb_camera.color_profile`` 会返回“设置成功”，但传感器根本
        不重开，出图尺寸一直不变 —— 静默失败。enable_color 先 false 再 true 才会
        真的把流重建。
        """
        node = self.config['switch']['node']
        self._call([_param('enable_color', False)], timeout=10.0)
        time.sleep(1.0)
        self._call([_param(self.config['switch']['param'], value)], timeout=10.0)
        self._call([_param('enable_color', True)], timeout=10.0)
        self.node.get_logger().info(f'{node} 重开彩色流 -> {value}')

    def _call(self, parameters, timeout: float) -> None:
        if not self._client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"连不上 {self.config['switch']['node']} 的参数服务")
        request = SetParameters.Request()
        request.parameters = parameters
        future = self._client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            raise RuntimeError('设置参数超时')
        for result in future.result().results:
            if not result.successful:
                raise RuntimeError(f'设置参数被拒绝：{result.reason}')


class MotionGate:
    """手臂静止才放行拍照。

    camera_node 打的是「收到帧的时刻」，RTSP 还叠着编解码和网络延迟，动着拍出来的图
    和同一时刻查到的 TF 根本不是一回事。宁可等，也别采一堆错位的样本。
    """

    def __init__(self, node, config: dict) -> None:
        self.max_speed = float(config.get('max_joint_speed', 0.01))
        self.settle = float(config.get('settle_seconds', 0.5))
        self._lock = threading.Lock()
        self._speed = None
        self._still_since = None
        self._stamp = 0.0
        self._previous = None
        node.create_subscription(JointState, config.get('joint_states_topic',
                                                        '/joint_states'),
                                 self._on_joints, 10)

    def _on_joints(self, message: JointState) -> None:
        now = time.monotonic()
        if message.velocity:
            speed = float(np.max(np.abs(message.velocity)))
        else:
            # 有的驱动只发位置，那就自己差分
            positions = np.asarray(message.position, float)
            speed = 0.0
            if self._previous is not None:
                previous, stamp = self._previous
                span = now - stamp
                if span > 1e-3 and previous.shape == positions.shape:
                    speed = float(np.max(np.abs(positions - previous)) / span)
            self._previous = (positions, now)
        with self._lock:
            self._speed = speed
            self._stamp = now
            if speed <= self.max_speed:
                if self._still_since is None:
                    self._still_since = now
            else:
                self._still_since = None

    def state(self) -> dict:
        with self._lock:
            speed, since, stamp = self._speed, self._still_since, self._stamp
        if speed is None:
            return {'ok': False, 'reason': '收不到 /joint_states'}
        if time.monotonic() - stamp > 2.0:
            return {'ok': False, 'reason': '/joint_states 已经 2 秒没更新', 'speed': speed}
        if since is None:
            return {'ok': False, 'speed': round(speed, 5),
                    'reason': f'手臂在动（{speed:.3f} rad/s > {self.max_speed}）'}
        held = time.monotonic() - since
        if held < self.settle:
            return {'ok': False, 'speed': round(speed, 5),
                    'reason': f'刚停稳 {held:.2f} s，再等 {self.settle - held:.2f} s'}
        return {'ok': True, 'speed': round(speed, 5), 'still_s': round(held, 2)}


class Frames:
    """TF 查询。查不到就抛，绝不返回单位阵糊弄过去。"""

    def __init__(self, node) -> None:
        self.node = node
        self.buffer = Buffer()
        self._listener = TransformListener(self.buffer, node)

    def lookup(self, parent: str, child: str, timeout: float = 1.0) -> np.ndarray:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.buffer.can_transform(parent, child, rclpy.time.Time()):
                break
            time.sleep(0.02)
        if not self.buffer.can_transform(parent, child, rclpy.time.Time()):
            raise RuntimeError(f'查不到 TF {parent} -> {child}')
        stamped = self.buffer.lookup_transform(parent, child, rclpy.time.Time())
        rotation = stamped.transform.rotation
        translation = stamped.transform.translation
        return transforms.rt_to_matrix(
            transforms.quat_to_matrix([rotation.x, rotation.y, rotation.z, rotation.w]),
            [translation.x, translation.y, translation.z])

    def available(self, parent: str, child: str) -> bool:
        return bool(self.buffer.can_transform(parent, child, rclpy.time.Time()))


def _param(name: str, value) -> Parameter:
    """按 Python 类型选 ParameterValue 的字段。bool 是 int 的子类，必须先判"""
    if isinstance(value, bool):
        inner = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    elif isinstance(value, int):
        inner = ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                               integer_value=value)
    else:
        inner = ParameterValue(type=ParameterType.PARAMETER_STRING,
                               string_value=str(value))
    return Parameter(name=name, value=inner)
