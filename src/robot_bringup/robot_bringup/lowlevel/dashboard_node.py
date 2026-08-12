#!/usr/bin/env python3
"""底层总线只读监控：``LowState`` 全字段 + 盆骨/躯干双 IMU。

    /lf/lowstate ──────┐
                       ├─► [ 本节点：stdlib HTTP + 静态页 ] ◄─轮询─ 浏览器
    /lf/secondary_imu ─┘                                    (表格 + 时序曲线)

和 ``g1_motion_control`` 的监控页分工：那边只管 IK 与末端位姿，这边把电机的
每一个字段都铺出来（``tau_est`` / ``vol`` / ``sensor`` / ``mode`` 旧页都没有），
外加 LowState 头部与两颗 IMU。

**纯只读**：不发布任何话题、不调用任何服务，所以不需要 ``ReentrantCallbackGroup``
+ ``MultiThreadedExecutor`` 那一套防死锁配置。

两层降载，缺一不可（实测 rclpy 单帧开销 439 us，raw 下）：

* **默认订固件的 ``/lf/*`` 低频版而不是 ``/lowstate`` 与 ``/secondary_imu``**。两者都是
  G1 固件直发、内容逐字段相同，只差频率：20 Hz vs 1040 Hz。两路全速加起来要烧接近
  一个核，而监控页根本用不上；真要看高频细节把两个 topic 参数改回去。
* **``raw=True`` 订阅**，回调里只接一串字节；反序列化（实测 712 us，占整条链路 79%）
  推迟到浏览器真的来问的那一刻，于是成本跟的是页面轮询频率而不是总线频率。
  不这么做时 spin 线程握着 GIL 不放，HTTP 线程被饿到单次响应 200 ms。

没有锁：回调只做属性赋值，GIL 保证单次赋值原子。快照里 ``seq`` 最多比帧快一拍，
对监控页无影响。另外**没人看页面就退订**，页面重开时自动订回。
"""

from __future__ import annotations

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from unitree_hg.msg import IMUState, LowState

# 前端按下标取值，顺序改了两边都要改。
MOTOR_FIELDS = (
    'mode', 'q', 'dq', 'ddq', 'tau_est',
    'temp_shell', 'temp_winding', 'vol',
    'motorstate', 'sensor0', 'sensor1',
)
# LowState.motor_state 固定 35 槽，前 29 个是本体，后面是宇树预留。
MOTOR_SLOTS = 35
G1_JOINT_NAMES = (
    'left_hip_pitch_joint',
    'left_hip_roll_joint',
    'left_hip_yaw_joint',
    'left_knee_joint',
    'left_ankle_pitch_joint',
    'left_ankle_roll_joint',
    'right_hip_pitch_joint',
    'right_hip_roll_joint',
    'right_hip_yaw_joint',
    'right_knee_joint',
    'right_ankle_pitch_joint',
    'right_ankle_roll_joint',
    'waist_yaw_joint',
    'waist_roll_joint',
    'waist_pitch_joint',
    'left_shoulder_pitch_joint',
    'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint',
    'left_elbow_joint',
    'left_wrist_roll_joint',
    'left_wrist_pitch_joint',
    'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint',
    'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint',
    'right_elbow_joint',
    'right_wrist_roll_joint',
    'right_wrist_pitch_joint',
    'right_wrist_yaw_joint',
)
MOTOR_NAMES = G1_JOINT_NAMES + tuple(
    f'reserved_{index}' for index in range(len(G1_JOINT_NAMES), MOTOR_SLOTS))
# 一整场都不变，就不要跟着每秒几百次的轮询一起重发（约占快照的四分之一）。
MOTOR_META = json.dumps({
    'motor_fields': MOTOR_FIELDS,
    'motor_names': MOTOR_NAMES,
    'named_motors': len(G1_JOINT_NAMES),
}, separators=(',', ':')).encode()
_CONTENT_TYPES = {
    '.html': 'text/html', '.css': 'text/css', '.js': 'text/javascript'}


def _number(value) -> float | None:
    """NaN / Inf 会被 json 写成非法字面量，前端 ``JSON.parse`` 直接抛。"""
    number = float(value)
    return number if math.isfinite(number) else None


def motor_rows(message: LowState | None) -> list[list]:
    """把 ``motor_state`` 摊成与 :data:`MOTOR_FIELDS` 同序的二维数组。"""
    if message is None:
        return []
    return [
        [
            int(motor.mode),
            _number(motor.q),
            _number(motor.dq),
            _number(motor.ddq),
            _number(motor.tau_est),
            int(motor.temperature[0]),
            int(motor.temperature[1]),
            _number(motor.vol),
            int(motor.motorstate),
            int(motor.sensor[0]),
            int(motor.sensor[1]),
        ]
        for motor in message.motor_state[:MOTOR_SLOTS]
    ]


def imu_payload(imu: IMUState | None) -> dict | None:
    """四元数按固件的 ``(w, x, y, z)`` 原样透传，不转 ROS 的 xyzw。"""
    if imu is None:
        return None
    return {
        'quaternion': [_number(value) for value in imu.quaternion],
        'gyroscope': [_number(value) for value in imu.gyroscope],
        'accelerometer': [_number(value) for value in imu.accelerometer],
        'rpy': [_number(value) for value in imu.rpy],
        'temperature': int(imu.temperature),
    }


def lowstate_header(message: LowState | None) -> dict | None:
    if message is None:
        return None
    return {
        'mode_pr': int(message.mode_pr),
        'mode_machine': int(message.mode_machine),
        'tick': int(message.tick),
        'crc': int(message.crc),
        'version': [int(value) for value in message.version],
        'wireless_remote': bytes(message.wireless_remote).hex(),
    }


def under(base: Path, relative: str) -> Path | None:
    """把 ``relative`` 接到 ``base`` 下，越界就返回 None。

    校验的是**相对路径本身**，不是拼出来再 ``resolve()`` 比前缀——
    ``--symlink-install`` 会把包目录做成指回 src/ 的符号链接，``resolve()``
    一路跟过去就跑出了 base，所有静态文件都会 404。
    """
    path = Path(relative)
    if not relative or path.is_absolute() or '..' in path.parts:
        return None
    return base / path


class LowLevelDashboard(Node):

    def __init__(self) -> None:
        super().__init__('lowlevel_dashboard')
        p = self.declare_parameter
        host = p('host', '0.0.0.0').get_parameter_value().string_value
        port = int(p('port', 8210).get_parameter_value().integer_value)
        self._idle_release = p(
            'idle_release_s', 3.0).get_parameter_value().double_value
        self._lowstate_topic = p(
            'lowstate_topic', '/lf/lowstate').get_parameter_value().string_value
        self._imu_topic = p(
            'secondary_imu_topic',
            '/lf/secondary_imu').get_parameter_value().string_value
        self._validate_configuration(port)

        self._lowstate: bytes | None = None
        self._torso_imu: bytes | None = None
        self._seq = 0
        self._last_poll = 0.0
        self._lowstate_sub = None
        self._imu_sub = None
        self._static = Path(__file__).parent / 'static'

        # 底层流全是 BEST_EFFORT + KEEP_LAST(1)，订阅端不匹配会静默收不到数据。
        self._stream = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                  reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_timer(0.2, self._gate)

        self._server = ThreadingHTTPServer((host, port), _handler(self))
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        self.get_logger().info(
            f'底层监控页已开：http://{host}:{self._server.server_address[1]}/')

    def _validate_configuration(self, port: int) -> None:
        if not 0 <= port <= 65535:
            raise ValueError('port must be between 0 and 65535')
        if not math.isfinite(self._idle_release) or self._idle_release <= 0.0:
            raise ValueError('idle_release_s must be greater than zero')

    # -- ROS 输入 --------------------------------------------------------------

    def _on_lowstate(self, data: bytes) -> None:
        self._lowstate = data
        self._seq += 1

    def _on_secondary_imu(self, data: bytes) -> None:
        self._torso_imu = data

    def _gate(self) -> None:
        """有人看才订阅，页面关掉就退订。订/退都在定时器里做，不碰 HTTP 线程。"""
        watching = time.monotonic() - self._last_poll < self._idle_release
        if watching and self._lowstate_sub is None:
            self._lowstate_sub = self.create_subscription(
                LowState, self._lowstate_topic, self._on_lowstate, self._stream,
                raw=True)
            self._imu_sub = self.create_subscription(
                IMUState, self._imu_topic, self._on_secondary_imu, self._stream,
                raw=True)
        elif not watching and self._lowstate_sub is not None:
            self.destroy_subscription(self._lowstate_sub)
            self.destroy_subscription(self._imu_sub)
            self._lowstate_sub = None
            self._imu_sub = None
            self._lowstate = None
            self._torso_imu = None

    # -- HTTP 取数 --------------------------------------------------------------

    def snapshot(self) -> dict:
        self._last_poll = time.monotonic()    # 有人在看，_gate 据此保持订阅
        raw, torso_raw = self._lowstate, self._torso_imu
        lowstate = deserialize_message(raw, LowState) if raw else None
        return {
            'motors': motor_rows(lowstate),
            'header': lowstate_header(lowstate),
            'imu': {
                'pelvis': imu_payload(lowstate.imu_state if lowstate else None),
                'torso': imu_payload(
                    deserialize_message(torso_raw, IMUState) if torso_raw else None),
            },
            'seq': self._seq,
        }

    def read_static(self, relative: str) -> tuple[bytes, str] | None:
        path = under(self._static, relative)
        if path is None or not path.is_file():
            return None
        return (path.read_bytes(),
                _CONTENT_TYPES.get(path.suffix, 'application/octet-stream'))

    def shutdown(self) -> None:
        self._server.shutdown()


def _handler(node: LowLevelDashboard):

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'
        # keep-alive 下每条连接占一个线程；客户端悄悄走掉时靠它回收。
        timeout = 30

        def log_message(self, *args) -> None:
            pass                            # 每次轮询打一行会把终端刷爆

        def _send(self, code: int, body: bytes, kind: str) -> None:
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:            # noqa: N802 (BaseHTTPRequestHandler 的约定)
            path = urlparse(self.path).path
            if path in ('/', '/index.html'):
                return self._static('index.html')
            if path == '/api/motors':
                return self._send(200, MOTOR_META, 'application/json')
            if path == '/api/state':
                body = json.dumps(node.snapshot(), separators=(',', ':')).encode()
                return self._send(200, body, 'application/json')
            return self._static(path.lstrip('/'))

        def _static(self, relative: str) -> None:
            found = node.read_static(relative)
            if found is None:
                return self._send(404, b'not found', 'text/plain')
            return self._send(200, *found)

    return Handler


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LowLevelDashboard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
