"""ROS 2 Gloria-M 夹爪设备节点（共享 CAN bridge 模式）。

节点不打开 SDK 自带的串口转 CAN 设备，只复用 Gloria-M SDK 的纯协议层，通过
``can_bridge_ros`` 与项目 CAN 设备通信。支持 MIT、PV 两种控制模式、模式回读确认、
反馈在线检测、安全位置限幅、诊断信息以及退出失能。
"""

from __future__ import annotations

import math
import struct
import threading
import time
from typing import Optional, Sequence, Tuple

import rclpy
from can_msgs.msg import Frame
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from gloria_ros.msg import MitCommand, PvCommand
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

try:
    from gloria_m_sdk.protocol_mit import pack_mit_command, unpack_mit_feedback
    from gloria_m_sdk.types import Limits
except Exception as _exc:  # noqa: BLE001
    raise ImportError(
        f"无法导入夹爪 SDK 协议层（gloria_m_sdk）：{_exc}. "
        "确认 submodule 已拉取，并先 source scripts/env.sh") from _exc


_CTRL_ENABLE = 0xFC
_CTRL_DISABLE = 0xFD
_CTRL_SET_ZERO = 0xFE
_RID_CTRL_MODE = 10
_RID_PMAX = 21
_RID_VMAX = 22
_RID_TMAX = 23
_MODE_MIT = 1
_MODE_POS_VEL = 2
_MODE_NAMES = {_MODE_MIT: "mit", _MODE_POS_VEL: "pos_vel"}


class GloriaGripperNode(Node):
    """一台 Gloria-M 夹爪的 ROS 2 设备节点。"""

    def __init__(self) -> None:
        super().__init__("gloria_gripper")
        self._cb_group = ReentrantCallbackGroup()
        self._lock = threading.RLock()
        self._operation_lock = threading.RLock()
        self._feedback_event = threading.Event()
        self._mode_event = threading.Event()
        self._parameter_event = threading.Event()

        self.declare_parameter("rx_topic", "/can0/rx")
        self.declare_parameter("tx_topic", "/can0/tx")
        self.declare_parameter("command_id", 0x01)
        self.declare_parameter("feedback_id", 0x101)
        self.declare_parameter("control_mode", "mit")
        self.declare_parameter("pmax", 3.14)
        self.declare_parameter("vmax", 10.0)
        self.declare_parameter("tmax", 12.0)
        self.declare_parameter("safe_position_min", 0.0)
        self.declare_parameter("safe_position_max", 2.77)
        self.declare_parameter("kp", 10.0)
        self.declare_parameter("kd", 1.0)
        self.declare_parameter("pv_velocity", 1.0)
        self.declare_parameter("joint_name", "gripper")
        self.declare_parameter("state_topic", "~/joint_states")
        self.declare_parameter("command_topic", "~/command")
        self.declare_parameter("mit_command_topic", "~/mit_command")
        self.declare_parameter("pv_command_topic", "~/pv_command")
        self.declare_parameter("enable_on_start", False)
        self.declare_parameter("configure_mode_on_enable", True)
        self.declare_parameter("verify_limits_on_configure", True)
        self.declare_parameter("allow_set_zero", False)
        self.declare_parameter("require_enabled_for_command", True)
        self.declare_parameter("require_fresh_feedback", True)
        self.declare_parameter("feedback_timeout_s", 0.5)
        self.declare_parameter("response_timeout_s", 0.5)
        self.declare_parameter("state_poll_period_s", 0.1)
        self.declare_parameter("disable_on_feedback_timeout", True)
        self.declare_parameter("disable_on_shutdown", True)
        self.declare_parameter("diagnostic_period_s", 1.0)

        gp = self.get_parameter
        rx_topic = str(gp("rx_topic").value)
        tx_topic = str(gp("tx_topic").value)
        self._command_id = int(gp("command_id").value)
        self._feedback_id = int(gp("feedback_id").value)
        self._desired_mode = self._parse_mode(str(gp("control_mode").value))
        self._limits = Limits(
            pmax=float(gp("pmax").value),
            vmax=float(gp("vmax").value),
            tmax=float(gp("tmax").value),
        )
        self._safe_min = float(gp("safe_position_min").value)
        self._safe_max = float(gp("safe_position_max").value)
        self._kp = float(gp("kp").value)
        self._kd = float(gp("kd").value)
        self._pv_velocity = float(gp("pv_velocity").value)
        self._joint_name = str(gp("joint_name").value)
        state_topic = str(gp("state_topic").value)
        command_topic = str(gp("command_topic").value)
        mit_command_topic = str(gp("mit_command_topic").value)
        pv_command_topic = str(gp("pv_command_topic").value)
        enable_on_start = bool(gp("enable_on_start").value)
        self._configure_mode_on_enable = bool(gp("configure_mode_on_enable").value)
        self._verify_limits_on_configure = bool(gp("verify_limits_on_configure").value)
        self._allow_set_zero = bool(gp("allow_set_zero").value)
        self._require_enabled = bool(gp("require_enabled_for_command").value)
        self._require_fresh = bool(gp("require_fresh_feedback").value)
        self._feedback_timeout = float(gp("feedback_timeout_s").value)
        self._response_timeout = float(gp("response_timeout_s").value)
        self._state_poll_period = float(gp("state_poll_period_s").value)
        self._disable_on_feedback_timeout = bool(
            gp("disable_on_feedback_timeout").value)
        self._disable_on_shutdown = bool(gp("disable_on_shutdown").value)
        diagnostic_period = float(gp("diagnostic_period_s").value)
        self._validate_configuration(diagnostic_period)

        self._got_feedback = False
        self._last_feedback_monotonic = 0.0
        self._last_position = 0.0
        self._last_velocity = 0.0
        self._last_torque = 0.0
        self._enabled_requested = False
        self._confirmed_mode: Optional[int] = None
        self._last_mode_value: Optional[int] = None
        self._mode_request_pending = False
        self._pending_parameter_id: Optional[int] = None
        self._last_parameter_value: Optional[float] = None
        self._disable_generation = 0
        self._shutting_down = False

        rx_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
        tx_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        self._tx_pub = self.create_publisher(Frame, tx_topic, tx_qos)
        self._state_pub = self.create_publisher(JointState, state_topic, state_qos)
        self._diag_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", QoSProfile(depth=10))
        self._rx_sub = self.create_subscription(
            Frame, rx_topic, self._on_frame, rx_qos,
            callback_group=self._cb_group)
        self._cmd_sub = self.create_subscription(
            Float64, command_topic, self._on_command, 10,
            callback_group=self._cb_group)
        self._mit_cmd_sub = self.create_subscription(
            MitCommand, mit_command_topic, self._on_mit_command, 10,
            callback_group=self._cb_group)
        self._pv_cmd_sub = self.create_subscription(
            PvCommand, pv_command_topic, self._on_pv_command, 10,
            callback_group=self._cb_group)
        self._srv_enable = self.create_service(
            Trigger, "~/enable", self._srv_enable_cb,
            callback_group=self._cb_group)
        self._srv_disable = self.create_service(
            Trigger, "~/disable", self._srv_disable_cb,
            callback_group=self._cb_group)
        self._srv_zero = self.create_service(
            Trigger, "~/set_zero", self._srv_zero_cb,
            callback_group=self._cb_group)
        self._srv_configure = self.create_service(
            Trigger, "~/configure", self._srv_configure_cb,
            callback_group=self._cb_group)
        self._srv_refresh = self.create_service(
            Trigger, "~/refresh", self._srv_refresh_cb,
            callback_group=self._cb_group)

        self._diag_timer = self.create_timer(
            diagnostic_period, self._publish_diagnostics,
            callback_group=self._cb_group)
        self._state_timer = self.create_timer(
            self._state_poll_period, self._poll_state,
            callback_group=self._cb_group)
        self._enable_timer = None
        if enable_on_start:
            self._enable_timer = self.create_timer(
                1.0, self._enable_once, callback_group=self._cb_group)

        self.get_logger().info(
            f"Gloria gripper: cmd_id=0x{self._command_id:X} "
            f"fb_id=0x{self._feedback_id:X} mode={self._mode_name(self._desired_mode)} "
            f"rx='{rx_topic}' tx='{tx_topic}' -> '{state_topic}', "
            f"safe_position=[{self._safe_min}, {self._safe_max}]")

    @staticmethod
    def _parse_mode(value: str) -> int:
        normalized = value.strip().lower().replace("-", "_")
        if normalized == "mit":
            return _MODE_MIT
        if normalized in ("pv", "pos_vel", "position_velocity"):
            return _MODE_POS_VEL
        raise ValueError("control_mode 必须是 'mit' 或 'pos_vel'")

    @staticmethod
    def _mode_name(mode: Optional[int]) -> str:
        return _MODE_NAMES.get(mode, "unknown")

    def _validate_configuration(self, diagnostic_period: float) -> None:
        if not 0 <= self._command_id <= 0x7FF:
            raise ValueError("command_id 必须是 11-bit CAN ID")
        if not 0 <= self._feedback_id <= 0x7FF:
            raise ValueError("feedback_id 必须是 11-bit CAN ID")
        if self._desired_mode == _MODE_POS_VEL and self._command_id > 0x6FF:
            raise ValueError("PV 模式要求 command_id <= 0x6FF")
        values = (
            self._limits.pmax, self._limits.vmax, self._limits.tmax,
            self._safe_min, self._safe_max, self._kp, self._kd,
            self._pv_velocity, self._feedback_timeout, self._response_timeout,
            self._state_poll_period, diagnostic_period,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("夹爪数值参数必须是有限数")
        if self._limits.pmax <= 0 or self._limits.vmax <= 0 or self._limits.tmax <= 0:
            raise ValueError("pmax/vmax/tmax 必须大于零")
        if self._safe_min >= self._safe_max:
            raise ValueError("safe_position_min 必须小于 safe_position_max")
        if self._safe_min < -self._limits.pmax or self._safe_max > self._limits.pmax:
            raise ValueError("安全位置范围必须位于 [-pmax, pmax] 内")
        if not 0.0 <= self._kp <= 500.0 or not 0.0 <= self._kd <= 5.0:
            raise ValueError("kp 必须在 [0,500]、kd 必须在 [0,5]")
        if self._feedback_timeout <= 0 or self._response_timeout <= 0:
            raise ValueError("反馈和响应超时必须大于零")
        if self._state_poll_period <= 0:
            raise ValueError("state_poll_period_s 必须大于零")
        if diagnostic_period <= 0:
            raise ValueError("diagnostic_period_s 必须大于零")

    # --- CAN 帧 ---------------------------------------------------------
    def _send_frame(self, can_id: int, data: bytes) -> None:
        if len(data) > 8:
            raise ValueError("CAN 数据长度不能超过 8 字节")
        frame = Frame()
        frame.header.stamp = self.get_clock().now().to_msg()
        frame.id = int(can_id)
        frame.is_extended = False
        frame.is_rtr = False
        frame.is_error = False
        frame.dlc = len(data)
        frame.data = list(bytes(data).ljust(8, b"\x00"))
        self._tx_pub.publish(frame)

    def _send_ctrl(self, command: int) -> None:
        self._send_frame(
            self._command_id,
            bytes([0xFF] * 7 + [command & 0xFF]),
        )

    def _send_mode_request(self, mode: int) -> None:
        data = bytes([
            self._command_id & 0xFF,
            (self._command_id >> 8) & 0xFF,
            0x55,
            _RID_CTRL_MODE,
        ]) + struct.pack("<I", int(mode))
        self._send_frame(0x7FF, data)

    def _send_parameter_read(self, register_id: int) -> None:
        data = bytes([
            self._command_id & 0xFF,
            (self._command_id >> 8) & 0xFF,
            0x33,
            register_id & 0xFF,
            0, 0, 0, 0,
        ])
        self._send_frame(0x7FF, data)

    def _request_state(self) -> None:
        data = bytes([
            self._command_id & 0xFF,
            (self._command_id >> 8) & 0xFF,
            0xCC, 0, 0, 0, 0, 0,
        ])
        self._send_frame(0x7FF, data)

    # --- 配置、使能和确认 ----------------------------------------------
    def _configure_mode(self) -> bool:
        if not self._operation_lock.acquire(blocking=False):
            self.get_logger().warn("configure rejected: another device operation is active")
            return False
        try:
            with self._lock:
                if self._enabled_requested:
                    self.get_logger().error("configure rejected: disable the gripper first")
                    return False
            self._mode_event.clear()
            with self._lock:
                self._last_mode_value = None
                self._confirmed_mode = None
                self._mode_request_pending = True
            self._send_mode_request(self._desired_mode)
            if not self._mode_event.wait(self._response_timeout):
                self.get_logger().error("control mode confirmation timed out")
                return False
            with self._lock:
                confirmed = self._last_mode_value
            if confirmed != self._desired_mode:
                self.get_logger().error(
                    f"control mode mismatch: requested={self._mode_name(self._desired_mode)} "
                    f"received={self._mode_name(confirmed)}")
                return False
            with self._lock:
                self._confirmed_mode = confirmed
            self.get_logger().info(
                f"control mode confirmed: {self._mode_name(confirmed)}")
            if self._verify_limits_on_configure and not self._verify_limits():
                with self._lock:
                    self._confirmed_mode = None
                return False
            return True
        finally:
            with self._lock:
                self._mode_request_pending = False
            self._operation_lock.release()

    def _read_parameter_f32(self, register_id: int) -> Optional[float]:
        self._parameter_event.clear()
        with self._lock:
            self._pending_parameter_id = register_id
            self._last_parameter_value = None
        self._send_parameter_read(register_id)
        received = self._parameter_event.wait(self._response_timeout)
        with self._lock:
            value = self._last_parameter_value
            self._pending_parameter_id = None
        if not received:
            self.get_logger().error(
                f"parameter read timed out: register {register_id}")
            return None
        return value

    def _verify_limits(self) -> bool:
        """读取固件 MIT 量程，防止主机与设备使用不同的缩放参数。"""
        expected = (
            (_RID_PMAX, "PMAX", self._limits.pmax),
            (_RID_VMAX, "VMAX", self._limits.vmax),
            (_RID_TMAX, "TMAX", self._limits.tmax),
        )
        for register_id, name, configured in expected:
            actual = self._read_parameter_f32(register_id)
            if actual is None:
                return False
            if not math.isclose(actual, configured, rel_tol=1e-4, abs_tol=1e-4):
                self.get_logger().error(
                    f"{name} mismatch: ROS={configured} firmware={actual}; "
                    "update the ROS parameters or configure the motor before enabling")
                return False
        self.get_logger().info("firmware MIT limits confirmed")
        return True

    def _enable(self) -> Tuple[bool, str]:
        if not self._operation_lock.acquire(blocking=False):
            return False, "another device operation is active"
        try:
            with self._lock:
                if self._enabled_requested:
                    if (self._confirmed_mode == self._desired_mode
                            and self._feedback_is_fresh()):
                        return True, "already enabled"
                    return False, "gripper state is inconsistent; disable it first"
                generation = self._disable_generation
            if self._configure_mode_on_enable and not self._configure_mode():
                return False, "control mode was not confirmed"
            if not self._configure_mode_on_enable:
                with self._lock:
                    self._confirmed_mode = self._desired_mode
                self.get_logger().warn(
                    "control mode confirmation is disabled; assuming configured mode")
            with self._lock:
                if generation != self._disable_generation:
                    return False, "enable cancelled by disable request"
            self._feedback_event.clear()
            self._send_ctrl(_CTRL_ENABLE)
            self._request_state()
            if self._feedback_event.wait(self._response_timeout):
                with self._lock:
                    cancelled = generation != self._disable_generation
                    if not cancelled:
                        self._enabled_requested = True
                if cancelled:
                    self._send_ctrl(_CTRL_DISABLE)
                    return False, "enable cancelled; disable command sent again"
                return True, "enabled and feedback confirmed"
            self._send_ctrl(_CTRL_DISABLE)
            with self._lock:
                self._disable_generation += 1
                self._enabled_requested = False
            return False, "enable was not confirmed; disable command sent"
        finally:
            self._operation_lock.release()

    def _enable_once(self) -> None:
        if self._enable_timer is not None:
            self._enable_timer.cancel()
        success, message = self._enable()
        if success:
            self.get_logger().info(f"enable_on_start: {message}")
        else:
            self.get_logger().error(f"enable_on_start: {message}")

    # --- 命令 -----------------------------------------------------------
    def _clamp_position(self, position: float) -> float:
        return min(max(position, self._safe_min), self._safe_max)

    def _feedback_is_fresh(self) -> bool:
        with self._lock:
            stamp = self._last_feedback_monotonic
        return stamp > 0.0 and (time.monotonic() - stamp) <= self._feedback_timeout

    def _command_allowed(self, required_mode: int) -> bool:
        with self._lock:
            enabled = self._enabled_requested
            mode = self._confirmed_mode
        if self._require_enabled and not enabled:
            self.get_logger().warn("command rejected: gripper is not enabled")
            return False
        if mode != required_mode:
            self.get_logger().warn(
                f"command rejected: {self._mode_name(required_mode)} mode is not confirmed")
            return False
        if self._require_fresh and not self._feedback_is_fresh():
            self.get_logger().warn("command rejected: feedback is stale")
            return False
        return True

    @staticmethod
    def _finite(values: Sequence[float]) -> bool:
        return all(math.isfinite(float(value)) for value in values)

    def _send_mit(self, q: float, dq: float, kp: float, kd: float, tau: float) -> None:
        if not self._finite((q, dq, kp, kd, tau)):
            self.get_logger().warn("MIT command rejected: values must be finite")
            return
        if not 0.0 <= kp <= 500.0 or not 0.0 <= kd <= 5.0:
            self.get_logger().warn("MIT command rejected: kp/kd outside protocol limits")
            return
        if abs(dq) > self._limits.vmax or abs(tau) > self._limits.tmax:
            self.get_logger().warn("MIT command rejected: dq/tau outside configured limits")
            return
        if not self._command_allowed(_MODE_MIT):
            return
        requested_q = float(q)
        q = self._clamp_position(requested_q)
        if q != requested_q:
            self.get_logger().warn(
                f"MIT position clamped from {requested_q} to {q}")
        data = pack_mit_command(
            kp=float(kp), kd=float(kd), q=q, dq=float(dq), tau=float(tau),
            limits=self._limits)
        self._send_frame(self._command_id, data)

    def _send_pv(self, position: float, velocity: float) -> None:
        if not self._finite((position, velocity)):
            self.get_logger().warn("PV command rejected: values must be finite")
            return
        if not self._command_allowed(_MODE_POS_VEL):
            return
        requested_position = float(position)
        position = self._clamp_position(requested_position)
        if position != requested_position:
            self.get_logger().warn(
                f"PV position clamped from {requested_position} to {position}")
        if not 0.0 <= velocity <= self._limits.vmax:
            self.get_logger().warn("PV command rejected: velocity outside [0, vmax]")
            return
        self._send_frame(
            0x100 + (self._command_id & 0x7FF),
            struct.pack("<ff", position, velocity),
        )

    def _on_command(self, msg: Float64) -> None:
        """兼容接口：按当前配置模式发送位置命令。"""
        if self._desired_mode == _MODE_MIT:
            self._send_mit(msg.data, 0.0, self._kp, self._kd, 0.0)
        else:
            self._send_pv(msg.data, self._pv_velocity)

    def _on_mit_command(self, msg: MitCommand) -> None:
        """处理强类型 MIT 阻抗/扭矩命令。"""
        self._send_mit(msg.q, msg.dq, msg.kp, msg.kd, msg.tau)

    def _on_pv_command(self, msg: PvCommand) -> None:
        """处理强类型 PV 位置速度命令。"""
        self._send_pv(msg.position, msg.velocity)

    # --- 服务 -----------------------------------------------------------
    def _srv_enable_cb(self, _request, response: Trigger.Response):
        response.success, response.message = self._enable()
        return response

    def _srv_disable_cb(self, _request, response: Trigger.Response):
        self._send_ctrl(_CTRL_DISABLE)
        with self._lock:
            self._disable_generation += 1
            self._enabled_requested = False
        response.success = True
        response.message = "disable command sent (firmware has no acknowledgement)"
        return response

    def _srv_zero_cb(self, _request, response: Trigger.Response):
        if not self._allow_set_zero:
            response.success = False
            response.message = "set_zero is disabled; set allow_set_zero=true explicitly"
            return response
        with self._lock:
            if self._enabled_requested:
                response.success = False
                response.message = "disable the gripper before changing its zero"
                return response
        if not self._operation_lock.acquire(blocking=False):
            response.success = False
            response.message = "another device operation is active"
            return response
        try:
            self._feedback_event.clear()
            self._send_ctrl(_CTRL_SET_ZERO)
            self._request_state()
            response.success = self._feedback_event.wait(self._response_timeout)
        finally:
            self._operation_lock.release()
        response.message = (
            "zero command sent and feedback confirmed" if response.success
            else "zero command sent, but no feedback was received")
        return response

    def _srv_configure_cb(self, _request, response: Trigger.Response):
        response.success = self._configure_mode()
        response.message = (
            f"mode confirmed: {self._mode_name(self._desired_mode)}"
            if response.success else "control mode confirmation failed")
        return response

    def _srv_refresh_cb(self, _request, response: Trigger.Response):
        if not self._operation_lock.acquire(blocking=False):
            response.success = False
            response.message = "another device operation is active"
            return response
        try:
            self._feedback_event.clear()
            self._request_state()
            response.success = self._feedback_event.wait(self._response_timeout)
        finally:
            self._operation_lock.release()
        response.message = (
            "feedback received" if response.success else "feedback timed out")
        return response

    def _poll_state(self) -> None:
        """使能期间周期请求状态，维持反馈健康度并检测掉线。"""
        if self._shutting_down:
            return
        with self._lock:
            enabled = self._enabled_requested
        if not enabled:
            return
        if (self._disable_on_feedback_timeout
                and not self._feedback_is_fresh()):
            self._send_ctrl(_CTRL_DISABLE)
            with self._lock:
                self._disable_generation += 1
                self._enabled_requested = False
            self.get_logger().error(
                "feedback timed out; disable command sent and motion blocked")
            return
        if self._operation_lock.acquire(blocking=False):
            try:
                self._request_state()
            finally:
                self._operation_lock.release()

    # --- 反馈 -----------------------------------------------------------
    def _can_id_belongs_to_device(self, can_id: int) -> bool:
        return can_id in (self._feedback_id, self._command_id, 0x00)

    def _state_feedback_belongs_to_device(self, can_id: int, data: bytes) -> bool:
        if can_id == 0x00:
            return (data[0] & 0x0F) == (self._command_id & 0x0F)
        return data[0] == (self._command_id & 0xFF)

    def _on_frame(self, frame: Frame) -> None:
        if frame.is_error or frame.is_rtr or frame.is_extended:
            return
        if int(frame.dlc) != 8:
            return
        data = bytes(bytearray(frame.data))[:8]
        can_id = int(frame.id)
        if not self._can_id_belongs_to_device(can_id):
            return

        with self._lock:
            mode_pending = self._mode_request_pending
            parameter_pending = self._pending_parameter_id
        is_mode_reply = (
            mode_pending and data[2] in (0x33, 0x55)
            and int(data[3]) == _RID_CTRL_MODE)
        is_parameter_reply = (
            parameter_pending is not None and data[2] in (0x33, 0x55)
            and int(data[3]) == parameter_pending)
        if is_mode_reply or is_parameter_reply:
            register_id = int(data[3])
            if is_mode_reply:
                value = struct.unpack("<I", data[4:8])[0]
                with self._lock:
                    self._last_mode_value = int(value)
                self._mode_event.set()
            if is_parameter_reply:
                value = struct.unpack("<f", data[4:8])[0]
                with self._lock:
                    self._last_parameter_value = float(value)
                self._parameter_event.set()
            return

        if not self._state_feedback_belongs_to_device(can_id, data):
            return

        try:
            feedback = unpack_mit_feedback(data, limits=self._limits)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"unpack_mit_feedback failed: {exc}")
            return

        now = time.monotonic()
        with self._lock:
            first = not self._got_feedback
            self._got_feedback = True
            self._last_feedback_monotonic = now
            self._last_position = float(feedback.position)
            self._last_velocity = float(feedback.velocity)
            self._last_torque = float(feedback.torque)
        self._feedback_event.set()
        if first:
            self.get_logger().info(
                f"first feedback: pos={feedback.position:+.3f} "
                f"vel={feedback.velocity:+.3f} tau={feedback.torque:+.3f}")

        state = JointState()
        if frame.header.stamp.sec or frame.header.stamp.nanosec:
            state.header.stamp = frame.header.stamp
        else:
            state.header.stamp = self.get_clock().now().to_msg()
        state.name = [self._joint_name]
        state.position = [float(feedback.position)]
        state.velocity = [float(feedback.velocity)]
        state.effort = [float(feedback.torque)]
        self._state_pub.publish(state)

    # --- 诊断与关闭 -----------------------------------------------------
    def _publish_diagnostics(self) -> None:
        if self._shutting_down:
            return
        with self._lock:
            got_feedback = self._got_feedback
            age = (time.monotonic() - self._last_feedback_monotonic
                   if got_feedback else math.inf)
            enabled = self._enabled_requested
            mode = self._confirmed_mode
            position = self._last_position
            velocity = self._last_velocity
            torque = self._last_torque

        status = DiagnosticStatus()
        status.name = f"{self.get_fully_qualified_name()}: Gloria-M"
        status.hardware_id = (
            f"can:0x{self._command_id:X}/0x{self._feedback_id:X}")
        if not got_feedback:
            status.level = DiagnosticStatus.ERROR if enabled else DiagnosticStatus.WARN
            status.message = "no feedback"
        elif age > self._feedback_timeout:
            status.level = DiagnosticStatus.ERROR if enabled else DiagnosticStatus.WARN
            status.message = "feedback stale"
        elif mode != self._desired_mode:
            status.level = DiagnosticStatus.WARN
            status.message = "control mode not confirmed"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "enabled" if enabled else "ready (disabled)"
        status.values = [
            KeyValue(key="enabled_requested", value=str(enabled)),
            KeyValue(key="desired_mode", value=self._mode_name(self._desired_mode)),
            KeyValue(key="confirmed_mode", value=self._mode_name(mode)),
            KeyValue(key="feedback_age_s", value=(f"{age:.3f}" if got_feedback else "inf")),
            KeyValue(key="position_rad", value=f"{position:.6f}"),
            KeyValue(key="velocity_rad_s", value=f"{velocity:.6f}"),
            KeyValue(key="torque_nm", value=f"{torque:.6f}"),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._diag_pub.publish(message)

    def destroy_node(self) -> bool:
        self._shutting_down = True
        if self._enable_timer is not None:
            self._enable_timer.cancel()
        if self._disable_on_shutdown and self._enabled_requested:
            try:
                self._send_ctrl(_CTRL_DISABLE)
                with self._lock:
                    self._enabled_requested = False
                self.get_logger().info("disable sent during shutdown")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"shutdown disable failed: {exc}")
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node: Optional[GloriaGripperNode] = None
    try:
        node = GloriaGripperNode()
    except Exception as exc:  # noqa: BLE001
        rclpy.logging.get_logger("gloria_gripper").fatal(str(exc))
        if rclpy.ok():
            rclpy.shutdown()
        return

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
