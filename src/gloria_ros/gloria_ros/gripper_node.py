"""ROS 2 Gloria-M 夹爪设备节点（共享 CAN bridge 模式）

节点不打开 SDK 自带的串口转 CAN 设备，只复用 Gloria-M SDK 的纯协议层，通过 ``can_bridge_ros`` 与项目 CAN 设备通信
支持 MIT、PV 两种控制模式、模式回读确认、反馈在线检测、安全位置限幅、诊断信息以及退出失能
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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from gloria_ros.can_feedback import (
    is_register_reply,
    register_reply_belongs_to_device,
    state_feedback_belongs_to_device,
)

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
_CTRL_FRAME_PREFIX = b"\xFF" * 7
_REQUEST_FRAME = struct.Struct("<HBBI")
_UINT32 = struct.Struct("<I")
_FLOAT32 = struct.Struct("<f")
_PV_COMMAND = struct.Struct("<ff")
_LOG_THROTTLE_S = 1.0


class GloriaGripperNode(Node):
    """一台 Gloria-M 夹爪的 ROS2 设备节点"""

    def __init__(self) -> None:
        super().__init__("gloria_gripper")
        # 服务会同步等待 CAN 应答，因此必须允许订阅回调在另一执行器线程中运行
        self._cb_group = ReentrantCallbackGroup()  # 允许服务和定时器回调并发执行
        self._rx_cb_group = MutuallyExclusiveCallbackGroup()  # 按接收顺序串行处理 CAN 反馈帧
        self._command_cb_group = MutuallyExclusiveCallbackGroup()  # 串行处理 MIT 与 PV 运动命令
        self._lock = threading.RLock()  # 保护反馈、模式和使能状态等共享数据
        # 操作锁串行化有应答的设备操作，RLock 允许使能流程内部调用配置流程
        self._operation_lock = threading.RLock()  # 串行化配置、使能、刷新和归零等设备操作
        self._feedback_event = threading.Event()  # 通知等待线程已收到有效状态反馈
        self._mode_event = threading.Event()       # 通知等待线程已收到控制模式回包
        self._parameter_event = threading.Event()  # 通知等待线程已收到参数读取回包

        self.declare_parameter("rx_topic", "/can0/rx")  # CAN bridge 转发设备反馈帧的输入话题
        self.declare_parameter("tx_topic", "/can0/tx")  # 向 CAN bridge 发送设备控制帧的输出话题
        self.declare_parameter("command_id", 0x01)  # 设备接收 MIT 和通用控制命令的标准 CAN ID
        self.declare_parameter("feedback_id", 0x101)  # 设备发送状态和寄存器回包的标准 CAN ID
        self.declare_parameter("control_mode", "mit")  # 节点启动后期望配置的控制模式
        self.declare_parameter("pmax", 3.14)  # MIT 协议位置字段的正负编解码量程
        self.declare_parameter("vmax", 10.0)  # MIT 协议速度字段的正负编解码量程
        self.declare_parameter("tmax", 12.0)  # MIT 协议力矩字段的正负编解码量程
        self.declare_parameter("safe_position_min", 0.0)  # 允许发送的最小目标位置
        self.declare_parameter("safe_position_max", 2.77)  # 允许发送的最大目标位置
        self.declare_parameter("joint_name", "gripper")  # JointState 消息使用的关节名称
        self.declare_parameter("state_topic", "~/joint_states")  # 发布夹爪状态的 ROS 话题
        self.declare_parameter("mit_command_topic", "~/mit_command")  # 接收 MIT 阻抗和力矩命令的 ROS 话题
        self.declare_parameter("pv_command_topic", "~/pv_command")  # 接收 PV 位置速度命令的 ROS 话题
        self.declare_parameter("enable_on_start", False)  # 是否在节点启动后延时自动配置并使能设备
        self.declare_parameter("configure_mode_on_enable", True)  # 调用使能服务时是否先配置并确认控制模式
        self.declare_parameter("verify_limits_on_configure", True)  # 配置模式后是否读取并核对固件 MIT 量程
        self.declare_parameter("allow_set_zero", False)  # 是否允许通过服务修改设备机械零点
        self.declare_parameter("require_enabled_for_command", True)  # 是否拒绝主机未请求使能时收到的运动命令
        self.declare_parameter("require_fresh_feedback", True)  # 是否拒绝设备反馈过期时收到的运动命令
        self.declare_parameter("feedback_timeout_s", 0.5)  # 将最近设备反馈判定为过期的时间阈值
        self.declare_parameter("response_timeout_s", 0.5)  # 等待模式、参数或状态应答的最大时长
        self.declare_parameter("state_poll_period_s", 0.1)  # 设备使能后主动请求状态的周期
        self.declare_parameter("disable_on_feedback_timeout", True)  # 反馈超时后是否自动发送设备失能命令
        self.declare_parameter("disable_on_shutdown", True)  # 节点关闭时是否向已使能设备发送失能命令
        self.declare_parameter("diagnostic_period_s", 0.0)  # 发布 ROS 诊断消息的周期，非正值表示关闭诊断

        # 参数只在启动时读取，节点运行期间不做动态参数更新
        gp = self.get_parameter
        rx_topic = gp("rx_topic").get_parameter_value().string_value
        tx_topic = gp("tx_topic").get_parameter_value().string_value
        self._command_id = gp("command_id").get_parameter_value().integer_value  # 设备控制命令使用的标准 CAN ID
        self._feedback_id = gp("feedback_id").get_parameter_value().integer_value  # 设备状态和寄存器回包使用的 CAN ID
        self._pv_command_id = 0x100 + self._command_id  # PV 位置速度命令对应的 CAN ID
        self._desired_mode = self._parse_mode(gp("control_mode").get_parameter_value().string_value)  # 当前期望写入设备的控制模式
        self._limits = Limits(
            pmax=gp("pmax").get_parameter_value().double_value,
            vmax=gp("vmax").get_parameter_value().double_value,
            tmax=gp("tmax").get_parameter_value().double_value,
        )  # MIT 协议位置、速度和力矩的编解码量程
        self._safe_min = gp("safe_position_min").get_parameter_value().double_value  # 软件允许的最小目标位置
        self._safe_max = gp("safe_position_max").get_parameter_value().double_value  # 软件允许的最大目标位置
        self._joint_name = gp("joint_name").get_parameter_value().string_value  # JointState 消息中的关节名称
        state_topic = gp("state_topic").get_parameter_value().string_value
        mit_command_topic = gp("mit_command_topic").get_parameter_value().string_value
        pv_command_topic = gp("pv_command_topic").get_parameter_value().string_value
        enable_on_start = gp("enable_on_start").get_parameter_value().bool_value
        self._configure_mode_on_enable = gp("configure_mode_on_enable").get_parameter_value().bool_value  # 使能前是否自动配置并确认控制模式
        self._verify_limits_on_configure = gp("verify_limits_on_configure").get_parameter_value().bool_value  # 配置模式时是否核对固件量程
        self._allow_set_zero = gp("allow_set_zero").get_parameter_value().bool_value  # 是否开放修改机械零点的服务
        self._require_enabled = gp("require_enabled_for_command").get_parameter_value().bool_value  # 是否只允许在已请求使能状态下发送运动命令
        self._require_fresh = gp("require_fresh_feedback").get_parameter_value().bool_value  # 是否要求反馈新鲜后才能发送运动命令
        self._feedback_timeout = gp("feedback_timeout_s").get_parameter_value().double_value  # 判断反馈过期的时间阈值
        self._response_timeout = gp("response_timeout_s").get_parameter_value().double_value  # 等待模式、参数和状态应答的最大时长
        self._state_poll_period = gp("state_poll_period_s").get_parameter_value().double_value  # 使能期间主动请求状态的周期
        self._disable_on_feedback_timeout = gp("disable_on_feedback_timeout").get_parameter_value().bool_value  # 反馈超时后是否自动发送失能
        self._disable_on_shutdown = gp("disable_on_shutdown").get_parameter_value().bool_value  # 节点关闭时是否向已使能设备发送失能
        diagnostic_period = gp("diagnostic_period_s").get_parameter_value().double_value
        self._validate_configuration(diagnostic_period)
        namespace = self.get_namespace().rstrip("/")
        self._diagnostic_name = f"{namespace}/{self.get_name()}: Gloria-M"  # /diagnostics 中用于匹配本节点的状态名称
        self._hardware_id = f"can:0x{self._command_id:X}/0x{self._feedback_id:X}"  # 诊断消息中显示的设备 CAN 标识

        # 共享状态只在锁内更新，等待事件仅负责跨回调唤醒
        self._got_feedback = False  # 标记节点是否至少收到过一次有效状态反馈
        self._last_feedback_monotonic = 0.0  # 最近有效反馈对应的单调时钟时间
        self._enabled_requested = False  # 记录主机侧是否已成功请求设备使能
        self._enable_in_progress = False  # 标记使能流程是否已开始但尚未完成
        self._confirmed_mode: Optional[int] = None  # 已通过设备回包确认的控制模式
        self._last_mode_value: Optional[int] = None  # 最近一次控制模式寄存器回包值
        self._mode_request_pending = False  # 标记当前是否正在等待控制模式回包
        self._pending_parameter_id: Optional[int] = None  # 当前等待回包的参数寄存器 ID
        self._last_parameter_value: Optional[float] = None  # 最近一次参数读取回包值
        self._disable_generation = 0  # 失能请求代次，用于取消并发中的使能流程
        self._shutting_down = False  # 标记节点是否进入关闭流程
        self._auto_enable_pending = enable_on_start  # 通信就绪后仍需执行启动自动使能

        rx_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        tx_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # 高频反馈使用 BEST_EFFORT，关键控制帧使用 RELIABLE
        self._tx_pub = self.create_publisher(Frame, tx_topic, tx_qos)  # 向 CAN bridge 发布设备控制帧
        self._state_pub = self.create_publisher(JointState, state_topic, state_qos)  # 发布夹爪位置、速度和反馈力矩
        self._diag_pub = None  # 发布节点在线状态、模式和反馈健康度，诊断关闭时不创建
        self._diag_timer = None  # 周期发布诊断消息，诊断关闭时不创建
        if diagnostic_period > 0.0:
            self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", QoSProfile(depth=1))
            self._diag_timer = self.create_timer(diagnostic_period, self._publish_diagnostics, callback_group=self._cb_group)
        self._rx_sub = self.create_subscription(
            Frame, rx_topic, self._on_frame, rx_qos,
            callback_group=self._rx_cb_group)  # 订阅 CAN bridge 转发的设备反馈帧
        self._mit_cmd_sub = self.create_subscription(
            MitCommand, mit_command_topic, self._on_mit_command, command_qos,
            callback_group=self._command_cb_group)  # 订阅 MIT 阻抗和力矩命令
        self._pv_cmd_sub = self.create_subscription(
            PvCommand, pv_command_topic, self._on_pv_command, command_qos,
            callback_group=self._command_cb_group)  # 订阅 PV 位置速度命令
        self._srv_enable = self.create_service(
            Trigger, "~/enable", self._srv_enable_cb,
            callback_group=self._cb_group)  # 提供模式配置、量程确认和设备使能服务
        self._srv_disable = self.create_service(
            Trigger, "~/disable", self._srv_disable_cb,
            callback_group=self._cb_group)  # 提供可取消并发使能流程的设备失能服务
        self._srv_zero = self.create_service(
            Trigger, "~/set_zero", self._srv_zero_cb,
            callback_group=self._cb_group)  # 提供受参数保护的机械零点设置服务
        self._srv_configure = self.create_service(
            Trigger, "~/configure", self._srv_configure_cb,
            callback_group=self._cb_group)  # 提供控制模式写入、回读和量程核验服务
        self._srv_set_mode = self.create_service(
            SetBool, "~/set_mode", self._srv_set_mode_cb,
            callback_group=self._cb_group)  # 提供失能状态下选择待配置模式的服务
        self._srv_refresh = self.create_service(
            Trigger, "~/refresh", self._srv_refresh_cb,
            callback_group=self._cb_group)  # 提供主动请求并等待一帧状态反馈的服务

        self._state_timer = self.create_timer(
            self._state_poll_period, self._poll_state,
            callback_group=self._cb_group)  # 使能期间周期请求状态并检查反馈超时
        self._enable_timer = None  # 保存等待通信就绪并自动使能的重试定时器
        if enable_on_start:
            self._enable_timer = self.create_timer(
            1.0, self._auto_enable, callback_group=self._cb_group)  # 收到新鲜反馈后尝试使能，失败则保留重试

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
        if mode is None:
            return "unknown"
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
            self._safe_min, self._safe_max,
            self._feedback_timeout, self._response_timeout,
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
        if self._feedback_timeout <= 0 or self._response_timeout <= 0:
            raise ValueError("反馈和响应超时必须大于零")
        if self._state_poll_period <= 0:
            raise ValueError("state_poll_period_s 必须大于零")

    # --- CAN 帧 ---------------------------------------------------------
    def _send_frame(self, can_id: int, data: bytes) -> None:
        if len(data) > 8:
            raise ValueError("CAN 数据长度不能超过 8 字节")
        frame = Frame()
        frame.id = can_id
        frame.is_extended = False
        frame.is_rtr = False
        frame.is_error = False
        frame.dlc = len(data)
        frame.data = list(data.ljust(8, b"\x00"))
        self._tx_pub.publish(frame)

    def _send_ctrl(self, command: int) -> None:
        self._send_frame(self._command_id, _CTRL_FRAME_PREFIX + bytes((command & 0xFF,)))

    def _send_mode_request(self, mode: int) -> None:
        self._send_frame(0x7FF, _REQUEST_FRAME.pack(self._command_id, 0x55, _RID_CTRL_MODE, mode))

    def _send_parameter_read(self, register_id: int) -> None:
        self._send_frame(0x7FF, _REQUEST_FRAME.pack(self._command_id, 0x33, register_id & 0xFF, 0))

    def _request_state(self) -> None:
        self._send_frame(0x7FF, _REQUEST_FRAME.pack(self._command_id, 0xCC, 0, 0))

    def _wait_for_feedback_after(self, wait_after: float) -> bool:
        # Event 只负责唤醒，时间戳用于排除本次请求发送前已进入回调的旧反馈
        deadline = time.monotonic() + self._response_timeout
        while True:
            self._feedback_event.clear()
            with self._lock:
                if self._last_feedback_monotonic >= wait_after:
                    return True
            remaining = deadline - time.monotonic()
            if remaining <= 0.0 or not self._feedback_event.wait(remaining):
                return False

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
        try:
            self._send_parameter_read(register_id)
            received = self._parameter_event.wait(self._response_timeout)
            with self._lock:
                value = self._last_parameter_value
            if not received:
                self.get_logger().error(f"parameter read timed out: register {register_id}")
                return None
            return value
        finally:
            # 即使发布请求或等待过程异常，也不能让后续回包误命中过期请求
            with self._lock:
                self._pending_parameter_id = None

    def _verify_limits(self) -> bool:
        """读取固件 MIT 量程，防止主机与设备使用不同的缩放参数"""
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

    def _request_disable(self) -> None:
        # 先更新主机状态以阻断并发运动命令，再向设备发送失能帧
        with self._lock:
            self._disable_generation += 1
            self._enabled_requested = False
            self._send_ctrl(_CTRL_DISABLE)

    def _enable(
            self, expected_generation: Optional[int] = None
            ) -> Tuple[bool, str]:
        if not self._operation_lock.acquire(blocking=False):
            return False, "another device operation is active"
        try:
            with self._lock:
                if self._shutting_down:
                    return False, "node is shutting down"
                if (expected_generation is not None
                        and (not self._auto_enable_pending
                             or expected_generation != self._disable_generation)):
                    return False, "automatic enable cancelled"
                if self._enabled_requested:
                    if (self._confirmed_mode == self._desired_mode
                            and self._feedback_stamp_is_fresh(self._last_feedback_monotonic)):
                        return True, "already enabled"
                    return False, "gripper state is inconsistent; disable it first"
                self._enable_in_progress = True
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
                if self._shutting_down:
                    return False, "node is shutting down"
                # 在同一临界区内确认取消代次并发送使能，保证并发失能命令的发送顺序
                self._feedback_event.clear()
                self._send_ctrl(_CTRL_ENABLE)
                wait_after = time.monotonic()
                self._request_state()
            if self._wait_for_feedback_after(wait_after):
                with self._lock:
                    cancelled = generation != self._disable_generation
                    if not cancelled:
                        self._enabled_requested = True
                if cancelled:
                    self._send_ctrl(_CTRL_DISABLE)
                    return False, "enable cancelled; disable command sent again"
                return True, "enabled and feedback confirmed"
            self._request_disable()
            return False, "enable was not confirmed; disable command sent"
        finally:
            with self._lock:
                self._enable_in_progress = False
            self._operation_lock.release()

    def _cancel_auto_enable(self) -> None:
        with self._lock:
            self._auto_enable_pending = False
        if self._enable_timer is not None:
            self._enable_timer.cancel()

    def _auto_enable(self) -> None:
        with self._lock:
            # 新鲜反馈表示 bridge 和设备已经可通信；generation 则防止显式
            # disable 与在途自动使能交错后，又把设备重新拉起。
            if (self._shutting_down
                    or not self._auto_enable_pending
                    or not self._feedback_stamp_is_fresh(
                        self._last_feedback_monotonic)):
                return
            generation = self._disable_generation
        success, message = self._enable(expected_generation=generation)
        if success:
            self._cancel_auto_enable()
            self.get_logger().info(f"enable_on_start: {message}")
        elif message not in (
                "another device operation is active",
                "automatic enable cancelled"):
            self.get_logger().warn(
                f"enable_on_start retry pending: {message}")

    # --- 命令 -----------------------------------------------------------
    def _clamp_position(self, position: float) -> float:
        return min(max(position, self._safe_min), self._safe_max)

    def _feedback_stamp_is_fresh(self, stamp: float) -> bool:
        return stamp > 0.0 and (time.monotonic() - stamp) <= self._feedback_timeout

    def _command_allowed(self, required_mode: int) -> bool:
        # 锁内只复制判断所需状态，日志和时间计算放到锁外
        with self._lock:
            enabled = self._enabled_requested
            mode = self._confirmed_mode
            feedback_stamp = self._last_feedback_monotonic
        if self._require_enabled and not enabled:
            self.get_logger().warn("command rejected: gripper is not enabled", throttle_duration_sec=_LOG_THROTTLE_S)
            return False
        if mode != required_mode:
            self.get_logger().warn(
                f"command rejected: {self._mode_name(required_mode)} mode is not confirmed",
                throttle_duration_sec=_LOG_THROTTLE_S)
            return False
        if self._require_fresh and not self._feedback_stamp_is_fresh(feedback_stamp):
            self.get_logger().warn("command rejected: feedback is stale", throttle_duration_sec=_LOG_THROTTLE_S)
            return False
        return True

    @staticmethod
    def _finite(values: Sequence[float]) -> bool:
        return all(math.isfinite(value) for value in values)

    def _send_mit(self, q: float, dq: float, kp: float, kd: float, tau: float) -> None:
        if not self._finite((q, dq, kp, kd, tau)):
            self.get_logger().warn("MIT command rejected: values must be finite", throttle_duration_sec=_LOG_THROTTLE_S)
            return
        if not 0.0 <= kp <= 500.0 or not 0.0 <= kd <= 5.0:
            self.get_logger().warn("MIT command rejected: kp/kd outside protocol limits", throttle_duration_sec=_LOG_THROTTLE_S)
            return
        if abs(dq) > self._limits.vmax or abs(tau) > self._limits.tmax:
            self.get_logger().warn("MIT command rejected: dq/tau outside configured limits", throttle_duration_sec=_LOG_THROTTLE_S)
            return
        requested_q = q
        q = self._clamp_position(requested_q)
        # 状态检查与发帧共用同一把锁，确保并发失能帧不会先于运动帧发出
        with self._lock:
            if not self._command_allowed(_MODE_MIT):
                return
            if q != requested_q:
                self.get_logger().warn(
                    f"MIT position clamped from {requested_q} to {q}",
                    throttle_duration_sec=_LOG_THROTTLE_S)
            data = pack_mit_command(
                kp=kp, kd=kd, q=q, dq=dq, tau=tau,
                limits=self._limits)
            self._send_frame(self._command_id, data)

    def _send_pv(self, position: float, velocity: float) -> None:
        if not self._finite((position, velocity)):
            self.get_logger().warn("PV command rejected: values must be finite", throttle_duration_sec=_LOG_THROTTLE_S)
            return
        requested_position = position
        position = self._clamp_position(requested_position)
        # 状态检查与发帧共用同一把锁，确保并发失能帧不会先于运动帧发出
        with self._lock:
            if not self._command_allowed(_MODE_POS_VEL):
                return
            if position != requested_position:
                self.get_logger().warn(
                    f"PV position clamped from {requested_position} to {position}",
                    throttle_duration_sec=_LOG_THROTTLE_S)
            if not 0.0 <= velocity <= self._limits.vmax:
                self.get_logger().warn("PV command rejected: velocity outside [0, vmax]", throttle_duration_sec=_LOG_THROTTLE_S)
                return
            self._send_frame(self._pv_command_id, _PV_COMMAND.pack(position, velocity))

    def _on_mit_command(self, msg: MitCommand) -> None:
        """处理强类型 MIT 阻抗/扭矩命令"""
        self._send_mit(msg.q, msg.dq, msg.kp, msg.kd, msg.tau)

    def _on_pv_command(self, msg: PvCommand) -> None:
        """处理强类型 PV 位置速度命令"""
        self._send_pv(msg.position, msg.velocity)

    # --- 服务 -----------------------------------------------------------
    def _srv_enable_cb(self, _request, response: Trigger.Response):
        response.success, response.message = self._enable()
        if response.success:
            self._cancel_auto_enable()
        return response

    def _srv_disable_cb(self, _request, response: Trigger.Response):
        # 失能不等待操作锁，确保它可以取消正在等待反馈的使能流程
        self._cancel_auto_enable()
        self._request_disable()
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
            with self._lock:
                self._feedback_event.clear()
                self._send_ctrl(_CTRL_SET_ZERO)
                wait_after = time.monotonic()
                self._request_state()
            response.success = self._wait_for_feedback_after(wait_after)
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

    def _srv_set_mode_cb(self, request: SetBool.Request, response: SetBool.Response):
        selected_mode = _MODE_POS_VEL if request.data else _MODE_MIT
        if selected_mode == _MODE_POS_VEL and self._command_id > 0x6FF:
            response.success = False
            response.message = "PV mode requires command_id <= 0x6FF"
            return response
        if not self._operation_lock.acquire(blocking=False):
            response.success = False
            response.message = "another device operation is active"
            return response
        try:
            with self._lock:
                if self._enabled_requested:
                    response.success = False
                    response.message = "disable the gripper before changing mode"
                    return response
                self._desired_mode = selected_mode
                self._confirmed_mode = None
            response.success = True
            response.message = f"selected mode: {self._mode_name(selected_mode)}"
            return response
        finally:
            self._operation_lock.release()

    def _srv_refresh_cb(self, _request, response: Trigger.Response):
        if not self._operation_lock.acquire(blocking=False):
            response.success = False
            response.message = "another device operation is active"
            return response
        try:
            with self._lock:
                self._feedback_event.clear()
                wait_after = time.monotonic()
                self._request_state()
            response.success = self._wait_for_feedback_after(wait_after)
        finally:
            self._operation_lock.release()
        response.message = (
            "feedback received" if response.success else "feedback timed out")
        return response

    def _poll_state(self) -> None:
        """周期请求状态；使能期间同时检测反馈超时。"""
        timed_out = False
        with self._lock:
            if self._shutting_down:
                return
            if (self._enabled_requested
                    and self._disable_on_feedback_timeout
                    and not self._feedback_stamp_is_fresh(self._last_feedback_monotonic)):
                # 超时判断和主机状态切换保持原子性，避免新反馈插入后仍误触发失能
                self._disable_generation += 1
                self._enabled_requested = False
                self._send_ctrl(_CTRL_DISABLE)
                timed_out = True
        if timed_out:
            self.get_logger().error(
                "feedback timed out; disable command sent and motion blocked")
            return
        if self._operation_lock.acquire(blocking=False):
            try:
                with self._lock:
                    if self._shutting_down:
                        return
                self._request_state()
            finally:
                self._operation_lock.release()

    # --- 反馈 -----------------------------------------------------------
    def _on_frame(self, frame: Frame) -> None:
        if frame.is_error or frame.is_rtr or frame.is_extended:
            return
        if frame.dlc != 8:
            return
        received_at = time.monotonic()
        data = bytes(frame.data)
        can_id = frame.id

        # 寄存器回包只唤醒当前等待者，避免配置响应被误判为状态反馈
        register_reply = is_register_reply(data, self._command_id)
        if register_reply:
            if not register_reply_belongs_to_device(
                    can_id, self._command_id, self._feedback_id):
                return
            register_id = data[3]
            mode_received = False
            parameter_received = False
            with self._lock:
                if self._mode_request_pending and register_id == _RID_CTRL_MODE:
                    self._last_mode_value = _UINT32.unpack_from(data, 4)[0]
                    mode_received = True
                if self._pending_parameter_id == register_id:
                    self._last_parameter_value = _FLOAT32.unpack_from(data, 4)[0]
                    parameter_received = True
            if mode_received:
                self._mode_event.set()
            if parameter_received:
                self._parameter_event.set()
            return

        if not state_feedback_belongs_to_device(
                can_id, data, self._command_id, self._feedback_id):
            return

        # 状态帧是高频路径，字节数据和解码结果都只转换一次
        try:
            feedback = unpack_mit_feedback(data, limits=self._limits)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"unpack_mit_feedback failed: {exc}",
                throttle_duration_sec=_LOG_THROTTLE_S)
            return

        position = feedback.position
        velocity = feedback.velocity
        torque = feedback.torque
        with self._lock:
            first = not self._got_feedback
            self._got_feedback = True
            self._last_feedback_monotonic = received_at
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
        state.position = [position]
        state.velocity = [velocity]
        state.effort = [torque]
        self._state_pub.publish(state)

    # --- 诊断与关闭 -----------------------------------------------------
    def _publish_diagnostics(self) -> None:
        if self._diag_pub is None:
            return
        # 先复制一致快照，避免构造诊断消息时长期占用状态锁
        with self._lock:
            if self._shutting_down:
                return
            got_feedback = self._got_feedback
            age = (time.monotonic() - self._last_feedback_monotonic
                   if got_feedback else math.inf)
            enabled = self._enabled_requested
            mode = self._confirmed_mode
            desired_mode = self._desired_mode

        status = DiagnosticStatus()
        status.name = self._diagnostic_name
        status.hardware_id = self._hardware_id
        if not got_feedback:
            status.level = DiagnosticStatus.ERROR if enabled else DiagnosticStatus.WARN
            status.message = "no feedback"
        elif age > self._feedback_timeout:
            status.level = DiagnosticStatus.ERROR if enabled else DiagnosticStatus.WARN
            status.message = "feedback stale"
        elif mode != desired_mode:
            status.level = DiagnosticStatus.WARN
            status.message = "control mode not confirmed"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "enabled" if enabled else "ready (disabled)"
        status.values = [
            KeyValue(key="enabled_requested", value=str(enabled)),
            KeyValue(key="desired_mode", value=self._mode_name(desired_mode)),
            KeyValue(key="confirmed_mode", value=self._mode_name(mode)),
            KeyValue(key="feedback_age_s", value=(f"{age:.3f}" if got_feedback else "inf")),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._diag_pub.publish(message)

    def destroy_node(self) -> bool:
        with self._lock:
            self._shutting_down = True
            self._auto_enable_pending = False
            should_disable = self._enabled_requested or self._enable_in_progress
        if self._enable_timer is not None:
            self._enable_timer.cancel()
        if self._disable_on_shutdown and should_disable:
            try:
                # 同时覆盖已使能状态和尚未完成的在途使能流程
                self._request_disable()
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
        get_logger("gloria_gripper").fatal(str(exc))
        if rclpy.ok():
            rclpy.shutdown()
        raise

    # 服务回调会等待 CAN 应答，第二个线程负责继续处理反馈订阅
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
