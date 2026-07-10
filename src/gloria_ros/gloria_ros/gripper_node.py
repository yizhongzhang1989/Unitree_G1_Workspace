"""ROS 2 Gloria-M 夹爪设备节点（bridge 模式，骨架 + 能收发帧）。

不直接开总线，经通用 ``can_bridge`` 共享总线：
  - 订阅 ``rx_topic``（can_msgs/Frame），过滤本电机的 ``feedback_id``，用夹爪 SDK 的
    ``unpack_mit_feedback`` 解析 MIT 反馈，发布 ``sensor_msgs/JointState``；
  - 收到目标位置命令（std_msgs/Float64）时用 SDK 的 ``pack_mit_command`` 打包 MIT 帧，
    发布 can_msgs/Frame 到 ``tx_topic``（发往 ``command_id``）；
  - 使能/失能通过 ``~/enable`` ``~/disable`` 服务下发控制帧。

**复用夹爪 SDK 的协议层**（``gloria_m_sdk.protocol_mit`` / ``types``）——这些是纯逻辑、
兼容 Python 3.8，无需 SDK 的串口传输层（SDK 整体要求 3.11，但协议层不涉及）。

一夹爪一节点；同一条总线多个夹爪就多起几个节点（不同 command_id/feedback_id）。
先启动 ``can_bridge`` 独占物理总线。

⚠️ 骨架版：控制目前是"目标位置 + 固定 kp/kd 的 MIT"，寄存器/模式切换等按夹爪手册后续完善。
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

import rclpy
from can_msgs.msg import Frame
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

# 复用夹爪 SDK 的协议层：把 submodule 的 src 加入 sys.path（--symlink-install 下 __file__ 指向源码）
_SDK_SRC = Path(__file__).resolve().parents[2] / "Gloria-M-SDK" / "src"
if _SDK_SRC.is_dir() and str(_SDK_SRC) not in sys.path:
    sys.path.insert(0, str(_SDK_SRC))
try:
    from gloria_m_sdk.protocol_mit import pack_mit_command, unpack_mit_feedback
    from gloria_m_sdk.types import Limits
except Exception as _exc:  # noqa: BLE001
    raise ImportError(
        f"无法导入夹爪 SDK 协议层（gloria_m_sdk）：{_exc}. "
        f"确认 submodule 已拉取：{_SDK_SRC}") from _exc

# 控制帧（使能/失能/置零）：data = [0xFF*7, cmd]
_CTRL_ENABLE = 0xFC
_CTRL_DISABLE = 0xFD
_CTRL_SET_ZERO = 0xFE


class GloriaGripperNode(Node):
    def __init__(self) -> None:
        super().__init__("gloria_gripper")

        self.declare_parameter("rx_topic", "/can0/rx")
        self.declare_parameter("tx_topic", "/can0/tx")
        self.declare_parameter("command_id", 0x01)
        self.declare_parameter("feedback_id", 0x101)
        self.declare_parameter("pmax", 3.14)
        self.declare_parameter("vmax", 10.0)
        self.declare_parameter("tmax", 12.0)
        self.declare_parameter("kp", 10.0)
        self.declare_parameter("kd", 1.0)
        self.declare_parameter("joint_name", "gripper")
        self.declare_parameter("state_topic", "~/joint_states")
        self.declare_parameter("command_topic", "~/command")
        self.declare_parameter("enable_on_start", False)

        gp = self.get_parameter
        rx_topic = str(gp("rx_topic").value)
        tx_topic = str(gp("tx_topic").value)
        self._command_id = int(gp("command_id").value)
        self._feedback_id = int(gp("feedback_id").value)
        self._limits = Limits(pmax=float(gp("pmax").value),
                              vmax=float(gp("vmax").value),
                              tmax=float(gp("tmax").value))
        self._kp = float(gp("kp").value)
        self._kd = float(gp("kd").value)
        self._joint_name = str(gp("joint_name").value)
        state_topic = str(gp("state_topic").value)
        command_topic = str(gp("command_topic").value)
        enable_on_start = bool(gp("enable_on_start").value)
        self._got_feedback = False

        rx_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=100)
        tx_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                            history=HistoryPolicy.KEEP_LAST, depth=50)
        state_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST, depth=50)

        self._tx_pub = self.create_publisher(Frame, tx_topic, tx_qos)
        self._state_pub = self.create_publisher(JointState, state_topic, state_qos)
        self._rx_sub = self.create_subscription(Frame, rx_topic, self._on_frame, rx_qos)
        self._cmd_sub = self.create_subscription(
            Float64, command_topic, self._on_command, 10)
        self._srv_enable = self.create_service(Trigger, "~/enable", self._srv_enable_cb)
        self._srv_disable = self.create_service(Trigger, "~/disable", self._srv_disable_cb)
        self._srv_zero = self.create_service(Trigger, "~/set_zero", self._srv_zero_cb)

        self.get_logger().info(
            f"Gloria gripper: cmd_id=0x{self._command_id:X} fb_id=0x{self._feedback_id:X} "
            f"rx='{rx_topic}' tx='{tx_topic}' -> {state_topic} "
            f"(limits pmax={self._limits.pmax} vmax={self._limits.vmax} tmax={self._limits.tmax})")

        # 启动后延时使能（等 bridge 的 tx 订阅建立），一次性
        self._enable_timer = None
        if enable_on_start:
            self._enable_timer = self.create_timer(1.0, self._enable_once)

    def _enable_once(self) -> None:
        if self._enable_timer is not None:
            self._enable_timer.cancel()
        self._send_ctrl(_CTRL_ENABLE)
        self.get_logger().info("enable sent on start")

    # --- CAN 帧下发 -------------------------------------------------------
    def _send_frame(self, can_id: int, data: bytes) -> None:
        f = Frame()
        f.header.stamp = self.get_clock().now().to_msg()
        f.id = int(can_id)
        f.is_extended = False
        f.dlc = len(data)
        f.data = list(bytes(data).ljust(8, b"\x00"))
        self._tx_pub.publish(f)

    def _send_ctrl(self, cmd: int) -> None:
        self._send_frame(self._command_id, bytes([0xFF] * 7 + [cmd & 0xFF]))

    # --- 命令：目标位置 -> MIT 帧 ----------------------------------------
    def _on_command(self, msg: Float64) -> None:
        q = float(msg.data)
        data = pack_mit_command(kp=self._kp, kd=self._kd, q=q, dq=0.0, tau=0.0,
                                limits=self._limits)
        self._send_frame(self._command_id, data)

    def _srv_enable_cb(self, _req, resp: Trigger.Response):
        self._send_ctrl(_CTRL_ENABLE)
        resp.success = True
        resp.message = "enable sent"
        return resp

    def _srv_disable_cb(self, _req, resp: Trigger.Response):
        self._send_ctrl(_CTRL_DISABLE)
        resp.success = True
        resp.message = "disable sent"
        return resp

    def _srv_zero_cb(self, _req, resp: Trigger.Response):
        self._send_ctrl(_CTRL_SET_ZERO)
        resp.success = True
        resp.message = "set_zero sent (angle origin reset)"
        return resp

    # --- 反馈：MIT 帧 -> JointState --------------------------------------
    def _on_frame(self, frame: Frame) -> None:
        if int(frame.id) != self._feedback_id:
            return
        data = bytes(bytearray(frame.data))
        if len(data) < 8:
            return
        try:
            fb = unpack_mit_feedback(data, limits=self._limits)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"unpack_mit_feedback failed: {exc}")
            return
        if not self._got_feedback:
            self._got_feedback = True
            self.get_logger().info(
                f"first feedback: pos={fb.position:+.3f} vel={fb.velocity:+.3f} "
                f"tau={fb.torque:+.3f}")
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self._joint_name]
        js.position = [float(fb.position)]
        js.velocity = [float(fb.velocity)]
        js.effort = [float(fb.torque)]
        self._state_pub.publish(js)


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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
