"""ROS 2 KWR57 device node (bridge mode)

一设备一节点：本节点**不直接开总线**，而是通过 ``can_bridge_ros`` 共享总线：
    - 订阅 ``rx_topic``（``can_msgs/Frame``，BEST_EFFORT）。
        高频部署由 bridge 按 CAN ID 路由到设备专属话题；未配置路由时，本节点仍会自行过滤总线帧；
    - 用 ``kwr57_sensor`` 的协议层组包，发布 ``geometry_msgs/WrenchStamped``；
  - 下发命令（起流/停止/采样率）时，把 ``can_msgs/Frame`` 发布到 ``tx_topic``（RELIABLE）。

同一条总线上要挂多个 KWR57：先用 ``examples/set_id.py`` 给每个设备设不同 CAN ID，
再为每个设备起一个本节点实例（不同 cmd_id/data_base_id/topic/命名空间）。

依赖：先启动 ``can_bridge_ros`` 独占物理总线。

Topics / Services
-----------------
subscribes: <rx_topic>  can_msgs/Frame        (来自 bridge 的所有总线帧)
publishes:  <tx_topic>  can_msgs/Frame        (下发给 bridge 的命令帧)
            <topic>     geometry_msgs/WrenchStamped
            ~/command   std_msgs/String        start | stop | tare | zero | reset_tare
services:   ~/start ~/stop ~/tare ~/reset_tare  std_srvs/Trigger

Parameters
----------
  rx_topic        string  default "/can0/rx"     bridge RX；bringup 会改为专属话题
  tx_topic        string  default "/can0/tx"     bridge 订阅的命令帧话题
  cmd_id          int     default 0x10           本设备命令(接收)CAN ID
  data_base_id    int     default 0x15           本设备数据起始 CAN ID (帧 base/+1/+2)
  topic           string  default "~/wrench_raw" 输出 wrench 话题
  frame_id        string  default "kwr57_ft_sensor_link"
  period_ms       int     default 1              上传周期 (1 -> ~1000 Hz)
  sample_rate_hz  int     default 1000           内部采样率
  publish_rate    double  default 0.0            0 = 每帧都发
  use_si          bool    default false          false=原始值(与库一致); true=换算 N/N*m
  autostart       bool    default true
  tare_on_start   bool    default false
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, List, Optional, Union

import rclpy
# ROS 2 包 can_msgs 提供的标准 CAN 帧消息；Foxy: apt install ros-foxy-can-msgs
# 它与负责物理总线 I/O 的 python-can/can_sdk 是不同层次的依赖。
from can_msgs.msg import Frame
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

# 见 KWR57-SDK；工作区由 scripts/env.sh 暴露源码，无需安装本地 SDK
from kwr57_sensor import Wrench, WrenchAssembler
from kwr57_sensor import protocol

_CMD_SETTLE_S = 0.1


class KWR57DeviceNode(Node):
    def __init__(
            self,
            *,
            node_name: str = "kwr57_ft_sensor",
            context: Any = None,
            parameter_overrides: Optional[List[Parameter]] = None,
            use_global_arguments: bool = True,
            direct_rx: bool = False,
            direct_tx: Optional[Callable[[int, bytes], bool]] = None,
            defer_autostart: bool = False) -> None:
        super().__init__(
            node_name,
            context=context,
            parameter_overrides=parameter_overrides or [],
            use_global_arguments=use_global_arguments,
        )

        self.declare_parameter("rx_topic", "/can0/rx")
        self.declare_parameter("tx_topic", "/can0/tx")
        self.declare_parameter("cmd_id", 0x10)
        self.declare_parameter("data_base_id", 0x15)
        self.declare_parameter("topic", "~/wrench_raw")
        self.declare_parameter("frame_id", "kwr57_ft_sensor_link")
        self.declare_parameter("period_ms", 1)
        self.declare_parameter("sample_rate_hz", 1000)
        self.declare_parameter("publish_rate", 0.0)
        self.declare_parameter("use_si", False)
        self.declare_parameter("autostart", True)
        self.declare_parameter("tare_on_start", False)

        gp = self.get_parameter
        rx_topic = gp("rx_topic").get_parameter_value().string_value
        tx_topic = gp("tx_topic").get_parameter_value().string_value
        self._cmd_id = gp("cmd_id").get_parameter_value().integer_value
        data_base_id = gp("data_base_id").get_parameter_value().integer_value
        topic = gp("topic").get_parameter_value().string_value
        frame_id = gp("frame_id").get_parameter_value().string_value
        self._period_ms = gp("period_ms").get_parameter_value().integer_value
        self._sample_rate_hz = gp("sample_rate_hz").get_parameter_value().integer_value
        self._publish_rate = gp("publish_rate").get_parameter_value().double_value
        self._use_si = gp("use_si").get_parameter_value().bool_value
        self._autostart = gp("autostart").get_parameter_value().bool_value
        self._tare_on_start = gp("tare_on_start").get_parameter_value().bool_value
        self._direct_tx = direct_tx

        self._data_ids = protocol.data_ids_from_base(data_base_id)
        self._assembler = WrenchAssembler(self._data_ids)

        # --- ROS interface ------------------------------------------------
        wrench_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=32)

        self._pub = self.create_publisher(WrenchStamped, topic, wrench_qos)
        self._tx_pub = None
        if direct_tx is None:
            tx_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST, depth=100)
            self._tx_pub = self.create_publisher(Frame, tx_topic, tx_qos)
        self._rx_sub = None
        if not direct_rx:
            rx_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=128)
            self._rx_sub = self.create_subscription(
                Frame, rx_topic, self._on_frame, rx_qos)
        self._cmd_sub = self.create_subscription(
            String, "~/command", self._on_command, 10)
        self._srv_start = self.create_service(Trigger, "~/start", self._srv_start_cb)
        self._srv_stop = self.create_service(Trigger, "~/stop", self._srv_stop_cb)
        self._srv_tare = self.create_service(Trigger, "~/tare", self._srv_tare_cb)
        self._srv_reset_tare = self.create_service(
            Trigger, "~/reset_tare", self._srv_reset_tare_cb)

        self._offsets: List[float] = [0.0] * 6
        self._tare_pending = False
        self._frames_seen = 0
        self._streaming = False
        self._last_pub = 0.0
        self._min_period = 1.0 / self._publish_rate if self._publish_rate > 0.0 else 0.0
        self._start_lock = threading.Lock()
        self._start_cancel: Optional[threading.Event] = None
        self._start_thread: Optional[threading.Thread] = None

        self._msg = WrenchStamped()
        self._msg.header.frame_id = frame_id

        self.get_logger().info(
            f"KWR57 device [bridge]: cmd_id=0x{self._cmd_id:X} "
            f"data_ids={'/'.join(f'0x{c:X}' for c in self._data_ids)}  "
            f"rx={'in-process' if direct_rx else repr(rx_topic)} "
            f"tx={'in-process' if direct_tx is not None else repr(tx_topic)} "
            f"-> {topic} (si={self._use_si})")

        if not defer_autostart:
            self.activate()

    def activate(self) -> None:
        if self._autostart:
            self._start_async(tare=self._tare_on_start)
        else:
            self.get_logger().info("waiting for start command")

    def stop_device(self) -> None:
        self._do_stop()

    # --- command frame helpers -------------------------------------------
    def _send_cmd(self, cmd_id: int, data: bytes) -> None:
        if self._direct_tx is not None:
            if not self._direct_tx(cmd_id, data):
                self.get_logger().error(
                    f"failed to enqueue CAN command for ID 0x{cmd_id:X}")
            return
        f = Frame()
        f.header.stamp = self.get_clock().now().to_msg()
        f.id = int(cmd_id)
        f.is_extended = False
        f.dlc = len(data)
        f.data = list(bytes(data).ljust(8, b"\x00"))
        if self._tx_pub is None:
            raise RuntimeError("KWR57 TX publisher is not available")
        self._tx_pub.publish(f)

    def _start_async(self, tare: bool) -> None:
        with self._start_lock:
            if self._start_thread is not None and self._start_thread.is_alive():
                self.get_logger().info("stream start already in progress")
                return
            cancel = threading.Event()
            thread = threading.Thread(
                target=self._start_sequence, args=(tare, cancel), daemon=True)
            self._start_cancel = cancel
            self._start_thread = thread
            thread.start()

    def _start_sequence(self, tare: bool, cancel: threading.Event) -> None:
        """按驱动时序起流：停止 -> 设采样率 -> 实时命令（未起流则重发）。"""
        self._streaming = False
        if cancel.is_set():
            return
        self._send_cmd(self._cmd_id, protocol.build_stop_command())
        if cancel.wait(_CMD_SETTLE_S):
            return
        self._assembler.reset()
        self._send_cmd(self._cmd_id, protocol.build_sample_rate_command(self._sample_rate_hz))
        if cancel.wait(_CMD_SETTLE_S):
            return
        for _ in range(3):
            if cancel.is_set():
                self._streaming = False
                return
            self._frames_seen = 0
            self._streaming = True   # 允许 rx 回调计数/发布
            self._send_cmd(self._cmd_id, protocol.build_realtime_command(self._period_ms))
            deadline = time.monotonic() + 0.3
            while time.monotonic() < deadline:
                if cancel.is_set():
                    self._streaming = False
                    return
                if self._frames_seen >= 3:
                    if tare:
                        self._tare_pending = True
                    self.get_logger().info("stream started")
                    return
                if cancel.wait(0.02):
                    self._streaming = False
                    return
        if tare:
            self._tare_pending = True
        self.get_logger().warn("stream start not confirmed (no frames); is the bridge up?")

    def _cancel_start_sequence(self) -> None:
        with self._start_lock:
            cancel = self._start_cancel
            thread = self._start_thread
            if cancel is not None:
                cancel.set()
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)
            if thread.is_alive():
                self.get_logger().warn("stream start thread did not stop within 1 second")

    def _do_stop(self) -> None:
        self._streaming = False
        self._cancel_start_sequence()
        self._send_cmd(self._cmd_id, protocol.build_stop_command())
        self.get_logger().info("stream stopped")

    # --- ROS callbacks ----------------------------------------------------
    def _on_command(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        if cmd == "start":
            self._start_async(tare=False)
        elif cmd == "stop":
            self._do_stop()
        elif cmd in ("tare", "zero"):
            self._tare_pending = True
            self.get_logger().info("tare requested (offset = next sample)")
        elif cmd in ("reset_tare", "untare", "clear_tare"):
            self._offsets = [0.0] * 6
            self._tare_pending = False
            self.get_logger().info("tare cleared")
        else:
            self.get_logger().warn(f"ignoring unknown command '{msg.data}'")

    def _srv_start_cb(self, _req, resp: Trigger.Response):
        self._start_async(tare=False)
        resp.success = True
        resp.message = "streaming"
        return resp

    def _srv_stop_cb(self, _req, resp: Trigger.Response):
        self._do_stop()
        resp.success = True
        resp.message = "stopped"
        return resp

    def _srv_tare_cb(self, _req, resp: Trigger.Response):
        self._tare_pending = True
        resp.success = True
        resp.message = "tare requested (offset captured from next sample)"
        return resp

    def _srv_reset_tare_cb(self, _req, resp: Trigger.Response):
        self._offsets = [0.0] * 6
        self._tare_pending = False
        resp.success = True
        resp.message = "tare cleared"
        return resp

    # --- data path (rx callback) -----------------------------------------
    def _on_frame(self, frame: Frame) -> None:
        self.handle_can_frame(
            int(frame.id),
            bytes(frame.data),
            is_extended=bool(frame.is_extended),
            is_rtr=bool(frame.is_rtr),
            is_error=bool(frame.is_error),
            dlc=int(frame.dlc),
        )

    def handle_can_frame(
            self,
            can_id: int,
            data: Union[bytes, bytearray],
            *,
            is_extended: bool = False,
            is_rtr: bool = False,
            is_error: bool = False,
            dlc: Optional[int] = None) -> bool:
        """Process one CAN frame; return whether it belongs to this device."""
        if can_id not in self._data_ids:
            return False
        frame_dlc = len(data) if dlc is None else dlc
        if is_extended or is_rtr or is_error or frame_dlc != 8 or len(data) < 8:
            return False
        self._frames_seen += 1
        wrench = self._assembler.push(can_id, data)
        if wrench is None:
            return True
        self._handle_wrench(wrench)
        return True

    def _handle_wrench(self, wrench: Wrench) -> None:
        if self._use_si:
            wrench = wrench.to_si()
        if self._tare_pending:
            self._offsets = [wrench.fx, wrench.fy, wrench.fz,
                             wrench.mx, wrench.my, wrench.mz]
            self._tare_pending = False
            self.get_logger().info(
                f"tare baseline set: Fx={wrench.fx:+.3f} Fy={wrench.fy:+.3f} "
                f"Fz={wrench.fz:+.3f}")
        if self._min_period > 0.0:
            now = time.monotonic()
            if (now - self._last_pub) < self._min_period:
                return
            self._last_pub = now
        self._publish(wrench)

    def _publish(self, w: Wrench) -> None:
        msg = self._msg
        msg.header.stamp = self.get_clock().now().to_msg()
        ox, oy, oz, omx, omy, omz = self._offsets
        msg.wrench.force.x = w.fx - ox
        msg.wrench.force.y = w.fy - oy
        msg.wrench.force.z = w.fz - oz
        msg.wrench.torque.x = w.mx - omx
        msg.wrench.torque.y = w.my - omy
        msg.wrench.torque.z = w.mz - omz
        self._pub.publish(msg)

    def destroy_node(self) -> bool:
        self._streaming = False
        self._cancel_start_sequence()
        try:
            self._send_cmd(self._cmd_id, protocol.build_stop_command())
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node: Optional[KWR57DeviceNode] = None
    try:
        node = KWR57DeviceNode()
    except Exception as exc:  # noqa: BLE001
        get_logger("kwr57_ft_sensor").fatal(str(exc))
        if rclpy.ok():
            rclpy.shutdown()
        return

    # 单个高频订阅使用线程池只会增加 CPython 的 GIL 竞争与任务调度开销。
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
