"""ROS 2 KWR57 device node (bridge mode).

一设备一节点：本节点**不直接开总线**，而是通过通用 ``can_bridge`` 共享总线：
  - 订阅 ``rx_topic``（``can_msgs/Frame``，BEST_EFFORT），按本设备的 CAN ID 过滤、
    用 ``kwr57_sensor`` 的协议层组包，发布 ``geometry_msgs/WrenchStamped``；
  - 下发命令（起流/停止/采样率）时，把 ``can_msgs/Frame`` 发布到 ``tx_topic``（RELIABLE）。

同一条总线上要挂多个 KWR57：先用 ``examples/set_id.py`` 给每个设备设不同 CAN ID，
再为每个设备起一个本节点实例（不同 cmd_id/data_base_id/topic/命名空间）。

依赖：先启动 ``can_bridge``（见 ~/can_bridge）独占物理总线。

Topics / Services
-----------------
subscribes: <rx_topic>  can_msgs/Frame        (来自 bridge 的所有总线帧)
publishes:  <tx_topic>  can_msgs/Frame        (下发给 bridge 的命令帧)
            <topic>     geometry_msgs/WrenchStamped
            ~/command   std_msgs/String        start | stop | tare | zero | reset_tare
services:   ~/start ~/stop ~/tare ~/reset_tare  std_srvs/Trigger

Parameters
----------
  rx_topic        string  default "/can0/rx"     bridge 发布的总线帧话题
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
from typing import List, Optional

import rclpy
from can_msgs.msg import Frame
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from kwr57_sensor import Wrench, WrenchAssembler
from kwr57_sensor import protocol
from kwr57_sensor import KWR57Sensor

_CMD_SETTLE_S = 0.1


class KWR57DeviceNode(Node):
    def __init__(self) -> None:
        super().__init__("kwr57_ft_sensor")

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
        # --- direct-bus (high-rate) mode ---------------------------------
        # bridge 模式逐帧走 DDS，在本机 1000Hz(3000 帧/s) 下会因 GIL 争用丢帧、
        # wrench 掉到 ~130Hz。direct 模式让本节点用 SDK 紧循环直接读总线+组包，
        # 只发布 ~1000Hz 的 WrenchStamped（已实测可满速）。注意：direct 模式
        # 独占物理 CAN 设备，同一设备上不能再跑 bridge 或其它节点。
        self.declare_parameter("direct_bus", False)
        self.declare_parameter("interface", "canalystii")
        self.declare_parameter("channel", "0")
        self.declare_parameter("bitrate", 1_000_000)

        gp = self.get_parameter
        rx_topic = str(gp("rx_topic").value)
        tx_topic = str(gp("tx_topic").value)
        self._cmd_id = int(gp("cmd_id").value)
        data_base_id = int(gp("data_base_id").value)
        self._data_base_id = data_base_id
        topic = str(gp("topic").value)
        frame_id = str(gp("frame_id").value)
        self._period_ms = int(gp("period_ms").value)
        self._sample_rate_hz = int(gp("sample_rate_hz").value)
        self._publish_rate = float(gp("publish_rate").value)
        self._use_si = bool(gp("use_si").value)
        autostart = bool(gp("autostart").value)
        tare_on_start = bool(gp("tare_on_start").value)
        self._direct = bool(gp("direct_bus").value)
        self._interface = str(gp("interface").value)
        self._channel = str(gp("channel").value)
        self._bitrate = int(gp("bitrate").value)

        self._data_ids = protocol.data_ids_from_base(data_base_id)
        self._assembler = WrenchAssembler(self._data_ids)

        # --- ROS interface ------------------------------------------------
        rx_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=200)
        tx_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=100)
        wrench_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=200)

        self._pub = self.create_publisher(WrenchStamped, topic, wrench_qos)
        # bridge 模式才需要 rx 订阅 / tx 发布；direct 模式直接开总线
        self._tx_pub = None
        self._rx_sub = None
        if not self._direct:
            self._tx_pub = self.create_publisher(Frame, tx_topic, tx_qos)
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

        # direct 模式的总线/线程状态
        self._sensor = None
        self._direct_thread: Optional[threading.Thread] = None

        self._msg = WrenchStamped()
        self._msg.header.frame_id = frame_id

        mode = "DIRECT" if self._direct else "bridge"
        src = (f"{self._interface}:{self._channel}@{self._bitrate}"
               if self._direct else f"rx='{rx_topic}' tx='{tx_topic}'")
        self.get_logger().info(
            f"KWR57 device [{mode}]: cmd_id=0x{self._cmd_id:X} "
            f"data_ids={'/'.join(f'0x{c:X}' for c in self._data_ids)}  "
            f"{src} -> {topic} (si={self._use_si})")

        if self._direct:
            self._tare_on_start = tare_on_start
            if autostart:
                self._start_direct()
            else:
                self.get_logger().info("waiting for start command (direct)")
            return

        if autostart:
            self._start_async(tare=tare_on_start)
        else:
            self.get_logger().info("waiting for start command")

    # --- command frame helpers -------------------------------------------
    def _send_cmd(self, cmd_id: int, data: bytes) -> None:
        f = Frame()
        f.header.stamp = self.get_clock().now().to_msg()
        f.id = int(cmd_id)
        f.is_extended = False
        f.dlc = len(data)
        f.data = list(bytes(data).ljust(8, b"\x00"))
        self._tx_pub.publish(f)

    def _start_async(self, tare: bool) -> None:
        if self._direct:
            self._start_direct(tare=tare)
            return
        threading.Thread(target=self._start_sequence, args=(tare,), daemon=True).start()

    # --- direct-bus mode (SDK tight loop, no per-frame DDS) ---------------
    def _start_direct(self, tare: bool = False) -> None:
        if self._direct_thread is not None and self._direct_thread.is_alive():
            self.get_logger().info("direct stream already running")
            return
        if tare:
            self._tare_pending = True
        self._streaming = True
        self._direct_thread = threading.Thread(target=self._direct_loop, daemon=True)
        self._direct_thread.start()

    def _direct_loop(self) -> None:
        """用 SDK 紧循环直接读总线+组包，只发布 WrenchStamped（可跑满 ~1000Hz）。"""
        try:
            if self._sensor is None:
                self._sensor = KWR57Sensor.open(
                    interface=self._interface, channel=self._channel,
                    bitrate=self._bitrate, cmd_id=self._cmd_id,
                    data_base_id=self._data_base_id)
            self._sensor.start_stream(period_ms=self._period_ms,
                                      rate_hz=self._sample_rate_hz)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"direct open/start failed: {exc}")
            self._streaming = False
            return
        self.get_logger().info("stream started (direct)")
        while self._streaming and rclpy.ok():
            try:
                wrench = self._sensor.read_wrench(timeout=0.05)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"direct read failed: {exc}")
                break
            if wrench is None:
                continue
            self._frames_seen += 1
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
                    continue
                self._last_pub = now
            self._publish(wrench)

    def _start_sequence(self, tare: bool) -> None:
        """按驱动时序起流：停止 -> 设采样率 -> 实时命令（未起流则重发）。"""
        self._streaming = False
        self._send_cmd(self._cmd_id, protocol.build_stop_command())
        time.sleep(_CMD_SETTLE_S)
        self._assembler.reset()
        self._send_cmd(self._cmd_id, protocol.build_sample_rate_command(self._sample_rate_hz))
        time.sleep(_CMD_SETTLE_S)
        for _ in range(3):
            self._frames_seen = 0
            self._streaming = True   # 允许 rx 回调计数/发布
            self._send_cmd(self._cmd_id, protocol.build_realtime_command(self._period_ms))
            deadline = time.monotonic() + 0.3
            while time.monotonic() < deadline:
                if self._frames_seen >= 3:
                    if tare:
                        self._tare_pending = True
                    self.get_logger().info("stream started")
                    return
                time.sleep(0.02)
        if tare:
            self._tare_pending = True
        self.get_logger().warn("stream start not confirmed (no frames); is the bridge up?")

    def _do_stop(self) -> None:
        self._streaming = False
        if self._direct:
            th = self._direct_thread
            if th is not None and th.is_alive() and th is not threading.current_thread():
                th.join(timeout=1.0)
            try:
                if self._sensor is not None:
                    self._sensor.stop_stream()
            except Exception:  # noqa: BLE001
                pass
            self.get_logger().info("stream stopped (direct)")
            return
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
        if frame.id not in self._data_ids:
            return
        self._frames_seen += 1
        wrench = self._assembler.push(int(frame.id), bytes(bytearray(frame.data)))
        if wrench is None:
            return
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
        if self._direct:
            th = self._direct_thread
            if th is not None and th.is_alive() and th is not threading.current_thread():
                th.join(timeout=1.0)
            try:
                if self._sensor is not None:
                    self._sensor.stop_stream()
                    self._sensor.close()
            except Exception:  # noqa: BLE001
                pass
            return super().destroy_node()
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
        rclpy.logging.get_logger("kwr57_ft_sensor").fatal(str(exc))
        if rclpy.ok():
            rclpy.shutdown()
        return

    executor = MultiThreadedExecutor()
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
