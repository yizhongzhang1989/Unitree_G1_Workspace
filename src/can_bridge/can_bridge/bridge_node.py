"""第2层 CAN Bus Bridge：python-can 总线 <-> can_msgs/Frame（支持多通道）。

一个 bridge 进程独占**一个物理 USB-CAN 设备**，可同时桥接该设备的多条 CAN 通道
（如 CANalyst-II 的 CAN1/CAN2 = 通道 0/1）：
  - 一个专用总线线程持续 recv()，按 ``msg.channel`` 把帧发布为 can_msgs/Frame 到
    对应通道的 ``/<bus_name>/rx``；
  - 订阅各通道的 ``/<bus_name>/tx``，把命令帧标记通道后经同一线程 send() 出去
    （单线程收发，避免个别 USB-CAN 后端并发 send/recv 出错）。

为什么必须一个进程开多通道：CANalyst-II 是**一个 USB 设备**，每个 python-can Bus
会各建一个 CanalystDevice 独占该 USB 设备，两个 Bus 会 `Resource busy`；而一个 Bus
用 ``channel="0,1"`` 只建一个 device、初始化两个通道，收发用 ``Message.channel`` 区分。

设备节点只需订阅对应总线的 ``rx``、按 CAN ID 过滤，发命令到该总线的 ``tx``。
消息用标准 can_msgs/Frame，与 ros2_socketcan 一致。
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Dict, Optional, Tuple

import rclpy
from can_msgs.msg import Frame
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from .canbus_backend import open_bus


class CanBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("can_bridge")

        self.declare_parameter("interface", "canalystii")
        self.declare_parameter("channel", "0")            # python-can channel(s), 如 "0" 或 "0,1"
        self.declare_parameter("bitrate", 1_000_000)
        self.declare_parameter("channel_ids", [0])        # msg.channel 值（与 bus_names 平行）
        self.declare_parameter("bus_names", ["can0"])     # 每个通道对应的 ROS 总线名
        self.declare_parameter("rx_queue_depth", 1000)
        self.declare_parameter("receive_own_messages", False)

        gp = self.get_parameter
        interface = str(gp("interface").value)
        channel = str(gp("channel").value)
        bitrate = int(gp("bitrate").value)
        channel_ids = [int(c) for c in (gp("channel_ids").value or [0])]
        bus_names = [str(b) for b in (gp("bus_names").value or ["can0"])]
        rx_depth = int(gp("rx_queue_depth").value)
        receive_own = bool(gp("receive_own_messages").value)
        if len(channel_ids) != len(bus_names):
            raise ValueError("channel_ids 与 bus_names 长度必须一致")

        # --- open the bus (layer 1) --------------------------------------
        try:
            self._bus = open_bus(interface, channel, bitrate,
                                 receive_own_messages=receive_own)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().fatal(
                f"could not open CAN bus ({interface}:{channel}): {exc}")
            raise

        rx_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=rx_depth)
        tx_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=100)

        # 每个通道一套 rx 发布 / tx 订阅
        self._rx_pub: Dict[int, object] = {}
        self._single = len(channel_ids) == 1
        self._only_cid = channel_ids[0] if self._single else None
        self._subs = []
        for cid, name in zip(channel_ids, bus_names):
            self._rx_pub[cid] = self.create_publisher(Frame, f"/{name}/rx", rx_qos)
            self._subs.append(self.create_subscription(
                Frame, f"/{name}/tx", self._make_tx_cb(cid), tx_qos))
            self.get_logger().info(
                f"bridge channel {cid} <-> /{name}/rx (BEST_EFFORT), /{name}/tx (RELIABLE)")

        self._tx_queue: "queue.Queue[Tuple[int, Frame]]" = queue.Queue(maxsize=2000)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._bus_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"CAN bridge on {interface}:{channel} @ {bitrate} bps, "
            f"channels {channel_ids} -> buses {bus_names}")

    def _make_tx_cb(self, cid: int):
        def _cb(frame: Frame) -> None:
            try:
                self._tx_queue.put_nowait((cid, frame))
            except queue.Full:
                self.get_logger().warn(f"tx queue full (ch {cid}), dropping frame")
        return _cb

    def _bus_loop(self) -> None:
        import can
        while not self._stop.is_set():
            # 1) flush pending outgoing frames
            while True:
                try:
                    cid, f = self._tx_queue.get_nowait()
                except queue.Empty:
                    break
                try:
                    self._bus.send(can.Message(
                        arbitration_id=int(f.id),
                        is_extended_id=bool(f.is_extended),
                        is_remote_frame=bool(f.is_rtr),
                        dlc=int(f.dlc),
                        data=bytes(bytearray(f.data))[:int(f.dlc)],
                        channel=cid,
                    ))
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(f"CAN send failed (ch {cid}): {exc}")

            # 2) receive one frame (short timeout keeps tx latency low)
            try:
                msg = self._bus.recv(timeout=0.005)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"CAN recv failed: {exc}")
                time.sleep(0.05)
                continue
            if msg is None:
                continue
            if not rclpy.ok() or self._stop.is_set():
                break
            self._publish(msg)

    def _publish(self, msg) -> None:
        # 路由：单总线时忽略 channel 全归它；多总线按 msg.channel 分发
        if self._single:
            pub = self._rx_pub[self._only_cid]
        else:
            ch = getattr(msg, "channel", None)
            pub = self._rx_pub.get(ch)
            if pub is None:
                return  # 帧来自未配置的通道，忽略
        frame = Frame()
        frame.header.stamp = self.get_clock().now().to_msg()
        frame.id = int(msg.arbitration_id)
        frame.is_extended = bool(msg.is_extended_id)
        frame.is_rtr = bool(msg.is_remote_frame)
        frame.is_error = bool(getattr(msg, "is_error_frame", False))
        dlc = int(msg.dlc)
        frame.dlc = dlc
        # can_msgs/Frame.data 是定长 uint8[8]，setter 走 numpy，需要 list of ints（不是 bytes）
        frame.data = list(bytes(bytearray(msg.data)[:8]).ljust(8, b"\x00"))
        try:
            pub.publish(frame)
        except Exception:  # noqa: BLE001 - shutting down
            pass

    def destroy_node(self) -> bool:
        self._stop.set()
        self._thread.join(timeout=1.0)
        try:
            self._bus.shutdown()
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node: Optional[CanBridgeNode] = None
    try:
        node = CanBridgeNode()
    except Exception as exc:  # noqa: BLE001
        rclpy.logging.get_logger("can_bridge").fatal(str(exc))
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
