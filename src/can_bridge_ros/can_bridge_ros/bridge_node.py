"""ROS 2 CAN Bus Bridge：python-can 总线 <-> can_msgs/Frame（支持多通道和 ID 路由）。

一个 bridge 进程独占一个物理 USB-CAN 设备，可同时桥接该设备的多条 CAN 通道。
接收帧默认发布到对应 ``/canX/rx``；通过启动参数配置 ``rx_routes`` 后，命中的帧改发
一个或多个设备专属话题，避免高频设备帧唤醒同总线上的所有 ROS 节点。
底层总线创建和适配器准备由无 ROS 的 ``can_sdk`` 统一提供；本模块只负责 ROS 参数、
消息转换、收发调度和话题分发。
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Dict, Optional, Tuple

import can
import rclpy
# ROS 2 标准 CAN 消息包 can_msgs 提供；Foxy: apt install ros-foxy-can-msgs
from can_msgs.msg import Frame
from can_sdk import open_bus
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from can_bridge_ros.handler_api import FrameDisposition, FrameHandlerContext
from can_bridge_ros.handler_runtime import FrameHandlerRuntime
from can_bridge_ros.routing import parse_rx_routes


class CanBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("can_bridge_ros")

        self.declare_parameter("interface", "canalystii")
        self.declare_parameter("channel", "0")
        self.declare_parameter("bitrate", 1_000_000)
        self.declare_parameter("channel_ids", [0])
        self.declare_parameter("bus_names", ["can0"])
        self.declare_parameter("rx_queue_depth", 128)
        self.declare_parameter("receive_own_messages", False)
        self.declare_parameter("rx_routes", [""])
        self.declare_parameter("frame_handler_specs", [""])

        get_parameter = self.get_parameter
        interface = str(get_parameter("interface").value)
        channel = str(get_parameter("channel").value)
        bitrate = int(get_parameter("bitrate").value)
        channel_ids = [int(value) for value in
                       (get_parameter("channel_ids").value or [0])]
        bus_names = [str(value) for value in
                     (get_parameter("bus_names").value or ["can0"])]
        rx_depth = int(get_parameter("rx_queue_depth").value)
        receive_own = bool(get_parameter("receive_own_messages").value)
        rx_route_specs = list(get_parameter("rx_routes").value or [])
        handler_specs = list(get_parameter("frame_handler_specs").value or [])
        if len(channel_ids) != len(bus_names):
            raise ValueError("channel_ids 与 bus_names 长度必须一致")
        rx_routes = parse_rx_routes(rx_route_specs, channel_ids)

        rx_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=rx_depth,
        )
        tx_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )

        self._rx_pub: Dict[int, object] = {}
        self._routed_rx_pub: Dict[Tuple[int, int], Tuple[object, ...]] = {}
        self._single = len(channel_ids) == 1
        self._only_cid = channel_ids[0] if self._single else None
        self._subs = []
        for channel_id, bus_name in zip(channel_ids, bus_names):
            self._rx_pub[channel_id] = self.create_publisher(
                Frame, f"/{bus_name}/rx", rx_qos)
            self._subs.append(self.create_subscription(
                Frame,
                f"/{bus_name}/tx",
                self._make_tx_cb(channel_id),
                tx_qos,
            ))
            self.get_logger().info(
                f"bridge channel {channel_id} <-> /{bus_name}/rx "
                "(BEST_EFFORT), /{bus_name}/tx (RELIABLE)")

        publishers_by_topic: Dict[str, object] = {}
        for route_key, route_topics in rx_routes.items():
            route_publishers = []
            for route_topic in route_topics:
                publisher = publishers_by_topic.get(route_topic)
                if publisher is None:
                    publisher = self.create_publisher(Frame, route_topic, rx_qos)
                    publishers_by_topic[route_topic] = publisher
                route_publishers.append(publisher)
                channel_id, can_id = route_key
                self.get_logger().info(
                    f"RX route channel {channel_id}, CAN ID 0x{can_id:X} -> "
                    f"{route_topic}")
            self._routed_rx_pub[route_key] = tuple(route_publishers)

        self._tx_queue: "queue.Queue[Tuple[int, Frame]]" = queue.Queue(maxsize=2000)
        self._tx_pending = threading.Event()
        self._tx_idle = threading.Event()
        self._tx_idle.set()
        self._tx_state_lock = threading.Lock()
        self._stop = threading.Event()
        handler_context = FrameHandlerContext(
            logger=self.get_logger(),
            send_frame=self._enqueue_data_frame,
            ros_context=self.context,
        )
        self._handler_runtime = FrameHandlerRuntime.from_specs(
            handler_specs, handler_context)
        for registration in self._handler_runtime.registrations:
            keys = ", ".join(
                f"ch {key.channel_id}/0x{key.can_id:X}"
                for key in registration.keys)
            self.get_logger().info(
                f"frame handler {registration.name!r}: {keys}")

        try:
            self._bus = open_bus(
                interface=interface,
                channel=channel,
                bitrate=bitrate,
                receive_own_messages=receive_own,
            )
        except Exception as exc:  # noqa: BLE001
            self._handler_runtime.destroy_auxiliary_nodes()
            self.get_logger().fatal(
                f"could not open CAN bus ({interface}:{channel}): {exc}")
            raise

        self._thread = threading.Thread(target=self._bus_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"CAN bridge on {interface}:{channel} @ {bitrate} bps, "
            f"channels {channel_ids} -> buses {bus_names}")

    def _make_tx_cb(self, channel_id: int):
        def _callback(frame: Frame) -> None:
            self._enqueue_frame(channel_id, frame)
        return _callback

    def _enqueue_frame(self, channel_id: int, frame: Frame) -> bool:
        with self._tx_state_lock:
            try:
                self._tx_queue.put_nowait((channel_id, frame))
                self._tx_idle.clear()
                self._tx_pending.set()
                return True
            except queue.Full:
                self.get_logger().warn(
                    f"tx queue full (ch {channel_id}), dropping frame")
                return False

    def _enqueue_data_frame(self, channel_id: int, can_id: int, data: bytes) -> bool:
        if channel_id not in self._rx_pub:
            raise ValueError(f"unknown CAN channel {channel_id}")
        if not 0 <= can_id <= 0x7FF:
            raise ValueError(f"CAN ID must be standard, got 0x{can_id:X}")
        payload = bytes(data)
        if len(payload) > 8:
            raise ValueError("classic CAN payload cannot exceed 8 bytes")
        frame = Frame()
        frame.header.stamp = self.get_clock().now().to_msg()
        frame.id = can_id
        frame.is_extended = False
        frame.dlc = len(payload)
        frame.data = list(payload.ljust(8, b"\x00"))
        return self._enqueue_frame(channel_id, frame)

    @property
    def auxiliary_nodes(self) -> Tuple[Node, ...]:
        return self._handler_runtime.auxiliary_nodes

    def start_frame_handlers(self) -> None:
        self._handler_runtime.start()

    def stop_frame_handlers(self) -> None:
        self._handler_runtime.stop()

    def _bus_loop(self) -> None:
        while not self._stop.is_set():
            if self._tx_pending.is_set():
                self._tx_pending.clear()
                while True:
                    try:
                        channel_id, frame = self._tx_queue.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        self._bus.send(can.Message(
                            arbitration_id=int(frame.id),
                            is_extended_id=bool(frame.is_extended),
                            is_remote_frame=bool(frame.is_rtr),
                            dlc=int(frame.dlc),
                            data=bytes(frame.data)[:int(frame.dlc)],
                            channel=channel_id,
                        ))
                    except Exception as exc:  # noqa: BLE001
                        self.get_logger().error(
                            f"CAN send failed (ch {channel_id}): {exc}")
                with self._tx_state_lock:
                    if self._tx_queue.empty():
                        self._tx_idle.set()

            try:
                message = self._bus.recv(timeout=0.005)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"CAN recv failed: {exc}")
                time.sleep(0.05)
                continue
            if message is None:
                continue
            if not rclpy.ok() or self._stop.is_set():
                break
            self._publish(message)

    def _publish(self, message) -> None:
        if self._single:
            channel_id = self._only_cid
        else:
            raw_channel = getattr(message, "channel", None)
            try:
                channel_id = int(raw_channel)
            except (TypeError, ValueError):
                return

        can_id = int(message.arbitration_id)
        is_data_frame = not (
            bool(message.is_extended_id)
            or bool(message.is_remote_frame)
            or bool(getattr(message, "is_error_frame", False))
        )
        if is_data_frame:
            disposition = self._handler_runtime.dispatch(channel_id, message)
            if disposition is FrameDisposition.CONSUME:
                return
        publishers = (self._routed_rx_pub.get((channel_id, can_id))
                      if is_data_frame else None)
        if publishers is None:
            default_publisher = self._rx_pub.get(channel_id)
            if default_publisher is None:
                return
            publishers = (default_publisher,)

        frame = Frame()
        frame.header.stamp = self.get_clock().now().to_msg()
        frame.id = can_id
        frame.is_extended = bool(message.is_extended_id)
        frame.is_rtr = bool(message.is_remote_frame)
        frame.is_error = bool(getattr(message, "is_error_frame", False))
        frame.dlc = int(message.dlc)
        frame.data = list(bytes(message.data)[:8].ljust(8, b"\x00"))
        for publisher in publishers:
            try:
                publisher.publish(frame)
            except Exception:  # noqa: BLE001 - ROS 正在关闭
                pass

    def destroy_node(self) -> bool:
        self.stop_frame_handlers()
        if not self._tx_idle.wait(timeout=0.5):
            self.get_logger().warn("timed out waiting for CAN TX queue to drain")
        self._stop.set()
        self._thread.join(timeout=1.0)
        try:
            self._bus.shutdown()
        except Exception:  # noqa: BLE001 - 关闭阶段尽量不抛异常
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node: Optional[CanBridgeNode] = None
    executor = SingleThreadedExecutor()
    try:
        node = CanBridgeNode()
    except Exception as exc:  # noqa: BLE001
        rclpy.logging.get_logger("can_bridge_ros").fatal(str(exc))
        if rclpy.ok():
            rclpy.shutdown()
        return
    executor.add_node(node)
    for auxiliary_node in node.auxiliary_nodes:
        executor.add_node(auxiliary_node)
    try:
        node.start_frame_handlers()
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_frame_handlers()
        for auxiliary_node in node.auxiliary_nodes:
            executor.remove_node(auxiliary_node)
            auxiliary_node.destroy_node()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
