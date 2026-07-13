"""Minimal ROS 2 demo subscriber for the KWR57 wrench topic

Subscribes with BEST_EFFORT QoS (matching the sensor node's publisher) and
prints the latest 6-axis wrench plus the measured message rate. Useful as a
copy-paste starting point and to verify the publisher from the CLI without
having to remember the QoS override for ``ros2 topic echo``.

Run:
    ros2 run kwr57_ros wrench_echo
    ros2 run kwr57_ros wrench_echo --ros-args -p topic:=/ft_left/wrench_raw
"""

from __future__ import annotations

import sys
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class WrenchEcho(Node):
    def __init__(self) -> None:
        super().__init__("kwr57_wrench_echo")
        self.declare_parameter("topic", "/kwr57_ft_sensor/wrench_raw")
        topic = self.get_parameter("topic").get_parameter_value().string_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sub = self.create_subscription(WrenchStamped, topic, self._cb, qos)
        self._latest: Optional[WrenchStamped] = None
        self._window_count = 0
        self._last_report = time.monotonic()
        self._report_timer = self.create_timer(1.0, self._report)
        self.get_logger().info(f"subscribing (BEST_EFFORT) to {topic}")

    def _cb(self, msg: WrenchStamped) -> None:
        self._window_count += 1
        self._latest = msg

    def _report(self) -> None:
        now = time.monotonic()
        elapsed = now - self._last_report
        hz = self._window_count / elapsed if elapsed > 0.0 else 0.0
        self._window_count = 0
        self._last_report = now
        msg = self._latest
        if msg is None:
            self.get_logger().warn("waiting for wrench messages")
            return
        f = msg.wrench.force
        t = msg.wrench.torque
        print(
            f"Fx={f.x:+9.3f} Fy={f.y:+9.3f} Fz={f.z:+9.3f}  |  "
            f"Mx={t.x:+8.4f} My={t.y:+8.4f} Mz={t.z:+8.4f}  [{hz:6.1f} Hz]",
            flush=True,
        )


def main() -> None:
    rclpy.init()
    node = WrenchEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write("\n")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
