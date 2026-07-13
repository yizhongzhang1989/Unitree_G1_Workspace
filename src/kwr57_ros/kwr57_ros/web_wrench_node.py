"""ROS2 web visualizer for the KWR57 wrench topic (SSH-friendly)

Same browser UI as ``examples/web_wrench.py`` (六轴条形图 + 力/力矩 XY 矢量投影),
but the data source is a ROS2 topic instead of the CAN bus: it subscribes to a
``geometry_msgs/WrenchStamped`` topic (BEST_EFFORT, matching the sensor node's
publisher) and feeds each sample into the exact same HTTP visualizer.

To avoid duplicating the ~250-line HTML/JS, this node reuses the handler and
shared-state helpers from ``examples/web_wrench.py`` by importing that file from
the (editable-installed) repository. No hardware/CAN access happens here.

Run:
    ros2 run kwr57_ft_sensor web_wrench
    ros2 run kwr57_ft_sensor web_wrench --ros-args -p port:=8080 -p topic:=/ft/wrench

Open in browser:  http://<ipv4>:8765
"""

from __future__ import annotations

import importlib.util
import sys
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path
from types import ModuleType
from typing import Optional

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

import kwr57_sensor
from kwr57_sensor import Wrench


def _load_web_wrench_module() -> ModuleType:
    """Import ``examples/web_wrench.py`` to reuse its HTML/handler/state helpers.

    Located relative to the installed ``kwr57_sensor`` package (editable install
    keeps it next to the repo's ``examples/``).
    """
    repo_root = Path(kwr57_sensor.__file__).resolve().parents[1]
    web_path = repo_root / "examples" / "web_wrench.py"
    if not web_path.exists():
        raise FileNotFoundError(
            f"could not find web viewer at {web_path}. Install the library "
            "editable: pip install -e <kwr57_can_sensor repo> (see ros2 README).")
    spec = importlib.util.spec_from_file_location("kwr57_web_wrench_ui", web_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    # Register before exec: Python 3.8 @dataclass resolves field types via
    # sys.modules[cls.__module__], which is None for an unregistered module.
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class WebWrenchNode(Node):
    def __init__(self) -> None:
        super().__init__("kwr57_web_wrench")

        self.declare_parameter("topic", "/kwr57_ft_sensor/wrench_raw")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8765)
        self.declare_parameter("force_scale", 10.0)
        self.declare_parameter("torque_scale", 0.25)

        topic = str(self.get_parameter("topic").value)
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        force_scale = float(self.get_parameter("force_scale").value)
        torque_scale = float(self.get_parameter("torque_scale").value)

        self._ui = _load_web_wrench_module()
        self._state = self._ui.SharedState(
            lock=threading.Lock(),
            latest=self._ui.initial_payload(
                force_scale=max(force_scale, 1e-9),
                torque_scale=max(torque_scale, 1e-9),
            ),
        )
        self._ui.set_status(self._state, f"waiting for {topic}")

        # BEST_EFFORT to match the sensor node's high-rate publisher.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sub = self.create_subscription(WrenchStamped, topic, self._on_wrench, qos)

        self._window_count = 0
        self._window_start = time.monotonic()
        self._hz = 0.0

        handler = self._ui.make_handler(self._state)
        self._httpd = ThreadingHTTPServer((host, port), handler)
        self._http_thread = threading.Thread(
            target=self._httpd.serve_forever, kwargs={"poll_interval": 0.2}, daemon=True)
        self._http_thread.start()
        self.get_logger().info(
            f"web visualizer on http://{host}:{port}  <-  topic {topic} "
            f"(BEST_EFFORT). SSH tunnel: ssh -L {port}:127.0.0.1:{port} user@server")

    def _on_wrench(self, msg: WrenchStamped) -> None:
        now = time.monotonic()
        self._window_count += 1
        elapsed = now - self._window_start
        if elapsed >= 1.0:
            self._hz = self._window_count / elapsed
            self._window_count = 0
            self._window_start = now
        f = msg.wrench.force
        t = msg.wrench.torque
        wrench = Wrench(f.x, f.y, f.z, t.x, t.y, t.z)
        self._ui.update_payload(
            self._state, wrench, self._hz, f"ros {self._hz:5.1f} Hz")

    def destroy_node(self) -> bool:
        try:
            self._httpd.shutdown()
            self._httpd.server_close()
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node: Optional[WebWrenchNode] = None
    try:
        node = WebWrenchNode()
    except Exception as exc:  # noqa: BLE001
        rclpy.logging.get_logger("kwr57_web_wrench").fatal(str(exc))
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
