"""Unified HTTP API for the dual-hand end-effector dashboard."""

from __future__ import annotations

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Tuple
from urllib.parse import urlsplit
from urllib.request import ProxyHandler, build_opener

import rclpy
from geometry_msgs.msg import WrenchStamped
from gloria_ros.msg import MitCommand
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from robot_bringup.end_effectors.topology import dashboard_topology_parameters


_MAX_REQUEST_BYTES = 64 * 1024
_HANDS = ("left", "right")
_SENSOR_ACTIONS = ("start", "stop", "tare", "reset_tare")
_GRIPPER_SERVICES = ("enable", "disable", "refresh")
_CAMERA_PATH_PREFIX = "/api/cameras/"
_STREAM_RATE_WINDOW_S = 3.0
_ROUNDTRIP_TARGET_TOLERANCE_RAD = 0.10
_ROUNDTRIP_MAX_SWITCH_PERIOD_S = 3.0


def _target_name(value: str) -> str:
    normalized = "/" + value.strip().strip("/")
    if normalized == "/":
        raise ValueError("target node must not be empty")
    return normalized


def _finite_fields(payload: object, names: Iterable[str]) -> Dict[str, float]:
    if not isinstance(payload, dict):
        raise ValueError("request body must be a JSON object")
    result: Dict[str, float] = {}
    for name in names:
        if name not in payload:
            raise ValueError(f"missing field: {name}")
        value = payload[name]
        if isinstance(value, bool):
            raise ValueError(f"{name} must be a finite number")
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be a finite number") from exc
        if not math.isfinite(number):
            raise ValueError(f"{name} must be a finite number")
        result[name] = number
    return result


def _roundtrip_should_switch(
        position: Optional[float], target: float,
        elapsed: float, timeout: float) -> bool:
    return (
        elapsed >= timeout
        or position is not None
        and abs(position - target) <= _ROUNDTRIP_TARGET_TOLERANCE_RAD
    )


def _child_name(target: str, child: str) -> str:
    return f"{target}/{child.strip('/')}"


def _request_path(raw_path: object) -> str:
    if isinstance(raw_path, bytes):
        return raw_path.decode("utf-8", errors="surrogatepass")
    return str(raw_path)


def _json_number(value: float) -> Optional[float]:
    number = float(value)
    return number if math.isfinite(number) else None


def _wrench_payload(
        message: Optional[WrenchStamped], received_at: Optional[float]) -> Optional[dict]:
    if message is None or received_at is None:
        return None
    force = message.wrench.force
    torque = message.wrench.torque
    return {
        "stamp": {
            "sec": int(message.header.stamp.sec),
            "nanosec": int(message.header.stamp.nanosec),
        },
        "frame_id": message.header.frame_id,
        "force": {
            "x": _json_number(force.x),
            "y": _json_number(force.y),
            "z": _json_number(force.z),
        },
        "torque": {
            "x": _json_number(torque.x),
            "y": _json_number(torque.y),
            "z": _json_number(torque.z),
        },
        "received_at": received_at,
    }


def _serialized_wrench_payload(
        serialized_message: Optional[bytes],
        received_at: Optional[float]) -> Optional[dict]:
    if serialized_message is None or received_at is None:
        return None
    message = deserialize_message(serialized_message, WrenchStamped)
    return _wrench_payload(message, received_at)


def _joint_state_payload(
        message: Optional[JointState], received_at: Optional[float]) -> Optional[dict]:
    if message is None or received_at is None:
        return None
    return {
        "stamp": {
            "sec": int(message.header.stamp.sec),
            "nanosec": int(message.header.stamp.nanosec),
        },
        "name": list(message.name),
        "position": [_json_number(value) for value in message.position],
        "velocity": [_json_number(value) for value in message.velocity],
        "effort": [_json_number(value) for value in message.effort],
        "received_at": received_at,
    }


def _control_route(path: str) -> Tuple[str, str, str]:
    parts = tuple(part for part in path.split("/") if part)
    if len(parts) < 5 or parts[:2] != ("api", "hands"):
        raise KeyError("not found")
    hand = parts[2]
    if hand not in _HANDS:
        raise ValueError("hand must be 'left' or 'right'")
    if (len(parts) == 5
            and parts[3] == "sensor"
            and parts[4] in _SENSOR_ACTIONS):
        return hand, "sensor_service", parts[4]
    if len(parts) == 6 and parts[3:5] == ("gripper", "service"):
        if parts[5] in _GRIPPER_SERVICES:
            return hand, "gripper_service", parts[5]
    if len(parts) == 6 and parts[3:] == ("gripper", "command", "mit"):
        return hand, "gripper_command", "mit"
    if len(parts) == 6 and parts[3:5] == ("gripper", "roundtrip"):
        if parts[5] in ("start", "stop"):
            return hand, "gripper_roundtrip", parts[5]
    raise KeyError("not found")


def _make_handler(node: Any):
    class Handler(BaseHTTPRequestHandler):
        server_version = "EndEffectorsDashboard/1.0"

        def _send_bytes(
                self, status: int, content_type: str, body: bytes) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def _send_json(self, status: int, payload: dict) -> None:
            body = json.dumps(
                payload, ensure_ascii=False, allow_nan=False,
                separators=(",", ":"),
            ).encode("utf-8")
            self._send_bytes(status, "application/json; charset=utf-8", body)

        def _read_json(self) -> object:
            raw_length = self.headers.get("Content-Length")
            if raw_length is None:
                raise ValueError("Content-Length is required")
            try:
                length = int(raw_length)
            except ValueError as exc:
                raise ValueError("invalid Content-Length") from exc
            if length < 0 or length > _MAX_REQUEST_BYTES:
                raise ValueError("request body is too large")
            try:
                return json.loads(self.rfile.read(length).decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise ValueError(
                    "request body must be valid UTF-8 JSON") from exc

        def do_GET(self) -> None:  # noqa: N802
            path = urlsplit(_request_path(self.path)).path
            if path == "/":
                self._send_bytes(200, "text/html; charset=utf-8", node.html)
                return
            if path == "/api/state":
                self._send_json(200, node.snapshot())
                return
            if (path.startswith(_CAMERA_PATH_PREFIX)
                    and path.endswith("/video_feed")):
                hand = path[len(_CAMERA_PATH_PREFIX): -len("/video_feed")]
                if hand not in _HANDS:
                    self._send_json(
                        404, {"ok": False, "message": "camera not found"})
                    return
                try:
                    node.proxy_camera_stream(hand, self)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                except Exception as exc:  # noqa: BLE001
                    node.get_logger().debug(
                        f"camera stream proxy ended for {hand}: {exc}")
                return
            if path == "/favicon.ico":
                self._send_bytes(204, "image/x-icon", b"")
                return
            self._send_json(404, {"ok": False, "message": "not found"})

        def do_POST(self) -> None:  # noqa: N802
            path = urlsplit(_request_path(self.path)).path
            if self.headers.get("X-Robot-Control") != "1":
                self._send_json(
                    403, {
                        "ok": False,
                        "message": "missing control request header",
                    })
                return
            try:
                hand, route_type, action = _control_route(path)
                if route_type == "sensor_service":
                    result = node.call_sensor_service(hand, action)
                elif route_type == "gripper_service":
                    result = node.call_gripper_service(hand, action)
                elif route_type == "gripper_command":
                    result = node.publish_mit(hand, self._read_json())
                elif action == "start":
                    result = node.start_roundtrip(hand, self._read_json())
                else:
                    result = node.stop_roundtrip(hand)
            except KeyError:
                self._send_json(404, {"ok": False, "message": "not found"})
                return
            except ValueError as exc:
                self._send_json(400, {"ok": False, "message": str(exc)})
                return
            except TimeoutError as exc:
                self._send_json(504, {"ok": False, "message": str(exc)})
                return
            except RuntimeError as exc:
                self._send_json(503, {"ok": False, "message": str(exc)})
                return
            except Exception as exc:  # noqa: BLE001
                node.get_logger().error(
                    f"unhandled HTTP error for {path}: {exc}")
                self._send_json(
                    500, {"ok": False, "message": "internal server error"})
                return
            self._send_json(200, result)

        def log_message(self, fmt: str, *args) -> None:
            node.get_logger().debug("HTTP " + (fmt % args))

    return Handler


class EndEffectorsDashboard(Node):
    """Expose the dual-hand ROS interfaces through one local web UI."""

    def __init__(self) -> None:
        super().__init__("end_effectors_dashboard")
        topology_defaults = dashboard_topology_parameters("dual")
        self._state_lock = threading.Lock()
        self._stream_locks = {
            hand: {
                "sensor": threading.Lock(),
                "gripper": threading.Lock(),
            }
            for hand in _HANDS
        }
        self._service_locks = {hand: threading.Lock() for hand in _HANDS}
        self._roundtrip_locks = {hand: threading.Lock() for hand in _HANDS}
        self._roundtrip_stops = {hand: threading.Event() for hand in _HANDS}
        self._roundtrip_threads: Dict[str, Optional[threading.Thread]] = {
            hand: None for hand in _HANDS
        }

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8770)
        self.declare_parameter("request_timeout_s", 3.0)
        self.declare_parameter("state_stale_s", 1.0)
        self.declare_parameter("safe_position_min", 0.0)
        self.declare_parameter("safe_position_max", 2.77)
        self.declare_parameter("mit_velocity_limit", 10.0)
        self.declare_parameter("mit_torque_limit", 12.0)
        for name, value in topology_defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter("left_camera_url", "http://127.0.0.1:8010")
        self.declare_parameter("right_camera_url", "http://127.0.0.1:8011")
        self.declare_parameter("camera_timeout_s", 1.0)
        self.declare_parameter("camera_poll_period_s", 2.0)
        self.declare_parameter("html_path", "")

        gp = self.get_parameter
        host = gp("host").get_parameter_value().string_value
        port = gp("port").get_parameter_value().integer_value
        self._request_timeout = gp(
            "request_timeout_s").get_parameter_value().double_value
        self._state_stale = gp(
            "state_stale_s").get_parameter_value().double_value
        self._safe_min = gp(
            "safe_position_min").get_parameter_value().double_value
        self._safe_max = gp(
            "safe_position_max").get_parameter_value().double_value
        self._velocity_limit = gp(
            "mit_velocity_limit").get_parameter_value().double_value
        self._torque_limit = gp(
            "mit_torque_limit").get_parameter_value().double_value
        self._camera_timeout = gp(
            "camera_timeout_s").get_parameter_value().double_value
        self._camera_poll_period = gp(
            "camera_poll_period_s").get_parameter_value().double_value
        html_override = gp(
            "html_path").get_parameter_value().string_value.strip()
        self._validate_configuration(port)

        self._config = {
            "left": {
                "label": "左手",
                "bus": gp("left_bus").get_parameter_value().string_value,
                "sensor_node": _target_name(
                    gp("left_sensor_node").get_parameter_value().string_value),
                "wrench_topic": gp(
                    "left_wrench_topic").get_parameter_value().string_value,
                "gripper_node": _target_name(
                    gp("left_gripper_node")
                    .get_parameter_value().string_value),
                "camera_url": gp(
                    "left_camera_url").get_parameter_value()
                .string_value.rstrip("/"),
            },
            "right": {
                "label": "右手",
                "bus": gp("right_bus").get_parameter_value().string_value,
                "sensor_node": _target_name(
                    gp("right_sensor_node")
                    .get_parameter_value().string_value),
                "wrench_topic": gp(
                    "right_wrench_topic").get_parameter_value().string_value,
                "gripper_node": _target_name(
                    gp("right_gripper_node")
                    .get_parameter_value().string_value),
                "camera_url": gp(
                    "right_camera_url").get_parameter_value()
                .string_value.rstrip("/"),
            },
        }
        for hand, config in self._config.items():
            if not config["bus"].strip("/"):
                raise ValueError(f"{hand}_bus must not be empty")
            if not config["wrench_topic"].startswith("/"):
                raise ValueError(
                    f"{hand}_wrench_topic must be an absolute topic")
            camera_url = urlsplit(config["camera_url"])
            if (camera_url.scheme not in ("http", "https")
                    or not camera_url.netloc):
                raise ValueError(
                    f"{hand}_camera_url must be an absolute HTTP URL")

        html_path = (
            Path(html_override).expanduser()
            if html_override
            else Path(__file__).with_name("dashboard.html")
        )
        try:
            self._html = html_path.read_bytes()
        except OSError as exc:
            raise RuntimeError(
                f"could not read web UI at {html_path}: {exc}") from exc

        now = time.monotonic()
        self._state = {
            hand: {
                "sensor": self._new_stream_state(now),
                "gripper": self._new_stream_state(now),
                "camera": {
                    "connected": False,
                    "message": "not checked",
                    "checked_at": None,
                },
                "last_action": None,
                "roundtrip": {
                    "running": False,
                    "phase": "idle",
                    "target": None,
                    "completed_segments": 0,
                    "message": "not started",
                },
            }
            for hand in _HANDS
        }
        self._camera_opener = build_opener(ProxyHandler({}))
        self._camera_stop = threading.Event()
        self._camera_thread = threading.Thread(
            target=self._poll_cameras,
            daemon=True,
            name="robot-web-cameras",
        )
        self._camera_thread.start()

        wrench_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=64,
        )
        latest_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._wrench_subscriptions = []
        self._joint_subscriptions = []
        self._mit_publishers = {}
        self._sensor_clients = {}
        self._gripper_clients = {}
        for hand, config in self._config.items():
            self._wrench_subscriptions.append(self.create_subscription(
                WrenchStamped,
                config["wrench_topic"],
                lambda message, selected=hand: self._on_wrench(
                    selected, message),
                wrench_qos,
                raw=True,
            ))
            joint_topic = _child_name(config["gripper_node"], "joint_states")
            self._joint_subscriptions.append(self.create_subscription(
                JointState,
                joint_topic,
                lambda message, selected=hand: self._on_joint_state(
                    selected, message),
                latest_qos,
            ))
            self._mit_publishers[hand] = self.create_publisher(
                MitCommand,
                _child_name(config["gripper_node"], "mit_command"),
                command_qos,
            )
            self._sensor_clients[hand] = {
                action: self.create_client(
                    Trigger,
                    _child_name(config["sensor_node"], action),
                )
                for action in _SENSOR_ACTIONS
            }
            self._gripper_clients[hand] = {
                action: self.create_client(
                    Trigger,
                    _child_name(config["gripper_node"], action),
                )
                for action in _GRIPPER_SERVICES
            }

        handler = _make_handler(self)
        self._httpd = ThreadingHTTPServer((host, port), handler)
        self._httpd.daemon_threads = True
        self._http_thread = threading.Thread(
            target=self._httpd.serve_forever,
            kwargs={"poll_interval": 0.2},
            daemon=True,
            name="robot-web-http",
        )
        self._http_thread.start()
        actual_port = int(self._httpd.server_address[1])
        self.get_logger().info(
            f"end-effector web dashboard on http://{host}:{actual_port}")

    @property
    def html(self) -> bytes:
        return self._html

    @staticmethod
    def _new_stream_state(now: float) -> dict:
        return {
            "sample": None,
            "received_monotonic": 0.0,
            "window_started": now,
            "window_count": 0,
            "rate_hz": 0.0,
        }

    def _validate_configuration(self, port: int) -> None:
        if not 0 <= port <= 65535:
            raise ValueError("port must be between 0 and 65535")
        positive = (
            ("request_timeout_s", self._request_timeout),
            ("state_stale_s", self._state_stale),
            ("mit_velocity_limit", self._velocity_limit),
            ("mit_torque_limit", self._torque_limit),
            ("camera_timeout_s", self._camera_timeout),
            ("camera_poll_period_s", self._camera_poll_period),
        )
        for name, value in positive:
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be greater than zero")
        if not (math.isfinite(self._safe_min)
                and math.isfinite(self._safe_max)
                and self._safe_min < self._safe_max):
            raise ValueError(
                "safe_position_min must be less than safe_position_max")

    def _record_stream_sample(
            self, hand: str, stream_name: str, sample: object) -> None:
        now = time.monotonic()
        with self._stream_locks[hand][stream_name]:
            stream = self._state[hand][stream_name]
            stream["sample"] = sample
            stream["received_monotonic"] = now
            stream["window_count"] += 1
            elapsed = now - stream["window_started"]
            if elapsed >= _STREAM_RATE_WINDOW_S:
                stream["rate_hz"] = stream["window_count"] / elapsed
                stream["window_count"] = 0
                stream["window_started"] = now

    def _on_wrench(self, hand: str, serialized_message: Any) -> None:
        self._record_stream_sample(hand, "sensor", serialized_message)

    def _on_joint_state(self, hand: str, message: Any) -> None:
        self._record_stream_sample(hand, "gripper", message)

    def _latest_gripper_position(self, hand: str) -> Optional[float]:
        with self._stream_locks[hand]["gripper"]:
            stream = self._state[hand]["gripper"]
            message = stream["sample"]
            received = stream["received_monotonic"]
            if (message is None or not message.position
                    or time.monotonic() - received > self._state_stale):
                return None
            position = float(message.position[0])
        return position if math.isfinite(position) else None

    def _record_action(
            self, hand: str, category: str, action: str,
            ok: bool, message: str) -> None:
        with self._state_lock:
            self._state[hand]["last_action"] = {
                "category": category,
                "action": action,
                "ok": bool(ok),
                "message": str(message),
                "time": time.time(),
            }

    def _set_roundtrip(self, hand: str, **values) -> None:
        with self._state_lock:
            self._state[hand]["roundtrip"].update(values)

    def snapshot(self) -> dict:
        now = time.monotonic()
        wall_now = time.time()
        stream_snapshots = {}
        for hand in _HANDS:
            for stream_name in ("sensor", "gripper"):
                with self._stream_locks[hand][stream_name]:
                    stream = self._state[hand][stream_name]
                    stream_snapshots[(hand, stream_name)] = (
                        stream["sample"],
                        stream["received_monotonic"],
                        stream["rate_hz"],
                    )

        hands = {}
        with self._state_lock:
            for hand, config in self._config.items():
                sensor_sample, sensor_received, sensor_rate = (
                    stream_snapshots[(hand, "sensor")])
                gripper_sample, gripper_received, gripper_rate = (
                    stream_snapshots[(hand, "gripper")])
                sensor_age = (
                    now - sensor_received
                    if sensor_received > 0.0 else None
                )
                gripper_age = (
                    now - gripper_received
                    if gripper_received > 0.0 else None
                )
                hands[hand] = {
                    "label": config["label"],
                    "bus": config["bus"],
                    "sensor": {
                        "target_node": config["sensor_node"],
                        "topic": config["wrench_topic"],
                        "connected": (
                            sensor_age is not None
                            and sensor_age <= self._state_stale),
                        "age_s": sensor_age,
                        "rate_hz": sensor_rate,
                        "wrench": _serialized_wrench_payload(
                            sensor_sample,
                            wall_now - sensor_age
                            if sensor_age is not None else None),
                    },
                    "gripper": {
                        "target_node": config["gripper_node"],
                        "topic": _child_name(
                            config["gripper_node"], "joint_states"),
                        "connected": (
                            gripper_age is not None
                            and gripper_age <= self._state_stale),
                        "age_s": gripper_age,
                        "rate_hz": gripper_rate,
                        "joint_state": _joint_state_payload(
                            gripper_sample,
                            wall_now - gripper_age
                            if gripper_age is not None else None),
                    },
                    "camera": {
                        **self._state[hand]["camera"],
                        "video_url": (
                            f"/api/cameras/{hand}/video_feed"),
                    },
                    "last_action": self._state[hand]["last_action"],
                    "roundtrip": dict(self._state[hand]["roundtrip"]),
                }
        for hand in _HANDS:
            hands[hand]["sensor"]["services"] = {
                name: client.service_is_ready()
                for name, client in self._sensor_clients[hand].items()
            }
            hands[hand]["gripper"]["services"] = {
                name: client.service_is_ready()
                for name, client in self._gripper_clients[hand].items()
            }
        return {
            "hands": hands,
            "safe_position": {"min": self._safe_min, "max": self._safe_max},
            "server_time": wall_now,
        }

    def _poll_cameras(self) -> None:
        while not self._camera_stop.is_set():
            for hand in _HANDS:
                if self._camera_stop.is_set():
                    return
                camera_url = self._config[hand]["camera_url"]
                connected = False
                message = "camera unavailable"
                try:
                    with self._camera_opener.open(
                            camera_url + "/status",
                            timeout=self._camera_timeout) as response:
                        payload = json.loads(response.read().decode("utf-8"))
                    connected = bool(payload.get("is_running"))
                    message = str(payload.get("message", "camera online"))
                except Exception as exc:  # noqa: BLE001
                    message = f"camera service unavailable: {exc}"
                with self._state_lock:
                    self._state[hand]["camera"].update({
                        "connected": connected,
                        "message": message,
                        "checked_at": time.time(),
                    })
            self._camera_stop.wait(self._camera_poll_period)

    def proxy_camera_stream(
            self, hand: str, downstream: BaseHTTPRequestHandler) -> None:
        if hand not in _HANDS:
            raise ValueError("unknown hand")
        camera_url = self._config[hand]["camera_url"]
        try:
            upstream = self._camera_opener.open(
                camera_url + "/video_feed", timeout=self._camera_timeout)
        except Exception as exc:  # noqa: BLE001
            downstream._send_json(  # type: ignore[attr-defined]
                503, {"ok": False, "message": f"camera unavailable: {exc}"})
            return
        with upstream:
            downstream.send_response(200)
            downstream.send_header(
                "Content-Type",
                upstream.headers.get(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=frame"),
            )
            downstream.send_header("Cache-Control", "no-store")
            downstream.send_header("X-Content-Type-Options", "nosniff")
            downstream.end_headers()
            while not self._camera_stop.is_set():
                chunk = upstream.read(64 * 1024)
                if not chunk:
                    return
                downstream.wfile.write(chunk)
                downstream.wfile.flush()

    def _call_trigger(
            self, hand: str, category: str, action: str,
            endpoint: str, client) -> dict:
        with self._service_locks[hand]:
            if not client.wait_for_service(timeout_sec=self._request_timeout):
                message = f"service unavailable: {endpoint}"
                self._record_action(hand, category, action, False, message)
                raise RuntimeError(message)
            future = client.call_async(Trigger.Request())
            completed = threading.Event()
            future.add_done_callback(lambda _future: completed.set())
            if not completed.wait(self._request_timeout):
                future.cancel()
                message = f"service timed out: {endpoint}"
                self._record_action(hand, category, action, False, message)
                raise TimeoutError(message)
            try:
                response = future.result()
            except Exception as exc:  # noqa: BLE001
                message = f"service call failed: {exc}"
                self._record_action(hand, category, action, False, message)
                raise RuntimeError(message) from exc
            if response is None:
                message = f"service returned no response: {endpoint}"
                self._record_action(hand, category, action, False, message)
                raise RuntimeError(message)
        result = {"ok": bool(response.success), "message": response.message}
        self._record_action(
            hand, category, action, result["ok"], result["message"])
        return result

    def call_sensor_service(self, hand: str, action: str) -> dict:
        if hand not in _HANDS or action not in _SENSOR_ACTIONS:
            raise ValueError("unknown sensor service")
        endpoint = _child_name(self._config[hand]["sensor_node"], action)
        return self._call_trigger(
            hand, "sensor", action, endpoint,
            self._sensor_clients[hand][action])

    def call_gripper_service(self, hand: str, action: str) -> dict:
        if hand not in _HANDS or action not in _GRIPPER_SERVICES:
            raise ValueError("unknown gripper service")
        if action == "disable":
            self._roundtrip_stops[hand].set()
        endpoint = _child_name(self._config[hand]["gripper_node"], action)
        return self._call_trigger(
            hand, "gripper", action, endpoint,
            self._gripper_clients[hand][action])

    def _validated_mit(self, payload: object) -> Dict[str, float]:
        values = _finite_fields(payload, ("q", "dq", "kp", "kd", "tau"))
        if not self._safe_min <= values["q"] <= self._safe_max:
            raise ValueError(
                f"q must be within [{self._safe_min}, {self._safe_max}]")
        if not 0.0 <= values["kp"] <= 500.0:
            raise ValueError("kp must be within [0, 500]")
        if not 0.0 <= values["kd"] <= 5.0:
            raise ValueError("kd must be within [0, 5]")
        if abs(values["dq"]) > self._velocity_limit:
            raise ValueError(
                f"abs(dq) must not exceed {self._velocity_limit}")
        if abs(values["tau"]) > self._torque_limit:
            raise ValueError(
                f"abs(tau) must not exceed {self._torque_limit}")
        return values

    def _publish_mit_values(self, hand: str, values: Dict[str, float]) -> None:
        message = MitCommand()
        message.q = values["q"]
        message.dq = values["dq"]
        message.kp = values["kp"]
        message.kd = values["kd"]
        message.tau = values["tau"]
        self._mit_publishers[hand].publish(message)

    def publish_mit(self, hand: str, payload: object) -> dict:
        if hand not in _HANDS:
            raise ValueError("unknown hand")
        values = self._validated_mit(payload)
        with self._roundtrip_locks[hand]:
            with self._state_lock:
                running = self._state[hand]["roundtrip"]["running"]
            if running:
                raise RuntimeError(
                    "stop the round trip before sending a manual command")
            self._publish_mit_values(hand, values)
        message = "MIT command published"
        self._record_action(hand, "gripper", "mit", True, message)
        return {"ok": True, "message": message}

    def start_roundtrip(self, hand: str, payload: object) -> dict:
        if hand not in _HANDS:
            raise ValueError("unknown hand")
        values = _finite_fields(
            payload,
            ("position_a", "position_b", "kp", "kd", "rate_hz",
             "switch_period_s"),
        )
        for name in ("position_a", "position_b"):
            if not self._safe_min <= values[name] <= self._safe_max:
                raise ValueError(
                    f"{name} must be within "
                    f"[{self._safe_min}, {self._safe_max}]")
        if values["position_a"] == values["position_b"]:
            raise ValueError("position_a and position_b must be different")
        if not 0.0 <= values["kp"] <= 500.0:
            raise ValueError("kp must be within [0, 500]")
        if not 0.0 <= values["kd"] <= 5.0:
            raise ValueError("kd must be within [0, 5]")
        if not 0.0 < values["rate_hz"] <= 100.0:
            raise ValueError("rate_hz must be within (0, 100]")
        if not 0.2 <= values["switch_period_s"] <= _ROUNDTRIP_MAX_SWITCH_PERIOD_S:
            raise ValueError("switch_period_s must be within [0.2, 3.0]")

        with self._roundtrip_locks[hand]:
            with self._state_lock:
                if self._state[hand]["roundtrip"]["running"]:
                    raise RuntimeError("round trip is already running")
            stop_event = self._roundtrip_stops[hand]
            stop_event.clear()
            self._set_roundtrip(
                hand,
                running=True,
                phase="enabling",
                target=None,
                completed_segments=0,
                message="enabling gripper in MIT mode",
            )

        enabled_succeeded = False
        try:
            enabled = self.call_gripper_service(hand, "enable")
            if not enabled["ok"]:
                raise RuntimeError(enabled["message"])
            enabled_succeeded = True
            if stop_event.is_set():
                raise RuntimeError("round trip cancelled while enabling")
        except Exception as exc:
            message = str(exc)
            if enabled_succeeded:
                try:
                    disabled = self.call_gripper_service(hand, "disable")
                    if not disabled["ok"]:
                        message = (
                            f"{message}; disable failed: "
                            f"{disabled['message']}")
                except Exception as disable_exc:  # noqa: BLE001
                    message = f"{message}; disable failed: {disable_exc}"
            self._set_roundtrip(
                hand, running=False, phase="error", target=None,
                message=message)
            self._record_action(hand, "roundtrip", "start", False, message)
            raise RuntimeError(message) from exc

        thread = threading.Thread(
            target=self._run_roundtrip,
            args=(hand, values),
            daemon=True,
            name=f"{hand}-mit-roundtrip",
        )
        with self._roundtrip_locks[hand]:
            self._roundtrip_threads[hand] = thread
            thread.start()
        message = "MIT round trip started"
        self._record_action(hand, "roundtrip", "start", True, message)
        return {"ok": True, "message": message}

    def stop_roundtrip(self, hand: str) -> dict:
        if hand not in _HANDS:
            raise ValueError("unknown hand")
        self._roundtrip_stops[hand].set()
        with self._state_lock:
            running = self._state[hand]["roundtrip"]["running"]
            if running:
                self._state[hand]["roundtrip"]["message"] = "stop requested"
        message = (
            "round trip stop requested"
            if running else "round trip is not running")
        self._record_action(hand, "roundtrip", "stop", True, message)
        return {"ok": True, "message": message}

    def _run_roundtrip(self, hand: str, values: Dict[str, float]) -> None:
        stop_event = self._roundtrip_stops[hand]
        period = 1.0 / values["rate_hz"]
        targets = (values["position_a"], values["position_b"])
        target_index = 0
        completed_segments = 0
        target_started = time.monotonic()
        error: Optional[str] = None
        try:
            while not stop_event.is_set():
                now = time.monotonic()
                target = targets[target_index]
                if _roundtrip_should_switch(
                        self._latest_gripper_position(hand), target,
                        now - target_started, values["switch_period_s"]):
                    target_index = 1 - target_index
                    completed_segments += 1
                    target_started = now
                    target = targets[target_index]
                self._set_roundtrip(
                    hand,
                    running=True,
                    phase="moving",
                    target=target,
                    completed_segments=completed_segments,
                    message=f"publishing MIT target {target:.3f} rad",
                )
                self._publish_mit_values(hand, {
                    "q": target,
                    "dq": 0.0,
                    "kp": values["kp"],
                    "kd": values["kd"],
                    "tau": 0.0,
                })
                stop_event.wait(period)
        except Exception as exc:  # noqa: BLE001
            error = str(exc)
            self.get_logger().error(f"{hand} round trip failed: {exc}")
        finally:
            try:
                disabled = self.call_gripper_service(hand, "disable")
                if not disabled["ok"] and error is None:
                    error = disabled["message"]
            except Exception as exc:  # noqa: BLE001
                if error is None:
                    error = f"disable failed: {exc}"
            phase = "error" if error is not None else "stopped"
            message = error or "round trip stopped; gripper disabled"
            self._set_roundtrip(
                hand, running=False, phase=phase, target=None, message=message)
            self._record_action(
                hand, "roundtrip", "finished", error is None, message)

    def destroy_node(self) -> bool:
        for stop_event in self._roundtrip_stops.values():
            stop_event.set()
        for thread in self._roundtrip_threads.values():
            if thread is not None and thread.is_alive():
                thread.join(timeout=self._request_timeout + 1.0)
        self._camera_stop.set()
        if self._camera_thread.is_alive():
            self._camera_thread.join(timeout=self._camera_timeout + 1.0)
        try:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._http_thread.join(timeout=1.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"web server shutdown failed: {exc}")
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node: Optional[EndEffectorsDashboard] = None
    try:
        node = EndEffectorsDashboard()
    except Exception as exc:  # noqa: BLE001
        get_logger("end_effectors_dashboard").fatal(str(exc))
        if rclpy.ok():
            rclpy.shutdown()
        return

    executor = SingleThreadedExecutor()
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
