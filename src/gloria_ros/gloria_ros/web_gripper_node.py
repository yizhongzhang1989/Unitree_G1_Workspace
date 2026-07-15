"""Gloria-M ROS 2 夹爪的浏览器控制与诊断节点"""

from __future__ import annotations

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.parse import urlsplit

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from gloria_ros.msg import MitCommand, PvCommand
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger


_MAX_REQUEST_BYTES = 64 * 1024
_SERVICE_NAMES = ("configure", "enable", "disable", "refresh", "set_zero")


def _diagnostic_level(value: object) -> int:
    if isinstance(value, int):
        return value
    if isinstance(value, (bytes, bytearray)) and len(value) == 1:
        return value[0]
    raise ValueError("diagnostic level must be an integer or one byte")


_DIAGNOSTIC_LEVELS = {
    _diagnostic_level(DiagnosticStatus.OK): "ok",
    _diagnostic_level(DiagnosticStatus.WARN): "warn",
    _diagnostic_level(DiagnosticStatus.ERROR): "error",
    _diagnostic_level(DiagnosticStatus.STALE): "stale",
}


def _target_name(value: str) -> str:
    normalized = "/" + value.strip().strip("/")
    if normalized == "/":
        raise ValueError("target_node must name a ROS node")
    return normalized


def _child_name(target: str, child: str) -> str:
    return f"{target}/{child.strip('/')}"


def _finite_number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float, str)):
        raise ValueError(f"{name} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number")
    return number


def _required_fields(payload: object, names) -> Dict[str, float]:
    if not isinstance(payload, dict):
        raise ValueError("request body must be a JSON object")
    missing = [name for name in names if name not in payload]
    if missing:
        raise ValueError(f"missing fields: {', '.join(missing)}")
    return {name: _finite_number(payload[name], name) for name in names}


class GloriaWebNode(Node):
    """通过本地 HTTP API 暴露 Gloria 夹爪的 ROS 接口"""

    def __init__(self) -> None:
        super().__init__("gloria_web_gripper")
        # HTTP 线程、往返线程和 ROS executor 会并发读写缓存状态
        self._cb_group = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()
        # 服务调用必须串行，避免多个 HTTP 请求并发驱动同一个设备状态机
        self._service_lock = threading.Lock()
        self._roundtrip_lock = threading.Lock()
        self._roundtrip_stop = threading.Event()
        self._roundtrip_thread: Optional[threading.Thread] = None

        self.declare_parameter("target_node", "/gloria_gripper")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8766)
        self.declare_parameter("request_timeout_s", 3.0)
        self.declare_parameter("state_stale_s", 1.0)
        self.declare_parameter("safe_position_min", 0.0)
        self.declare_parameter("safe_position_max", 2.77)
        self.declare_parameter("html_path", "")

        gp = self.get_parameter
        self._target = _target_name(gp("target_node").get_parameter_value().string_value)
        host = gp("host").get_parameter_value().string_value
        port = gp("port").get_parameter_value().integer_value
        self._request_timeout = gp("request_timeout_s").get_parameter_value().double_value
        self._state_stale = gp("state_stale_s").get_parameter_value().double_value
        self._safe_min = gp("safe_position_min").get_parameter_value().double_value
        self._safe_max = gp("safe_position_max").get_parameter_value().double_value
        html_override = gp("html_path").get_parameter_value().string_value.strip()
        if not 0 <= port <= 65535:
            raise ValueError("port must be between 0 and 65535")
        if not math.isfinite(self._request_timeout) or self._request_timeout <= 0.0:
            raise ValueError("request_timeout_s must be greater than zero")
        if not math.isfinite(self._state_stale) or self._state_stale <= 0.0:
            raise ValueError("state_stale_s must be greater than zero")
        if not self._safe_min < self._safe_max:
            raise ValueError("safe_position_min must be less than safe_position_max")

        html_path = (
            Path(html_override).expanduser()
            if html_override else Path(__file__).with_name("web_gripper.html")
        )
        try:
            # 页面启动时读入内存，HTTP 请求不再重复访问磁盘
            self._html = html_path.read_bytes()
        except OSError as exc:
            raise RuntimeError(f"could not read web UI at {html_path}: {exc}") from exc

        self._state_topic = _child_name(self._target, "joint_states")
        self._diagnostic_name = f"{self._target}: Gloria-M"
        self._joint_state: Optional[dict] = None
        self._joint_received_monotonic = 0.0
        self._diagnostic: Optional[dict] = None
        self._last_action: Optional[dict] = None
        self._roundtrip = {
            "running": False,
            "phase": "idle",
            "target": None,
            "message": "not started",
            "completed_segments": 0,
        }

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self._state_sub = self.create_subscription(
            JointState, self._state_topic, self._on_joint_state, state_qos,
            callback_group=self._cb_group)
        self._diagnostic_sub = self.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diagnostics, 10,
            callback_group=self._cb_group)

        self._mit_pub = self.create_publisher(
            MitCommand, _child_name(self._target, "mit_command"), 10)
        self._pv_pub = self.create_publisher(
            PvCommand, _child_name(self._target, "pv_command"), 10)
        self._service_clients = {
            name: self.create_client(
                Trigger, _child_name(self._target, name),
                callback_group=self._cb_group)
            for name in _SERVICE_NAMES
        }
        self._mode_client = self.create_client(
            SetBool, _child_name(self._target, "set_mode"),
            callback_group=self._cb_group)

        # HTTP 连接线程设为守护线程，节点关闭时不等待闲置客户端
        handler = _make_handler(self)
        self._httpd = ThreadingHTTPServer((host, port), handler)
        self._httpd.daemon_threads = True
        self._http_thread = threading.Thread(
            target=self._httpd.serve_forever,
            kwargs={"poll_interval": 0.2},
            daemon=True,
        )
        self._http_thread.start()
        actual_port = int(self._httpd.server_address[1])
        self.get_logger().info(
            f"Gloria web control on http://{host}:{actual_port} -> {self._target}. "
            f"SSH tunnel: ssh -L {actual_port}:127.0.0.1:{actual_port} user@server")

    def _record_action(self, action: str, ok: bool, message: str) -> None:
        with self._state_lock:
            self._last_action = {
                "action": action,
                "ok": bool(ok),
                "message": str(message),
                "time": time.time(),
            }

    def _on_joint_state(self, msg: JointState) -> None:
        # ROS 回调内生成不可变快照，HTTP 线程只读取已完成的数据
        state = {
            "stamp": {
                "sec": int(msg.header.stamp.sec),
                "nanosec": int(msg.header.stamp.nanosec),
            },
            "name": list(msg.name),
            "position": [float(value) for value in msg.position],
            "velocity": [float(value) for value in msg.velocity],
            "effort": [float(value) for value in msg.effort],
            "received_at": time.time(),
        }
        with self._state_lock:
            self._joint_state = state
            self._joint_received_monotonic = time.monotonic()

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name != self._diagnostic_name:
                continue
            level = _diagnostic_level(status.level)
            diagnostic = {
                "name": status.name,
                "hardware_id": status.hardware_id,
                "level": level,
                "level_name": _DIAGNOSTIC_LEVELS.get(level, "unknown"),
                "message": status.message,
                "values": {item.key: item.value for item in status.values},
                "received_at": time.time(),
            }
            with self._state_lock:
                self._diagnostic = diagnostic
            return

    def snapshot(self) -> dict:
        # 锁内只获取引用和复制可变往返状态，服务可用性查询放到锁外
        with self._state_lock:
            joint_state = self._joint_state
            joint_age = (
                time.monotonic() - self._joint_received_monotonic
                if joint_state is not None else None
            )
            diagnostic = self._diagnostic
            last_action = self._last_action
            roundtrip = dict(self._roundtrip)
        return {
            "target_node": self._target,
            "state_topic": self._state_topic,
            "connected": joint_age is not None and joint_age <= self._state_stale,
            "joint_age_s": joint_age,
            "joint_state": joint_state,
            "diagnostic": diagnostic,
            "services": {
                **{
                    name: client.service_is_ready()
                    for name, client in self._service_clients.items()
                },
                "set_mode": self._mode_client.service_is_ready(),
            },
            "last_action": last_action,
            "roundtrip": roundtrip,
            "server_time": time.time(),
        }

    def publish_mit(self, payload: object) -> dict:
        values = _required_fields(payload, ("q", "dq", "kp", "kd", "tau"))
        msg = MitCommand()
        msg.q = values["q"]
        msg.dq = values["dq"]
        msg.kp = values["kp"]
        msg.kd = values["kd"]
        msg.tau = values["tau"]
        self._mit_pub.publish(msg)
        message = "MIT command published"
        self._record_action("mit", True, message)
        return {"ok": True, "message": message}

    def publish_pv(self, payload: object) -> dict:
        values = _required_fields(payload, ("position", "velocity"))
        msg = PvCommand()
        msg.position = values["position"]
        msg.velocity = values["velocity"]
        self._pv_pub.publish(msg)
        message = "PV command published"
        self._record_action("pv", True, message)
        return {"ok": True, "message": message}

    def _call_service(self, action: str, service_name: str, client, request) -> dict:
        endpoint = _child_name(self._target, service_name)
        with self._service_lock:
            if not client.wait_for_service(timeout_sec=self._request_timeout):
                message = f"service unavailable: {endpoint}"
                self._record_action(action, False, message)
                raise RuntimeError(message)
            future = client.call_async(request)
            # HTTP 线程不能 spin ROS future，使用事件等待 executor 的完成回调
            completed = threading.Event()
            future.add_done_callback(lambda _future: completed.set())
            if not completed.wait(self._request_timeout):
                future.cancel()
                message = f"service timed out: {endpoint}"
                self._record_action(action, False, message)
                raise TimeoutError(message)
            try:
                response = future.result()
            except Exception as exc:  # noqa: BLE001
                message = f"service call failed: {exc}"
                self._record_action(action, False, message)
                raise RuntimeError(message) from exc
            if response is None:
                message = f"service returned no response: {endpoint}"
                self._record_action(action, False, message)
                raise RuntimeError(message)
        result = {"ok": bool(response.success), "message": response.message}
        self._record_action(action, result["ok"], result["message"])
        return result

    def call_service(self, name: str) -> dict:
        if name not in self._service_clients:
            raise ValueError(f"unknown service: {name}")
        if name == "disable":
            self._roundtrip_stop.set()
        return self._call_service(
            name, name, self._service_clients[name], Trigger.Request())

    def select_mode(self, payload: object) -> dict:
        if not isinstance(payload, dict):
            raise ValueError("request body must be a JSON object")
        mode = str(payload.get("mode", "")).strip().lower()
        if mode not in ("mit", "pos_vel"):
            raise ValueError("mode must be 'mit' or 'pos_vel'")
        request = SetBool.Request()
        request.data = mode == "pos_vel"
        return self._call_service("set_mode", "set_mode", self._mode_client, request)

    def _set_roundtrip(self, **values) -> None:
        with self._state_lock:
            self._roundtrip.update(values)

    def _latest_position(self) -> Optional[float]:
        with self._state_lock:
            state = self._joint_state
            received = self._joint_received_monotonic
        if (state is None or not state["position"]
                or time.monotonic() - received > self._state_stale):
            return None
        return float(state["position"][0])

    def start_roundtrip(self, payload: object) -> dict:
        values = _required_fields(
            payload,
            ("position_a", "position_b", "velocity", "rate_hz", "settle_threshold",
             "settle_time_s", "segment_timeout_s"),
        )
        position_a = values["position_a"]
        position_b = values["position_b"]
        if position_a == position_b:
            raise ValueError("position_a and position_b must be different")
        if not all(
                self._safe_min <= position <= self._safe_max
                for position in (position_a, position_b)):
            raise ValueError(
                f"round trip positions must be within "
                f"[{self._safe_min}, {self._safe_max}]")
        if not 0.0 < values["rate_hz"] <= 100.0:
            raise ValueError("rate_hz must be in (0, 100]")
        if values["velocity"] <= 0.0:
            raise ValueError("velocity must be greater than zero")
        if values["settle_threshold"] <= 0.0:
            raise ValueError("settle_threshold must be greater than zero")
        if values["settle_time_s"] < 0.0:
            raise ValueError("settle_time_s must not be negative")
        if values["segment_timeout_s"] <= 0.0:
            raise ValueError("segment_timeout_s must be greater than zero")

        with self._roundtrip_lock:
            if self._roundtrip_thread is not None and self._roundtrip_thread.is_alive():
                raise RuntimeError("round trip is already running")
            self._roundtrip_stop.clear()
            self._set_roundtrip(
                running=True,
                phase="starting",
                target=None,
                message="configuring and enabling",
                completed_segments=0,
            )
            self._roundtrip_thread = threading.Thread(
                target=self._run_roundtrip,
                args=(values,),
                daemon=True,
                name="gloria-roundtrip",
            )
            # 先记录启动结果，后台线程随后产生的错误才能成为最新状态
            self._record_action("roundtrip", True, "round trip started")
            self._roundtrip_thread.start()
        return {"ok": True, "message": "round trip started"}

    def stop_roundtrip(self) -> dict:
        self._roundtrip_stop.set()
        self._set_roundtrip(message="stop requested")
        return {"ok": True, "message": "round trip stop requested"}

    def _run_roundtrip(self, values: Dict[str, float]) -> None:
        error: Optional[str] = None
        try:
            disabled = self._call_service(
                "roundtrip_prepare", "disable",
                self._service_clients["disable"], Trigger.Request())
            if not disabled["ok"]:
                raise RuntimeError(disabled["message"])
            selected = self.select_mode({"mode": "pos_vel"})
            if not selected["ok"]:
                raise RuntimeError(selected["message"])
            enabled = self.call_service("enable")
            if not enabled["ok"]:
                raise RuntimeError(enabled["message"])

            period = 1.0 / values["rate_hz"]
            targets = (values["position_a"], values["position_b"])
            # 高频发布循环复用消息对象，避免每个周期分配新 ROS 消息
            command = PvCommand()
            command.velocity = values["velocity"]
            completed_segments = 0
            while not self._roundtrip_stop.is_set():
                target_index = completed_segments % len(targets)
                target = targets[target_index]
                command.position = target
                phase = "moving to A" if target_index == 0 else "moving to B"
                self._set_roundtrip(
                    phase=phase,
                    target=target,
                    message=f"{phase}: {target:.4f} rad",
                )
                segment_started = time.monotonic()
                settled_at: Optional[float] = None
                while not self._roundtrip_stop.is_set():
                    self._pv_pub.publish(command)
                    position = self._latest_position()
                    now = time.monotonic()
                    if position is not None and abs(position - target) < values["settle_threshold"]:
                        if settled_at is None:
                            settled_at = now
                        elif now - settled_at >= values["settle_time_s"]:
                            break
                    else:
                        settled_at = None
                    if now - segment_started >= values["segment_timeout_s"]:
                        self._set_roundtrip(
                            message=f"{phase} timed out; switching endpoint")
                        break
                    self._roundtrip_stop.wait(period)
                completed_segments += 1
                self._set_roundtrip(completed_segments=completed_segments)
        except Exception as exc:  # noqa: BLE001
            error = str(exc)
            self.get_logger().error(f"round trip failed: {exc}")
        finally:
            # 无论用户中止还是流程异常，退出前都尝试失能设备
            stopped = self._roundtrip_stop.is_set()
            try:
                disabled = self.call_service("disable")
                if not disabled["ok"] and error is None:
                    error = disabled["message"]
            except Exception as exc:  # noqa: BLE001
                if error is None:
                    error = f"disable failed: {exc}"
            if error is not None:
                phase = "error"
                message = error
            elif stopped:
                phase = "stopped"
                message = "round trip stopped; disable sent"
            else:
                phase = "complete"
                message = "round trip complete; disable sent"
            self._set_roundtrip(
                running=False, phase=phase, target=None, message=message)
            self._record_action("roundtrip", error is None, message)

    def destroy_node(self) -> bool:
        # 先停止运动线程，再关闭 HTTP 服务，避免销毁期间继续产生 ROS 请求
        self._roundtrip_stop.set()
        thread = self._roundtrip_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=self._request_timeout + 1.0)
        try:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._http_thread.join(timeout=1.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"web server shutdown failed: {exc}")
        return super().destroy_node()


def _make_handler(node: Any):
    # Handler 闭包直接访问节点，避免为每个请求额外查找全局实例
    class Handler(BaseHTTPRequestHandler):
        server_version = "GloriaWeb/1.0"

        def _send_bytes(self, status: int, content_type: str, body: bytes) -> None:
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
            raw = self.rfile.read(length)
            try:
                return json.loads(raw.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise ValueError("request body must be valid UTF-8 JSON") from exc

        def do_GET(self) -> None:  # noqa: N802
            path = urlsplit(self.path).path
            if path == "/":
                self._send_bytes(200, "text/html; charset=utf-8", node._html)
                return
            if path == "/api/state":
                self._send_json(200, node.snapshot())
                return
            if path == "/favicon.ico":
                self._send_bytes(204, "image/x-icon", b"")
                return
            self._send_json(404, {"ok": False, "message": "not found"})

        def do_POST(self) -> None:  # noqa: N802
            path = urlsplit(self.path).path
            if self.headers.get("X-Gloria-Control") != "1":
                self._send_json(
                    403, {"ok": False, "message": "missing control request header"})
                return
            try:
                if path == "/api/command/mit":
                    result = node.publish_mit(self._read_json())
                elif path == "/api/command/pv":
                    result = node.publish_pv(self._read_json())
                elif path == "/api/mode":
                    result = node.select_mode(self._read_json())
                elif path == "/api/roundtrip/start":
                    result = node.start_roundtrip(self._read_json())
                elif path == "/api/roundtrip/stop":
                    result = node.stop_roundtrip()
                elif path.startswith("/api/service/"):
                    result = node.call_service(path[len("/api/service/"):])
                else:
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
                node.get_logger().error(f"unhandled HTTP error for {path}: {exc}")
                self._send_json(
                    500, {"ok": False, "message": "internal server error"})
                return
            self._send_json(200, result)

        def log_message(self, fmt: str, *args) -> None:
            node.get_logger().debug("HTTP " + (fmt % args))

    return Handler


def main() -> None:
    rclpy.init()
    node: Optional[GloriaWebNode] = None
    try:
        node = GloriaWebNode()
    except Exception as exc:  # noqa: BLE001
        get_logger("gloria_web_gripper").fatal(str(exc))
        if rclpy.ok():
            rclpy.shutdown()
        return

    # 一个线程处理 ROS 状态，另一个线程完成 HTTP 发起的异步服务回调
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
