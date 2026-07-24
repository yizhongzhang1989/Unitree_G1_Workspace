import json
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

from geometry_msgs.msg import WrenchStamped
from rclpy.serialization import serialize_message

from robot_bringup.end_effectors.dashboard_node import (
    _control_route,
    _finite_fields,
    _make_handler,
    _roundtrip_should_switch,
    _serialized_wrench_payload,
    _target_name,
)


class _Logger:
    def __init__(self) -> None:
        self.errors = []

    def debug(self, message: str) -> None:
        del message
        pass

    def error(self, message: str) -> None:
        self.errors.append(message)


class _FakeDashboard:
    html = b"<!doctype html><title>Robot dashboard test</title>"

    def __init__(self) -> None:
        self.calls = []
        self.logger = _Logger()
        self.camera_available = True

    def get_logger(self):
        return self.logger

    def snapshot(self):
        return {"hands": {"left": {"bus": "can0"}, "right": {"bus": "can1"}}}

    def call_sensor_service(self, hand, action):
        self.calls.append(("sensor", hand, action))
        return {"ok": True, "message": action}

    def call_gripper_service(self, hand, action):
        self.calls.append(("gripper_service", hand, action))
        return {"ok": True, "message": action}

    def publish_mit(self, hand, payload):
        self.calls.append(("mit", hand, payload))
        return {"ok": True, "message": "mit"}

    def start_roundtrip(self, hand, payload):
        self.calls.append(("roundtrip_start", hand, payload))
        return {"ok": True, "message": "roundtrip started"}

    def stop_roundtrip(self, hand):
        self.calls.append(("roundtrip_stop", hand))
        return {"ok": True, "message": "roundtrip stopped"}

    def proxy_camera_stream(self, hand, handler):
        self.calls.append(("camera", hand))
        if self.camera_available:
            handler._send_bytes(200, "image/jpeg", b"camera-frame")
        else:
            handler._send_json(
                503, {"ok": False, "message": "camera unavailable"})


class HelpersTest(unittest.TestCase):
    def test_normalizes_target_names(self) -> None:
        self.assertEqual(_target_name("grip_arm0"), "/grip_arm0")
        with self.assertRaises(ValueError):
            _target_name("/")

    def test_requires_finite_numeric_fields(self) -> None:
        self.assertEqual(_finite_fields({"q": "1.25"}, ("q",)), {"q": 1.25})
        for payload in ({}, {"q": True}, {"q": "nan"}):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                _finite_fields(payload, ("q",))

    def test_roundtrip_switches_on_target_or_timeout(self) -> None:
        self.assertTrue(_roundtrip_should_switch(0.09, 0.0, 0.1, 3.0))
        self.assertFalse(_roundtrip_should_switch(0.11, 0.0, 2.9, 3.0))
        self.assertFalse(_roundtrip_should_switch(None, 0.0, 2.9, 3.0))
        self.assertTrue(_roundtrip_should_switch(None, 0.0, 3.0, 3.0))

    def test_routes_only_known_hands_and_actions(self) -> None:
        self.assertEqual(
            _control_route("/api/hands/left/sensor/tare"),
            ("left", "sensor_service", "tare"),
        )
        self.assertEqual(
            _control_route("/api/hands/right/gripper/command/mit"),
            ("right", "gripper_command", "mit"),
        )
        self.assertEqual(
            _control_route("/api/hands/right/gripper/roundtrip/start"),
            ("right", "gripper_roundtrip", "start"),
        )
        with self.assertRaises(ValueError):
            _control_route("/api/hands/center/sensor/start")
        with self.assertRaises(KeyError):
            _control_route("/api/hands/left/sensor/reboot")

    def test_serialized_wrench_payload_preserves_message_fields(self) -> None:
        message = WrenchStamped()
        message.header.stamp.sec = 123
        message.header.stamp.nanosec = 456
        message.header.frame_id = "arm0_ft_link"
        message.wrench.force.x = 1.25
        message.wrench.force.y = -2.5
        message.wrench.force.z = 3.75
        message.wrench.torque.x = -0.1
        message.wrench.torque.y = 0.2
        message.wrench.torque.z = -0.3

        payload = _serialized_wrench_payload(
            serialize_message(message), 1000.5)

        self.assertIsNotNone(payload)
        assert payload is not None
        self.assertEqual(payload["stamp"], {"sec": 123, "nanosec": 456})
        self.assertEqual(payload["frame_id"], "arm0_ft_link")
        self.assertEqual(payload["force"], {
            "x": 1.25, "y": -2.5, "z": 3.75})
        self.assertAlmostEqual(payload["torque"]["x"], -0.1)
        self.assertAlmostEqual(payload["torque"]["y"], 0.2)
        self.assertAlmostEqual(payload["torque"]["z"], -0.3)
        self.assertEqual(payload["received_at"], 1000.5)


class HandlerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.node = _FakeDashboard()
        self.server = ThreadingHTTPServer(
            ("127.0.0.1", 0), _make_handler(self.node))
        self.thread = threading.Thread(
            target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base_url = f"http://127.0.0.1:{self.server.server_address[1]}"
        self.opener = build_opener(ProxyHandler({}))

    def tearDown(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1.0)

    def _post(self, path: str, payload=None, include_header: bool = True):
        headers = {"Content-Type": "application/json"}
        if include_header:
            headers["X-Robot-Control"] = "1"
        request = Request(
            self.base_url + path,
            data=json.dumps(
                payload if payload is not None else {}).encode("utf-8"),
            headers=headers,
            method="POST",
        )
        with self.opener.open(request, timeout=2.0) as response:
            return response.status, json.loads(response.read())

    def test_serves_page_and_combined_state(self) -> None:
        with self.opener.open(self.base_url + "/", timeout=2.0) as response:
            self.assertIn(b"Robot dashboard test", response.read())
        with self.opener.open(
                self.base_url + "/api/state", timeout=2.0) as response:
            state = json.loads(response.read())
        self.assertEqual(state["hands"]["left"]["bus"], "can0")
        self.assertEqual(state["hands"]["right"]["bus"], "can1")

    def test_routes_sensor_and_gripper_controls_by_hand(self) -> None:
        cases = (
            ("/api/hands/left/sensor/start", None,
             ("sensor", "left", "start")),
            ("/api/hands/right/gripper/service/enable", None,
             ("gripper_service", "right", "enable")),
            ("/api/hands/left/gripper/command/mit", {"q": 1.0},
             ("mit", "left", {"q": 1.0})),
            ("/api/hands/right/gripper/roundtrip/start", {"position_a": 0.2},
             ("roundtrip_start", "right", {"position_a": 0.2})),
            ("/api/hands/right/gripper/roundtrip/stop", None,
             ("roundtrip_stop", "right")),
        )
        for path, payload, expected in cases:
            with self.subTest(path=path):
                status, result = self._post(path, payload)
                self.assertEqual(status, 200)
                self.assertTrue(result["ok"])
                self.assertEqual(self.node.calls[-1], expected)

    def test_proxies_camera_without_affecting_dashboard(self) -> None:
        with self.opener.open(
                self.base_url + "/api/cameras/left/video_feed",
                timeout=2.0) as response:
            self.assertEqual(response.headers.get_content_type(), "image/jpeg")
            self.assertEqual(response.read(), b"camera-frame")
        self.assertEqual(self.node.calls[-1], ("camera", "left"))

        self.node.camera_available = False
        with self.assertRaises(HTTPError) as raised:
            self.opener.open(
                self.base_url + "/api/cameras/right/video_feed",
                timeout=2.0)
        self.assertEqual(raised.exception.code, 503)

    def test_rejects_unknown_camera(self) -> None:
        with self.assertRaises(HTTPError) as raised:
            self.opener.open(
                self.base_url + "/api/cameras/center/video_feed",
                timeout=2.0)
        self.assertEqual(raised.exception.code, 404)

    def test_rejects_requests_without_control_header(self) -> None:
        with self.assertRaises(HTTPError) as raised:
            self._post("/api/hands/left/sensor/stop", include_header=False)
        self.assertEqual(raised.exception.code, 403)
        self.assertEqual(self.node.calls, [])

    def test_rejects_unknown_control_route(self) -> None:
        with self.assertRaises(HTTPError) as raised:
            self._post("/api/hands/left/gripper/service/reboot")
        self.assertEqual(raised.exception.code, 404)
        self.assertEqual(self.node.calls, [])


class HtmlContractTest(unittest.TestCase):
    def test_page_exposes_six_streams_camera_and_mit_controls(self) -> None:
        html_path = (
            Path(__file__).parents[1]
            / "robot_bringup"
            / "end_effectors"
            / "dashboard.html")
        html = " ".join(html_path.read_text(encoding="utf-8").split())
        for stream in (
            "left-camera", "left-sensor", "left-gripper",
            "right-camera", "right-sensor", "right-gripper"):
            with self.subTest(stream=stream):
                self.assertIn(f'data-stream="{stream}"', html)
        self.assertIn("左右手末端联调", html)
        self.assertIn('data-roundtrip="start"', html)
        self.assertIn('data-roundtrip="stop"', html)
        self.assertIn('data-command="mit"', html)
        self.assertIn('data-gripper="refresh"', html)
        self.assertIn('data-role="camera-feed"', html)
        self.assertIn('data-role="camera-placeholder"', html)
        self.assertIn("function updateCamera(hand, panel, camera)", html)
        self.assertIn("cameraRetryAt", html)
        for axis in ("fx", "fy", "fz", "mx", "my", "mz"):
            with self.subTest(axis=axis):
                self.assertIn(f'data-axis="{axis}"', html)
                self.assertIn(f'data-value="{axis}"', html)
        self.assertNotIn('data-vector="force"', html)
        self.assertNotIn('data-vector="torque"', html)
        self.assertNotIn("function drawVector(", html)
        self.assertNotIn('data-norm="force"', html)
        self.assertNotIn('data-norm="torque"', html)
        roundtrip_defaults = (
            'data-field="position-a" value="0.00"',
            'data-field="position-b" value="2.50"',
            'data-field="roundtrip-kp" value="10"',
            'data-field="roundtrip-kd" value="1"',
            'data-field="rate-hz" value="100"',
            'data-field="switch-period" value="3" min="0.2" max="3"',
        )
        for default in roundtrip_defaults:
            with self.subTest(roundtrip_default=default):
                self.assertIn(default, html)
        self.assertIn("`/api/hands/${hand}/${endpoint}`", html)
        self.assertNotIn("gripper/mode", html)
        self.assertNotIn("command/pv", html)


if __name__ == "__main__":
    unittest.main()
