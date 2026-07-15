import json
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

from robot_bringup.web_dashboard_node import (
    _control_route,
    _finite_fields,
    _make_handler,
    _target_name,
)


class _Logger:
    def __init__(self) -> None:
        self.errors = []

    def debug(self, _message: str) -> None:
        pass

    def error(self, message: str) -> None:
        self.errors.append(message)


class _FakeDashboard:
    html = b"<!doctype html><title>Robot dashboard test</title>"

    def __init__(self) -> None:
        self.calls = []
        self.logger = _Logger()

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
    def test_page_exposes_four_streams_and_mit_controls(self) -> None:
        html_path = (
            Path(__file__).parents[1]
            / "robot_bringup"
            / "web_dashboard.html")
        html = " ".join(html_path.read_text(encoding="utf-8").split())
        for stream in (
                "left-sensor", "left-gripper",
                "right-sensor", "right-gripper"):
            with self.subTest(stream=stream):
                self.assertIn(f'data-stream="{stream}"', html)
        self.assertIn("CAN0 左手 / CAN1 右手", html)
        self.assertIn('data-roundtrip="start"', html)
        self.assertIn('data-roundtrip="stop"', html)
        self.assertIn('data-command="mit"', html)
        self.assertIn('data-gripper="refresh"', html)
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
            'data-field="switch-period" value="8"',
        )
        for default in roundtrip_defaults:
            with self.subTest(roundtrip_default=default):
                self.assertIn(default, html)
        self.assertIn("`/api/hands/${hand}/${endpoint}`", html)
        self.assertNotIn("gripper/mode", html)
        self.assertNotIn("command/pv", html)


if __name__ == "__main__":
    unittest.main()
