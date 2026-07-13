import json
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

from gloria_ros.web_gripper_node import (
    _child_name,
    _diagnostic_level,
    _make_handler,
    _required_fields,
    _target_name,
)


class _Logger:
    def __init__(self) -> None:
        self.errors = []

    def debug(self, _message: str) -> None:
        pass

    def error(self, message: str) -> None:
        self.errors.append(message)


class _FakeNode:
    def __init__(self) -> None:
        self._html = b"<!doctype html><title>Gloria test</title>"
        self.calls = []
        self._logger = _Logger()
        self.raise_on_pv = False

    def get_logger(self):
        return self._logger

    def snapshot(self):
        return {"target_node": "/gripper", "connected": True}

    def publish_mit(self, payload):
        self.calls.append(("mit", payload))
        return {"ok": True, "message": "mit"}

    def publish_pv(self, payload):
        if self.raise_on_pv:
            raise ZeroDivisionError("division by zero")
        self.calls.append(("pv", payload))
        return {"ok": True, "message": "pv"}

    def call_service(self, name):
        self.calls.append(("service", name))
        return {"ok": True, "message": name}

    def select_mode(self, payload):
        self.calls.append(("mode", payload))
        return {"ok": True, "message": "mode selected"}

    def start_roundtrip(self, payload):
        self.calls.append(("roundtrip_start", payload))
        return {"ok": True, "message": "round trip started"}

    def stop_roundtrip(self):
        self.calls.append(("roundtrip_stop", None))
        return {"ok": True, "message": "round trip stopped"}


    # 辅助函数测试覆盖进入 ROS 消息层前的输入边界
class WebHelpersTest(unittest.TestCase):
    def test_diagnostic_level_accepts_foxy_bytes_and_integer(self) -> None:
        self.assertEqual(_diagnostic_level(b"\x02"), 2)
        self.assertEqual(_diagnostic_level(3), 3)
        with self.assertRaises(ValueError):
            _diagnostic_level(b"")

    def test_target_and_private_interface_names(self) -> None:
        self.assertEqual(_target_name("left/gloria"), "/left/gloria")
        self.assertEqual(
            _child_name("/left/gloria", "/mit_command"),
            "/left/gloria/mit_command",
        )
        with self.assertRaises(ValueError):
            _target_name("/")

    def test_required_fields_reject_missing_and_non_finite_values(self) -> None:
        self.assertEqual(
            _required_fields({"position": "1.25"}, ("position",)),
            {"position": 1.25},
        )
        for payload in ({}, {"position": True}, {"position": "nan"}):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                _required_fields(payload, ("position",))


class WebHandlerTest(unittest.TestCase):
    def setUp(self) -> None:
        # 使用本地临时端口并禁用系统代理，测试不会访问外部网络
        self.node = _FakeNode()
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), _make_handler(self.node))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base_url = f"http://127.0.0.1:{self.server.server_address[1]}"
        self.opener = build_opener(ProxyHandler({}))

    def tearDown(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1.0)

    def _post(self, path: str, payload=None, control_header: bool = True):
        headers = {"Content-Type": "application/json"}
        if control_header:
            headers["X-Gloria-Control"] = "1"
        request = Request(
            self.base_url + path,
            data=json.dumps(payload if payload is not None else {}).encode("utf-8"),
            headers=headers,
            method="POST",
        )
        with self.opener.open(request, timeout=2.0) as response:
            return response.status, json.loads(response.read())

    def test_serves_independent_html_and_state_api(self) -> None:
        with self.opener.open(self.base_url + "/", timeout=2.0) as response:
            self.assertEqual(response.status, 200)
            self.assertIn(b"Gloria test", response.read())
        with self.opener.open(
                self.base_url + "/api/state", timeout=2.0) as response:
            self.assertEqual(
                json.loads(response.read()),
                {"target_node": "/gripper", "connected": True},
            )

    def test_routes_every_command_and_service(self) -> None:
        cases = (
            ("/api/command/mit", {"q": 1.0}, ("mit", {"q": 1.0})),
            ("/api/command/pv", {"position": 1.0}, ("pv", {"position": 1.0})),
        )
        for path, payload, expected in cases:
            with self.subTest(path=path):
                status, result = self._post(path, payload)
                self.assertEqual(status, 200)
                self.assertTrue(result["ok"])
                self.assertEqual(self.node.calls[-1], expected)
        for name in ("configure", "enable", "disable", "refresh", "set_zero"):
            with self.subTest(service=name):
                self._post(f"/api/service/{name}")
                self.assertEqual(self.node.calls[-1], ("service", name))
            self._post("/api/mode", {"mode": "pos_vel"})
            self.assertEqual(self.node.calls[-1], ("mode", {"mode": "pos_vel"}))
        self._post("/api/roundtrip/start", {"position_a": 0.2})
        self.assertEqual(
            self.node.calls[-1], ("roundtrip_start", {"position_a": 0.2}))
        self._post("/api/roundtrip/stop")
        self.assertEqual(self.node.calls[-1], ("roundtrip_stop", None))

    def test_rejects_removed_compatible_position_route(self) -> None:
        with self.assertRaises(HTTPError) as raised:
            self._post("/api/command/position", {"position": 1.0})
        self.assertEqual(raised.exception.code, 404)
        self.assertEqual(self.node.calls, [])

    def test_rejects_post_without_control_header(self) -> None:
        with self.assertRaises(HTTPError) as raised:
            self._post(
                "/api/service/enable", control_header=False)
        self.assertEqual(raised.exception.code, 403)
        self.assertEqual(self.node.calls, [])

    def test_returns_json_for_unexpected_backend_error(self) -> None:
        self.node.raise_on_pv = True
        with self.assertRaises(HTTPError) as raised:
            self._post("/api/command/pv", {"position": 1.0})
        self.assertEqual(raised.exception.code, 500)
        self.assertEqual(
            json.loads(raised.exception.read()),
            {"ok": False, "message": "internal server error"},
        )
        self.assertIn("division by zero", self.node._logger.errors[-1])


class HtmlContractTest(unittest.TestCase):
    def test_html_is_separate_and_exposes_all_routes(self) -> None:
        html_path = Path(__file__).parents[1] / "gloria_ros" / "web_gripper.html"
        html = html_path.read_text(encoding="utf-8")
        # 折叠格式化空白，只验证页面契约而不限制属性必须写在同一行
        compact_html = " ".join(html.split())
        self.assertNotIn("<!doctype html>", Path(
            __file__).parents[1].joinpath(
                "gloria_ros", "web_gripper_node.py").read_text(encoding="utf-8").lower())
        self.assertIn("`/api/command/${endpoint}`", compact_html)
        self.assertIn("/api/roundtrip/start", compact_html)
        self.assertIn("/api/roundtrip/stop", compact_html)
        self.assertIn('postJson("/api/mode", { mode: tab.dataset.mode })', compact_html)
        self.assertIn('data-tab="mit" data-mode="mit"', compact_html)
        self.assertIn('data-tab="pv" data-mode="pos_vel"', compact_html)
        roundtrip_defaults = (
            'name="position_a" value="0.00"',
            'name="position_b" value="2.50"',
            'name="velocity" value="1.00"',
            'name="rate_hz" value="100"',
            'name="settle_threshold" value="0.10" min="0.001" step="0.001"',
            'name="segment_timeout_s" value="8"',
            'name="settle_time_s" value="0.30"',
        )
        for field in roundtrip_defaults:
            with self.subTest(roundtrip_default=field):
                self.assertIn(field, compact_html)
        for endpoint in ("mit", "pv"):
            with self.subTest(endpoint=endpoint):
                self.assertIn(f'submitCommand(event.currentTarget, "{endpoint}")', compact_html)
        for service in ("configure", "enable", "disable", "refresh", "set_zero"):
            with self.subTest(service=service):
                self.assertIn(f'data-service="{service}"', compact_html)


if __name__ == "__main__":
    unittest.main()