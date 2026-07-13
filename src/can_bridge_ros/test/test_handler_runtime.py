import json
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from can_bridge_ros.handler_api import (
    FrameDisposition,
    FrameHandlerContext,
    FrameHandlerRegistration,
    FrameKey,
)
from can_bridge_ros.handler_runtime import FrameHandlerRuntime


class _Logger:
    def __init__(self) -> None:
        self.errors = []
        self.warnings = []

    def error(self, message: str) -> None:
        self.errors.append(message)

    def warning(self, message: str) -> None:
        self.warnings.append(message)


class _Node:
    def __init__(self) -> None:
        self.destroyed = False

    def destroy_node(self) -> None:
        self.destroyed = True


class FrameHandlerRuntimeTest(unittest.TestCase):
    def setUp(self) -> None:
        self.logger = _Logger()
        self.context = FrameHandlerContext(
            logger=self.logger,
            send_frame=lambda _channel, _can_id, _data: True,
            ros_context=None,
        )

    def _runtime(self, callback, **kwargs) -> FrameHandlerRuntime:
        registration = FrameHandlerRegistration(
            name="test", keys=(FrameKey(0, 0x15),), callback=callback)
        return FrameHandlerRuntime([registration], self.logger, **kwargs)

    def test_dispatches_matching_key_and_consumes(self) -> None:
        calls = []
        runtime = self._runtime(
            lambda channel, message: calls.append((channel, message))
            or FrameDisposition.CONSUME)
        message = SimpleNamespace(arbitration_id=0x15)

        self.assertIs(runtime.dispatch(0, message), FrameDisposition.FORWARD)
        runtime.start()
        self.assertIs(runtime.dispatch(0, message), FrameDisposition.CONSUME)
        self.assertEqual(calls, [(0, message)])
        self.assertIs(
            runtime.dispatch(1, message), FrameDisposition.FORWARD)

    def test_rejects_duplicate_handler_keys(self) -> None:
        registrations = [
            FrameHandlerRegistration(
                name=name,
                keys=(FrameKey(0, 0x15),),
                callback=lambda _channel, _message: FrameDisposition.CONSUME,
            )
            for name in ("first", "second")
        ]
        with self.assertRaisesRegex(ValueError, "claimed by both"):
            FrameHandlerRuntime(registrations, self.logger)

    def test_callback_failure_forwards_then_disables_handler(self) -> None:
        calls = 0

        def fail(_channel, _message):
            nonlocal calls
            calls += 1
            raise RuntimeError("broken")

        runtime = self._runtime(fail, failure_limit=2)
        runtime.start()
        message = SimpleNamespace(arbitration_id=0x15)
        self.assertIs(runtime.dispatch(0, message), FrameDisposition.FORWARD)
        self.assertIs(runtime.dispatch(0, message), FrameDisposition.FORWARD)
        self.assertIs(runtime.dispatch(0, message), FrameDisposition.FORWARD)
        self.assertEqual(calls, 2)
        self.assertTrue(any("disabled" in error for error in self.logger.errors))

    def test_loads_explicit_json_factory_and_runs_lifecycle(self) -> None:
        events = []
        node = _Node()

        def create(_context, config):
            self.assertEqual(config, {"device": "fixture"})
            return FrameHandlerRegistration(
                name="loaded",
                keys=(FrameKey(0, 0x16),),
                callback=lambda _channel, _message: FrameDisposition.CONSUME,
                auxiliary_nodes=(node,),
                start=lambda: events.append("start"),
                stop=lambda: events.append("stop"),
            )

        spec = json.dumps({
            "factory": "fixture.handlers:create",
            "config": {"device": "fixture"},
        })
        with patch(
                "can_bridge_ros.handler_runtime.importlib.import_module",
                return_value=SimpleNamespace(create=create)):
            runtime = FrameHandlerRuntime.from_specs([spec], self.context)

        self.assertEqual(runtime.auxiliary_nodes, (node,))
        runtime.start()
        runtime.start()
        runtime.stop()
        runtime.stop()
        self.assertEqual(events, ["start", "stop"])

    def test_rejects_invalid_json_and_unknown_fields(self) -> None:
        with self.assertRaisesRegex(ValueError, "invalid frame handler JSON"):
            FrameHandlerRuntime.from_specs(["{"], self.context)
        spec = json.dumps({"factory": "x:y", "unexpected": True})
        with self.assertRaisesRegex(ValueError, "unknown frame handler fields"):
            FrameHandlerRuntime.from_specs([spec], self.context)

    def test_factory_failure_destroys_previously_created_nodes(self) -> None:
        node = _Node()

        def first(_context, _config):
            return FrameHandlerRegistration(
                name="first",
                keys=(FrameKey(0, 0x15),),
                callback=lambda _channel, _message: FrameDisposition.CONSUME,
                auxiliary_nodes=(node,),
            )

        module = SimpleNamespace(first=first, invalid=lambda _context, _config: None)
        specs = [
            json.dumps({"factory": "fixture:first"}),
            json.dumps({"factory": "fixture:invalid"}),
        ]
        with patch(
                "can_bridge_ros.handler_runtime.importlib.import_module",
                return_value=module):
            with self.assertRaisesRegex(TypeError, "did not return"):
                FrameHandlerRuntime.from_specs(specs, self.context)
        self.assertTrue(node.destroyed)


if __name__ == "__main__":
    unittest.main()