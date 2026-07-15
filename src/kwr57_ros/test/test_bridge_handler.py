import json
import struct
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from can_msgs.msg import Frame

from can_bridge_ros.handler_api import FrameDisposition, FrameHandlerContext
from kwr57_ros.bridge_handler import (
    build_frame_handler_spec,
    create_frame_handler,
)
from kwr57_ros.ft_sensor_node import KWR57DeviceNode
from kwr57_sensor import WrenchAssembler


class _CoreNode:
    """Minimal state needed to exercise KWR57DeviceNode's shared RX core."""

    def __init__(self) -> None:
        self._data_ids = (0x15, 0x16, 0x17)
        self._assembler = WrenchAssembler(self._data_ids)
        self._frames_seen = 0
        self.wrenches = []

    def _handle_wrench(self, wrench) -> None:
        self.wrenches.append(wrench)


class _FakeDeviceNode:
    def __init__(self, **kwargs) -> None:
        self.kwargs = kwargs
        self.frames = []
        self.activated = 0
        self.stopped = 0

    def handle_can_frame(self, can_id, data, **flags) -> bool:
        self.frames.append((can_id, bytes(data), flags))
        return not flags["is_extended"] and flags["dlc"] == 8

    def activate(self) -> None:
        self.activated += 1

    def stop_device(self) -> None:
        self.stopped += 1


class Kwr57SharedDataPathTest(unittest.TestCase):
    def setUp(self) -> None:
        self.node = _CoreNode()

    def test_direct_frames_assemble_one_wrench(self) -> None:
        payloads = (
            struct.pack("<2f", 1.0, 2.0),
            struct.pack("<2f", 3.0, 4.0),
            struct.pack("<2f", 5.0, 6.0),
        )
        for can_id, payload in zip(self.node._data_ids, payloads):
            handled = KWR57DeviceNode.handle_can_frame(
                self.node, can_id, payload, dlc=8)
            self.assertTrue(handled)

        self.assertEqual(self.node._frames_seen, 3)
        self.assertEqual(len(self.node.wrenches), 1)
        wrench = self.node.wrenches[0]
        self.assertEqual(
            (wrench.fx, wrench.fy, wrench.fz, wrench.mx, wrench.my, wrench.mz),
            (1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
        )

    def test_invalid_or_unrelated_frame_is_not_consumed(self) -> None:
        self.assertFalse(KWR57DeviceNode.handle_can_frame(
            self.node, 0x15, b"\x00" * 8, is_extended=True, dlc=8))
        self.assertFalse(KWR57DeviceNode.handle_can_frame(
            self.node, 0x15, b"\x00" * 7, dlc=7))
        self.assertFalse(KWR57DeviceNode.handle_can_frame(
            self.node, 0x20, b"\x00" * 8, dlc=8))
        self.assertEqual(self.node._frames_seen, 0)

    def test_ros_frame_adapter_uses_shared_core(self) -> None:
        calls = []
        adapter = SimpleNamespace(
            handle_can_frame=lambda *args, **kwargs: calls.append((args, kwargs)))
        frame = Frame()
        frame.id = 0x15
        frame.dlc = 8
        frame.data = list(range(8))

        KWR57DeviceNode._on_frame(adapter, frame)

        self.assertEqual(calls[0][0], (0x15, bytes(range(8))))
        self.assertEqual(calls[0][1]["dlc"], 8)


class Kwr57HandlerFactoryTest(unittest.TestCase):
    def setUp(self) -> None:
        self.context = FrameHandlerContext(
            logger=SimpleNamespace(),
            send_frame=lambda _channel, _can_id, _data: True,
            ros_context=object(),
        )

    def test_factory_registers_three_ids_and_lifecycle(self) -> None:
        with patch(
                "kwr57_ros.bridge_handler.KWR57DeviceNode",
                _FakeDeviceNode):
            registration = create_frame_handler(self.context, {
                "channel_id": 1,
                "node_name": "ft_test",
                "data_base_id": 0x18,
                "cmd_id": 0x11,
                "topic": "/ft_test/wrench_raw",
            })

        self.assertEqual(
            [(key.channel_id, key.can_id) for key in registration.keys],
            [(1, 0x18), (1, 0x19), (1, 0x1A)],
        )
        node = registration.auxiliary_nodes[0]
        self.assertFalse(node.kwargs["use_global_arguments"])
        self.assertTrue(node.kwargs["direct_rx"])
        self.assertTrue(callable(node.kwargs["direct_tx"]))
        self.assertTrue(node.kwargs["defer_autostart"])

        message = SimpleNamespace(
            arbitration_id=0x18,
            data=b"\x00" * 8,
            is_extended_id=False,
            is_remote_frame=False,
            is_error_frame=False,
            dlc=8,
        )
        self.assertIs(
            registration.callback(1, message), FrameDisposition.CONSUME)
        message.is_extended_id = True
        self.assertIs(
            registration.callback(1, message), FrameDisposition.FORWARD)

        registration.start()
        registration.stop()
        self.assertEqual((node.activated, node.stopped), (1, 1))

    def test_rejects_unknown_config_and_conflicting_ids(self) -> None:
        with self.assertRaisesRegex(ValueError, "unknown KWR57"):
            create_frame_handler(self.context, {"unknown": 1})
        with self.assertRaisesRegex(ValueError, "conflicts"):
            create_frame_handler(self.context, {
                "cmd_id": 0x16,
                "data_base_id": 0x15,
            })
        with self.assertRaisesRegex(ValueError, "sample_rate_hz"):
            build_frame_handler_spec({"sample_rate_hz": 333})
        with self.assertRaisesRegex(ValueError, "period_ms"):
            build_frame_handler_spec({"period_ms": 65536})

    def test_builds_validated_handler_spec(self) -> None:
        spec = json.loads(build_frame_handler_spec({
            "channel_id": 1,
            "node_name": "ft_right",
            "cmd_id": 0x11,
            "data_base_id": 0x18,
            "topic": "/ft_right/wrench_raw",
        }))

        self.assertEqual(
            spec["factory"],
            "kwr57_ros.bridge_handler:create_frame_handler")
        self.assertEqual(spec["config"]["channel_id"], 1)
        self.assertEqual(spec["config"]["sample_rate_hz"], 1000)
        with self.assertRaisesRegex(ValueError, "unknown KWR57"):
            build_frame_handler_spec({"unknown": True})


if __name__ == "__main__":
    unittest.main()