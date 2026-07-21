import unittest
from typing import List, cast

from robot_bringup.end_effectors.topology import (
    CanBus,
    GloriaDevice,
    Kwr57Device,
    build_bridge_parameters,
)


def _sensor(name: str, bus: CanBus, cmd_id: int,
            data_base_id: int) -> Kwr57Device:
    return Kwr57Device(
        name=name,
        bus=bus,
        cmd_id=cmd_id,
        data_base_id=data_base_id,
        wrench_topic=f"/{name}/wrench_raw",
        frame_id=f"{name}_link",
    )


def _gripper(name: str, bus: CanBus, command_id: int, feedback_id: int,
             rx_topic: str) -> GloriaDevice:
    return GloriaDevice(
        name=name,
        bus=bus,
        command_id=command_id,
        feedback_id=feedback_id,
        rx_topic=rx_topic,
        joint_name=name,
    )


class BuildBridgeParametersTest(unittest.TestCase):
    def test_kwr57_inventory_uses_handlers_without_frame_routes(self) -> None:
        can0 = CanBus("can0", 0)
        can1 = CanBus("can1", 1)
        devices = [
            _sensor("left", can0, 0x10, 0x15),
            _sensor("right", can1, 0x10, 0x15),
        ]

        parameters = build_bridge_parameters([can0, can1], devices)

        self.assertEqual(parameters["channel_ids"], [0, 1])
        self.assertEqual(parameters["bus_names"], ["can0", "can1"])
        self.assertEqual(parameters["rx_routes"], [""])

    def test_kwr57_handler_config_contains_runtime_parameters(self) -> None:
        device = _sensor("left", CanBus("can0", 0), 0x10, 0x15)

        self.assertEqual(device.handler_config["channel_id"], 0)
        self.assertEqual(device.handler_config["node_name"], "left")
        self.assertEqual(device.handler_config["sample_rate_hz"], 1000)
        self.assertEqual(device.handler_config["period_ms"], 1)
        self.assertNotIn("rx_topic", device.handler_config)
        self.assertNotIn("tx_topic", device.handler_config)

    def test_uses_typed_placeholder_for_empty_routes(self) -> None:
        parameters = build_bridge_parameters([CanBus("can0", 0)], [])
        self.assertEqual(parameters["rx_routes"], [""])

    def test_builds_gripper_routes_including_shared_zero(self) -> None:
        can0 = CanBus("can0", 0)
        grippers = [
            _gripper("left", can0, 0x01, 0x101, "/can0/grip_left/rx"),
            _gripper("right", can0, 0x02, 0x102, "/can0/grip_right/rx"),
        ]

        parameters = build_bridge_parameters([can0], [], grippers)

        self.assertEqual(parameters["rx_routes"], [
            "0:0x101:/can0/grip_left/rx",
            "0:0x1:/can0/grip_left/rx",
            "0:0x0:/can0/grip_left/rx",
            "0:0x102:/can0/grip_right/rx",
            "0:0x2:/can0/grip_right/rx",
            "0:0x0:/can0/grip_right/rx",
        ])

    def test_builds_complete_single_bus_inventory(self) -> None:
        can0 = CanBus("can0", 0)
        sensors = [
            _sensor("ft_left", can0, 0x10, 0x15),
            _sensor("ft_right", can0, 0x11, 0x18),
        ]
        grippers = [
            _gripper("grip_left", can0, 0x01, 0x101, "/can0/grip_left/rx"),
            _gripper("grip_right", can0, 0x02, 0x102, "/can0/grip_right/rx"),
        ]

        routes = cast(
            List[str],
            build_bridge_parameters([can0], sensors, grippers)["rx_routes"],
        )

        self.assertEqual(len(routes), 6)
        self.assertEqual(routes.count("0:0x0:/can0/grip_left/rx"), 1)
        self.assertEqual(routes.count("0:0x0:/can0/grip_right/rx"), 1)

    def test_rejects_overlapping_data_ids_on_same_channel(self) -> None:
        can0 = CanBus("can0", 0)
        devices = [
            _sensor("left", can0, 0x10, 0x15),
            _sensor("right", can0, 0x11, 0x17),
        ]
        with self.assertRaisesRegex(ValueError, "共用数据 ID"):
            build_bridge_parameters([can0], devices)

    def test_rejects_duplicate_command_id_on_same_channel(self) -> None:
        can0 = CanBus("can0", 0)
        devices = [
            _sensor("left", can0, 0x10, 0x15),
            _sensor("right", can0, 0x10, 0x18),
        ]
        with self.assertRaisesRegex(ValueError, "共用 cmd_id"):
            build_bridge_parameters([can0], devices)

    def test_rejects_command_and_data_id_collision(self) -> None:
        can0 = CanBus("can0", 0)
        devices = [
            _sensor("left", can0, 0x10, 0x15),
            _sensor("right", can0, 0x16, 0x18),
        ]
        with self.assertRaisesRegex(ValueError, "数据 ID 冲突"):
            build_bridge_parameters([can0], devices)

    def test_allows_same_ids_on_different_channels(self) -> None:
        can0 = CanBus("can0", 0)
        can1 = CanBus("can1", 1)
        devices = [
            _sensor("left", can0, 0x10, 0x15),
            _sensor("right", can1, 0x10, 0x15),
        ]
        build_bridge_parameters([can0, can1], devices)

    def test_rejects_duplicate_wrench_topic(self) -> None:
        can0 = CanBus("can0", 0)
        devices = [
            _sensor("left", can0, 0x10, 0x15),
            Kwr57Device(
                name="right", bus=can0, cmd_id=0x11, data_base_id=0x18,
                wrench_topic="/left/wrench_raw", frame_id="right_link"),
        ]
        with self.assertRaisesRegex(ValueError, "Wrench 话题重复"):
            build_bridge_parameters([can0], devices)

    def test_rejects_wrench_and_frame_topic_collision(self) -> None:
        can0 = CanBus("can0", 0)
        sensor = Kwr57Device(
            name="ft", bus=can0, cmd_id=0x10, data_base_id=0x15,
            wrench_topic="/can0/grip/rx", frame_id="ft_link")
        gripper = _gripper(
            "grip", can0, 0x01, 0x101, "/can0/grip/rx")
        with self.assertRaisesRegex(ValueError, "话题冲突"):
            build_bridge_parameters([can0], [sensor], [gripper])

    def test_rejects_unconfigured_bus(self) -> None:
        configured_bus = CanBus("can0", 0)
        wrong_bus = CanBus("can0", 1)
        device = _sensor("left", wrong_bus, 0x10, 0x15)
        with self.assertRaisesRegex(ValueError, "未配置的总线"):
            build_bridge_parameters([configured_bus], [device])

    def test_rejects_gripper_id_colliding_with_kwr57_data(self) -> None:
        can0 = CanBus("can0", 0)
        sensor = _sensor("ft", can0, 0x10, 0x15)
        gripper = _gripper("grip", can0, 0x01, 0x16, "/can0/grip/rx")
        with self.assertRaisesRegex(ValueError, "KWR57.*数据 ID 冲突"):
            build_bridge_parameters([can0], [sensor], [gripper])

    def test_rejects_gripper_active_id_collision(self) -> None:
        can0 = CanBus("can0", 0)
        grippers = [
            _gripper("left", can0, 0x01, 0x101, "/can0/left/rx"),
            _gripper("right", can0, 0x02, 0x101, "/can0/right/rx"),
        ]
        with self.assertRaisesRegex(ValueError, "共用活动 ID"):
            build_bridge_parameters([can0], [], grippers)

    def test_rejects_gripper_payload_id_alias(self) -> None:
        can0 = CanBus("can0", 0)
        grippers = [
            _gripper("left", can0, 0x01, 0x101, "/can0/left/rx"),
            _gripper("right", can0, 0x11, 0x111, "/can0/right/rx"),
        ]
        with self.assertRaisesRegex(ValueError, r"Data\[0\] 设备号相同"):
            build_bridge_parameters([can0], [], grippers)

    def test_rejects_zero_feedback_id(self) -> None:
        can0 = CanBus("can0", 0)
        with self.assertRaisesRegex(ValueError, "非零 feedback_id"):
            _gripper("grip", can0, 0x01, 0x00, "/can0/grip/rx")

    def test_rejects_zero_payload_device_id(self) -> None:
        can0 = CanBus("can0", 0)
        with self.assertRaisesRegex(ValueError, "低 4 位不能为 0"):
            _gripper("grip", can0, 0x20, 0x120, "/can0/grip/rx")

    def test_rejects_gloria_active_id_using_broadcast_id(self) -> None:
        can0 = CanBus("can0", 0)
        with self.assertRaisesRegex(ValueError, "固定请求 ID 0x7FF"):
            _gripper("grip", can0, 0x01, 0x7FF, "/can0/grip/rx")

    def test_rejects_kwr57_using_gloria_broadcast_id(self) -> None:
        can0 = CanBus("can0", 0)
        sensor = _sensor("ft", can0, 0x10, 0x7FD)
        gripper = _gripper("grip", can0, 0x01, 0x101, "/can0/grip/rx")
        with self.assertRaisesRegex(ValueError, "固定请求 ID 0x7FF"):
            build_bridge_parameters([can0], [sensor], [gripper])

    def test_allows_same_gripper_ids_on_different_channels(self) -> None:
        can0 = CanBus("can0", 0)
        can1 = CanBus("can1", 1)
        grippers = [
            _gripper("left", can0, 0x01, 0x101, "/can0/grip/rx"),
            _gripper("right", can1, 0x01, 0x101, "/can1/grip/rx"),
        ]

        routes = cast(
            List[str],
            build_bridge_parameters(
                [can0, can1], [], grippers)["rx_routes"],
        )

        self.assertIn("0:0x0:/can0/grip/rx", routes)
        self.assertIn("1:0x0:/can1/grip/rx", routes)

    def test_rejects_shared_zero_colliding_with_kwr57_data(self) -> None:
        can0 = CanBus("can0", 0)
        sensor = _sensor("ft", can0, 0x10, 0x00)
        gripper = _gripper("grip", can0, 0x03, 0x103, "/can0/grip/rx")
        with self.assertRaisesRegex(ValueError, "共享反馈 ID"):
            build_bridge_parameters([can0], [sensor], [gripper])


if __name__ == "__main__":
    unittest.main()