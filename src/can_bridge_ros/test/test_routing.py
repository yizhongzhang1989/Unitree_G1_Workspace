import unittest

from can_bridge_ros.routing import parse_rx_routes


class ParseRxRoutesTest(unittest.TestCase):
    def test_parses_single_destination(self) -> None:
        routes = parse_rx_routes(["0:0x15:/can0/ft/rx"], [0])
        self.assertEqual(routes, {(0, 0x15): ("/can0/ft/rx",)})

    def test_fans_out_same_id_to_distinct_topics(self) -> None:
        routes = parse_rx_routes([
            "0:0x0:/can0/grip_left/rx",
            "0:0x0:/can0/grip_right/rx",
        ], [0])
        self.assertEqual(routes, {
            (0, 0): ("/can0/grip_left/rx", "/can0/grip_right/rx"),
        })

    def test_rejects_exact_duplicate(self) -> None:
        with self.assertRaisesRegex(ValueError, "路由重复"):
            parse_rx_routes([
                "0:0x15:/can0/ft/rx",
                "0:0x15:/can0/ft/rx",
            ], [0])

    def test_rejects_unknown_channel(self) -> None:
        with self.assertRaisesRegex(ValueError, "不在 channel_ids"):
            parse_rx_routes(["1:0x15:/can1/ft/rx"], [0])

    def test_rejects_extended_id(self) -> None:
        with self.assertRaisesRegex(ValueError, "CAN ID 超出范围"):
            parse_rx_routes(["0:0x800:/can0/ft/rx"], [0])

    def test_rejects_empty_topic(self) -> None:
        with self.assertRaisesRegex(ValueError, "话题不能为空"):
            parse_rx_routes(["0:0x15:"], [0])


if __name__ == "__main__":
    unittest.main()