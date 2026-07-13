import unittest

from gloria_ros.can_feedback import (
    is_register_reply,
    register_reply_belongs_to_device,
    state_feedback_belongs_to_device,
)


# 纯协议测试不依赖 ROS executor，用于快速验证共享总线分流边界
class CanFeedbackRoutingTest(unittest.TestCase):
    def test_state_matches_low_nibble_with_enabled_status(self) -> None:
        data = bytes([0x11, 0, 0, 0, 0, 0, 0, 0])
        self.assertTrue(state_feedback_belongs_to_device(
            0x101, data, command_id=0x01, feedback_id=0x101))

    def test_state_matches_low_nibble_with_fault_status(self) -> None:
        data = bytes([0xA1, 0, 0, 0, 0, 0, 0, 0])
        self.assertTrue(state_feedback_belongs_to_device(
            0x00, data, command_id=0x01, feedback_id=0x101))

    def test_state_rejects_other_payload_device(self) -> None:
        data = bytes([0x12, 0, 0, 0, 0, 0, 0, 0])
        self.assertFalse(state_feedback_belongs_to_device(
            0x00, data, command_id=0x01, feedback_id=0x101))

    def test_state_rejects_unconfigured_can_id(self) -> None:
        data = bytes([0x01, 0, 0, 0, 0, 0, 0, 0])
        self.assertFalse(state_feedback_belongs_to_device(
            0x102, data, command_id=0x01, feedback_id=0x101))

    def test_register_reply_shape(self) -> None:
        self.assertTrue(is_register_reply(bytes([0, 0, 0x33, 10, 0, 0, 0, 0])))
        self.assertTrue(is_register_reply(bytes([0, 0, 0x55, 10, 0, 0, 0, 0])))
        self.assertTrue(is_register_reply(
            bytes([1, 0, 0x55, 10, 1, 0, 0, 0]), command_id=0x01))
        self.assertFalse(is_register_reply(
            bytes([2, 0, 0x55, 10, 1, 0, 0, 0]), command_id=0x01))
        self.assertFalse(is_register_reply(bytes([0, 0, 0xCC, 0, 0, 0, 0, 0])))

    def test_state_with_register_opcode_in_position_byte_is_not_reply(self) -> None:
        self.assertFalse(is_register_reply(
            bytes([0x11, 0x80, 0x33, 10, 0, 0, 0, 0])))
        self.assertFalse(is_register_reply(
            bytes([0xA1, 0x80, 0x55, 21, 0, 0, 0, 0])))

    def test_register_reply_requires_unique_nonzero_can_id(self) -> None:
        self.assertTrue(register_reply_belongs_to_device(0x101, 0x01, 0x101))
        self.assertTrue(register_reply_belongs_to_device(0x01, 0x01, 0x101))
        self.assertFalse(register_reply_belongs_to_device(0x00, 0x01, 0x101))
        self.assertFalse(register_reply_belongs_to_device(0x102, 0x01, 0x101))


if __name__ == "__main__":
    unittest.main()