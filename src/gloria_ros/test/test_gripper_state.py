import threading
import time
import unittest
from unittest.mock import Mock

from gloria_ros.gripper_node import GloriaGripperNode, _CTRL_DISABLE


class GripperStateTest(unittest.TestCase):
    def setUp(self) -> None:
        # 绕过 ROS Node 初始化，只测试不依赖 ROS 上下文的并发状态辅助方法
        self.node = object.__new__(GloriaGripperNode)
        self.node._lock = threading.RLock()
        self.node._feedback_event = threading.Event()
        self.node._response_timeout = 0.02
        self.node._last_feedback_monotonic = 0.0

    def test_feedback_before_request_does_not_confirm(self) -> None:
        self.node._last_feedback_monotonic = 1.0
        self.node._feedback_event.set()
        self.assertFalse(self.node._wait_for_feedback_after(2.0))

    def test_feedback_after_request_wakes_waiter(self) -> None:
        wait_after = time.monotonic()
        result = []
        waiter = threading.Thread(
            target=lambda: result.append(
                self.node._wait_for_feedback_after(wait_after)))
        waiter.start()
        with self.node._lock:
            self.node._last_feedback_monotonic = time.monotonic()
        self.node._feedback_event.set()
        waiter.join(timeout=1.0)
        self.assertFalse(waiter.is_alive())
        self.assertEqual(result, [True])

    def test_disable_updates_state_before_sending_frame(self) -> None:
        self.node._disable_generation = 2
        self.node._enabled_requested = True
        observed = []
        self.node._send_ctrl = Mock(side_effect=lambda command: observed.append(
            (command, self.node._disable_generation,
             self.node._enabled_requested)))

        self.node._request_disable()

        self.assertEqual(observed, [(_CTRL_DISABLE, 3, False)])


if __name__ == "__main__":
    unittest.main()