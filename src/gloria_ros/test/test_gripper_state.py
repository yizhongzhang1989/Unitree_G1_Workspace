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

    def test_poll_requests_state_while_disabled(self) -> None:
        self.node._operation_lock = threading.RLock()
        self.node._shutting_down = False
        self.node._enabled_requested = False
        self.node._disable_on_feedback_timeout = True
        self.node._request_state = Mock()

        self.node._poll_state()

        self.node._request_state.assert_called_once_with()

    def test_poll_disables_enabled_gripper_on_feedback_timeout(self) -> None:
        self.node._operation_lock = threading.RLock()
        self.node._shutting_down = False
        self.node._enabled_requested = True
        self.node._disable_on_feedback_timeout = True
        self.node._disable_generation = 0
        self.node._feedback_timeout = 0.5
        self.node._send_ctrl = Mock()
        self.node._request_state = Mock()
        self.node.get_logger = Mock()

        self.node._poll_state()

        self.node._send_ctrl.assert_called_once_with(_CTRL_DISABLE)
        self.node._request_state.assert_not_called()
        self.assertFalse(self.node._enabled_requested)

    def test_auto_enable_waits_for_fresh_feedback(self) -> None:
        self.node._shutting_down = False
        self.node._auto_enable_pending = True
        self.node._disable_generation = 4
        self.node._feedback_timeout = 0.5
        self.node._enable = Mock()

        self.node._auto_enable()

        self.node._enable.assert_not_called()

    def test_auto_enable_retries_until_success_then_cancels(self) -> None:
        self.node._shutting_down = False
        self.node._auto_enable_pending = True
        self.node._disable_generation = 4
        self.node._feedback_timeout = 0.5
        self.node._last_feedback_monotonic = time.monotonic()
        self.node._enable = Mock(side_effect=[
            (False, "control mode was not confirmed"),
            (True, "enabled and feedback confirmed"),
        ])
        self.node._enable_timer = Mock()
        self.node.get_logger = Mock()

        self.node._auto_enable()
        self.node._auto_enable()

        self.assertEqual(self.node._enable.call_count, 2)
        self.node._enable.assert_called_with(expected_generation=4)
        self.node._enable_timer.cancel.assert_called_once_with()
        self.assertFalse(self.node._auto_enable_pending)

    def test_explicit_disable_cancels_pending_auto_enable(self) -> None:
        self.node._auto_enable_pending = True
        self.node._enable_timer = Mock()
        self.node._disable_generation = 0
        self.node._enabled_requested = False
        self.node._send_ctrl = Mock()
        response = Mock()

        self.node._srv_disable_cb(None, response)

        self.node._enable_timer.cancel.assert_called_once_with()
        self.assertFalse(self.node._auto_enable_pending)
        self.node._send_ctrl.assert_called_once_with(_CTRL_DISABLE)


if __name__ == "__main__":
    unittest.main()