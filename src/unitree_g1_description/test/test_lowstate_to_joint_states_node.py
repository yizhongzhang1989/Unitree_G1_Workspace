from types import MethodType, SimpleNamespace
from typing import cast

from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from unitree_hg.msg import LowState

from unitree_g1_description.joint_state_mapping import G1_JOINT_NAMES
from unitree_g1_description.lowstate_to_joint_states_node import (
    LowStateToJointStates,
)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Clock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: Time(sec=123, nanosec=456))


def _motor_states():
    return [
        SimpleNamespace(q=float(index), dq=0.1, tau_est=0.2)
        for index in range(len(G1_JOINT_NAMES))
    ]


def test_inputs_update_cache_and_timer_publishes_latest_unified_state():
    publisher = _Publisher()
    state_size = len(G1_JOINT_NAMES) + 2
    node = SimpleNamespace(
        _positions=[None] * state_size,
        _velocities=[None] * state_size,
        _efforts=[None] * state_size,
        _require_pr_mode=True,
        _frame_id="",
        _publisher=publisher,
        _warn_invalid=lambda reason: None,
        get_clock=lambda: _Clock(),
    )
    node._update_cache = MethodType(
        LowStateToJointStates._update_cache, node)
    typed_node = cast(LowStateToJointStates, node)

    left = JointState()
    left.position = [0.3]
    left.velocity = [0.4]
    left.effort = [0.5]
    right = JointState()
    right.position = [0.6]
    right.velocity = [0.7]
    right.effort = [0.8]

    LowStateToJointStates._on_gripper_state(typed_node, "left_", left)
    LowStateToJointStates._on_gripper_state(typed_node, "right_", right)
    assert publisher.messages == []

    lowstate = cast(
        LowState, SimpleNamespace(mode_pr=0, motor_state=_motor_states()))
    LowStateToJointStates._on_lowstate(typed_node, lowstate)
    assert publisher.messages == []
    LowStateToJointStates._publish_cached_state(typed_node)

    assert len(publisher.messages) == 1
    message = publisher.messages[0]
    assert len(message.name) == len(G1_JOINT_NAMES) + 2
    assert message.name[-2:] == [
        "left_eccentric_joint", "right_eccentric_joint"]
    assert list(message.position[-2:]) == [0.3, 0.6]
    assert list(message.velocity[-2:]) == [0.4, 0.7]
    assert list(message.effort[-2:]) == [0.5, 0.8]
    assert message.header.stamp == Time(sec=123, nanosec=456)

    left.position = [0.9]
    LowStateToJointStates._on_gripper_state(typed_node, "left_", left)
    LowStateToJointStates._publish_cached_state(typed_node)

    latest = publisher.messages[-1]
    assert list(latest.position[:29]) == list(message.position[:29])
    assert list(latest.position[-2:]) == [0.9, 0.6]


def test_timer_does_not_publish_before_any_input_arrives():
    publisher = _Publisher()
    state_size = len(G1_JOINT_NAMES) + 2
    node = SimpleNamespace(
        _positions=[None] * state_size,
        _velocities=[None] * state_size,
        _efforts=[None] * state_size,
        _frame_id="",
        _publisher=publisher,
        get_clock=lambda: _Clock(),
    )

    LowStateToJointStates._publish_cached_state(
        cast(LowStateToJointStates, node))

    assert publisher.messages == []