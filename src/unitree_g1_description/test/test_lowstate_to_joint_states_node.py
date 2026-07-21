from pathlib import Path
from types import MethodType, SimpleNamespace
from typing import cast

from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
from unitree_hg.msg import LowState

from unitree_g1_description.joint_state_mapping import (
    G1_JOINT_NAMES,
    load_gripper_model_spec,
)
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


def _state_node(publisher):
    model_path = Path(__file__).parents[1] / "model" / "final.urdf"
    gripper_models = {
        prefix: load_gripper_model_spec(model_path, prefix)
        for prefix in ("left_", "right_")
    }
    joint_state_order = (
        *G1_JOINT_NAMES,
        *(name for prefix in ("left_", "right_")
          for name in gripper_models[prefix].joint_names),
    )
    return SimpleNamespace(
        _positions=[None] * len(joint_state_order),
        _velocities=[None] * len(joint_state_order),
        _efforts=[None] * len(joint_state_order),
        _gripper_models=gripper_models,
        _gripper_state_offsets={
            prefix: joint_state_order.index(gripper_models[prefix].source_name)
            for prefix in ("left_", "right_")
        },
        _joint_state_order=joint_state_order,
        _require_pr_mode=True,
        _frame_id="",
        _publisher=publisher,
        _warn_invalid=lambda reason: None,
        get_clock=lambda: _Clock(),
    )


def test_inputs_update_cache_and_timer_publishes_latest_unified_state():
    publisher = _Publisher()
    node = _state_node(publisher)
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
    positions = dict(zip(message.name, message.position))
    velocities = dict(zip(message.name, message.velocity))
    efforts = dict(zip(message.name, message.effort))
    assert len(message.name) == len(G1_JOINT_NAMES) + 2 * 33
    assert positions["left_eccentric_joint"] == 0.3
    assert positions["right_eccentric_joint"] == 0.6
    assert velocities["left_eccentric_joint"] == 0.4
    assert velocities["right_eccentric_joint"] == 0.7
    assert efforts["left_eccentric_joint"] == 0.5
    assert efforts["right_eccentric_joint"] == 0.8
    assert efforts["left_internal_left_slider_spline_00_joint"] == 0.0
    assert message.header.stamp == Time(sec=123, nanosec=456)

    left.position = [0.9]
    LowStateToJointStates._on_gripper_state(typed_node, "left_", left)
    LowStateToJointStates._publish_cached_state(typed_node)

    latest = publisher.messages[-1]
    assert list(latest.position[:29]) == list(message.position[:29])
    latest_positions = dict(zip(latest.name, latest.position))
    assert latest_positions["left_eccentric_joint"] == 0.9
    assert latest_positions["right_eccentric_joint"] == 0.6


def test_timer_does_not_publish_before_any_input_arrives():
    publisher = _Publisher()
    node = _state_node(publisher)

    LowStateToJointStates._publish_cached_state(
        cast(LowStateToJointStates, node))

    assert publisher.messages == []