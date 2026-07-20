import threading
from types import MethodType, SimpleNamespace
from unittest.mock import patch

import pytest
from controller_manager_msgs.srv import ListControllers, SwitchController

from unitree_g1_description.joint_state_mapping import G1_JOINT_NAMES
from unitree_g1_description.mit_command import low_cmd_crc
from unitree_g1_description.mit_position_controller_node import (
    CONTROLLED_JOINT_NAMES,
    MitPositionController,
)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_distributes_targets_to_g1_lowcmd_with_configured_gains():
    target = tuple(index * 0.01 for index in range(
        len(CONTROLLED_JOINT_NAMES)))
    publisher = _Publisher()
    node = SimpleNamespace(
        _stiffness=tuple(index + 10.0 for index in range(29)),
        _damping=tuple(index + 1.0 for index in range(29)),
        _lowcmd_publisher=publisher,
        _g1_message_key=None,
        _g1_message=None,
        _command_publish_lock=threading.Lock(),
        _lock=threading.Lock(),
        _active=True,
        _target_positions=target,
        _mode_machine=5,
    )

    MitPositionController._update_g1_message(node, target, 5)
    MitPositionController._publish_g1(node)

    assert len(publisher.messages) == 1
    message = publisher.messages[0]
    assert message.mode_pr == 0
    assert message.mode_machine == 5
    for index, position in enumerate(target[:29]):
        command = message.motor_cmd[index]
        assert command.mode == 1
        assert command.q == pytest.approx(position)
        assert command.dq == 0.0
        assert command.tau == 0.0
        assert command.kp == pytest.approx(index + 10.0)
        assert command.kd == pytest.approx(index + 1.0)
    assert all(command.mode == 0 for command in message.motor_cmd[29:])
    assert message.crc == low_cmd_crc(message)


def test_distributes_last_two_targets_to_gloria_at_sdk_kd_limit():
    target = tuple(index * 0.01 for index in range(
        len(CONTROLLED_JOINT_NAMES)))
    publishers = (_Publisher(), _Publisher())
    node = SimpleNamespace(
        _ready_target=lambda: (target, 5),
        _gripper_publishers=publishers,
        _gripper_kp=10.0,
        _gripper_kd=5.0,
        _command_publish_lock=threading.Lock(),
        _lock=threading.Lock(),
        _active=True,
        _update_g1_message=lambda target, mode_machine: None,
    )

    MitPositionController._publish_grippers(node)

    assert [publisher.messages[0].q for publisher in publishers] == pytest.approx(
        target[-2:])
    for publisher in publishers:
        message = publisher.messages[0]
        assert message.dq == 0.0
        assert message.kp == 10.0
        assert message.kd == 5.0
        assert message.tau == 0.0


def test_reports_dashboard_controller_in_exact_command_order():
    node = SimpleNamespace(
        _controller_name="whole_body_controller",
        _active=False,
        _lock=threading.Lock(),
    )
    response = ListControllers.Response()

    MitPositionController._list_controllers(
        node, ListControllers.Request(), response)

    assert len(response.controller) == 1
    controller = response.controller[0]
    assert controller.name == "whole_body_controller"
    assert controller.state == "inactive"
    assert controller.type == (
        "forward_command_controller/ForwardCommandController")
    assert controller.claimed_interfaces == [
        f"{name}/position" for name in CONTROLLED_JOINT_NAMES]


def test_dashboard_timeout_holds_latest_feedback_instead_of_deactivating():
    feedback = {
        name: index * 0.02
        for index, name in enumerate(CONTROLLED_JOINT_NAMES)
    }
    warnings = []
    node = SimpleNamespace(
        _active=True,
        _lock=threading.Lock(),
        _state_error_locked=lambda now: "",
        _target_positions=tuple(0.0 for _ in CONTROLLED_JOINT_NAMES),
        _last_command_at=9.0,
        _activated_at=9.0,
        _command_timeout=0.25,
        _joint_positions=feedback,
        _mode_machine=5,
        _warn_locked=lambda key, message: warnings.append((key, message)),
    )

    with patch(
            "unitree_g1_description.mit_position_controller_node.time.monotonic",
            return_value=10.0):
        ready = MitPositionController._ready_target(node)

    assert ready == (
        tuple(feedback[name] for name in CONTROLLED_JOINT_NAMES), 5)
    assert node._active
    assert node._last_command_at == 10.0
    assert warnings == [(
        "command_timeout",
        "dashboard command timed out; holding latest feedback pose",
    )]


def test_rejects_initial_target_far_from_feedback():
    feedback = {name: 0.0 for name in CONTROLLED_JOINT_NAMES}
    limits = {name: (-3.0, 3.0) for name in CONTROLLED_JOINT_NAMES}
    node = SimpleNamespace(
        _state_error_locked=lambda now: "",
        _position_limits=limits,
        _target_positions=None,
        _joint_positions=feedback,
        _max_initial_error=0.2,
        _max_command_step=0.1,
    )
    target = [0.0] * len(CONTROLLED_JOINT_NAMES)
    target[len(G1_JOINT_NAMES)] = 0.21

    error = MitPositionController._command_error_locked(node, target, 1.0)

    assert "first target differs from feedback" in error


def _switch_node(restore_result):
    target = tuple(index * 0.01 for index in range(
        len(CONTROLLED_JOINT_NAMES)))
    logger = SimpleNamespace(error=lambda message: None, warning=lambda message: None)
    node = SimpleNamespace(
        _controller_name="whole_body_controller",
        _switch_lock=threading.Lock(),
        _lock=threading.Lock(),
        _command_publish_lock=threading.Lock(),
        _active=True,
        _target_positions=target,
        _last_command_at=1.0,
        _activated_at=1.0,
        _restore_previous_mode=lambda: restore_result,
        get_logger=lambda: logger,
    )
    node._wait_for_command_publishers_idle = MethodType(
        MitPositionController._wait_for_command_publishers_idle, node)
    node._deactivate_locked = MethodType(
        MitPositionController._deactivate_locked, node)
    return node, target


def _stop_request():
    request = SwitchController.Request()
    request.stop_controllers = ["whole_body_controller"]
    request.strictness = SwitchController.Request.BEST_EFFORT
    return request


def test_disengage_restoration_failure_keeps_low_level_output_stopped():
    node, _target = _switch_node((False, "SelectMode failed"))

    response = MitPositionController._switch_controller(
        node, _stop_request(), SwitchController.Response())

    assert not response.ok
    assert not node._active
    assert node._target_positions is None
    assert node._last_command_at == 0.0


def test_disengage_success_stops_hold_stream():
    node, _target = _switch_node((True, ""))

    response = MitPositionController._switch_controller(
        node, _stop_request(), SwitchController.Response())

    assert response.ok
    assert not node._active
    assert node._target_positions is None


def test_restore_accepts_selected_mode_after_select_response_timeout():
    checks = iter([
        (True, "", ""),
        (True, "ai", ""),
    ])
    selects = []
    motion_switcher = SimpleNamespace(
        select_mode=lambda name, timeout: (
            selects.append((name, timeout))
            or (False, f"motion switcher SelectMode({name!r}) timed out")),
        check_mode=lambda timeout: next(checks),
    )
    node = SimpleNamespace(
        _manage_motion_mode=True,
        _restore_motion_mode=True,
        _previous_motion_mode="ai",
        _fallback_motion_mode="ai",
        _motion_switch_timeout=1.0,
        _motion_select_timeout=3.0,
        _motion_switcher=motion_switcher,
    )

    result = MitPositionController._restore_previous_mode(node)

    assert result == (True, "")
    assert len(selects) == 1


def test_restore_retries_transient_select_failures_until_mode_is_active():
    checks = iter([
        (True, "", ""),
        (True, "", ""),
        (True, "ai", ""),
    ])
    selects = iter([
        (False, "motion switcher SelectMode('ai') failed: status=7002"),
        (False, "motion switcher SelectMode('ai') timed out"),
    ])
    node = SimpleNamespace(
        _manage_motion_mode=True,
        _restore_motion_mode=True,
        _previous_motion_mode="ai",
        _fallback_motion_mode="ai",
        _motion_switch_timeout=1.0,
        _motion_select_timeout=3.0,
        _motion_switcher=SimpleNamespace(
            select_mode=lambda name, timeout: next(selects),
            check_mode=lambda timeout: next(checks),
        ),
    )

    result = MitPositionController._restore_previous_mode(node)

    assert result == (True, "")


def test_shutdown_stops_hold_stream_before_restoring_motion_mode():
    active_during_restore = []
    logger = SimpleNamespace(error=lambda message: None, warning=lambda message: None)
    node = SimpleNamespace(
        _controller_name="whole_body_controller",
        _switch_lock=threading.Lock(),
        _lock=threading.Lock(),
        _command_publish_lock=threading.Lock(),
        _active=True,
        _target_positions=tuple(0.0 for _ in CONTROLLED_JOINT_NAMES),
        _last_command_at=1.0,
        _restore_previous_mode=lambda: (
            active_during_restore.append(node._active) or (True, "")),
        get_logger=lambda: logger,
    )
    node._deactivate_locked = MethodType(
        MitPositionController._deactivate_locked, node)
    node._wait_for_command_publishers_idle = MethodType(
        MitPositionController._wait_for_command_publishers_idle, node)

    MitPositionController.shutdown_control(node)

    assert active_during_restore == [False]
    assert not node._active
    assert node._target_positions is None