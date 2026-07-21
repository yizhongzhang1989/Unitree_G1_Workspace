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


class _Future:
    def __init__(self, response=None, error=None):
        self._response = response
        self._error = error

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        if self._error is not None:
            raise self._error
        return self._response


class _ServiceClient:
    def __init__(self, response=None, available=True, error=None):
        self._response = response
        self._available = available
        self._error = error

    def wait_for_service(self, timeout_sec):
        if self._error is not None:
            raise self._error
        return self._available

    def call_async(self, request):
        return _Future(self._response)


class _ReactivateOnEnter:
    def __init__(self, node):
        self._node = node

    def __enter__(self):
        self._node._active = True

    def __exit__(self, exc_type, exc_value, traceback):
        return False


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


def test_old_state_fault_cleanup_does_not_disable_new_active_session():
    cleanups = []
    node = SimpleNamespace(
        _active=True,
        _lock=threading.Lock(),
        _state_error_locked=lambda now: "JointState is stale",
        _deactivate_locked=lambda reason: setattr(node, "_active", False),
        _wait_for_command_publishers_idle=lambda: None,
        _disable_grippers_and_restore_mode=lambda restore_mode: (
            cleanups.append(restore_mode) or (True, "")),
        get_logger=lambda: SimpleNamespace(error=lambda message: None),
    )
    node._switch_lock = _ReactivateOnEnter(node)

    ready = MitPositionController._ready_target(node)

    assert ready is None
    assert node._active
    assert cleanups == []


def test_state_error_rejects_control_after_g1_publisher_failure():
    node = SimpleNamespace(
        _g1_publisher_failed=True,
        _lowstate_at=1.0,
        _joint_state_at=1.0,
        _state_timeout=1.0,
        _require_pr_mode=True,
        _mode_pr=0,
        _joint_positions={name: 0.0 for name in CONTROLLED_JOINT_NAMES},
    )

    error = MitPositionController._state_error_locked(node, 1.1)

    assert error == "G1 command publisher has stopped"


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
        _disable_grippers_and_restore_mode=lambda restore_mode: (
            restore_result if restore_mode else (True, "")),
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


def _start_request():
    request = SwitchController.Request()
    request.start_controllers = ["whole_body_controller"]
    request.strictness = SwitchController.Request.BEST_EFFORT
    return request


def test_rejects_start_and_stop_for_same_controller_in_one_request():
    node, _events = _start_node()
    request = _start_request()
    request.stop_controllers = ["whole_body_controller"]

    response = MitPositionController._switch_controller(
        node, request, SwitchController.Response())

    assert not response.ok
    assert not node._active


def _start_node(enable_result=(True, "")):
    events = []
    feedback = {
        name: index * 0.01
        for index, name in enumerate(CONTROLLED_JOINT_NAMES)
    }
    logger = SimpleNamespace(
        error=lambda message: events.append(("error", message)),
        warning=lambda message: None,
        info=lambda message: events.append(("info", message)),
    )
    node = SimpleNamespace(
        _controller_name="whole_body_controller",
        _switch_lock=threading.Lock(),
        _lock=threading.Lock(),
        _active=False,
        _target_positions=None,
        _joint_positions=feedback,
        _last_command_at=0.0,
        _activated_at=0.0,
        _state_error_locked=lambda now: "",
        _prepare_low_level_control=lambda: (
            events.append(("prepare", node._active)) or (True, "")),
        _enable_grippers=lambda: (
            events.append(("enable", node._active)) or enable_result),
        _disable_grippers_and_restore_mode=lambda restore_mode: (
            events.append(("rollback", restore_mode, node._active))
            or (True, "")),
        _warn_locked=lambda key, message: events.append((key, message)),
        get_logger=lambda: logger,
    )
    return node, events


def test_engage_enables_both_grippers_before_activating_output():
    node, events = _start_node()

    response = MitPositionController._switch_controller(
        node, _start_request(), SwitchController.Response())

    assert response.ok
    assert node._active
    assert events[:2] == [("prepare", False), ("enable", False)]


def test_engage_gripper_failure_rolls_back_and_stays_inactive():
    node, events = _start_node((False, "right gripper enable failed"))

    response = MitPositionController._switch_controller(
        node, _start_request(), SwitchController.Response())

    assert not response.ok
    assert not node._active
    assert ("rollback", True, False) in events


def test_engage_precheck_failure_disables_grippers_and_stays_inactive():
    node, events = _start_node()
    node._state_error_locked = lambda now: "JointState is stale"
    node._disable_grippers = lambda: (
        events.append(("disable", node._active)) or (True, ""))

    response = MitPositionController._switch_controller(
        node, _start_request(), SwitchController.Response())

    assert not response.ok
    assert not node._active
    assert ("disable", False) in events
    assert not any(event[0] == "prepare" for event in events)


def test_engage_prepare_failure_disables_grippers_and_stays_inactive():
    node, events = _start_node()
    node._prepare_low_level_control = lambda: (
        events.append(("prepare", node._active))
        or (False, "ReleaseMode failed"))
    node._disable_grippers = lambda: (
        events.append(("disable", node._active)) or (True, ""))

    response = MitPositionController._switch_controller(
        node, _start_request(), SwitchController.Response())

    assert not response.ok
    assert not node._active
    assert events[:2] == [("prepare", False), ("disable", False)]


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


def test_gripper_services_attempt_both_sides_and_aggregate_failures():
    calls = []
    node = SimpleNamespace(
        _call_gripper_service=lambda side, action: (
            calls.append((side, action))
            or ((False, f"{side} failed") if side == "left" else (True, "")))
    )

    result = MitPositionController._call_gripper_services(node, "disable")

    assert set(calls) == {("left", "disable"), ("right", "disable")}
    assert result == (False, "left failed")


@pytest.mark.parametrize(
    "response,available,error,expected",
    [
        (SimpleNamespace(success=True, message="enabled"), True, None,
         (True, "enabled")),
        (SimpleNamespace(success=False, message="not ready"), True, None,
         (False, "/grip_arm0/enable failed: not ready")),
        (None, False, None,
         (False, "service unavailable: /grip_arm0/enable")),
        (None, True, RuntimeError("client closed"),
         (False, "service check failed: /grip_arm0/enable: client closed")),
    ],
)
def test_gripper_service_reports_success_and_failures(
        response, available, error, expected):
    client = _ServiceClient(response, available, error)
    node = SimpleNamespace(
        _gripper_service_clients={
            "left": {"enable": ("/grip_arm0/enable", client)},
        },
        _gripper_service_timeout=0.1,
    )

    result = MitPositionController._call_gripper_service(
        node, "left", "enable")

    assert result == expected


def test_parallel_gripper_service_converts_unexpected_exception_to_failure():
    node = SimpleNamespace(
        _call_gripper_service=lambda side, action: (
            (_ for _ in ()).throw(RuntimeError("client failed"))
            if side == "left" else (True, "disabled"))
    )

    result = MitPositionController._call_gripper_services(node, "disable")

    assert result == (
        False, "unexpected left gripper service error: client failed")


def test_disengage_transaction_restores_mode_even_if_gripper_disable_fails():
    events = []
    node = SimpleNamespace(
        _disable_grippers=lambda: (
            events.append("disable") or (False, "left disable failed")),
        _restore_previous_mode=lambda: (
            events.append("restore") or (True, "")),
    )

    result = MitPositionController._disable_grippers_and_restore_mode(
        node, True)

    assert events == ["disable", "restore"]
    assert result == (False, "left disable failed")


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
        _disable_grippers_and_restore_mode=lambda restore_mode: (
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