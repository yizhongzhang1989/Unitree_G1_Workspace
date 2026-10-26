"""Exercise the actual callbacks without ROS nodes or hardware publishers."""


import threading
from collections import deque
from unittest.mock import Mock

import numpy as np
import pytest
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from upper_vla.vla_backend import ActionChunk, Observation, SIDES
from upper_vla.vla_node import VlaBridgeNode
from upper_vla.backends.cogact_unitree import SPEC, CogACTUnitreeBackend, parse_action
from upper_vla.transforms import quat_angle


class ExecutorTestNode(VlaBridgeNode):
    publish_control_spy: Mock
    request_inference_spy: Mock
    control_measurement_spy: Mock
    stop_spy: Mock
    solve_ik_spy: Mock
    commit_ik_spy: Mock
    publish_target_spy: Mock
    reset_ik_spy: Mock
    observe_spy: Mock
    fail_spy: Mock


def executor_fixture(mode='manual'):
    pose = np.array([0., 0., 0., 0., 0., 0., 1.])
    node = ExecutorTestNode.__new__(ExecutorTestNode)
    patch = pytest.MonkeyPatch()
    node._lock = threading.Lock()
    node._running = threading.Event()
    node._command = {side: pose.copy() for side in SIDES}
    grippers: dict[str, float] = {side: 0. for side in SIDES}
    node._grip_command = grippers
    node._active = {side: True for side in SIDES}
    node._horizon = 0
    node._cursor = 0
    node._max_step_pos = 0.02
    node._max_step_ori = 0.1
    node._skip_intermediate = False
    node._action_rate = 10.
    node._execution_rate = 10.
    node._cartesian_limit_enabled = True
    patch.setattr(node, '_arms_ready', Mock(return_value=''))
    node._execution_mode = mode
    node.request_inference_spy = Mock()
    patch.setattr(node, '_request_inference', node.request_inference_spy)
    node._generation = 0
    node._infer_requested = threading.Event()
    node._inference_active = False
    node._delta = False
    node._spec = SPEC
    node._timed = None
    node._history = None
    node._pending_control = deque()
    node._last_control_stamp = None
    node._continuous_next_delay = 0.5
    node._continuous_next_at = None
    node._task = 'pick up the basketball'
    node._async_min_overlap = 7
    node._async_step = {}
    node._async_merge = {}
    node._async_last_publish = None
    node._error = ''
    patch.setattr(node, 'get_logger', Mock(return_value=Mock()))
    node._running.set()
    poses = np.tile(pose, (30, 1))
    poses[-1, 0] = 0.1
    node._chunk = ActionChunk(
        poses={side: poses.copy() for side in SIDES},
        grippers={side: np.zeros(30) for side in SIDES})
    node.publish_control_spy = Mock(wraps=node._publish_control)
    patch.setattr(node, '_publish_control', node.publish_control_spy)
    return node


def test_limited_final_target_is_not_discarded():
    node = executor_fixture()
    for _ in range(30):
        VlaBridgeNode._on_tick(node)
    assert node._command['left'][0] == 0.02
    assert node._chunk is not None
    for _ in range(5):
        VlaBridgeNode._on_tick(node)
    assert np.isclose(node._command['left'][0], 0.1)
    assert node._chunk is None
    node.request_inference_spy.assert_not_called()


@pytest.mark.parametrize('mode', ['manual', 'continuous'])
def test_thirty_hz_playback_interpolates_without_speeding_up(mode, monkeypatch):
    from upper_vla import vla_node

    now = [10.]
    monkeypatch.setattr(vla_node.time, 'monotonic', lambda: now[0])
    node = executor_fixture(mode)
    node._execution_rate = 30.
    node._cartesian_limit_enabled = False
    assert node._chunk is not None
    for side in SIDES:
        node._chunk.poses[side][:, 0] = np.arange(30)
        node._chunk.grippers[side][:] = np.arange(30) / 30.
        node._chunk.poses[side][1, 3:] = [0., 0., 1., 0.]
    for _ in range(89):
        VlaBridgeNode._on_tick(node)
    assert node._chunk is not None
    node.request_inference_spy.assert_not_called()
    VlaBridgeNode._on_tick(node)
    assert node._chunk is None
    sent = [call.args[0] for call in node.publish_control_spy.call_args_list]
    arms = np.array([[*command['left'], *command['right']] for command in sent])
    grips = np.array([[call.args[1][side] for side in SIDES]
                      for call in node.publish_control_spy.call_args_list])
    np.testing.assert_allclose(arms[:, 0], np.minimum(np.arange(90) / 3., 29.))
    np.testing.assert_allclose(grips[:, 0], np.minimum(np.arange(90) / 3., 29.) / 30.)
    np.testing.assert_allclose(arms[1, 3:7], [0., 0., .5, np.sqrt(3) / 2.], atol=1e-12)
    assert node.request_inference_spy.call_count == 0
    if mode == 'continuous':
        now[0] = 10.5
        VlaBridgeNode._on_tick(node)
        node.request_inference_spy.assert_called_once()


def test_skip_intermediate_sends_only_final_waypoint():
    node = executor_fixture()
    node._cartesian_limit_enabled = False
    node._skip_intermediate = True
    assert node._chunk is not None
    final_pose = node._chunk.poses['left'][-1].copy()
    VlaBridgeNode._on_tick(node)
    sent = [call.args[0] for call in node.publish_control_spy.call_args_list]
    arms = [[*command['left'], *command['right']] for command in sent]
    assert len(arms) == 1
    assert np.allclose(arms[0][:7], final_pose)
    assert node._chunk is None
    for _ in range(3):
        node._on_tick()
    node.request_inference_spy.assert_not_called()


@pytest.mark.parametrize('mode,enabled', [('manual', True), ('continuous', False)])
def test_auto_switch_requires_stop_and_keeps_current_chunk(mode, enabled):
    node = executor_fixture(mode)
    chunk = node._chunk
    response = VlaBridgeNode._on_set_auto(
        node, SetBool.Request(data=enabled), SetBool.Response())
    assert not response.success
    assert node._execution_mode == mode
    assert node._chunk is chunk
    node.request_inference_spy.assert_not_called()


def test_set_skip_intermediate_keeps_current_chunk():
    node = executor_fixture()
    chunk = node._chunk
    response = VlaBridgeNode._on_set_skip_intermediate(
        node, SetBool.Request(data=True), SetBool.Response())
    assert response.success
    assert node._skip_intermediate is True
    assert node._chunk is chunk


def test_continuous_requests_next_chunk_after_configured_delay(monkeypatch):
    from upper_vla import vla_node

    now = [10.0]
    monkeypatch.setattr(vla_node.time, 'monotonic', lambda: now[0])
    node = executor_fixture('continuous')
    node._cartesian_limit_enabled = False
    node._skip_intermediate = True
    VlaBridgeNode._on_tick(node)
    assert node._chunk is None
    assert node._continuous_next_at == 10.5
    node.request_inference_spy.assert_not_called()
    now[0] = 10.49
    VlaBridgeNode._on_tick(node)
    node.request_inference_spy.assert_not_called()
    now[0] = 10.5
    VlaBridgeNode._on_tick(node)
    node.request_inference_spy.assert_called_once()
    VlaBridgeNode._on_tick(node)
    node.request_inference_spy.assert_called_once()


@pytest.mark.parametrize('mode', ['continuous', 'async'])
def test_task_change_discards_old_execution_and_requests_new_task(mode):
    node = executor_fixture(mode)
    node._history = Mock()
    node._inference_active = True
    node._infer_requested.set()
    node._continuous_next_at = 123.

    VlaBridgeNode._on_task(node, String(data='place the basketball on the table'))

    assert node._running.is_set()
    assert node._generation == 1
    assert node._task == 'place the basketball on the table'
    assert node._chunk is None
    assert node._cursor == 0
    assert not node._inference_active
    assert not node._infer_requested.is_set()
    assert node._continuous_next_at is None
    node.request_inference_spy.assert_called_once_with(1)
    if mode == 'async':
        assert node._timed is not None
        assert node._timed.rate == node._action_rate
        assert node._timed.execution_rate == node._execution_rate
        assert node._timed.first_offset_steps == 1


def worker_fixture():
    node = executor_fixture()
    node._alive = True
    node._infer_requested = threading.Event()
    node._inference_active = False
    node._generation = 0
    node._delta = False
    node._chunk = None
    node._task = 'pick up the pink bowl using the left arm.'
    node._spec = SPEC
    patch = pytest.MonkeyPatch()
    grippers: dict[str, float] = {side: node._grip_command[side] for side in SIDES}
    node.observe_spy = Mock(return_value=Observation(
        node._task, {}, dict(node._command), grippers, dict(node._active)))
    patch.setattr(node, '_observe', node.observe_spy)
    patch.setattr(node, '_measured_pose', lambda side: node._command[side].copy())
    node.fail_spy = Mock()
    patch.setattr(node, '_fail', node.fail_spy)
    node._retry_delay = 0.
    del node._request_inference
    return node


def test_worker_response_matches_all_30_commands_and_grippers():
    from scipy.spatial.transform import Rotation

    node = worker_fixture()
    node._cartesian_limit_enabled = False
    backend = CogACTUnitreeBackend('http://unused.invalid')
    body, expected = {}, {}
    fix = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    for side_index, side in enumerate(SIDES):
        positions = np.array([[0.2 + index * 0.002, 0.2 - side_index * 0.4, 0.1]
                              for index in range(30)])
        rotations = Rotation.from_euler('z', np.arange(30) * 0.01 + side_index * 0.2)
        expected[side] = np.column_stack((positions, rotations.as_quat()))
        body[f'ROBOT_{side.upper()}_TRANS'] = positions.tolist()
        body[f'ROBOT_{side.upper()}_ROT_MAT'] = (rotations.as_matrix() @ fix).tolist()
        body[f'ROBOT_{side.upper()}_GRIPPER'] = np.linspace(side_index, 1 - side_index, 30)[:, None].tolist()
    chunk = backend._to_chunk(parse_action({'action': body, 'action_type_info': {'type': 'abs'}}))
    backend.close()
    backend_spy = Mock(spec=CogACTUnitreeBackend)
    backend_spy.infer.return_value = chunk
    node._backend = backend_spy
    accepted = threading.Event()

    def accept(*args, **kwargs):
        VlaBridgeNode._accept(node, *args, **kwargs)
        accepted.set()

    pytest.MonkeyPatch().setattr(node, '_accept', accept)
    assert node._request_inference() == ''
    assert node._request_inference() != ''
    worker = threading.Thread(target=VlaBridgeNode._infer_loop, args=(node,))
    worker.start()
    try:
        assert accepted.wait(2)
        for index in range(30):
            assert node._request_inference() != ''
            VlaBridgeNode._on_tick(node)
            calls = node.publish_control_spy.call_args_list
            arms, grips, _ = calls[-1].args
            for side_index, side in enumerate(SIDES):
                assert np.allclose(arms[side][:3], expected[side][index, :3], atol=1e-12)
                assert quat_angle(arms[side][3:], expected[side][index, 3:]) < 1e-7
                expected_grip = (1 - body[f'ROBOT_{side.upper()}_GRIPPER'][index][0]) * node._spec.gripper.robot_open_rad
                assert grips[side] == pytest.approx(expected_grip)
        assert node._chunk is None
        VlaBridgeNode._on_tick(node)
        assert node.publish_control_spy.call_count == 30
        backend_spy.infer.assert_called_once()
        assert not node._infer_requested.is_set()
    finally:
        node._alive = False
        node._running.clear()
        node._infer_requested.set()
        worker.join(2)
        assert not worker.is_alive()


@pytest.mark.parametrize('failure_generation, remains_active, expected_error', [
    (1, True, 'new task status'),
    (2, False, 'server failure'),
], ids=['stale', 'current'])
def test_manual_failure_only_updates_the_matching_generation(
        failure_generation, remains_active, expected_error):
    """旧请求失败保留新任务状态，当前请求失败需重新发起推理"""
    node = worker_fixture()
    node._generation = 2
    node._inference_active = True
    node._error = 'new task status'
    VlaBridgeNode._fail(node, 'server failure', failure_generation)
    assert node._inference_active is remains_active
    assert node._error == expected_error
    assert not node._infer_requested.is_set()


def test_stop_restart_discards_late_response_without_new_next():
    node = worker_fixture()
    chunk = executor_fixture()._chunk
    assert chunk is not None
    entered, release, handled = threading.Event(), threading.Event(), threading.Event()

    def infer(observation):
        _ = observation
        entered.set()
        assert release.wait(3)
        return chunk

    def accept(*args, **kwargs):
        VlaBridgeNode._accept(node, *args, **kwargs)
        handled.set()

    node._backend = Mock(spec=CogACTUnitreeBackend, infer=Mock(side_effect=infer))
    pytest.MonkeyPatch().setattr(node, '_accept', accept)
    node._request_inference()
    worker = threading.Thread(target=VlaBridgeNode._infer_loop, args=(node,))
    worker.start()
    try:
        assert entered.wait(2)
        VlaBridgeNode._stop(node, 'test stop')
        response = VlaBridgeNode._on_start(node, Trigger.Request(), Trigger.Response())
        assert response.success
        release.set()
        assert handled.wait(2)
        assert node._chunk is None
        assert not node._inference_active
        node.publish_control_spy.assert_not_called()
    finally:
        release.set()
        node._alive = False
        node._running.clear()
        node._infer_requested.set()
        worker.join(2)
        assert not worker.is_alive()


def test_shutdown_waits_for_inference_before_closing_backend():
    node = worker_fixture()
    entered, release, closed = threading.Event(), threading.Event(), threading.Event()

    def infer(observation):
        _ = observation
        entered.set()
        assert release.wait(3)
        return executor_fixture()._chunk

    node._backend = Mock(spec=CogACTUnitreeBackend,
                         infer=Mock(side_effect=infer), close=Mock(side_effect=closed.set))
    node._readers = {}
    node._worker = threading.Thread(target=VlaBridgeNode._infer_loop, args=(node,))
    node._request_inference()
    node._worker.start()
    try:
        assert entered.wait(2)
        shutdown = threading.Thread(target=VlaBridgeNode.shutdown, args=(node,))
        shutdown.start()
        assert not closed.wait(.1)
        release.set()
        shutdown.join(2)
        assert not shutdown.is_alive()
        assert closed.is_set()
    finally:
        release.set()
        node._alive = False
        node._running.clear()
        node._infer_requested.set()
        node._worker.join(2)
