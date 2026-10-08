"""Exercise the actual callbacks without ROS nodes or hardware publishers."""

# pyright: reportArgumentType=false, reportAttributeAccessIssue=false, reportOptionalMemberAccess=false

import threading
from collections import deque
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from g1_vla_bridge.vla_backend import ActionChunk, SIDES
from g1_vla_bridge.vla_node import VlaBridgeNode
from g1_motion_control.command_protocol import join_command, split_command
from g1_vla_bridge.backends.cogact_unitree import SPEC, CogACTUnitreeBackend, parse_action
from g1_vla_bridge.transforms import pose_matrix, quat_angle


def executor_fixture(mode='manual'):
    pose = np.array([0., 0., 0., 0., 0., 0., 1.])
    node = object.__new__(VlaBridgeNode)
    node.__dict__.update(
        _lock=threading.Lock(), _running=threading.Event(),
        _command={side: pose.copy() for side in SIDES},
        _grip_command={side: 0. for side in SIDES},
        _active={side: True for side in SIDES},
        _horizon=0, _cursor=0, _max_step_pos=0.02, _max_step_ori=0.1,
        _skip_intermediate=False,
        _action_rate=10., _execution_rate=10.,
        _cartesian_limit_enabled=True,
        _publisher=Mock(), _arms_ready=lambda: '', _execution_mode=mode,
        _request_inference=Mock(),
        _generation=0, _infer_requested=threading.Event(), _inference_active=False,
        _delta=False, _spec=SPEC, _timed=None, _history=None, _pending_control=deque(),
        _last_control_stamp=None,
        _continuous_next_delay=0.5, _continuous_next_at=None,
        _task='pick up the basketball', _async_min_overlap=7,
        _async_step={}, _async_merge={}, _async_last_publish=None, _error='',
        get_logger=Mock(return_value=Mock()),
    )
    node._running.set()
    poses = np.tile(pose, (30, 1))
    poses[-1, 0] = 0.1
    node._chunk = ActionChunk(
        poses={side: poses.copy() for side in SIDES},
        grippers={side: np.zeros(30) for side in SIDES})
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
    node._request_inference.assert_not_called()


@pytest.mark.parametrize('mode', ['manual', 'continuous'])
def test_thirty_hz_playback_interpolates_without_speeding_up(mode, monkeypatch):
    from g1_vla_bridge import vla_node

    now = [10.]
    monkeypatch.setattr(vla_node.time, 'monotonic', lambda: now[0])
    node = executor_fixture(mode)
    node._execution_rate = 30.
    node._cartesian_limit_enabled = False
    for side in SIDES:
        node._chunk.poses[side][:, 0] = np.arange(30)
        node._chunk.grippers[side][:] = np.arange(30) / 30.
        node._chunk.poses[side][1, 3:] = [0., 0., 1., 0.]
    for _ in range(89):
        VlaBridgeNode._on_tick(node)
    assert node._chunk is not None
    node._request_inference.assert_not_called()
    VlaBridgeNode._on_tick(node)
    assert node._chunk is None
    sent = [call.args[0].data for call in node._publisher.publish.call_args_list]
    arms = np.array(sent[::2])
    grips = np.array(sent[1::2])
    np.testing.assert_allclose(arms[:, 0], np.minimum(np.arange(90) / 3., 29.))
    np.testing.assert_allclose(grips[:, 0], np.minimum(np.arange(90) / 3., 29.) / 30.)
    np.testing.assert_allclose(arms[1, 3:7], [0., 0., .5, np.sqrt(3) / 2.], atol=1e-12)
    assert node._request_inference.call_count == 0
    if mode == 'continuous':
        now[0] = 10.5
        VlaBridgeNode._on_tick(node)
        node._request_inference.assert_called_once()


def test_disabled_limit_preserves_position_and_orientation():
    node = executor_fixture()
    node._cartesian_limit_enabled = False
    target = np.array([0.3, -0.2, 0.4, 1., 0., 0., 0.])
    actual = VlaBridgeNode._limit(node, node._command['left'], target)
    assert np.array_equal(actual, target)
    assert not np.shares_memory(actual, target)


def test_disabled_limit_sends_each_waypoint_without_extra_ticks():
    node = executor_fixture()
    node._cartesian_limit_enabled = False
    expected = node._chunk.poses['left'].copy()
    expected[-1, 3:] = [1., 0., 0., 0.]
    node._chunk.poses['left'][:] = expected
    for _ in range(30):
        VlaBridgeNode._on_tick(node)
    sent = [call.args[0].data for call in node._publisher.publish.call_args_list]
    arms = np.array([row for row in sent if len(row) == 14])
    assert np.allclose(arms[:, :7], expected)
    assert len(sent) == 60
    assert node._chunk is None
    VlaBridgeNode._on_tick(node)
    assert node._publisher.publish.call_count == 60


def test_skip_intermediate_sends_only_final_waypoint():
    node = executor_fixture()
    node._cartesian_limit_enabled = False
    node._skip_intermediate = True
    final_pose = node._chunk.poses['left'][-1].copy()
    VlaBridgeNode._on_tick(node)
    sent = [call.args[0].data for call in node._publisher.publish.call_args_list]
    arms = [row for row in sent if len(row) == 14]
    assert len(arms) == 1
    assert np.allclose(arms[0][:7], final_pose)
    assert node._chunk is None
    for _ in range(3):
        node._on_tick()
    node._request_inference.assert_not_called()


@pytest.mark.parametrize('mode,enabled', [('manual', True), ('continuous', False)])
def test_auto_switch_requires_stop_and_keeps_current_chunk(mode, enabled):
    node = executor_fixture(mode)
    chunk = node._chunk
    response = VlaBridgeNode._on_set_auto(
        node, SimpleNamespace(data=enabled), SimpleNamespace())
    assert not response.success
    assert node._execution_mode == mode
    assert node._chunk is chunk
    node._request_inference.assert_not_called()


def test_set_skip_intermediate_keeps_current_chunk():
    node = executor_fixture()
    chunk = node._chunk
    response = VlaBridgeNode._on_set_skip_intermediate(
        node, SimpleNamespace(data=True), SimpleNamespace())
    assert response.success
    assert node._skip_intermediate is True
    assert node._chunk is chunk


def test_continuous_requests_next_chunk_after_configured_delay(monkeypatch):
    from g1_vla_bridge import vla_node

    now = [10.0]
    monkeypatch.setattr(vla_node.time, 'monotonic', lambda: now[0])
    node = executor_fixture('continuous')
    node._cartesian_limit_enabled = False
    node._skip_intermediate = True
    VlaBridgeNode._on_tick(node)
    assert node._chunk is None
    assert node._continuous_next_at == 10.5
    node._request_inference.assert_not_called()
    now[0] = 10.49
    VlaBridgeNode._on_tick(node)
    node._request_inference.assert_not_called()
    now[0] = 10.5
    VlaBridgeNode._on_tick(node)
    node._request_inference.assert_called_once()
    VlaBridgeNode._on_tick(node)
    node._request_inference.assert_called_once()


@pytest.mark.parametrize('mode', ['continuous', 'async'])
def test_task_change_discards_old_execution_and_requests_new_task(mode):
    node = executor_fixture(mode)
    node._history = Mock()
    node._inference_active = True
    node._infer_requested.set()
    node._continuous_next_at = 123.

    VlaBridgeNode._on_task(node, SimpleNamespace(data='place the basketball on the table'))

    assert node._running.is_set()
    assert node._generation == 1
    assert node._task == 'place the basketball on the table'
    assert node._chunk is None
    assert node._cursor == 0
    assert not node._inference_active
    assert not node._infer_requested.is_set()
    assert node._continuous_next_at is None
    node._request_inference.assert_called_once_with(1)
    if mode == 'async':
        assert node._timed.rate == node._action_rate
        assert node._timed.execution_rate == node._execution_rate
        assert node._timed.first_offset_steps == 1


def test_returned_unified_poses_recover_raw_commands():
    from scipy.spatial.transform import Rotation

    backend = CogACTUnitreeBackend('http://unused.invalid')
    body = {}
    expected = {}
    fix_rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    for index, side in enumerate(SIDES):
        rotation = Rotation.from_euler('xyz', [0.4, -0.2, index + 0.3])
        position = np.array([0.25, 0.2 - 0.4 * index, 0.1])
        expected[side] = np.r_[position, rotation.as_quat()]
        body[f'ROBOT_{side.upper()}_TRANS'] = [position.tolist()] * 30
        body[f'ROBOT_{side.upper()}_ROT_MAT'] = [(rotation.as_matrix() @ fix_rotation).tolist()] * 30
        body[f'ROBOT_{side.upper()}_GRIPPER'] = [[float(index)]] * 30
    try:
        chunk = backend._to_chunk(parse_action({'action': body, 'action_type_info': {'type': 'abs'}}))
        command = join_command(left=chunk.poses['left'][0], right=chunk.poses['right'][0])
        decoded = split_command(command)
        for side in SIDES:
            actual = decoded[side]
            assert np.allclose(actual[:3], expected[side][:3], atol=1e-12)
            assert quat_angle(actual[3:], expected[side][3:]) < 1e-7
            assert np.allclose(pose_matrix(actual[3:], actual[:3]),
                               pose_matrix(expected[side][3:], expected[side][:3]), atol=1e-12)
        assert np.isclose(chunk.grippers['left'][0], backend.spec.gripper.robot_open_rad)
        assert chunk.grippers['right'][0] == backend.spec.gripper.robot_closed_rad
    finally:
        backend.close()


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
    node._observe = Mock(return_value=object())
    node._measured_pose = lambda side: node._command[side].copy()
    node.get_logger = Mock(return_value=Mock())
    node._fail = Mock()
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
    node._backend = Mock()
    node._backend.infer.return_value = chunk
    accepted = threading.Event()

    def accept(*args, **kwargs):
        VlaBridgeNode._accept(node, *args, **kwargs)
        accepted.set()

    node._accept = accept
    assert node._request_inference() == ''
    assert node._request_inference() != ''
    worker = threading.Thread(target=VlaBridgeNode._infer_loop, args=(node,))
    worker.start()
    try:
        assert accepted.wait(2)
        for index in range(30):
            assert node._request_inference() != ''
            VlaBridgeNode._on_tick(node)
            calls = node._publisher.publish.call_args_list
            arms = split_command(calls[-2].args[0].data)
            grips = split_command(calls[-1].args[0].data)['grip']
            for side_index, side in enumerate(SIDES):
                assert np.allclose(arms[side][:3], expected[side][index, :3], atol=1e-12)
                assert quat_angle(arms[side][3:], expected[side][index, 3:]) < 1e-7
                expected_grip = (1 - body[f'ROBOT_{side.upper()}_GRIPPER'][index][0]) * node._spec.gripper.robot_open_rad
                assert grips[side_index] == pytest.approx(expected_grip)
        assert node._chunk is None
        VlaBridgeNode._on_tick(node)
        assert node._publisher.publish.call_count == 60
        node._backend.infer.assert_called_once()
        assert not node._infer_requested.is_set()
    finally:
        node._alive = False
        node._running.clear()
        node._infer_requested.set()
        worker.join(2)
        assert not worker.is_alive()


def test_old_failure_does_not_clear_new_request():
    node = worker_fixture()
    node._generation = 2
    node._inference_active = True
    node._error = 'new task status'
    VlaBridgeNode._fail(node, 'old failure', 1)
    assert node._inference_active
    assert node._error == 'new task status'
    assert not node._infer_requested.is_set()


def test_manual_failure_requires_another_request():
    node = worker_fixture()
    node._inference_active = True
    VlaBridgeNode._fail(node, 'server failure', node._generation)
    assert not node._inference_active
    assert node._error == 'server failure'
    assert not node._infer_requested.is_set()


def test_stop_restart_discards_late_response_without_new_next():
    node = worker_fixture()
    chunk = executor_fixture()._chunk
    entered, release, handled = threading.Event(), threading.Event(), threading.Event()

    def infer(observation):
        _ = observation
        entered.set()
        assert release.wait(3)
        return chunk

    def accept(*args, **kwargs):
        VlaBridgeNode._accept(node, *args, **kwargs)
        handled.set()

    node._backend = SimpleNamespace(infer=infer)
    node._accept = accept
    node._request_inference()
    worker = threading.Thread(target=VlaBridgeNode._infer_loop, args=(node,))
    worker.start()
    try:
        assert entered.wait(2)
        VlaBridgeNode._stop(node, 'test stop')
        response = VlaBridgeNode._on_start(node, None, SimpleNamespace())
        assert response.success
        release.set()
        assert handled.wait(2)
        assert node._chunk is None
        assert not node._inference_active
        node._publisher.publish.assert_not_called()
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

    node._backend = SimpleNamespace(infer=infer, close=closed.set)
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
