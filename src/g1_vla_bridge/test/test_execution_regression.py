"""Exercise the actual callbacks without ROS nodes or hardware publishers."""

import threading
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
    node = SimpleNamespace(
        _lock=threading.Lock(), _running=threading.Event(),
        _command={side: pose.copy() for side in SIDES},
        _grip_command={side: 0. for side in SIDES},
        _active={side: True for side in SIDES},
        _horizon=0, _cursor=0, _max_step_pos=0.02, _max_step_ori=0.1,
        _skip_intermediate=False,
        _cartesian_limit_enabled=True,
        _publisher=Mock(), _arms_ready=lambda: '', _execution_mode=mode,
        _request_inference=Mock(),
    )
    node._running.set()
    node._limit = lambda current, target: VlaBridgeNode._limit(node, current, target)
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


def test_enable_auto_requests_immediately_when_running_and_idle():
    node = executor_fixture()
    node._chunk = None
    node._request_inference = Mock(return_value='')
    response = VlaBridgeNode._on_set_auto(
        node, SimpleNamespace(data=True), SimpleNamespace())
    assert response.success
    assert node._execution_mode == 'continuous'
    node._request_inference.assert_called_once_with()


def test_disable_auto_keeps_current_chunk():
    node = executor_fixture('continuous')
    chunk = node._chunk
    node._request_inference = Mock()
    response = VlaBridgeNode._on_set_auto(
        node, SimpleNamespace(data=False), SimpleNamespace())
    assert response.success
    assert node._execution_mode == 'manual'
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


def test_continuous_does_not_request_before_final_target_sent():
    node = executor_fixture('continuous')
    for _ in range(30):
        VlaBridgeNode._on_tick(node)
    node._request_inference.assert_not_called()
    for _ in range(5):
        VlaBridgeNode._on_tick(node)
    node._request_inference.assert_called_once()


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
        body[f'ROBOT_{side.upper()}_TRANS'] = [position.tolist()]
        body[f'ROBOT_{side.upper()}_ROT_MAT'] = [(rotation.as_matrix() @ fix_rotation).tolist()]
        body[f'ROBOT_{side.upper()}_GRIPPER'] = [float(index)]
    try:
        chunk = backend._to_chunk(parse_action(body))
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
    node._decode_images = lambda: ({}, '')
    node.get_logger = Mock(return_value=Mock())
    node._fail = Mock()
    node._retry_delay = 0.
    node._request_inference = lambda: VlaBridgeNode._request_inference(node)
    node._accept = lambda *args: VlaBridgeNode._accept(node, *args)
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
        body[f'ROBOT_{side.upper()}_GRIPPER'] = np.linspace(side_index, 1 - side_index, 30).tolist()
    chunk = backend._to_chunk(parse_action(body))
    backend.close()
    node._backend = Mock()
    node._backend.infer.return_value = chunk
    accepted = threading.Event()

    def accept(*args):
        VlaBridgeNode._accept(node, *args)
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
                expected_grip = (1 - body[f'ROBOT_{side.upper()}_GRIPPER'][index]) * node._spec.gripper.robot_open_rad
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
        entered.set()
        assert release.wait(3)
        return chunk

    def accept(*args):
        VlaBridgeNode._accept(node, *args)
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
