
from unittest.mock import Mock
import threading
from pathlib import Path

import numpy as np
import pytest
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from g1_motion_session.arm_ik import ArmIK
from g1_motion_session.arm_ik_mixin import IKStatus
from g1_motion_session.joint_output import JointOutput
from g1_motion_session.task_base import State

from upper_vla.timed_actions import TimedActions
from upper_vla.vla_node import HOME_POSES, VlaBridgeNode
from test_execution_regression import executor_fixture


def home_node(mode='manual'):
    node = executor_fixture(mode)
    node._base_frame = 'torso_link'
    node._tip_frames = {'left': 'left_gripper_base', 'right': 'right_gripper_base'}
    node._infer_requested.set()
    node._inference_active = True
    node._timed = Mock(spec=TimedActions)
    node.stop_spy = Mock(side_effect=lambda reason: VlaBridgeNode._stop(node, reason))
    pytest.MonkeyPatch().setattr(node, '_stop', node.stop_spy)
    return node


@pytest.mark.parametrize('mode', ['manual', 'continuous', 'async'])
@pytest.mark.parametrize('running', [False, True])
def test_home_immediately_replaces_both_targets_and_invalidates_playback(mode, running):
    node = home_node(mode)
    if not running:
        node._running.clear()
    node._active['right'] = False
    node._grip_command = {'left': .4, 'right': .7}
    generation = node._generation
    old_chunk = node._chunk
    assert old_chunk is not None
    response = VlaBridgeNode._on_home(node, Trigger.Request(), Trigger.Response())
    assert response.success
    node.publish_control_spy.assert_not_called()
    sent = [*node._command['left'], *node._command['right']]
    np.testing.assert_allclose(sent, [*HOME_POSES['left'], *HOME_POSES['right']])
    assert len(sent) == 14
    assert node._grip_command == {'left': .4, 'right': .7}
    assert not node._running.is_set()
    assert not node._infer_requested.is_set()
    assert not node._inference_active
    assert node._chunk is None and node._timed is None and node._cursor == 0
    assert node._generation > generation
    node.stop_spy.assert_called_once_with('收到 ~/home')
    VlaBridgeNode._accept(node, old_chunk, 10., generation)
    VlaBridgeNode._on_tick(node)
    node.publish_control_spy.assert_not_called()
    node.request_inference_spy.assert_not_called()


@pytest.mark.parametrize('problem', ['arms', 'base', 'tip'])
def test_home_rejection_never_publishes(problem):
    node = home_node()
    if problem == 'arms':
        pytest.MonkeyPatch().setattr(node, '_arms_ready', Mock(return_value='not engaged'))
    elif problem == 'base':
        node._base_frame = 'pelvis'
    else:
        node._tip_frames['right'] = 'right_wrist_yaw_link'
    response = VlaBridgeNode._on_home(node, Trigger.Request(), Trigger.Response())
    assert not response.success
    node.publish_control_spy.assert_not_called()
    assert node._running.is_set()


@pytest.mark.parametrize('running', [False, True])
def test_home_clears_control_history(running):
    node = home_node()
    history_spy = Mock()
    node._history = history_spy
    if not running:
        node._running.clear()
    response = VlaBridgeNode._on_home(node, Trigger.Request(), Trigger.Response())
    assert response.success
    history_spy.clear.assert_called_once_with()


def test_local_executor_admission_requires_running_and_ik():
    node = home_node()
    del node._arms_ready
    node.state = State.RUNNING
    node._ik = Mock(spec=ArmIK)
    assert node._arms_ready() == ''
    node.state = State.STOPPED
    assert 'engage' in node._arms_ready()
    node.state = State.RUNNING
    node._ik = None
    assert 'IK' in node._arms_ready()


def test_base_stop_ends_inflight_inference():
    node = home_node()
    del node._stop
    node._stop_executor()
    assert not node._running.is_set()
    assert node._chunk is None
    assert not node._infer_requested.is_set()


def ik_node():
    node = home_node()
    node.lock = threading.RLock()
    node.execution_lock = threading.RLock()
    node.state = State.RUNNING
    node.generation = 7
    node._ik = Mock(spec=ArmIK)
    node.arm_slots = np.arange(15, 29)
    node.output = Mock(spec=JointOutput, positions31=np.arange(31, dtype=float))
    node.solve_ik_spy = Mock(return_value=(np.arange(14, dtype=float) / 10., 0., 0., 10))
    node.commit_ik_spy = Mock()
    patch = pytest.MonkeyPatch()
    patch.setattr(node, 'solve_ik', node.solve_ik_spy)
    patch.setattr(node, 'commit_ik', node.commit_ik_spy)

    def publish(positions):
        node.output.positions31[:] = positions
        return True

    node.publish_target_spy = Mock(side_effect=publish)
    patch.setattr(node, 'publish_target', node.publish_target_spy)
    return node


@pytest.mark.parametrize('operation', ['stop', 'home'])
def test_business_stop_and_home_keep_local_ik_tracking(operation):
    node = ik_node()
    node._grip_command = dict(left=.4, right=.7)
    if operation == 'home':
        assert node._on_home(Trigger.Request(), Trigger.Response()).success
    else:
        node._stop('operator stopped')
    command = {side: pose.copy() for side, pose in node._command.items()}
    for _ in range(5):
        node._on_ik_tick()
    assert not node._running.is_set()
    assert node.publish_target_spy.call_count == node.commit_ik_spy.call_count == 5
    solve_call = node.solve_ik_spy.call_args
    assert solve_call is not None
    for side in command:
        np.testing.assert_array_equal(solve_call.args[0][side], command[side])
    np.testing.assert_array_equal(node.output.positions31[:15], np.arange(15))
    np.testing.assert_allclose(node.output.positions31[15:29], np.arange(14) / 10.)
    np.testing.assert_allclose(node.output.positions31[29:31], [.4, .7])
    commit_call = node.commit_ik_spy.call_args
    assert commit_call is not None
    np.testing.assert_array_equal(commit_call.args[0], node.output.positions31)


@pytest.mark.parametrize('invalidate', ['state', 'generation', 'publish'])
def test_cancelled_local_ik_never_commits(invalidate):
    node = ik_node()
    previous = node.output.positions31.copy()

    def solve(command):
        _ = command
        if invalidate == 'state':
            node.state = State.STOPPED
        elif invalidate == 'generation':
            node.generation += 1
        return np.zeros(14), 0., 0., 1

    node.solve_ik_spy.side_effect = solve
    if invalidate == 'publish':
        node.publish_target_spy.side_effect = None
        node.publish_target_spy.return_value = False
    node._on_ik_tick()
    node.commit_ik_spy.assert_not_called()
    np.testing.assert_array_equal(node.output.positions31, previous)


def test_base_stopped_suppresses_local_ik_publish():
    node = ik_node()
    node.state = State.STOPPED
    node._on_ik_tick()
    node.solve_ik_spy.assert_not_called()
    node.publish_target_spy.assert_not_called()


def test_broadcast_start_does_not_start_inference():
    node = ik_node()
    node._running.clear()
    node.reset_ik_spy = Mock()
    pytest.MonkeyPatch().setattr(node, 'reset_ik', node.reset_ik_spy)
    positions = np.arange(31, dtype=float)
    node.prepared(JointState(position=positions.tolist()))
    node.reset_ik_spy.assert_called_once()
    reset_call = node.reset_ik_spy.call_args
    assert reset_call is not None
    np.testing.assert_array_equal(reset_call.args[0], positions)
    assert not node._running.is_set()
    node.request_inference_spy.assert_not_called()


def test_real_local_ik_tracks_cartesian_command_with_committed_feedback():
    node = ik_node()
    node.dt = .02
    node.base_frame = 'torso_link'
    node.tip_frames = dict(node._tip_frames)
    node.arm_joints = [f'{side}_{joint}_joint' for side in ('left', 'right')
                       for joint in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw',
                                     'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')]
    description = Path(__file__).resolve().parents[2] / 'unitree_g1_description/model/final.urdf'
    model = ArmIK(description.read_text(), node.arm_joints, node.tip_frames,
                  rotation_weight=.01, reference=[0.] * 14)
    node._ik = model
    node.arm_indices = np.array([15 + node.arm_joints.index(name) for name in model.joint_names])
    node._arm_from_model = np.array([model.joint_names.index(name) for name in node.arm_joints])
    node.output.positions31[:] = 0.
    node._ik_targets = {}
    node._ik_status = IKStatus(
        upper_ready=False, upper_executor='ik', arm_mode='ik', ik_ready=False,
        base_frame=node.base_frame, tip_frames=dict(node.tip_frames),
        ik_rotation_weight=.01, ik_pos_err=0., ik_ori_err=0., ik_iterations=0,
        ik_ms=0., limited_pose={})
    node._ik_seed = np.zeros(14)
    del node.solve_ik
    del node.commit_ik
    node.reset_ik(node.output.positions31)
    assert node._ik_seed is not None
    node._command = {side: pose.copy() for side, pose in model.fk(node._ik_seed).items()}
    for side in node._command:
        node._command[side][0] += .01
    requested = {side: pose.copy() for side, pose in node._command.items()}
    before = model.fk(node._ik_seed)
    initial_error = max(np.linalg.norm(before[side][:3] - requested[side][:3]) for side in requested)
    node._running.clear()
    for _ in range(50):
        node._on_ik_tick()
    assert node._ik_seed is not None
    reached = model.fk(node._ik_seed)
    final_error = max(np.linalg.norm(reached[side][:3] - requested[side][:3]) for side in requested)
    assert node.publish_target_spy.call_count == 50
    assert final_error < initial_error and final_error < .001
    np.testing.assert_array_equal(node._ik_seed, node.output.positions31[node.arm_indices])
    for side in requested:
        np.testing.assert_array_equal(node._command[side], requested[side])
