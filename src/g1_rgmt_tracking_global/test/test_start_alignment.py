import threading

import numpy as np
import pytest
from rclpy.publisher import Publisher
from std_srvs.srv import Trigger

from unitree_g1_ros2_control.msg import JointTargets
from g1_mocap.consumer import FrameBuffer
from g1_rgmt_tracking_global.mocap_clip import MocapClip
from g1_rgmt_tracking_global.motion_library import MotionClip
from g1_rgmt_tracking_global.odometry import OdometryFuser
from g1_rgmt_tracking_global.rgmt_runtime import RgmtPolicy, RgmtSpec
from g1_rgmt_tracking_global.rotations import quat_apply, quat_conj, quat_from_axis, quat_mul
from g1_rgmt_tracking_global.tracking_node import RgmtTrackingNode, State


@pytest.mark.parametrize('streaming', [False, True])
@pytest.mark.parametrize('correction', [0.0, -1.4, 0.9])
def test_start_aligns_position_and_orientation_in_same_world(streaming, correction, monkeypatch):
    position_world = np.array([1.2, -0.7, 0.8])
    pelvis_quat = quat_from_axis('z', 0.3)
    torso_quat = quat_mul(pelvis_quat, quat_from_axis('z', 0.1))
    correction_quat = quat_from_axis('z', correction)
    calls = []
    node = RgmtTrackingNode.__new__(RgmtTrackingNode)
    if streaming:
        clip = MocapClip.__new__(MocapClip)
        node._mocap_clip = clip
    else:
        clip = MotionClip.__new__(MotionClip)
        node._mocap_clip = None
    clip.name = 'reference'
    monkeypatch.setattr(clip, 'align', lambda *args: calls.append(args))
    measured = np.zeros(31)
    node._lock = threading.Lock()
    node._state = State.IDLE
    node._measured = measured
    node._imu_quat = pelvis_quat
    node._odom = OdometryFuser.__new__(OdometryFuser)
    monkeypatch.setattr(node._odom, 'torso_position', lambda: position_world.copy())
    monkeypatch.setattr(node._odom, 'orientation_in_world', lambda quat: quat_mul(correction_quat, quat))
    node._mocap = FrameBuffer(n_joints=31, n_keys=0) if streaming else None
    if node._mocap is not None:
        node._mocap._calibrated = True
    node._clip = clip
    node._tracking = False
    node._stand_s = 3.0
    monkeypatch.setattr(node, '_stale', lambda: '')
    monkeypatch.setattr(node, '_snapshot', lambda: (measured.copy(), np.zeros(31), pelvis_quat.copy()))
    monkeypatch.setattr(node, '_now', lambda: 100.0)
    node._policy = RgmtPolicy.__new__(RgmtPolicy)
    monkeypatch.setattr(node._policy, 'reset', lambda: None)
    monkeypatch.setattr(node, '_torso_quat', lambda measured, pelvis: torso_quat.copy())
    monkeypatch.setattr(node, '_robot_ground_z', lambda measured, pelvis, position: 0.035)
    response = node._on_start(Trigger.Request(), Trigger.Response())
    assert response.success
    assert node._state is State.STAND
    assert len(calls) == 1
    assert np.allclose(calls[0][0], position_world)
    assert np.allclose(calls[0][1], quat_mul(correction_quat, torso_quat))
    foot_separation_local = np.array([0.0, 0.237, 0.0])
    separation_world = quat_apply(calls[0][1], foot_separation_local)
    running_orientation = quat_mul(correction_quat, torso_quat)
    separation_in_running_frame = quat_apply(quat_conj(running_orientation), separation_world)
    assert np.allclose(separation_in_running_frame, foot_separation_local)
    if streaming:
        assert calls[0][2] == 0.035


@pytest.mark.parametrize('state', [State.STAND, State.RUNNING])
def test_control_publishes_fixed_order_joint_targets(state, monkeypatch):
    positions = np.linspace(-0.2, 0.2, 29)
    measured = np.zeros(31)
    published = []
    message = JointTargets(mask=0x7FFFFFFF)
    node = RgmtTrackingNode.__new__(RgmtTrackingNode)
    node._lock = threading.Lock()
    node._state = state
    node._mocap_clip = None
    node._tracking = False
    monkeypatch.setattr(node, '_stale', lambda: '')
    monkeypatch.setattr(node, '_snapshot', lambda: (measured, measured, np.array([1., 0., 0., 0.])))
    node._imu_omega = np.zeros(3)
    node._odom = OdometryFuser.__new__(OdometryFuser)
    monkeypatch.setattr(node._odom, 'orientation_in_world', lambda quat: quat)
    monkeypatch.setattr(node._odom, 'torso_position', lambda: np.zeros(3))
    monkeypatch.setattr(node, '_torso_quat', lambda measured, quat: quat)
    node._stand_from = measured
    node._stand_start = 0.0
    monkeypatch.setattr(node, '_now', lambda: 1.0)
    node._stand_s = 2.0
    node._stand_tracks_mocap = False
    node._clip = MotionClip.__new__(MotionClip)
    monkeypatch.setattr(node._clip, 'stand_joint_pos', lambda: positions)
    node._clip.num_frames = 100
    node._gripper_targets = np.array([0.4, 0.5])
    node._gripper_slots = np.array([29, 30])
    node._joints = [str(index) for index in range(31)]
    node._action_slots = np.arange(29)
    node._obs_slots = np.arange(29)
    node._policy_uncontrolled_slots = np.array([], dtype=np.intp)
    node._policy = RgmtPolicy.__new__(RgmtPolicy)
    node._policy._spec = RgmtSpec(
        obs_joint_names=tuple(node._joints[:29]),
        action_joint_names=tuple(node._joints[:29]),
        all_body_names=(), anchor_body_name='torso_link', reference_key_bodies=(),
        default_joint_pos=np.zeros(29), action_scale=np.ones(29),
        window_offsets=np.array([0]), token_dim=38, token_layout=(('joint_pos', 29),),
        history_len=1, control_dt=0.02)
    monkeypatch.setattr(node._policy, 'step', lambda **kwargs: positions)
    node._policy.frame = 0
    node._tilt_limit = 1.0
    node._message = message
    node._message_positions = np.frombuffer(message.positions, dtype=np.float64)
    node._publisher = Publisher.__new__(Publisher)
    monkeypatch.setattr(node._publisher, 'publish', lambda value: published.append(value))
    node._control()
    expected = np.concatenate((positions, [0.4, 0.5]))
    if state is State.STAND:
        expected *= 0.5
    assert published == [message]
    assert message.mask == 0x7FFFFFFF
    np.testing.assert_array_equal(message.positions, expected)
