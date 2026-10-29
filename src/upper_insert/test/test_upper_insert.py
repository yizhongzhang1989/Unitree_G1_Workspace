"""Hardware-free regressions for insertion geometry, input handling and control guards."""

import json
import threading
from collections import deque
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pinocchio as pin
import pytest
from geometry_msgs.msg import TransformStamped, WrenchStamped
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import ExtrapolationException

from g1_motion_session.task_base import State
from upper_insert import package_file
from upper_insert.insert_node import InsertTask
from upper_insert.insert_policy import InsertPolicy, WrenchProcessor, build_observation
from upper_insert.tool_calibration import RobotToolCalibrator
from upper_insert.vision import (
    ARUCO_DICT_ID, MARKER_LENGTH_M, VisionPoseEstimator, VisionResult, _PLUG_TO_HOLE_BASIS,
    annotate_plug_and_holes, compute_plug_hole_delta, draw_aruco_marker,
    get_marker_object_points, load_pos_config,
)


@pytest.fixture
def task(monkeypatch):
    import upper_insert.insert_node as module

    monkeypatch.setattr(module.time, 'monotonic', lambda: 1000.)
    obj = object.__new__(InsertTask)
    obj.lock, obj.execution_lock = threading.RLock(), threading.RLock()
    obj._sensor_lock = threading.Lock()
    obj._fallback_error = ''
    obj._logger = SimpleNamespace(error=Mock(), warning=Mock())
    obj.state, obj.mode, obj.active_arm = State.RUNNING, 'policy_insert', 'right'
    obj._preparing = obj._ft_calib_running = False
    obj._imu_required = False
    obj._seeded = True
    obj._state_timeout = obj.wrench_timeout_s = .2
    obj.vision_timeout_s, obj.dt = .5, .02
    obj.vision_rate_hz = 30.
    obj._joint_stamp = 999.99
    obj._q = np.linspace(-.1, .1, 31)
    obj.arm_slots = obj.arm_indices = np.arange(15, 29)
    obj.gripper_slots, obj._grip = np.array([29, 30]), np.zeros(2)
    obj._grip_lo, obj._grip_hi = 0., 2.8
    obj.base_frame = 'torso_link'
    obj.action_frame = obj.delta_frame = 'id0'
    obj.delta_unit, obj.rot_mode = 'm', 'full'
    obj.align_orientation, obj.insert_depth_mm, obj.target_hole = True, 15., 'auto'
    obj.left_wrench_topic, obj.right_wrench_topic = '/arm0/wrench_net', '/arm1/wrench_net'
    obj._safety_warning, obj._mech_goal_pose = '', None
    obj._last_action, obj._last_action_6d = np.zeros(3), np.zeros(6)
    obj._target_stamps = deque(maxlen=251)
    obj._vision_frames = SimpleNamespace(write=lambda frame: frame)
    sides = ('left', 'right')
    obj._wrench = {s: (np.zeros(3), np.zeros(3), 999.99) for s in sides}
    obj._wrench_bias = {s: (np.zeros(3), np.zeros(3)) for s in sides}
    obj._wrench_net_stamp = dict.fromkeys(sides, 999.99)
    obj._wrench_source = dict.fromkeys(sides, 'wrench_net')
    obj._wrench_inputs = {s: {'raw': (999.99, True), 'net': (999.99, True)} for s in sides}
    obj._wrench_proc = {s: WrenchProcessor() for s in sides}
    obj._vision_state = dict(
        valid=True, stamp=999.99, seq=2, stamp_source='camera_read', missing_ids=[],
        error='', algo_cpu_ms=0., preprocess_backend='cpu',
        processed_frames=0, processed_frame_age_ms=0.,
        insert_depth_mm=15., target_hole='circle_tight', distance_mm=10.,
        axis_angle_deg=0., yaw_angle_deg=0., delta_pos=np.array([.002, -.003, -.01]),
        delta_quat=np.array([1., 0., 0., 0.]), delta_quat_wxyz=np.array([1., 0., 0., 0.]),
        R_HT=pin.exp3(np.array([-.2, .3, .1])), delta_id0_m=np.zeros(3),
        delta_id1_m=np.array([.01, -.02, .03]), delta_rot_id1=np.eye(3), r1_t_r0=np.eye(3),
    )
    calibration = dict(
        R_G_T=pin.exp3(np.array([.3, -.1, 2.9])), p_G_T_m=np.array([.03, -.02, .2]),
        R_TS=np.eye(3), r_ST_T_m=np.array([0., 0., -.25]), source='head_camera_tf',
    )
    obj.tool_calibrator = SimpleNamespace(get_calibration=lambda side: calibration)
    pose = np.r_[[.25, -.2, .1], pin.Quaternion(pin.exp3(np.array([.1, -.3, .4]))).coeffs()]
    obj._ik = SimpleNamespace(fk=Mock(return_value={s: pose.copy() for s in sides}))
    obj._ik_kwargs = {'rate_limit': 10.}
    obj._pose = {s: pose.copy() for s in sides}
    obj.output = SimpleNamespace(positions=np.zeros(31), positions31=np.zeros(31))
    obj.solve_ik = Mock(return_value=(np.zeros(14), 0., 0., 0))
    obj.publish_target, obj.commit_ik = Mock(return_value=True), Mock()
    obj._sync_ik_to_measured = Mock(return_value=True)
    obj._loc_pub = obj._detail_pub = SimpleNamespace(publish=Mock())
    obj.estop = Mock()
    obj.policy = SimpleNamespace(
        loaded=True, obs_dim=13, action_dim=6,
        contract=json.loads(package_file('config/contract.json').read_text()),
        step=Mock(return_value=np.array([.0028, -.0033, .0015, .013, -.022, .031])),
        reset=Mock(),
    )
    holes, plug = load_pos_config()
    obj.tracker = SimpleNamespace(
        target_hole=None, holes_mm=holes, plug_mm=plug,
        cam_cfg={'fps': 30, 'pixel_format': 'YUYV'},
    )
    return obj


@pytest.mark.parametrize('mode', ['float', 'mech_insert', 'policy_insert'])
@pytest.mark.parametrize('hole', ['auto', 'rect_tight'])
def test_dashboard_hole_selection_updates_the_shared_target_without_motion(task, mode, hole):
    task.mode = mode
    task.target_hole = task.tracker.target_hole = 'circle_tight'
    task._mech_goal_pose = np.zeros(7)
    task.handle_dashboard_command({'action': 'set_mech_params', 'target_hole': hole})
    assert task.target_hole == hole
    assert task.tracker.target_hole == (None if hole == 'auto' else hole)
    assert task._mech_goal_pose is None
    assert task.mode == mode
    assert task.insert_depth_mm == 15.
    task.publish_target.assert_not_called()
    task.policy.reset.assert_not_called()


def test_dashboard_rejects_unknown_holes_without_changing_the_target(task):
    previous_goal = task._mech_goal_pose = np.zeros(7)
    with pytest.raises(ValueError, match='未知孔位'):
        task.handle_dashboard_command({'action': 'set_mech_params', 'target_hole': 'missing_hole'})
    assert task.target_hole == 'auto' and task.tracker.target_hole is None
    assert task._mech_goal_pose is previous_goal
    task.publish_target.assert_not_called()


def test_vision_detects_markers_and_preserves_depth_geometry():
    k = np.array([[900., 0., 640.], [0., 900., 360.], [0., 0., 1.]])
    image = np.full((720, 1280, 3), 180, np.uint8)
    poses = [(np.array([2.85, -.12, .05]), np.array([-.04, .02, .42])),
             (np.array([2.88, .10, -.06]), np.array([.05, -.03, .40]))]
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    for marker, (rotation, translation) in enumerate(poses):
        card = np.full((140, 140), 255, np.uint8)
        card[10:130, 10:130] = draw_aruco_marker(dictionary, marker, 120)
        outer = get_marker_object_points(MARKER_LENGTH_M * 140 / 120)
        uv, _ = cv2.projectPoints(outer, rotation, translation, k, np.zeros(5))
        transform = cv2.getPerspectiveTransform(
            np.float32([[0, 0], [139, 0], [139, 139], [0, 139]]), uv.reshape(4, 2).astype(np.float32),
        )
        warped = cv2.warpPerspective(card, transform, (1280, 720), borderValue=180)
        mask = cv2.warpPerspective(np.full_like(card, 255), transform, (1280, 720))
        image[mask > 0] = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)[mask > 0]
    estimator = VisionPoseEstimator(k, np.zeros(5))
    _, detected = estimator.estimate_poses(image)
    assert set(detected) == {0, 1}
    for marker in detected:
        np.testing.assert_allclose(detected[marker][1].ravel(), poses[marker][1], atol=.01)
    delta = compute_plug_hole_delta(*poses[0], *poses[1], target_hole='circle_tight', insert_depth_mm=-15.)
    assert delta['hole_in_id0_mm'][2] == 15.
    np.testing.assert_allclose(delta['delta_id0_m'],
                               (delta['hole_in_id0_mm'] - delta['plug_in_id0_mm']) * .001)
    assert delta['policy_obs'].shape == (7,)
    _, lost = estimator.estimate_poses(np.full_like(image, 180))
    assert lost == {}


def test_policy_observation_and_action_scale():
    policy = InsertPolicy(action_scale=1.5)
    assert policy.loaded and (policy.obs_dim, policy.action_dim) == (13, 6)
    assert policy.translation_limit_m == .001
    assert policy.rotation_limit_rad == pytest.approx(np.deg2rad(.5))
    observation = build_observation([1, 2, 3], [.1, .2, .3], [.01, -.02, .03], [1, 0, 0, 0])
    np.testing.assert_allclose(observation, [.01, -.02, .03, 1, 0, 0, 0, 1, 2, 3, .1, .2, .3])
    policy.reset()
    expected = policy.infer_raw(observation) * np.r_[[.0015] * 3, [np.deg2rad(.75)] * 3]
    assert np.any(policy.hidden_state != 0.)
    policy.reset()
    assert not np.any(policy.hidden_state)
    np.testing.assert_allclose(policy.step([1, 2, 3], [.1, .2, .3], [.01, -.02, .03], [1, 0, 0, 0]),
                               expected, rtol=1e-6)


def test_wrench_filter_and_unfiltered_lever_arm_protection():
    processor = WrenchProcessor()
    processor.update_sample([10., 0., 0.], [0., 0., 0.], dt_s=.015)
    assert processor.filtered_s[0] == pytest.approx(10 * (1 - np.exp(-1)))
    force, moment = processor.tip_wrench([0., 0., -.25], np.eye(3), use_filtered=False)
    np.testing.assert_array_equal(force, [10., 0., 0.])
    np.testing.assert_array_equal(moment, [0., 2.5, 0.])
    assert not processor.check_safety([0., 0., -.25], np.eye(3))[0]


@pytest.mark.parametrize('arm', ['left', 'right'])
@pytest.mark.parametrize('align', [False, True])
def test_mechanical_goal_is_full_calibrated_tip_pose(task, arm, align):
    task.active_arm, task.align_orientation = arm, align
    pose = task._ik.fk.return_value[arm]
    calibration = task.tool_calibrator.get_calibration(arm)
    gripper = pin.SE3(pin.Quaternion(pose[3:]).matrix(), pose[:3])
    tool = pin.SE3(calibration['R_G_T'], calibration['p_G_T_m'])
    tip = gripper * tool
    displacement, rotation = np.array([.02, -.03, .04]), pin.exp3(np.array([.2, -.3, .1]))
    basis = gripper.rotation @ tool.rotation @ _PLUG_TO_HOLE_BASIS.T
    goal = task._compute_mech_target(True, basis.T @ displacement, basis.T @ rotation @ basis)
    result = pin.SE3(pin.Quaternion(goal[3:]).matrix(), goal[:3]) * tool
    np.testing.assert_allclose(result.translation, tip.translation + displacement, atol=1e-12)
    np.testing.assert_allclose(result.rotation, rotation @ tip.rotation if align else tip.rotation, atol=1e-12)
    assert task._compute_mech_target(False, displacement, rotation) is None


def test_mechanical_goal_is_frozen_and_commit_follows_publish(task):
    task.mode = 'mech_insert'
    task._control()
    goal = task._mech_goal_pose.copy()
    assert task.commit_ik.call_count == 1
    task._vision_state.update(valid=False, delta_id1_m=np.ones(3))
    task.publish_target.return_value = False
    task._control()
    np.testing.assert_array_equal(task.solve_ik.call_args.kwargs['targets']['right'], goal)
    assert task.commit_ik.call_count == 1
    task.policy.step.assert_not_called()
    task.set_mode('hold')
    assert task._mech_goal_pose is None
    task._sync_ik_to_measured.assert_called_once()


@pytest.mark.parametrize('arm', ['left', 'right'])
@pytest.mark.parametrize('action', ['mixed', 'rotation', 'zero'])
def test_policy_actions_are_anchored_to_measured_tip(task, arm, action):
    task.active_arm = arm
    if action in ('rotation', 'zero'):
        task.policy.step.return_value[:3] = 0
    if action == 'zero':
        task.policy.step.return_value[:] = 0
    step = task.policy.step.return_value
    pose = task._ik.fk.return_value[arm]
    calibration = task.tool_calibrator.get_calibration(arm)
    gripper = pin.SE3(pin.Quaternion(pose[3:]).matrix(), pose[:3])
    tool = pin.SE3(calibration['R_G_T'], calibration['p_G_T_m'])
    tip = gripper * tool
    basis = tip.rotation @ task._vision_state['R_HT'].T
    for _ in range(2):
        task._control()
        target = task.solve_ik.call_args.kwargs['targets'][arm]
        actual = pin.SE3(pin.Quaternion(target[3:]).matrix(), target[:3]) * tool
        np.testing.assert_allclose(actual.translation, tip.translation + basis @ step[:3], atol=1e-12)
        np.testing.assert_allclose(actual.rotation, basis @ pin.exp3(step[3:]) @ basis.T @ tip.rotation, atol=1e-12)
    assert task._ik.fk.call_args.kwargs == {'clamp': False}


@pytest.mark.parametrize('failure', ['force', 'joint', 'vision', 'wrench'])
def test_existing_control_guards_and_stale_warn_only_behavior(task, failure):
    if failure == 'force':
        task._wrench_proc['right'].update_sample([0., 0., 40.], np.zeros(3), stamp_s=999.)
    elif failure == 'joint':
        task._joint_stamp = 999.
    elif failure == 'vision':
        task._vision_state['stamp'] = 999.
    else:
        task._wrench['right'] = (np.zeros(3), np.zeros(3), 999.)
    task._control()
    if failure == 'force':
        assert task.mode == 'hold'
        task.policy.step.assert_not_called()
    elif failure == 'joint':
        task.estop.assert_called_once_with('/joint_states timeout')
    elif failure == 'vision':
        assert task.mode == 'policy_insert'
        task.policy.step.assert_not_called()
        task.policy.reset.assert_not_called()
        task._logger.warning.assert_called_once()
    else:
        assert task.mode == 'policy_insert'
        task.policy.step.assert_called_once()
        task.policy.reset.assert_not_called()
        task._logger.warning.assert_called_once()


@pytest.mark.parametrize('net_age', [.01, 1.])
def test_raw_wrench_fallback_only_activates_when_net_is_stale(task, net_age):
    task._wrench_net_stamp['right'] = 1000. - net_age
    task._compensate_raw_wrench_in_sensor = Mock(return_value=(np.ones(3), np.zeros(3)))
    message = WrenchStamped()
    message.wrench.force.x = 1.
    task._on_wrench('right', message, is_net=False, fallback_only=True)
    selected = net_age > .2
    assert task._compensate_raw_wrench_in_sensor.call_count == int(selected)
    assert task._wrench_source['right'] == ('wrench_raw+urdf_grav' if selected else 'wrench_net')


@pytest.fixture
def head_task(task, tmp_path):
    task.mode = 'float'
    task.tool_calibrator = RobotToolCalibrator(
        calibration_json=tmp_path / 'tool_calibration.json'
    )
    task.tip_frames = {'left': 'left_gripper_base', 'right': 'right_gripper_base'}
    task._head_image = task._head_camera_info = None
    task._head_calibration_jpeg = b''
    task._head_capture_lock = threading.Lock()
    task._head_capture_requested = threading.Event()
    task._head_capture_ready = threading.Event()
    task._head_qos = object()
    task._context = Mock()
    task.get_namespace = lambda: '/'
    task.resolve_topic_name = lambda topic: '/remapped' + topic
    task._head_capture_requested.set()
    task.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=1000.))
    task._tf_buffer = Mock()
    transform = TransformStamped()
    transform.transform.rotation.w = 1.
    task._tf_buffer.lookup_transform.return_value = transform
    message = Image(height=96, width=128, encoding='rgb8', step=384,
                    data=np.full((96, 128, 3), [220, 40, 20], np.uint8).tobytes())
    message.header.frame_id = 'camera_optical'
    message.header.stamp = Time(seconds=999.8).to_msg()
    info = CameraInfo(height=96, width=128, distortion_model='plumb_bob')
    info.header, info.k, info.d = message.header, [100., 0., 64., 0., 100., 48., 0., 0., 1.], [0.] * 5
    task._on_head_image(message)
    task._on_head_camera_info(info)
    task._head_capture_requested.clear()
    task._capture_head_inputs = task._head_calibration_inputs
    return task


@pytest.mark.parametrize('failure', [None, 'missing_id', 'arm_tf', 'old_image'])
def test_head_calibration_uses_capture_tf_and_preserves_result_on_failure(head_task, monkeypatch, failure):
    task = head_task
    original = task.tool_calibrator.calibrations['right']
    detect = Mock(return_value={} if failure == 'missing_id' else {1: (np.zeros(3), np.array([0., 0., .4]))})
    monkeypatch.setattr(task.tool_calibrator, 'detect_marker_multiscale', detect)
    if failure == 'old_image':
        task._head_image.header.stamp = Time(seconds=998.).to_msg()
    elif failure == 'arm_tf':
        def lookup(base, frame, stamp, **kwargs):
            if frame == 'right_gripper_base':
                raise ExtrapolationException('missing same-time arm TF')
            return task._tf_buffer.lookup_transform.return_value
        task._tf_buffer.lookup_transform.side_effect = lookup
    command = {'action': 'calibrate_tool', 'mode': 'head_camera'}
    if failure:
        with pytest.raises(ValueError):
            task.handle_dashboard_command(command)
        assert task.tool_calibrator.calibrations['right'] is original
        assert not task.tool_calibrator.calibration_path.is_file()
    else:
        assert task.handle_dashboard_command(command)['calibration']['source'] == 'head_camera_tf'
        assert task.tool_calibrator.calibration_path.is_file()
        reloaded = RobotToolCalibrator(calibration_json=task.tool_calibrator.calibration_path)
        assert reloaded.get_calibration('right')['source'] == 'head_camera_tf'
        np.testing.assert_allclose(
            reloaded.get_calibration('right')['p_G_T_m'],
            task.tool_calibrator.get_calibration('right')['p_G_T_m'],
            atol=1e-5,
        )
    assert task._head_calibration_jpeg
    for call in task._tf_buffer.lookup_transform.call_args_list:
        assert call.args[2].to_msg() == task._head_image.header.stamp
    task.publish_target.assert_not_called()

    task._registered = True
    task.state = State.ERROR
    task._error = '/joint_states timeout'
    task.set_mode('float')
    assert task.state == State.RUNNING and task._error == ''


def test_vision_updates_keep_capture_time_and_solved_geometry(task):
    poses = {0: (np.array([.1, .2, .3]), np.array([0., 0., .4])),
             1: (np.array([.2, -.1, .3]), np.array([.02, .01, .4]))}
    frame = np.zeros((8, 8, 3), np.uint8)
    task.tracker.camera_reader = SimpleNamespace(running=True, read_latest_timed=lambda: (True, frame, 7, 999.8))

    def result(**kwargs):
        task._vision_alive = False
        return VisionResult(poses, 400.)

    future = Mock()
    future.result.side_effect = result
    task._vision_executor = SimpleNamespace(submit=Mock(return_value=future))
    task._vision_alive, task._preview_requested_until = True, 0.
    task._vision_loop()
    assert task._vision_state['valid'] and task._vision_state['stamp'] == 999.8
    assert task._vision_state['processed_frames'] == 1
    expected = cv2.Rodrigues(poses[1][0])[0].T @ cv2.Rodrigues(poses[0][0])[0]
    np.testing.assert_allclose(task._vision_state['r1_t_r0'], expected, atol=1e-12)


def test_degenerate_pose_and_invalid_vision_hold_without_preview_crash(task, monkeypatch):
    k = np.array([[619.69, 0., 639.41], [0., 618.46, 362.08], [0., 0., 1.]])
    dist = np.array([.09296, -.04049, .00035, .00065, -.04315])
    estimator = VisionPoseEstimator(k, dist)
    bad_quad = np.array([[[302.4, 491.7], [326., 497.5], [309.4, 504.6], [306., 517.5]]], np.float32)
    monkeypatch.setattr(estimator, 'detect_markers', lambda frame: ((bad_quad,), np.array([[1]], np.int32)))
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    _, poses = estimator.estimate_poses(frame, draw=False)
    assert poses == {}

    holes, plug = load_pos_config()
    extreme_poses = {
        0: (np.array([2.8, -.1, .05]), np.array([-.04, .02, .45])),
        1: (np.array([1.2, -.8, .5]), np.array([.01, -.02, 1e-4])),
    }
    vis, _ = annotate_plug_and_holes(frame, k, dist, holes, plug, poses=extreme_poses)
    assert vis.shape == frame.shape

    task._vision_state['valid'] = False
    task._vision_state['missing_ids'] = [1]
    initial_pose = task._pose['right'].copy()
    task._control()
    task.policy.step.assert_not_called()
    np.testing.assert_array_equal(task._last_action, np.zeros(3))
    np.testing.assert_array_equal(task.solve_ik.call_args.kwargs['targets']['right'], initial_pose)


def test_step_policy_single_step_and_dry_run(task):
    task.mode = 'float'
    task.policy._hidden = np.zeros((2, 1, 128), dtype=np.float32)
    task.policy.hidden_state = task.policy._hidden.copy()
    dry = task.handle_dashboard_command({'action': 'step_policy', 'dry_run': True})
    assert dry['policy_step']['dry_run'] is True
    assert dry['policy_step']['step_index'] == 0
    assert task.mode == 'float'

    step1 = task.handle_dashboard_command({'action': 'step_policy'})
    assert step1['policy_step']['dry_run'] is False
    assert step1['policy_step']['step_index'] == 1
    assert task.mode == 'hold'
    pose_after_1 = task._pose['right'].copy()

    step2 = task.handle_dashboard_command({'action': 'step_policy'})
    assert step2['policy_step']['step_index'] == 2
    assert not np.allclose(task._pose['right'], pose_after_1)

    task.handle_dashboard_command({'action': 'reset_policy'})
    assert task._policy_step_count == 0
    assert task._last_policy_step_info is None


def test_id0_reuses_previous_pose_while_missing_id1_stops(task, monkeypatch):
    import upper_insert.vision.vision_worker as vw

    r0, t0 = np.array([2.8, -.1, .05]), np.array([-.04, .02, .45])
    r1, t1 = np.array([2.8, .1, -.05]), np.array([.03, -.02, .42])
    seq = [{0: (r0, t0), 1: (r1, t1)}, {1: (r1, t1)}, {}]
    vw._last_id0_pose = None
    vw._tracker = SimpleNamespace(
        estimator=SimpleNamespace(
            estimate_poses=lambda frame, draw=False, **_: (None, seq.pop(0)),
            color_preprocessor=None,
        ),
        cam_cfg={'camera_matrix': np.eye(3), 'dist_coeffs': np.zeros(5)},
        holes_mm=task.tracker.holes_mm,
        plug_mm=task.tracker.plug_mm,
    )
    frame = np.zeros((16, 16, 3), dtype=np.uint8)
    res_both = vw.process_vision_frame(frame, False, None, 15.)
    assert set(res_both.poses) == {0, 1}

    res_id1_only = vw.process_vision_frame(frame, False, None, 15.)
    assert set(res_id1_only.poses) == {0, 1}
    np.testing.assert_array_equal(res_id1_only.poses[0][0], r0)
    np.testing.assert_array_equal(res_id1_only.poses[0][1], t0)

    res_missing_id1 = vw.process_vision_frame(frame, False, None, 15.)
    assert set(res_missing_id1.poses) == {0}
    assert 1 not in res_missing_id1.poses


def test_fk_latency_compensation_and_rotation_limit_and_cached_id0_fast_path(task, monkeypatch):
    import upper_insert.vision.vision_locator as vl

    # 1. Rotation limit caps rotation vector norm while preserving translation and axis
    task.max_policy_rot_rad = float(np.deg2rad(1.0))
    raw = np.array([.002, -.001, .003, np.deg2rad(3.0), np.deg2rad(4.0), 0.])
    limited = task._limit_policy_action(raw)
    np.testing.assert_allclose(limited[:3], raw[:3])
    assert np.isclose(float(np.linalg.norm(limited[3:6])), np.deg2rad(1.0), rtol=1e-12)
    np.testing.assert_allclose(limited[3:6] / np.linalg.norm(limited[3:6]),
                               raw[3:6] / np.linalg.norm(raw[3:6]))

    # 2. FK latency compensation advances H-frame delta by exact tip motion between capture and control
    cal = task.tool_calibrator.get_calibration('right')
    r_gt = np.asarray(cal['R_G_T'], dtype=np.float64)
    p_gt = np.asarray(cal['p_G_T_m'], dtype=np.float64)
    cap_pose = task._pose['right'].copy()
    r_bg_cam = pin.Quaternion(cap_pose[3:7]).matrix()
    r_bt_cam = r_bg_cam @ r_gt
    p_bt_cam = cap_pose[:3] + r_bg_cam @ p_gt

    # Move tip by +3 mm along +X_H and +2 deg around +Y_H
    r_ht_cam = task._vision_state['R_HT'].copy()
    dp_h_actual = np.array([.003, -.001, .002])
    dr_h_actual = np.array([0., float(np.deg2rad(2.0)), 0.])
    dp_t_actual = r_ht_cam.T @ dp_h_actual
    dr_t_actual = r_ht_cam.T @ dr_h_actual
    r_bt_now = r_bt_cam @ pin.exp3(dr_t_actual)
    p_bt_now = p_bt_cam + r_bt_cam @ dp_t_actual
    r_bg_now = r_bt_now @ r_gt.T
    p_bg_now = p_bt_now - r_bg_now @ p_gt
    cur_pose = np.r_[p_bg_now, pin.Quaternion(r_bg_now).coeffs()]

    d_id0_cam = np.array([.010, -.005, .015])
    q_cam = vl.rotation_matrix_to_quat(r_ht_cam.T, order='wxyz')
    r1_r0_cam = _PLUG_TO_HOLE_BASIS @ r_ht_cam.T
    d_now, q_now, r_ht_now, r1_r0_now = task._compensate_vision_with_fk(
        d_id0_cam, q_cam, r_ht_cam, r1_r0_cam, cap_pose, cur_pose, cal,
    )
    np.testing.assert_allclose(d_now, d_id0_cam - dp_h_actual, atol=1e-12)
    np.testing.assert_allclose(r_ht_now, pin.exp3(dr_h_actual) @ r_ht_cam, atol=1e-12)
    np.testing.assert_allclose(q_now, vl.rotation_matrix_to_quat(r_ht_now.T, order='wxyz'), atol=1e-12)
    np.testing.assert_allclose(r1_r0_now, _PLUG_TO_HOLE_BASIS @ r_ht_now.T, atol=1e-12)

    # 3. Cached ID0 fast path skips occlusion recovery when ID1 is already detected
    k = np.array([[600., 0., 320.], [0., 600., 240.], [0., 0., 1.]])
    est = VisionPoseEstimator(k, np.zeros(5))
    frame = np.full((480, 640, 3), 240, dtype=np.uint8)
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    m1 = cv2.cvtColor(draw_aruco_marker(aruco_dict, 1, 80), cv2.COLOR_GRAY2BGR)
    frame[180:260, 260:340] = m1
    recovery_called = False

    def fail_recovery(*args, **kwargs):
        nonlocal recovery_called
        recovery_called = True

    monkeypatch.setattr(vl, '_recover_occluded_markers_single_frame', fail_recovery)
    corners, ids = est.detect_markers(frame, required_ids=(1,))
    assert not recovery_called
    assert ids is not None and list(ids.flatten()) == [1]
    assert len(corners) == 1

    # 4. SE(3) vision+FK fusion in torso_link and FK coasting when ID1 disappears
    task.vision_fk_alpha_pos = 0.5
    task.vision_fk_alpha_rot = 0.5
    task.vision_fk_coast_s = 2.0
    task.quat_order = 'wxyz'
    raw_delta = {
        'target_hole': 'circle_tight',
        'nearest_hole': 'circle_tight',
        'insert_depth_mm': 15.0,
        'R_HT': r_ht_cam.copy(),
        'delta_id0_m': d_id0_cam.copy(),
    }
    fused_1 = task._fuse_vision_delta_with_fk(raw_delta, cap_pose, 1000.0)
    np.testing.assert_allclose(fused_1['delta_id0_m'], d_id0_cam, atol=1e-12)
    np.testing.assert_allclose(fused_1['R_HT'], r_ht_cam, atol=1e-12)

    # When ID1 is lost at t=1000.5s and gripper moved to cur_pose, coasted delta matches exact FK motion
    coasted = task._coast_vision_delta_with_fk(cur_pose, 1000.5, 'circle_tight', 15.0)
    assert coasted is not None
    np.testing.assert_allclose(coasted['delta_id0_m'], d_id0_cam - dp_h_actual, atol=1e-12)
    np.testing.assert_allclose(coasted['R_HT'], pin.exp3(dr_h_actual) @ r_ht_cam, atol=1e-12)
    assert task._coast_vision_delta_with_fk(cur_pose, 1003.0, 'circle_tight', 15.0) is None
