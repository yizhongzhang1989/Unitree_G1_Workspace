"""Migrated arm mathematics/regressions and the in-process step/mixin contract."""

import math

import numpy as np
import pytest
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String

from g1_motion_session.arm_ik import ArmIK
from _arm_ik_testhelper import (
    ARM_JOINTS, TIP_FRAMES, Host, arm_pipeline, config, configured_ik as make_ik,
    passive_targets, stand_posture, urdf_xml)


@pytest.fixture(scope='module')
def ik():
    return ArmIK(urdf_xml(), ARM_JOINTS, TIP_FRAMES)


@pytest.fixture
def configured_ik():
    return make_ik()


@pytest.fixture
def bound_host():
    host = Host()
    host.bind()
    return host


def test_reduced_model_keeps_only_the_arms(ik):
    assert ik.model.nq == 14
    assert sorted(ik.joint_names) == sorted(ARM_JOINTS)
    assert np.all(ik.lower < ik.upper)


def test_missing_joint_or_frame_fails_at_construction(ik):
    with pytest.raises(ValueError):
        ArmIK(urdf_xml(), ARM_JOINTS + ['no_such_joint'], TIP_FRAMES)
    with pytest.raises(ValueError):
        ArmIK(urdf_xml(), ARM_JOINTS, {'left': 'no_such_frame'})


def test_fk_then_ik_recovers_the_pose(ik):
    rng = np.random.default_rng(0)
    for _ in range(50):
        truth = np.clip(rng.normal(0.0, 0.4, 14), ik.lower, ik.upper)
        targets = ik.fk(truth)
        seed = np.clip(truth + rng.normal(0.0, 0.05, 14), ik.lower, ik.upper)
        solved, pos_err, ori_err, _ = ik.solve(seed, targets)
        assert pos_err < 2e-3 and ori_err < 1e-2
        assert np.all(np.isfinite(solved))


def test_unreachable_target_is_best_effort_not_an_exception(ik):
    solved, pos_err, _, iters = ik.solve(
        np.zeros(14), {'right': [1.5, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0]})
    assert iters == 11 and pos_err > 0.5
    assert np.all(np.isfinite(solved))
    assert np.all(solved >= ik.lower - 1e-9) and np.all(solved <= ik.upper + 1e-9)


def test_solving_one_side_leaves_the_other_untouched(ik):
    seed = np.full(14, 0.1)
    solved, _, _, _ = ik.solve(seed, {'right': ik.fk(np.full(14, 0.3))['right']})
    left = [index for index, name in enumerate(ik.joint_names) if name.startswith('left')]
    right = [index for index, name in enumerate(ik.joint_names) if name.startswith('right')]
    assert np.allclose(solved[left], seed[left])
    assert not np.allclose(solved[right], seed[right])


def test_seed_outside_limits_is_clamped(ik):
    solved, _, _, _ = ik.solve(np.full(14, 1e3), {})
    assert np.allclose(solved, ik.upper)


def test_rotation_weight_zero_releases_orientation_and_updates_live(ik):
    seed = np.zeros(14)
    target = ik.fk(seed)['right'].copy()
    half = math.sin(math.pi / 8.0)
    target[3:7] = [0.0, 0.0, half, math.cos(math.pi / 8.0)]
    ik.set_rotation_weight(0.0)
    position_only, pos_err, ori_err, iters = ik.solve(seed, {'right': target})
    assert np.array_equal(position_only, seed)
    assert pos_err == pytest.approx(0.0)
    assert ori_err > 0.5
    assert iters == 1
    ik.set_rotation_weight(1.0)
    full_pose, _, full_ori_err, iters = ik.solve(seed, {'right': target})
    assert not np.allclose(full_pose, seed)
    assert full_ori_err < ori_err
    assert iters > 0


def test_rotation_weight_rejects_invalid_values(ik):
    for bad in (-0.01, 1.01, math.nan, math.inf):
        with pytest.raises(ValueError):
            ik.set_rotation_weight(bad)


def test_redundant_dof_does_not_drift_along_a_closed_path(configured_ik):
    model = configured_ik
    seed = stand_posture(model)
    center = model.fk(seed)['right']
    yaw = model.joint_names.index('right_shoulder_yaw_joint')
    track = []
    for index in range(300):
        angle = 2 * np.pi * index / 100
        target = dict(model.fk(seed))
        target['right'] = center.copy()
        target['right'][1] += 0.03 * np.cos(angle) - 0.03
        target['right'][2] += 0.03 * np.sin(angle)
        solved, _, _, _ = model.solve(seed, target)
        track.append(solved.copy())
    track = np.asarray(track)
    assert abs(track[-1, yaw] - track[100, yaw]) < 0.02
    assert np.abs(np.diff(track[100:], axis=0)).max() < 0.05


def test_unreachable_target_settles_instead_of_chattering(ik):
    seed = np.zeros(14)
    goal = ik.fk(seed)['right'].copy()
    goal[0] += 0.35
    track = [ik.solve(seed, {'right': goal})[0] for _ in range(60)]
    assert np.abs(np.diff(np.asarray(track)[20:], axis=0)).max() < 1e-9


def test_far_unreachable_target_does_not_blow_up_the_step(ik):
    goal = {side: pose.copy() for side, pose in ik.fk(np.zeros(14)).items()}
    for pose in goal.values():
        pose[0] += 0.60
    seed = np.zeros(14)
    track = []
    for _ in range(120):
        solved, _, _, _ = ik.solve(seed, goal)
        track.append(solved.copy())
    assert np.abs(np.diff(np.asarray(track)[60:], axis=0)).max() < 1e-9
    assert np.all(np.isfinite(solved))
    assert np.all(solved >= ik.lower - 1e-9) and np.all(solved <= ik.upper + 1e-9)


def test_warm_start_never_gets_trapped_past_the_elbow_singularity(configured_ik):
    model = configured_ik
    seed = stand_posture(model)
    home = model.fk(seed)
    far = {side: pose.copy() for side, pose in home.items()}
    for pose in far.values():
        pose[0] += 0.50
    solved = seed.copy()
    for _ in range(4):
        for _ in range(40):
            solved, _, _, _ = model.solve(solved, far)
        for _ in range(60):
            solved, pos_err, _, _ = model.solve(solved, home)
        assert pos_err < 5e-3


def test_elbow_limit_must_stay_below_the_extension_singularity(configured_ik):
    for index, name in enumerate(configured_ik.joint_names):
        if 'elbow' in name:
            assert configured_ik.upper[index] < 1.571


def test_arm_pipeline_always_comes_back_from_an_unreachable_target(configured_ik):
    model = configured_ik
    stand = stand_posture(model)
    home = model.fk(stand)
    rng = np.random.default_rng(1)
    for trial in range(8):
        offset = rng.uniform(0.25, 0.75, 3) * rng.choice([-1.0, 1.0], 3)
        far = {side: np.concatenate([pose[:3] + offset, pose[3:]]) for side, pose in home.items()}
        solved = stand.copy()
        for goal, frames in ((far, 80), (home, 200)):
            for _ in range(frames):
                solved, _ = arm_pipeline(model, solved, goal)
        back = model.fk(solved)
        residual = max(float(np.linalg.norm(back[side][:3] - home[side][:3])) for side in home)
        assert residual < 5e-3, f'trial {trial}: return error {residual}'


def test_arm_pipeline_rate_limit_caps_every_frame(configured_ik):
    model = configured_ik
    rate = config()['arm_rate_limit'] * 0.02
    stand = stand_posture(model)
    home = model.fk(stand)
    rng = np.random.default_rng(5)
    solved, published = stand.copy(), [stand.copy()]
    for _ in range(200):
        offset = rng.uniform(-0.2, 0.2, 3)
        goal = {side: np.concatenate([pose[:3] + offset, pose[3:]]) for side, pose in home.items()}
        solved, _ = arm_pipeline(model, solved, goal)
        published.append(solved.copy())
    assert np.abs(np.diff(np.asarray(published), axis=0)).max() <= rate + 1e-9


def test_rescue_seed_never_fires_during_normal_tracking(configured_ik, monkeypatch):
    model = configured_ik
    stand = stand_posture(model)
    home = model.fk(stand)
    calls = []
    original = model.solve

    def tracked(seed, targets):
        calls.append(1)
        return original(seed, targets)

    monkeypatch.setattr(model, 'solve', tracked)
    solved = stand.copy()
    for index in range(400):
        angle = 2 * math.pi * index / 200
        goal = {side: np.concatenate(
            [pose[:3] + [0.0, 0.06 * math.cos(angle), 0.06 * math.sin(angle)], pose[3:]])
            for side, pose in home.items()}
        solved, _ = arm_pipeline(model, solved, goal)
    assert len(calls) == 400


def test_null_gain_rejects_bad_gains(ik):
    with pytest.raises(ValueError):
        ArmIK(urdf_xml(), ARM_JOINTS, TIP_FRAMES, null_gain={'no_such_joint': 0.2})
    with pytest.raises(ValueError):
        ArmIK(urdf_xml(), ARM_JOINTS, TIP_FRAMES, null_gain={'right_elbow_joint': -0.2})
    with pytest.raises(ValueError):
        ArmIK(urdf_xml(), ARM_JOINTS, TIP_FRAMES, null_gain={'right_elbow_joint': math.nan})
    with pytest.raises(ValueError):
        ArmIK(urdf_xml(), ARM_JOINTS, TIP_FRAMES, null_gain={'right_elbow_joint': 0.2}, null_gate=(1.2, 0.8))
    with pytest.raises(ValueError):
        ArmIK(urdf_xml(), ARM_JOINTS, TIP_FRAMES,
              null_gain={'right_shoulder_roll_joint': 0.05}, null_target={'no_such_joint': 0.2})
    with pytest.raises(ValueError):
        ArmIK(urdf_xml(), ARM_JOINTS, TIP_FRAMES,
              null_gain={'right_shoulder_roll_joint': 0.05}, null_target={'right_shoulder_roll_joint': math.nan})
    with pytest.raises(ValueError):
        ArmIK(urdf_xml(), ARM_JOINTS, TIP_FRAMES, null_target={'right_shoulder_roll_joint': math.radians(-20.0)})


def test_null_target_is_ungated_and_keeps_the_tip_fixed(configured_ik):
    model, values = configured_ik, config()
    targets = dict(zip(values['arm_joints'], values['ik_null_target']))
    left_target = targets['left_shoulder_roll_joint']
    right_target = targets['right_shoulder_roll_joint']
    assert left_target == pytest.approx(math.radians(20.0), abs=math.radians(1.0))
    assert right_target == pytest.approx(-left_target)
    seed = stand_posture(model)
    goal = model.fk(seed)
    solved, _, _, _ = model.solve(seed, goal)
    for side, reference in (('left', left_target), ('right', right_target)):
        index = model.joint_names.index(f'{side}_shoulder_roll_joint')
        before = abs(seed[index] - reference)
        after = abs(solved[index] - reference)
        assert before < values['ik_null_gate'][0]
        assert after < before
        actual = model.fk(solved)[side]
        assert np.linalg.norm(actual[:3] - goal[side][:3]) < 1e-3
        orientation_error = 2.0 * math.acos(min(1.0, abs(float(np.dot(actual[3:], goal[side][3:])))))
        assert orientation_error < values['ik_tol_ori']


def test_zero_reference_null_gate_stays_shut_during_normal_tracking():
    values = config()
    zero_reference_gain = {
        name: gain for name, gain, target in zip(values['arm_joints'], values['ik_null_gain'], values['ik_null_target'])
        if gain and target == 0.0}
    plain_ik = make_ik(null_gain={}, null_target={})
    gated_ik = make_ik(null_gain=zero_reference_gain, null_target={})
    stand = stand_posture(plain_ik)
    home = plain_ik.fk(stand)
    plain, biased = stand.copy(), stand.copy()
    plain_iters = biased_iters = 0
    for index in range(200):
        angle = 2 * math.pi * index / 100
        goal = {side: np.concatenate(
            [pose[:3] + [0.0, 0.03 * math.cos(angle), 0.03 * math.sin(angle)], pose[3:]])
            for side, pose in home.items()}
        plain, _, _, plain_iters = plain_ik.solve(plain, goal)
        biased, _, _, biased_iters = gated_ik.solve(biased, goal)
    assert np.allclose(plain, biased)
    assert biased_iters == plain_iters


def test_null_gain_moves_the_posture_when_the_elbow_is_pinned(configured_ik, ik):
    stand = stand_posture(ik)
    goal = {side: np.concatenate([pose[:3] + [0.20, 0.0, 0.0], pose[3:]]) for side, pose in ik.fk(stand).items()}
    plain, biased = stand.copy(), stand.copy()
    for _ in range(300):
        plain, _, _, _ = ik.solve(plain, goal)
        biased, _, _, _ = configured_ik.solve(biased, goal)
    triple = [index for index, name in enumerate(ik.joint_names)
              if any(part in name for part in ('shoulder_yaw', 'elbow', 'wrist_roll'))]
    assert np.abs(biased[triple]).max() < np.abs(plain[triple]).max()
    for side in goal:
        biased_err = float(np.linalg.norm(ik.fk(biased)[side][:3] - goal[side][:3]))
        assert biased_err < 0.065
        third = configured_ik.joint_names.index(f'{side}_shoulder_yaw_joint')
        fifth = configured_ik.joint_names.index(f'{side}_wrist_roll_joint')
        assert abs(biased[third]) < math.radians(35)
        assert abs(biased[fifth]) < math.radians(50)


@pytest.mark.parametrize('max_iters', [0, 1, 10])
def test_final_update_runs_once_and_reports_final_fk(monkeypatch, max_iters):
    model = make_ik()
    model._iters = max_iters
    seed = stand_posture(model)
    target = {'right': model.fk(seed)['right'].copy()}
    target['right'][0] += 0.03
    calls = []
    original = model._final_update

    def tracked(joints, goals, task_scale, iterations):
        calls.append(iterations)
        return original(joints, goals, task_scale, iterations)

    monkeypatch.setattr(model, '_final_update', tracked)
    solved, pos_err, ori_err, count = model.solve(seed, target)
    assert len(calls) == 1
    assert count == calls[0] + 1
    assert count <= max_iters + 1
    actual = model.fk(solved)['right']
    assert pos_err == pytest.approx(np.linalg.norm(actual[:3] - target['right'][:3]), abs=1e-12)
    orientation = 2 * math.acos(min(1.0, abs(float(np.dot(actual[3:], target['right'][3:])))))
    assert ori_err == pytest.approx(orientation, abs=1e-8)
    left = [index for index, name in enumerate(model.joint_names) if name.startswith('left')]
    assert np.array_equal(solved[left], seed[left])
    assert np.all(solved >= model.lower) and np.all(solved <= model.upper)
    calls.clear()
    held, pos_err, ori_err, count = model.solve(seed, {})
    assert np.array_equal(held, seed)
    assert pos_err == ori_err == count == 0


def test_final_update_extension_return_with_rate_limit(configured_ik):
    model = configured_ik
    stand = stand_posture(model)
    home = model.fk(stand)
    rate = config()['arm_rate_limit'] * 0.02
    joints = stand.copy()
    previous = model.fk(joints)
    biggest_tip_step = 0.0
    extensions = np.concatenate([np.linspace(0, 0.3, 150), np.full(100, 0.3), np.linspace(0.3, 0, 15), np.zeros(250)])
    for index, extension in enumerate(extensions):
        targets = {side: np.concatenate((pose[:3] + [extension, 0, 0], pose[3:])) for side, pose in home.items()}
        updated, _ = arm_pipeline(model, joints, targets)
        assert np.max(np.abs(updated - joints)) <= rate + 1e-12
        joints = updated
        actual = model.fk(joints)
        biggest_tip_step = max(biggest_tip_step, max(
            float(np.linalg.norm(actual[side][:3] - previous[side][:3])) for side in home))
        previous = actual
        if index == 249:
            for side in home:
                third = model.joint_names.index(f'{side}_shoulder_yaw_joint')
                fifth = model.joint_names.index(f'{side}_wrist_roll_joint')
                assert abs(joints[third]) < math.radians(35)
                assert abs(joints[fifth]) < math.radians(50)
    assert biggest_tip_step < 0.05
    assert max(np.linalg.norm(actual[side][:3] - home[side][:3]) for side in home) < 0.001


def test_null_gain_settles_instead_of_chattering(configured_ik):
    model = configured_ik
    stand = stand_posture(model)
    goal = {side: np.concatenate([pose[:3] + [0.20, 0.0, 0.0], pose[3:]]) for side, pose in model.fk(stand).items()}
    track, solved = [], stand.copy()
    for _ in range(160):
        solved, _, _, _ = model.solve(solved, goal)
        track.append(solved.copy())
    assert np.abs(np.diff(np.asarray(track)[80:], axis=0)).max() < 1e-9


def test_same_seed_and_target_give_the_same_solution(configured_ik):
    model = configured_ik
    seed = stand_posture(model)
    target = model.fk(np.full(14, 0.3))
    first, _, _, _ = model.solve(seed, target)
    for _ in range(5):
        again, _, _, _ = model.solve(seed, target)
        assert np.array_equal(first, again)


@pytest.mark.parametrize('dt', [0.0, 0.01, 0.02, 0.04])
def test_step_limits_only_after_solve_and_reports_limited_fk(configured_ik, monkeypatch, dt):
    model = configured_ik
    seed = stand_posture(model)
    desired = seed + 0.5
    targets = model.fk(desired)
    calls = []

    def solve(actual_seed, goals):
        calls.append(actual_seed.copy())
        return desired.copy(), 0.0, 0.0, 11

    monkeypatch.setattr(model, 'solve', solve)
    target, position, orientation, iterations = model.step(seed, targets, dt)
    expected = seed + np.clip(desired - seed, -10.0 * dt, 10.0 * dt)
    np.testing.assert_array_equal(target, expected)
    np.testing.assert_array_equal(calls[0], seed)
    assert len(calls) == 1 and iterations == 11
    reached = model.fk(target, clamp=False)
    assert position == pytest.approx(max(np.linalg.norm(reached[side][:3] - targets[side][:3]) for side in targets))
    errors = [2 * math.acos(min(1.0, abs(float(np.dot(reached[side][3:], targets[side][3:]))))) for side in targets]
    assert orientation == pytest.approx(max(errors), abs=1e-8)


def test_rescue_giveup_rearms_and_accepts_reference_only_on_improvement(configured_ik, monkeypatch):
    model = configured_ik
    seed = stand_posture(model) + 0.05
    targets = model.fk(seed)
    misses = []
    responses = [(seed.copy(), 0.1, 0.2, 11), (seed.copy(), 0.099, 0.3, 7)] * 3
    responses += [(seed.copy(), 0.1, 0.2, 11), (seed.copy(), 0.0, 0.0, 1),
                  (seed.copy(), 0.1, 0.2, 11), (seed + 0.5, 0.08, 0.3, 7)]
    calls = []

    def solve(actual_seed, goals):
        calls.append(actual_seed.copy())
        return responses.pop(0)

    monkeypatch.setattr(model, 'solve', solve)
    for _ in range(4):
        _, _, _, iterations = model.step(seed, targets, 0.02)
        misses.append(model._rescue_miss)
    assert misses == [1, 2, 3, 3]
    assert len(calls) == 7 and iterations == 11
    for index in (1, 3, 5):
        np.testing.assert_array_equal(calls[index], model.reference)
    model.step(seed, targets, 0.02)
    assert model._rescue_miss == 0
    accepted, _, _, iterations = model.step(seed, targets, 0.02)
    assert model._rescue_miss == 0 and iterations == 18
    np.testing.assert_allclose(accepted, seed + 0.2, atol=1e-15)


def test_one_side_rescue_holds_uncommanded_joints(configured_ik, monkeypatch):
    model = configured_ik
    seed = stand_posture(model) + 0.1
    responses = [(seed.copy(), 0.1, 0.0, 11), (model.reference.copy(), 0.0, 0.0, 11)]
    monkeypatch.setattr(model, 'solve', lambda *args: responses.pop(0))
    target = model.step(seed, {'right': model.fk(seed)['right']}, 0.02)[0]
    left = [index for index, name in enumerate(model.joint_names) if name.startswith('left')]
    np.testing.assert_array_equal(target[left], seed[left])


@pytest.mark.parametrize('dt', [-0.01, math.inf, math.nan])
def test_step_rejects_invalid_dt(configured_ik, dt):
    with pytest.raises(ValueError):
        configured_ik.step(stand_posture(configured_ik), {}, dt)


def test_mixin_binds_once_and_maps_joint_names(monkeypatch):
    host = Host({'arm_joints': list(reversed(ARM_JOINTS))})
    assert not host.ik_ready
    assert not host.status_ik()['ik_ready']
    subscription = host.subscriptions[0]
    assert subscription.topic == '/robot_description'
    assert subscription.qos.reliability == ReliabilityPolicy.RELIABLE
    assert subscription.qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert subscription.callback_group is host.control_group
    assert {'passive_targets', 'robot_description_topic'}.isdisjoint(host.declared)
    host.bind()
    original = host._ik
    assert original is not None
    assert host._ik_seed is not None
    np.testing.assert_array_equal(host.arm_slots, [host.joints31.index(name) for name in host.arm_joints])
    np.testing.assert_array_equal(host.arm_indices, [host.joints31.index(name) for name in original.joint_names])
    np.testing.assert_array_equal(original.reference, stand_posture(original))
    host._bind_arm_ik(String(data='not a URDF'))
    host.init_arm_ik()
    assert host._ik is original and len(host.subscriptions) == 1
    assert not host.errors
    desired = host._ik_seed + 0.1
    targets = original.fk(desired)
    monkeypatch.setattr(original, 'step', lambda *args: (desired, 0.01, 0.02, 11))
    result = host.solve_ik(targets)
    np.testing.assert_array_equal(result[0], desired[host._arm_from_model])
    assert result[1:] == (0.01, 0.02, 11)


def test_mixin_solve_commit_reset_use_only_final_published_seed(bound_host, monkeypatch):
    host = bound_host
    original_seed = host._ik_seed.copy()
    original_pose = host.status_ik()['limited_pose']
    targets = host._ik.fk(original_seed + 0.2)
    calls = []
    real_step = host._ik.step

    def tracked(seed, poses, dt):
        calls.append((seed.copy(), dt))
        return real_step(seed, poses, dt)

    monkeypatch.setattr(host._ik, 'step', tracked)
    result = host.solve_ik(targets=targets)
    host.solve_ik(targets['left'], targets['right'])
    for seed, dt in calls:
        np.testing.assert_array_equal(seed, original_seed)
        assert dt == 0.02
    np.testing.assert_array_equal(host._ik_seed, original_seed)
    assert host.status_ik()['limited_pose'] == original_pose
    final = host.output.positions31.copy()
    final[host.arm_slots] = result[0] + 0.01
    host.commit_ik(final)
    final[:] = 100.0
    expected_seed = result[0][np.argsort(host._arm_from_model)] + 0.01
    np.testing.assert_array_equal(host._ik_seed, expected_seed)
    expected_fk = host._ik.fk(expected_seed, clamp=False)
    status = host.status_ik()
    for side in targets:
        np.testing.assert_array_equal(status['limited_pose'][side], expected_fk[side])
    assert status['ik_pos_err'] == round(max(np.linalg.norm(expected_fk[side][:3] - targets[side][:3]) for side in targets), 4)
    assert status['upper_executor'] == status['arm_mode'] == 'ik'
    assert status['ik_iterations'] > 0
    status['limited_pose']['left'][0] = 100.0
    assert host.status_ik()['limited_pose']['left'][0] != 100.0
    host._ik._rescue_miss = 3
    measured = host.output.positions31.copy()
    measured[host.arm_slots] = 0.1
    host.reset_ik(measured)
    assert host._ik._rescue_miss == 0
    assert host.status_ik()['ik_iterations'] == 0
    np.testing.assert_array_equal(host._ik_seed, measured[host.arm_indices])
    host.solve_ik(right=host._ik.fk(host._ik_seed)['right'])
    np.testing.assert_array_equal(calls[-1][0], measured[host.arm_indices])


def test_mixin_actual_fk_does_not_clip_to_solver_limits(bound_host):
    host = bound_host
    positions = host.output.positions31.copy()
    elbow = host.joints31.index('right_elbow_joint')
    positions[elbow] = 1.6
    host.reset_ik(positions)
    actual = host._ik.fk(host._ik_seed, clamp=False)['right']
    clamped = host._ik.fk(host._ik_seed)['right']
    np.testing.assert_array_equal(host.status_ik()['limited_pose']['right'], actual)
    assert not np.allclose(actual, clamped)
    target = host.solve_ik({})[0]
    np.testing.assert_array_equal(target, positions[host.arm_slots])


def test_mixin_weight_updates_before_and_after_binding():
    host = Host()
    parameter = Parameter('ik_rotation_weight', value=0.03)
    assert host._set_ik_parameters([parameter]).successful
    host.bind()
    assert host._ik is not None
    assert host._ik.rotation_weight == 0.03
    parameter = Parameter('ik_rotation_weight', value=0.1)
    assert host._set_ik_parameters([parameter]).successful
    assert host._ik.rotation_weight == host.status_ik()['ik_rotation_weight'] == 0.1
    parameter = Parameter('ik_rotation_weight', value=math.nan)
    assert not host._set_ik_parameters([parameter]).successful
    assert host._ik.rotation_weight == 0.1


def test_mixin_rejects_invalid_poses_and_positions():
    host = Host()
    with pytest.raises(RuntimeError):
        host.solve_ik({})
    host.bind()
    assert host._ik_seed is not None
    seed = host._ik_seed.copy()
    for bad in ({'unknown': [0.0] * 7}, {'left': [0.0] * 6}, {'left': [0.0] * 7},
                {'right': [math.nan, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]}):
        with pytest.raises(ValueError):
            host.solve_ik(bad)
    with pytest.raises(ValueError):
        host.solve_ik({}, [0.0] * 7)
    for bad in (np.zeros(30), np.full(31, math.nan)):
        with pytest.raises(ValueError):
            host.commit_ik(bad)
    np.testing.assert_array_equal(host._ik_seed, seed)


def test_reference_maps_arm_order_and_preserves_input():
    names = list(reversed(ARM_JOINTS))
    reference = list(reversed(passive_targets()[:14]))
    model = ArmIK(urdf_xml(), names, TIP_FRAMES, reference=reference)
    np.testing.assert_array_equal(model.reference, stand_posture(model))
    reference[0] = 100.0
    assert model.reference[-1] != 100.0
