"""Separate staged IK entry point, precision budgets and the shared control pipeline."""

from unittest.mock import Mock

import numpy as np
import pytest
from rclpy.parameter import Parameter

from g1_motion_session.arm_ik import _pose_errors
from _arm_ik_testhelper import ARM_JOINTS, Host, config, configured_ik, stand_posture


@pytest.fixture
def model():
    result = configured_ik()
    result._iters = 20
    result._tol_pos = 2e-5
    result._tol_ori = 7e-5
    return result


def posture(model):
    seed = stand_posture(model)
    for index, name in enumerate(model.joint_names):
        if any(part in name for part in ('elbow', 'wrist_roll', 'shoulder_yaw')):
            seed[index] = .8
    return seed


def test_original_preferences_reduce_the_full_weighted_cost(model):
    seed = posture(model)
    elbows = [i for i, name in enumerate(model.joint_names) if 'elbow' in name]
    seed[elbows] = 1.2
    solved, position, orientation, _ = model.solve(seed, model.fk(seed))
    assert position <= model._tol_pos + 1e-12
    assert orientation <= model._tol_ori + 1e-12
    assert np.all(solved >= model.lower) and np.all(solved <= model.upper)
    status = model.null_status
    assert status is not None
    assert status['iterations'] > 1
    assert not status['elbow_priority']
    assert status['final_cost'] < status['initial_cost']
    assert status['state'] in ('stationary', 'iteration_limit', 'blocked')
    if status['state'] == 'stationary':
        assert status['projected_gradient'] <= 1e-8


def test_shoulder_roll_preference_remains_active_with_elbow_preference(model):
    values = config()
    gains = dict(zip(values['arm_joints'], values['ik_null_gain']))
    for name in gains:
        if 'shoulder_roll' in name:
            gains[name] = 0.
    disabled = configured_ik(null_gain=gains, null_target={})
    disabled._iters, disabled._tol_pos, disabled._tol_ori = 20, model._tol_pos, model._tol_ori
    seed = stand_posture(model)
    for index, name in enumerate(model.joint_names):
        if 'elbow' in name:
            seed[index] = 1.2
    targets = model.fk(seed)
    solved, position, orientation, _ = model.solve(seed, targets)
    without_preference = disabled.solve(seed, targets)[0]
    for side, target in (('left', .34), ('right', -.34)):
        index = model.joint_names.index(f'{side}_shoulder_roll_joint')
        assert abs(solved[index] - target) < abs(without_preference[index] - target) - 1e-5
    assert position <= model._tol_pos + 1e-12
    assert orientation <= model._tol_ori + 1e-12


def test_null_optimization_continues_after_the_first_accepted_step():
    model = configured_ik(null_gain={'left_elbow_joint': .2, 'right_elbow_joint': .2}, null_target={})
    model._tol_pos, model._tol_ori = 2e-5, 7e-5
    seed = posture(model)
    elbows = [i for i, name in enumerate(model.joint_names) if 'elbow' in name]
    seed[elbows] = 1.2
    targets = model.fk(seed)
    model._iters = 1
    once = model.solve(seed, targets)[0]
    first = model.null_status
    model._iters = 20
    solved, position, orientation, _ = model.solve(seed, targets)
    status = model.null_status
    assert first is not None and status is not None
    assert status['iterations'] > first['iterations']
    assert status['final_elbow_cost'] < first['final_elbow_cost']
    assert np.all(solved[elbows] < once[elbows] - 1e-4)
    assert position <= model._tol_pos + 1e-12
    assert orientation <= model._tol_ori + 1e-12


def test_null_status_is_a_snapshot(model):
    seed = posture(model)
    targets = model.fk(seed)
    model.solve(seed, targets)
    status = model.null_status
    assert status is not None
    original = status['state']
    status['state'] = 'incorrect'
    assert model.null_status['state'] == original


def test_zero_budget_reports_actual_unoptimized_preference_cost(model):
    seed = posture(model)
    model._iters = 0
    solved, _, _, iterations = model.solve(seed, model.fk(seed))
    np.testing.assert_array_equal(solved, seed)
    status = model.null_status
    assert status is not None
    assert iterations == status['iterations'] == 0
    assert status['state'] == 'iteration_limit'
    assert status['initial_cost'] == status['final_cost'] > 0
    assert status['initial_elbow_cost'] == status['final_elbow_cost'] > 0
    assert status['projected_gradient'] > 1e-8


def test_null_window_preserves_pose_before_joint_rate_clipping(model):
    seed = posture(model)
    limit = model.rate_limit * .001
    window = seed - limit, seed + limit
    solved, position, orientation, _ = model.solve(seed, model.fk(seed), joint_window=window)
    assert np.all(solved >= window[0] - 1e-12)
    assert np.all(solved <= window[1] + 1e-12)
    assert np.linalg.norm(solved - seed) > 1e-6
    assert position <= model._tol_pos + 1e-12
    assert orientation <= model._tol_ori + 1e-12


@pytest.mark.parametrize('bad_window', ['missing', 'size', 'nan', 'reversed', 'outside_limits'])
def test_invalid_null_windows_are_explicit_errors(model, bad_window):
    seed = posture(model)
    lower, upper = seed - .01, seed + .01
    if bad_window == 'missing':
        window = (lower,)
    elif bad_window == 'size':
        window = (lower[:-1], upper)
    elif bad_window == 'nan':
        lower[0] = np.nan
        window = (lower, upper)
    elif bad_window == 'outside_limits':
        window = (model.upper + 1., model.upper + 2.)
    else:
        window = (upper, lower)
    with pytest.raises(ValueError):
        model.solve(seed, model.fk(seed), joint_window=window)
    assert model.null_status is None


def test_infeasible_zero_width_window_reports_blocked_without_motion(model):
    seed = posture(model)
    targets = model.fk(seed)
    targets['right'][0] += .01
    solved, position, _, _ = model.solve(seed, targets, joint_window=(seed, seed))
    np.testing.assert_array_equal(solved, seed)
    assert position == pytest.approx(.01)
    status = model.null_status
    assert status is not None
    assert status['state'] == 'blocked'
    assert status['task_fraction'] == 0.


def test_joint_at_hard_limit_does_not_block_primary_motion(model):
    import pinocchio as pin

    q = np.array([
        .0615, .2881, -.0448, .8007, .1884, .1878, -.0973,
        -.1861, -.2484, .6185, -.2457, -1.6416, .8670, 1.5695,
    ])
    targets = model.fk(q, clamp=False)
    r_tg = pin.Quaternion(targets['right'][3:7]).matrix()
    r_gt = np.array([[.02175, -.99976, .00253], [-.99973, -.02177, -.00871], [.00876, -.00234, -.99996]])
    p_gt = np.array([-.0072, .00299, .17009])
    dq = np.array([.99424, -.1061, -.01254, .00789])
    dq /= np.linalg.norm(dq)
    r_bh = r_tg @ r_gt @ pin.Quaternion(np.r_[dq[1:], dq[0]]).matrix()
    r_goal = r_bh @ r_gt.T
    goal_pos = targets['right'][:3] + r_tg @ p_gt + r_bh @ [-.00014, -.02093, .01241] - r_goal @ p_gt
    targets['right'] = np.r_[goal_pos, pin.Quaternion(r_goal).coeffs()]
    for _ in range(5):
        q, pos_err, ori_err, _ = model.step(q, targets, .02)
    assert pos_err <= model._tol_pos + 1e-12
    assert ori_err <= model._tol_ori + 1e-12


def test_position_only_task_backtracking_ignores_target_rotation(model):
    seed = posture(model)
    model.set_rotation_weight(0.)
    targets = model.fk(seed)
    targets['right'][:3] += [.08, -.04, .02]
    without_rotation = model.step(seed, targets, .001)[0]
    targets['right'][3:] = [0., 0., 0., 1.]
    with_rotation = model.step(seed, targets, .001)[0]
    np.testing.assert_array_equal(with_rotation, without_rotation)


def test_high_precision_recovers_diverse_reachable_poses(model):
    rng = np.random.default_rng(20261027)
    home = posture(model)
    for index in range(80):
        truth = np.clip(home + rng.normal(0., .15, 14), model.lower, model.upper)
        seed = np.clip(truth + rng.normal(0., .05, 14), model.lower, model.upper)
        targets = model.fk(truth)
        if index % 3:
            side = ('left', 'right')[index % 2]
            targets = {side: targets[side]}
        solved, position, orientation, iterations = model.solve(seed, targets)
        assert position <= model._tol_pos + 1e-12
        assert orientation <= model._tol_ori + 1e-12
        assert np.isfinite(solved).all()
        assert np.all(solved >= model.lower) and np.all(solved <= model.upper)
        actual = _pose_errors(model.fk(solved, clamp=False), targets)
        np.testing.assert_allclose((position, orientation), actual, atol=1e-12)
        assert iterations > 0
        for side in ('left', 'right'):
            if side not in targets:
                columns = model._tip[side][1]
                np.testing.assert_array_equal(solved[columns], seed[columns])


@pytest.mark.parametrize('gain', [.05, 1., 20., 200.])
def test_posture_adjustment_cannot_spend_more_than_the_pose_budget(model, gain):
    seed = posture(model)
    gains = {'right_shoulder_roll_joint': gain}
    biased = configured_ik(null_gain=gains, null_target={'right_shoulder_roll_joint': -.34})
    biased._iters, biased._tol_pos, biased._tol_ori = 20, model._tol_pos, model._tol_ori
    targets = biased.fk(seed)
    solved, position, orientation, _ = biased.solve(seed, targets)
    assert position <= model._tol_pos + 1e-12
    assert orientation <= model._tol_ori + 1e-12
    index = biased.joint_names.index('right_shoulder_roll_joint')
    assert abs(solved[index] + .34) <= abs(seed[index] + .34)
    if gain == .05:
        assert abs(solved[index] + .34) < abs(seed[index] + .34) - 1e-6
        assert np.linalg.norm(solved - seed) > 1e-5


def test_high_precision_handles_singular_position_only_and_unreachable_goals(model):
    model.set_rotation_weight(0.)
    seed = np.zeros(14)
    goal = model.fk(seed)
    goal['right'][:3] += [1., 0., 0.]
    solved, position, orientation, _ = model.solve(seed, goal)
    assert np.isfinite(solved).all() and np.isfinite([position, orientation]).all()
    assert position > .1
    assert np.all(solved >= model.lower) and np.all(solved <= model.upper)


@pytest.mark.parametrize('weight', [0., .01, 1.])
def test_secondary_task_cannot_worsen_an_unconverged_primary_solution(model, weight):
    plain = configured_ik(null_gain={}, null_target={})
    for candidate in (plain, model):
        candidate._iters, candidate._tol_pos, candidate._tol_ori = 4, 2e-5, 7e-5
        candidate.set_rotation_weight(weight)
    seed = posture(model)
    targets = model.fk(seed)
    for pose in targets.values():
        pose[:3] += [.5, .1, .2]
    primary = plain.solve(seed, targets)[0]
    adjusted = model.solve(seed, targets)[0]
    for side in targets:
        before = _pose_errors(plain.fk(primary, clamp=False), {side: targets[side]})
        after = _pose_errors(model.fk(adjusted, clamp=False), {side: targets[side]})
        assert after[0] <= max(plain._tol_pos, before[0]) + 1e-12
        if weight:
            assert after[1] <= max(plain._tol_ori, before[1]) + 1e-12


def test_empty_goals_and_zero_budget_do_not_move_joints(model):
    seed = posture(model)
    held, position, orientation, iterations = model.solve(seed, {})
    np.testing.assert_array_equal(held, seed)
    assert position == orientation == iterations == 0
    targets = model.fk(seed)
    targets['right'][0] += .01
    model._iters = 0
    held, position, _, iterations = model.solve(seed, targets)
    np.testing.assert_array_equal(held, seed)
    assert position == pytest.approx(.01) and iterations == 0


def test_fixed_pose_keeps_precision_during_repeated_posture_adjustments(model):
    seed = posture(model)
    targets = model.fk(seed)
    track = [seed.copy()]
    for _ in range(500):
        seed, position, orientation, _ = model.step(seed, targets, .02)
        assert position <= model._tol_pos + 1e-12
        assert orientation <= model._tol_ori + 1e-12
        track.append(seed.copy())
    track = np.array(track)
    assert np.max(np.abs(np.diff(track, axis=0))) <= model.rate_limit * .02 + 1e-12
    assert np.max(np.abs(np.diff(track[-100:], axis=0))) < 1e-3
    assert np.linalg.norm(track[-1] - track[0]) > 1e-3


def test_high_precision_native_failure_restores_python_execution(model, monkeypatch):
    seed = posture(model)
    targets = model.fk(seed)
    tip = model._tip['right']
    with monkeypatch.context() as patch:
        patch.setitem(model._tip, 'right', (model.model.nframes + 1, tip[1]))
        with pytest.raises(ValueError, match='IK frame out of range'):
            model.solve(seed, targets)
    solved, position, _, _ = model.solve(seed, targets)
    assert np.isfinite(solved).all() and position <= model._tol_pos


def test_high_precision_control_step_reuses_one_native_model(model, monkeypatch):
    from g1_motion_session import _arm_ik_native

    seed = posture(model)
    targets = model.fk(seed + .03)
    solve = Mock(wraps=_arm_ik_native.solve)
    create = Mock(wraps=_arm_ik_native.create)
    monkeypatch.setattr(_arm_ik_native, 'solve', solve)
    monkeypatch.setattr(_arm_ik_native, 'create', create)
    for _ in range(5):
        model.step(seed, targets, .02)
    assert solve.call_count == 5
    create.assert_not_called()


@pytest.mark.parametrize('dt', [0., .01, .02])
def test_high_precision_step_preserves_rescue_hold_rate_limit_and_error_reporting(model, monkeypatch, dt):
    seed = posture(model)
    targets = {'right': model.fk(seed + .3)['right']}
    alternative = seed + .5
    responses = [(seed.copy(), .1, .2, 5), (alternative.copy(), 0., 0., 3)]
    calls = []

    def solve(actual_seed, poses, *, joint_window):
        calls.append(actual_seed.copy())
        np.testing.assert_array_equal(joint_window[0], seed - model.rate_limit * dt)
        np.testing.assert_array_equal(joint_window[1], seed + model.rate_limit * dt)
        return responses.pop(0)

    monkeypatch.setattr(model, 'solve', solve)
    solved, position, orientation, iterations = model.step(seed, targets, dt)
    assert iterations == 8 and model._rescue_miss == 0
    np.testing.assert_array_equal(calls[0], seed)
    np.testing.assert_array_equal(calls[1], model.reference)
    expected = seed.copy()
    columns = model._tip['right'][1]
    expected[columns] += min(.5, model.rate_limit * dt)
    np.testing.assert_allclose(solved, expected, atol=1e-15)
    np.testing.assert_allclose(
        (position, orientation), _pose_errors(model.fk(expected, clamp=False), targets), atol=1e-12,
    )


def test_rejected_rescue_keeps_the_selected_solutions_null_status(model, monkeypatch):
    seed = posture(model)
    targets = model.fk(seed)
    model.solve(seed, targets)
    primary = model.null_status
    assert primary is not None
    alternate = primary.copy()
    alternate['iterations'] += 100
    replies = [(seed.copy(), .1, .2, 1, primary), (model.reference.copy(), .099, .2, 1, alternate)]

    def solve(actual_seed, poses, *, joint_window):
        solved, position, orientation, iterations, status = replies.pop(0)
        model._null_status = status.copy()
        return solved, position, orientation, iterations

    monkeypatch.setattr(model, 'solve', solve)
    model.step(seed, targets, .02)
    assert model.null_status['iterations'] == primary['iterations']


def test_executable_partial_task_does_not_trigger_reference_seed_rescue(model, monkeypatch):
    seed = posture(model)
    targets = model.fk(seed)
    model.solve(seed, targets)
    status = model.null_status
    assert status is not None
    status['task_fraction'] = .5
    targets['right'][0] += .04

    def solve(actual_seed, poses, *, joint_window):
        model._null_status = status.copy()
        return seed.copy(), .02, 0., 1

    callback = Mock(side_effect=solve)
    monkeypatch.setattr(model, 'solve', callback)
    model.step(seed, targets, .02)
    assert callback.call_count == 1
    assert model._rescue_miss == 0


def test_mixin_publishes_defensive_copy_of_null_status(monkeypatch):
    host = Host({'arm_joints': list(reversed(ARM_JOINTS))})
    host.bind()
    ik = host._ik
    assert ik is not None
    step = Mock(wraps=ik.step)
    monkeypatch.setattr(ik, 'step', step)
    targets = ik.fk(host._ik_seed)
    result = host.solve_ik(targets)
    assert step.call_count == 1
    status = host.status_ik()['ik_null_status']
    assert status is not None
    assert status == ik.null_status
    expected = status['state']
    status['state'] = 'incorrect'
    published = host.status_ik()['ik_null_status']
    assert published is not None
    assert published['state'] == expected
    assert result[0].shape == (14,)
