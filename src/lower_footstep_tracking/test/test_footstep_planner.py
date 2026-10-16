import math

import numpy as np
import pytest

from lower_footstep_tracking.footstep_planner import (
    Footstep, WorldFootstepQueue, direction_steps, relative, route_steps, simple_steps,
    validate_steps)
from lower_footstep_tracking.footstep_policy import FootstepScheduler


FEET = np.array(((0., .12, 0.), (0., -.12, 0.)))


def test_lateral_move_subdivides_and_finishes_at_requested_distance():
    shift = [0., .25]
    steps = direction_steps(FEET, 0, 'left', .25)
    assert [step.side for step in steps] == [0, 1] * 4
    final = validate_steps(steps, FEET)
    np.testing.assert_allclose(final[:, :2] - FEET[:, :2], [shift, shift])
    result = queue()
    result.append(steps)
    result.start()
    finish(result)
    np.testing.assert_allclose(result.scheduler.slots[:2, :2] - FEET[:, :2], [shift, shift], atol=1e-12)


def test_route_drawing_order_and_atomic_validation():
    forward = route_steps([[0., 0.], [.6, 0.]], FEET, 0)
    backward = route_steps([[0., 0.], [-.6, 0.]], FEET, 1, follow_heading=False)
    assert [step.side for step in forward] == [0, 1, 0, 1]
    assert forward[-1].pose[0] == pytest.approx(.6)
    assert backward[-1].pose[0] == pytest.approx(-.6)
    sideways = route_steps([[0., 0.], [0., .6]], FEET, 0)
    assert sideways[-1].pose[2] == pytest.approx(math.pi / 2)
    validate_steps([Footstep(0, np.array((.1, -.067, 0.)))], FEET)
    with pytest.raises(ValueError, match='spacing'):
        validate_steps([Footstep(0, np.array((.1, .3, 0.)))], FEET)
    with pytest.raises(ValueError, match='alternate'):
        validate_steps([forward[0], forward[0]], FEET)


def test_continuous_yaw_boundary_and_single_training_limit():
    for yaw in (-math.pi / 2, math.pi / 2):
        step = Footstep(0, np.array((.1, .12, yaw)))
        validate_steps([step], FEET)
        with pytest.raises(ValueError, match='30 degrees'):
            validate_steps([step], FEET, single=True)
        step.pose[2] = math.copysign(math.pi / 2 + .001, yaw)
        with pytest.raises(ValueError, match='90 degrees'):
            validate_steps([step], FEET)


def test_right_angle_route_generates_feasible_footsteps():
    steps = route_steps([[0., 0.], [.6, 0.], [.6, .6]], FEET, 0)
    feet = validate_steps(steps, FEET)
    assert len(steps) == 7
    assert steps[3].pose[2] == pytest.approx(math.pi / 2)
    assert steps[3].pose[1] < steps[4].pose[1]
    assert steps[-1].pose[2] == pytest.approx(math.pi / 2)
    result = queue()
    result.append(steps)
    result.start()
    finish(result)
    np.testing.assert_allclose(result.scheduler.slots[:2].mean(axis=0)[:2], feet.mean(axis=0)[:2], atol=.1)


def queue():
    contract = {'phase': {'left_touchdown_rad': math.pi / 2,
                          'right_touchdown_rad': 3 * math.pi / 2,
                          'contact_schedule': {
                              'double_support_windows_rad': [[.4 * math.pi, .6 * math.pi],
                                                             [1.4 * math.pi, 1.6 * math.pi]],
                              'liftoff_rad': [1.6 * math.pi, .6 * math.pi]}}}
    scheduler = FootstepScheduler(contract)
    scheduler.reset(FEET)
    return WorldFootstepQueue(scheduler, assumed_landing=True)


def finish(result, feet=None):
    for _ in range(1000):
        previous = result.phase
        result.tick(.02, result.scheduler.slots[:2] if feet is None else feet)
        if result.state == 'standing':
            assert result.scheduler.frequency == 0.
            return
        assert result.phase - previous == pytest.approx(math.tau * result.scheduler.frequency * .02)
    raise AssertionError('Queue did not finish')


@pytest.mark.parametrize('align', [False, True])
def test_single_profile_exact_phase_and_standing_hold(align):
    result = queue()
    result.append(simple_steps(FEET, 0, 'forward', .1, align=align), kind='single')
    result.start(1.2)
    first = result.phase
    result.tick(.02, FEET)
    assert result.scheduler.frequency == pytest.approx(1.2 * .02 / .5)
    finish(result)
    assert result.phase - first == pytest.approx((2 if align else 1) * math.pi + .05 * math.pi)
    assert result.scheduler.frequency == 0.
    frozen = result.phase
    result.tick(.02, FEET)
    assert result.phase == frozen


def test_continuous_commits_four_closes_and_resume_keeps_phase():
    result = queue()
    result.append(route_steps([[0., 0.], [2., 0.]], FEET, 0))
    result.start()
    assert len(result.committed) == 4
    for _ in range(30):
        result.tick(.02, result.scheduler.slots[:2])
    committed = [(phase, step.pose.copy()) for phase, step in result.committed]
    result.request_pause()
    result.tick(.02, result.scheduler.slots[:2])
    assert result.state == 'closing'
    for (phase, step), (expected, pose) in zip(result.committed, committed):
        assert phase == expected
        np.testing.assert_array_equal(step.pose, pose)
    finish(result)
    feet = result.scheduler.slots[:2]
    np.testing.assert_allclose(relative(feet[0], feet[1]), [0., .24, 0.], atol=1e-12)
    frozen = result.phase
    assert result.pending
    result.start()
    assert result.phase == frozen
    assert np.array_equal(result.committed[0][1].pose, feet[result.committed[0][1].side])


def test_queue_exhaustion_with_lagging_feet_auto_pause_and_frozen_standing():
    result = queue()
    result.assumed_landing = False
    result.append(route_steps([[0., 0.], [.6, 0.]], FEET, 0))
    result.start()
    finish(result, FEET)
    assert not result.pending and not result.committed
    np.testing.assert_array_equal(result.scheduler.anchor, FEET[result.scheduler.anchor_side])
    phase, anchor = result.phase, result.scheduler.anchor.copy()
    result.tick(.02, FEET)
    assert result.phase == phase and result.scheduler.frequency == 0.
    np.testing.assert_array_equal(result.scheduler.anchor, anchor)
