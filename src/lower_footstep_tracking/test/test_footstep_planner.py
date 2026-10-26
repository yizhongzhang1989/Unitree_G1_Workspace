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
    landed = []
    for _ in range(1000):
        previous = result.phase
        anchor_id = result.scheduler.anchor_id
        result.tick(.02, result.scheduler.slots[:2] if feet is None else feet)
        if result.scheduler.anchor_id != anchor_id:
            assert result.scheduler.anchor_id == anchor_id + 1
            side = result.scheduler.anchor_side
            landed.append((side, result.scheduler.slots[side].copy()))
        if result.state == 'standing':
            assert result.scheduler.frequency == 0.
            return landed
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
    assert result.phase - first == pytest.approx((2 if align else 1) * math.pi)
    assert result.scheduler.frequency == 0.
    frozen = result.phase
    result.tick(.02, FEET)
    assert result.phase == frozen


def test_continuous_pause_finishes_only_committed_and_resume_keeps_phase():
    result = queue()
    result.append(route_steps([[0., 0.], [2., 0.]], FEET, 0))
    result.start()
    assert len(result.committed) == 4
    for _ in range(30):
        result.tick(.02, result.scheduler.slots[:2])
    committed = [(phase, step.side, step.pose.copy()) for phase, step in result.committed]
    result.request_pause()
    result.tick(.02, result.scheduler.slots[:2])
    assert result.state == 'stopping'
    assert len(result.committed) == len(committed)
    for (phase, step), (expected, side, pose) in zip(result.committed, committed):
        assert phase == expected
        assert step.side == side
        np.testing.assert_array_equal(step.pose, pose)
    landed = finish(result)
    assert [side for side, _ in landed] == [side for _, side, _ in committed]
    for (_, pose), (_, _, expected) in zip(landed, committed):
        np.testing.assert_array_equal(pose, expected)
    frozen = result.phase
    assert result.pending
    pending = list(result.pending)
    result.start()
    assert result.phase == frozen
    assert result.committed[0][1].side == pending[0].side
    np.testing.assert_array_equal(result.committed[0][1].pose, pending[0].pose)
    landed = finish(result)
    assert [side for side, _ in landed] == [step.side for step in pending]
    for (_, pose), step in zip(landed, pending):
        np.testing.assert_array_equal(pose, step.pose)


@pytest.mark.parametrize('count', [1, 2, 3, 4, 5, 9])
@pytest.mark.parametrize('frequency', [.8, 1.2, 1.8])
@pytest.mark.parametrize('first_side', [0, 1])
def test_continuous_executes_exactly_planned_landings(count, frequency, first_side):
    result = queue()
    steps = [Footstep((first_side + index) % 2,
                      np.array((.1 * (index + 1),
                                .12 if (first_side + index) % 2 == 0 else -.12, 0.)))
             for index in range(count)]
    result.append(steps)
    result.start(frequency)
    landed = finish(result)
    assert [side for side, _ in landed] == [step.side for step in steps]
    for (_, pose), step in zip(landed, steps):
        np.testing.assert_array_equal(pose, step.pose)
    np.testing.assert_array_equal(result.scheduler.slots[:2], validate_steps(steps, FEET))
    assert not result.pending and not result.committed
    phase, anchor_id = result.phase, result.scheduler.anchor_id
    for _ in range(100):
        result.tick(.02, result.scheduler.slots[:2])
    assert result.phase == phase and result.scheduler.anchor_id == anchor_id


@pytest.mark.parametrize('kind,count', [
    ('continuous', 1), ('continuous', 2), ('continuous', 9),
    ('single', 1), ('single', 2), ('pause', 9), ('clear', 9)])
@pytest.mark.parametrize('frequency', [.8, 1.2, 1.8])
@pytest.mark.parametrize('first_side', [0, 1])
def test_final_touchdown_sets_zero_frequency_in_the_same_tick(kind, count, frequency, first_side):
    result = queue()
    steps = [Footstep((first_side + index) % 2,
                      np.array((.1 * (index + 1),
                                .12 if (first_side + index) % 2 == 0 else -.12, 0.)))
             for index in range(count)]
    result.append(steps, kind='single' if kind == 'single' else 'continuous')
    result.start(frequency)
    final_phase = result.phase + count * math.pi
    if kind in ('pause', 'clear'):
        for _ in range(10):
            result.tick(.02, result.scheduler.slots[:2])
        final_phase = result.committed[-1][0]
        if kind == 'pause':
            result.request_pause()
        else:
            result.replace_pending([])
    for _ in range(1000):
        result.tick(.02, result.scheduler.slots[:2])
        assert result.phase <= final_phase + 1e-9
        if not result.committed:
            assert result.state == 'standing'
            assert result.scheduler.frequency == 0.
            assert result.phase == pytest.approx(final_phase, abs=1e-12)
            assert result.scheduler.phase == pytest.approx(final_phase % math.tau, abs=1e-12)
            assert any(low <= result.scheduler.phase < high for low, high in result.scheduler.windows)
            break
    else:
        raise AssertionError('Final touchdown was not reached')
    for _ in range(50):
        result.tick(.02, result.scheduler.slots[:2])
        assert result.scheduler.frequency == 0.
        assert result.phase == pytest.approx(final_phase, abs=1e-12)


def test_continuous_resume_does_not_insert_stationary_landing():
    result = queue()
    for count in (1, 2, 5, 1):
        side = result.next_side()
        feet = result.tail_feet()
        steps = [Footstep((side + index) % 2,
                          np.array((feet[:, 0].max() + .1 * (index + 1),
                                    .12 if (side + index) % 2 == 0 else -.12, 0.)))
                 for index in range(count)]
        result.append(steps)
        phase = result.phase
        has_walked = result.has_walked
        result.start(1.8)
        if has_walked:
            assert result.phase == phase
        landed = finish(result)
        assert [side for side, _ in landed] == [step.side for step in steps]
        for (_, pose), step in zip(landed, steps):
            np.testing.assert_array_equal(pose, step.pose)


def test_continuous_wrong_resume_side_is_rejected_without_inserting_steps():
    result = queue()
    result.append([Footstep(0, np.array((.1, .12, 0.)))])
    result.start()
    finish(result)
    result.append([Footstep(0, np.array((.2, .12, 0.)))])
    pending, phase = list(result.pending), result.phase
    with pytest.raises(ValueError, match='next swing side'):
        result.start()
    assert result.state == 'standing' and result.phase == phase
    assert list(result.pending) == pending and not result.committed


@pytest.mark.parametrize('count', [1, 2, 9])
@pytest.mark.parametrize('ticks', [1, 15, 30])
def test_pause_during_start_or_walk_does_not_add_landings(count, ticks):
    result = queue()
    steps = [Footstep(index % 2, np.array((.1 * (index + 1), .12 if index % 2 == 0 else -.12, 0.)))
             for index in range(count)]
    result.append(steps)
    result.start(1.8)
    for _ in range(ticks):
        result.tick(.02, result.scheduler.slots[:2])
    committed, pending = list(result.committed), list(result.pending)
    result.request_pause()
    landed = finish(result)
    assert [side for side, _ in landed] == [step.side for _, step in committed]
    for (_, pose), (_, step) in zip(landed, committed):
        np.testing.assert_array_equal(pose, step.pose)
    assert list(result.pending) == pending


@pytest.mark.parametrize('ticks', [5, 30])
def test_appending_during_short_queue_start_or_stop_never_inserts_steps(ticks):
    result = queue()
    result.append([Footstep(0, np.array((.1, .12, 0.)))])
    result.start()
    for _ in range(ticks):
        result.tick(.02, result.scheduler.slots[:2])
    committed = list(result.committed)
    steps = [Footstep(1, np.array((.2, -.12, 0.))), Footstep(0, np.array((.3, .12, 0.)))]
    result.append(steps)
    landed = finish(result)
    expected = [step for _, step in committed] + (steps if ticks == 5 else [])
    assert [side for side, _ in landed] == [step.side for step in expected]
    for (_, pose), step in zip(landed, expected):
        np.testing.assert_array_equal(pose, step.pose)
    if ticks == 30:
        assert [step.json() for step in result.pending] == [step.json() for step in steps]
        result.start()
        assert len(finish(result)) == len(steps)
    assert not result.pending and not result.committed


def test_short_queue_lowers_peak_frequency_instead_of_adding_a_step():
    result = queue()
    result.append([Footstep(0, np.array((.1, .12, 0.)))])
    result.start(1.8)
    phase = result.phase
    frequencies = []
    for _ in range(50):
        result.tick(.02, result.scheduler.slots[:2])
        frequencies.append(result.scheduler.frequency)
    assert result.state == 'standing'
    assert result.phase - phase == pytest.approx(math.pi)
    assert max(frequencies) < 1.8
    assert np.max(np.abs(np.diff([0., *frequencies]))) <= 1.8 * .02 / .5 + 1e-9
    assert result.scheduler.anchor_side == 0


def test_clearing_pending_stops_at_last_committed_footstep():
    result = queue()
    result.append(route_steps([[0., 0.], [2., 0.]], FEET, 0))
    result.start()
    committed = list(result.committed)
    result.replace_pending([])
    landed = finish(result)
    assert [side for side, _ in landed] == [step.side for _, step in committed]
    for (_, pose), (_, step) in zip(landed, committed):
        np.testing.assert_array_equal(pose, step.pose)
    assert not result.pending and not result.committed


@pytest.mark.parametrize('frequency', [.8, 1.2, 1.8])
@pytest.mark.parametrize('dt', [.01, .02])
def test_continuous_stop_matches_exact_coast_and_linear_deceleration(frequency, dt):
    result = queue()
    result.append(route_steps([[0., 0.], [2., 0.]], FEET, 0))
    result.start(frequency)
    for _ in range(round(.6 / dt)):
        result.tick(dt, result.scheduler.slots[:2])
    result.request_pause()
    result.tick(dt, result.scheduler.slots[:2])
    assert result.state == 'stopping'
    phase, final_phase = result.phase, result.committed[-1][0]
    frequency = result.scheduler.frequency
    coast = (final_phase - phase) / (math.tau * frequency) - .25
    assert coast >= 0.

    def area(time):
        time = min(time, coast + .5)
        tail = max(0., time - coast)
        return frequency * (time - tail * tail)

    for index in range(1000):
        result.tick(dt, result.scheduler.slots[:2])
        time = (index + 1) * dt
        expected_phase = phase + math.tau * area(time)
        assert result.phase == pytest.approx(expected_phase, abs=1e-11)
        if result.state == 'standing':
            assert result.scheduler.frequency == 0.
            assert result.phase == pytest.approx(final_phase, abs=1e-12)
            break
        expected_frequency = (area(time) - area(index * dt)) / dt
        assert result.scheduler.frequency == pytest.approx(expected_frequency, abs=1e-11)
    else:
        raise AssertionError('Continuous stop did not finish')


def test_active_queue_refreshes_preview_only_once_per_control_tick(monkeypatch):
    result = queue()
    calls = []
    preview = result._preview

    def counted_preview():
        calls.append(None)
        preview()

    monkeypatch.setattr(result, '_preview', counted_preview)
    result.append(route_steps([[0., 0.], [1., 0.]], FEET, 0))
    result.start()
    assert len(calls) == 1
    for _ in range(1000):
        before = len(calls)
        result.tick(.02, result.scheduler.slots[:2])
        assert len(calls) == before + 1
        if result.state == 'standing':
            break
    else:
        raise AssertionError('Queue did not finish')


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
