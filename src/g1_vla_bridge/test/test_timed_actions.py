"""No ROS nodes, HTTP requests or hardware commands."""

import numpy as np
import pytest

from g1_vla_bridge.timed_actions import TimedActions
from g1_vla_bridge.vla_backend import ActionChunk, SIDES


def chunk(position=0., grip=0.):
    poses = np.tile([position, 0., 0., 0., 0., 0., 1.], (30, 1))
    return ActionChunk(poses={side: poses.copy() for side in SIDES},
                       grippers={side: np.full(30, grip) for side in SIDES})


def test_two_requests_overlap_on_execution_time():
    queue = TimedActions(30., 0.25, 0.)
    assert queue.merge(chunk(0.), 0., 0.2) == 24
    assert queue.last_merge['overlap'] == 0
    assert queue.last_merge['new'] == 24
    assert sorted(queue.samples) == list(range(6, 30))
    for tick in range(6, 12):
        assert queue.take(tick / 30)[0]['left'][0] == 0.
    assert queue.merge(chunk(4., 1.), 0.2, 0.4) == 24
    assert queue.last_merge['accepted'] == 24
    assert queue.last_merge['overlap'] == 18
    assert queue.last_merge['new'] == 6
    assert queue.last_merge['responses'] == 2
    assert queue.last_merge['overlap_total'] == 18
    assert queue.last_merge['new_total'] == 30
    assert queue.last_merge['observation_to_response_s'] == pytest.approx(.2)
    assert queue.last_merge['prediction_remaining_s'] == pytest.approx(.8)
    assert queue.last_merge['first_action_offset_s'] == pytest.approx(.2)
    assert sorted(queue.samples) == list(range(12, 36))
    for tick in range(12, 30):
        assert queue.samples[tick][0]['left'][0] == 1.
        assert queue.samples[tick][1]['left'] == 1.
    for tick in range(30, 36):
        assert queue.samples[tick][0]['left'][0] == 4.
    queue.merge(chunk(8.), 0.4, 0.6)
    assert queue.samples[18][0]['left'][0] == 2.75


def test_fractional_request_time_interpolates_on_shared_grid():
    queue = TimedActions(30., 0.5, 10.)
    prediction = chunk()
    for side in SIDES:
        prediction.poses[side][:, 0] = np.arange(30)
    queue.merge(prediction, 10.01, 10.215)
    assert min(queue.samples) == 7
    assert queue.samples[7][0]['left'][0] == pytest.approx(6.7)


def test_expired_results_and_delayed_ticks_never_replay():
    queue = TimedActions(30., 0.5, 0.)
    assert queue.merge(chunk(), 0., 1.1) == 0
    assert queue.last_merge['accepted'] == 0
    assert queue.last_merge['first_action_offset_s'] is None
    assert queue.last_merge['prediction_remaining_s'] == 0.
    assert queue.end == 0.
    queue.merge(chunk(2.), 1.1, 1.3)
    assert queue.take(1.5)[0]['left'][0] == 2.
    assert queue.take(1.5) is None
    assert queue.take(2.2) is None
    assert not queue.samples


def test_constant_predictions_cannot_self_amplify_through_ema():
    queue = TimedActions(30., .5, 0.)
    prediction = chunk(.02, 1.)
    for requested in np.arange(0., 2., .1):
        queue.merge(prediction, float(requested), float(requested + .2))
        for poses, grippers in queue.samples.values():
            for side in SIDES:
                np.testing.assert_allclose(poses[side], prediction.poses[side][0], atol=1e-14)
                assert grippers[side] == 1.


def test_already_consumed_tick_is_not_reinserted():
    queue = TimedActions(30., 0.5, 0.)
    queue.merge(chunk(), 0., 0.2)
    queue.take(0.4)
    queue.merge(chunk(3.), 0.2, 0.4)
    assert min(queue.samples) == 13


def test_rotation_uses_shortest_arc():
    queue = TimedActions(30., 0.5, 0.)
    queue.merge(chunk(), 0., 0.2)
    prediction = chunk()
    for side in SIDES:
        prediction.poses[side][:, 3:] = [0., 0., 1., 0.]
    queue.merge(prediction, 0.2, 0.4)
    assert np.allclose(np.abs(queue.samples[12][0]['left'][3:]),
                       [0., 0., 2 ** -0.5, 2 ** -0.5])


@pytest.mark.parametrize(('rate', 'alpha'), [
    (0., .5), (30., 0.), (30., 1.1), (float('nan'), .5), (30., float('nan'))])
def test_invalid_configuration(rate, alpha):
    with pytest.raises(ValueError):
        TimedActions(rate, alpha, 0.)
