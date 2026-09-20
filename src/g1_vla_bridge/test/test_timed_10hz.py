import numpy as np
import pytest

from g1_vla_bridge.timed_actions import TimedActions
from g1_vla_bridge.vla_backend import ActionChunk, SIDES


def chunk():
    poses = np.zeros((30, 7))
    poses[:, 0] = np.arange(1, 31)
    poses[:, 6] = 1.
    return ActionChunk({side: poses.copy() for side in SIDES},
                       {side: np.arange(1, 31) for side in SIDES})


def test_observation_relative_first_and_last():
    queue = TimedActions(10., 0., first_offset_steps=1)
    assert queue.merge(chunk(), 0., 0.) == 30
    assert queue.take(0.) is None
    assert queue.take(.1)[0]['left'][0] == 1
    assert queue.take(3.)[0]['left'][0] == 30
    assert queue.end == 3.


def test_ten_hz_prediction_interpolated_on_thirty_hz_grid():
    queue = TimedActions(10., 0., first_offset_steps=1, execution_rate=30.)
    prediction = chunk()
    for side in SIDES:
        prediction.poses[side][1, 3:] = [0., 0., 1., 0.]
    assert queue.merge(prediction, 0., 0.) == 30
    assert queue.take(2 / 30.) is None
    values = [queue.take(tick / 30.) for tick in range(3, 91)]
    np.testing.assert_allclose([value[0]['left'][0] for value in values],
                               np.arange(3, 91) / 3.)
    np.testing.assert_allclose([value[1]['left'] for value in values],
                               np.arange(3, 91) / 3.)
    np.testing.assert_allclose(values[1][0]['left'][3:],
                               [0., 0., .5, np.sqrt(3) / 2.], atol=1e-12)
    assert queue.end == 3.
    assert queue.take(3.01) is None


def test_late_final_tick_is_not_executed():
    queue = TimedActions(10., 0., first_offset_steps=1)
    queue.merge(chunk(), 0., .37)
    assert queue.take(3.01) is None


def test_model_grid_blending_is_independent_of_execution_rate():
    queues = [TimedActions(10., 0., first_offset_steps=1, execution_rate=rate)
              for rate in (10., 30.)]
    for queue in queues:
        queue.merge(chunk(), 0., .2)
        incoming = chunk()
        for side in SIDES:
            incoming.poses[side][:, 0] += 10.
            incoming.grippers[side] = incoming.grippers[side] + 20.
        queue.merge(incoming, .2, .4)
        assert queue.last_merge['overlap'] == 27
        assert queue.last_merge['accepted'] == 29
        for index, tick in enumerate(range(4, 31), 1):
            weight = index / 28.
            for side in SIDES:
                assert queue.samples[tick][0][side][0] == pytest.approx(tick + 8 * weight)
                assert queue.samples[tick][1][side] == pytest.approx(tick + 18)
    assert queues[0].samples.keys() == queues[1].samples.keys()
    for tick in queues[0].samples:
        for side in SIDES:
            np.testing.assert_array_equal(queues[0].samples[tick][0][side],
                                          queues[1].samples[tick][0][side])
            assert queues[0].samples[tick][1][side] == queues[1].samples[tick][1][side]
    start, finish = queues[1].samples[4], queues[1].samples[5]
    actual = queues[1].take(13 / 30.)
    for side in SIDES:
        np.testing.assert_allclose(actual[0][side][:3],
                                   (2 * start[0][side][:3] + finish[0][side][:3]) / 3.)
        assert actual[1][side] == pytest.approx((2 * start[1][side] + finish[1][side]) / 3.)


def test_latency_drops_expired_rows_without_retiming():
    queue = TimedActions(10., 0., first_offset_steps=1)

    assert queue.merge(chunk(), 0., .37) == 27
    assert queue.take(.37) is None
    assert queue.take(.4)[0]['left'][0] == pytest.approx(4.)
    assert queue.last_merge['prediction_remaining_s'] == pytest.approx(2.63)
    assert queue.merge(chunk(), 0., 3.01) == 0
    assert queue.end == 3.


def test_blend_after_execution_keeps_only_model_nodes_and_interpolates_them():
    queue = TimedActions(10., 0., first_offset_steps=1, execution_rate=30.)
    queue.merge(chunk(), 0., 0.)
    queue.take(.4)
    incoming = chunk()
    for side in SIDES:
        incoming.poses[side][:, 0] += 10.
        incoming.grippers[side] = incoming.grippers[side] + 20.
    queue.merge(incoming, .2, .41)
    assert queue.last_merge['overlap'] == 26
    assert queue.last_merge['accepted'] == 28
    assert queue.samples[4][0]['left'][0] == 4.
    assert queue.samples[5][0]['left'][0] == pytest.approx(5 + 8 / 27.)
    assert queue.samples[5][1]['left'] == pytest.approx(23.)
    assert queue.take(.4) is None
    output = queue.take(13 / 30.)
    assert output[0]['left'][0] == pytest.approx(4 + (1 + 8 / 27.) / 3.)
    assert output[1]['left'] == pytest.approx(4 + 19 / 3.)
