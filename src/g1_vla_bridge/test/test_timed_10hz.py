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
    queue = TimedActions(10., 1., 0., first_offset_steps=1)
    assert queue.merge(chunk(), 0., 0.) == 30
    assert queue.take(0.) is None
    assert queue.take(.1)[0]['left'][0] == 1
    assert queue.take(3.)[0]['left'][0] == 30
    assert queue.end == 3.


def test_ten_hz_prediction_interpolated_on_thirty_hz_grid():
    queue = TimedActions(10., 1., 0., first_offset_steps=1, execution_rate=30.)
    prediction = chunk()
    for side in SIDES:
        prediction.poses[side][1, 3:] = [0., 0., 1., 0.]
    assert queue.merge(prediction, 0., 0.) == 88
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
    queue = TimedActions(10., 1., 0., first_offset_steps=1)
    queue.merge(chunk(), 0., .37)
    assert queue.take(3.01) is None


def test_latency_drops_expired_rows_without_retiming():
    queue = TimedActions(10., 1., 0., first_offset_steps=1)
    assert queue.merge(chunk(), 0., .37) == 27
    assert queue.take(.37) is None
    assert queue.take(.4)[0]['left'][0] == pytest.approx(4.)
    assert queue.last_merge['prediction_remaining_s'] == pytest.approx(2.63)
    assert queue.merge(chunk(), 0., 3.01) == 0
    assert queue.end == 3.
