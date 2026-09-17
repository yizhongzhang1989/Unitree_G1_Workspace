import numpy as np
import pytest

from g1_vla_bridge.control_history import ControlHistory


def poses(position):
    return {side: np.array([position, 0., 0., 0., 0., 0., 1.])
            for side in ('left', 'right')}


@pytest.mark.parametrize('count', [0, 3, 15, 22])
def test_real_steps_bounded_paired_and_ordered(count):
    history = ControlHistory()
    for index in range(count):
        history.append(index, poses(index + 100), poses(index))
    snapshot = history.snapshot()
    assert len(snapshot) == min(count, 15)
    for step, index in zip(snapshot, range(max(0, count - 15), count)):
        assert step.action['left'][0] == index + 100
        assert step.state['left'][0] == index


def test_reset_isolation_cutoff_and_snapshot_ownership():
    first, second = ControlHistory(), ControlHistory()
    action, state = poses(100), poses(1)
    first.append(1., action, state)
    first.append(2., poses(200), poses(2))
    snapshot = first.snapshot(before=2.)
    assert len(snapshot) == 1
    action['left'][0] = -1
    state['left'][0] = -1
    assert snapshot[0].action['left'][0] == 100
    snapshot[0].state['left'][0] = -2
    assert first.snapshot()[0].state['left'][0] == 1
    first.clear()
    assert first.snapshot() == second.snapshot() == ()
    assert snapshot[0].action['left'][0] == 100


def test_delayed_observation_still_gets_full_fifteen_step_window():
    history = ControlHistory()
    for index in range(30):
        history.append(index, poses(index + 100), poses(index))
    snapshot = history.snapshot(before=25.)
    assert [step.stamp for step in snapshot] == list(range(10, 25))
    assert len(history.snapshot()) == 15


def test_delayed_queries_survive_more_than_sixty_four_publications():
    history = ControlHistory()
    for index in range(100):
        history.append(index, poses(index), poses(index))
    assert len(history.snapshot()) == 15
    assert [step.stamp for step in history.snapshot(before=10.)] == list(range(10))
    history.clear()
    assert history.snapshot(before=10.) == ()


def test_thirty_hz_publications_produce_ten_hz_actual_pairs():
    history = ControlHistory()
    for index in range(300):
        stamp = 1789886000. + index / 30.
        history.append(stamp, poses(index), poses(-index), state_stamp=stamp + .01)
    rows = history.snapshot()
    np.testing.assert_allclose([row.action['left'][0] for row in rows], np.arange(255, 300, 3))
    np.testing.assert_allclose(np.diff([row.stamp for row in rows]), .1, atol=3e-7)
    rows = history.snapshot(before=1789886001.51)
    assert len(rows) == 15
    assert rows[-1].stamp < 1789886001.51


def test_no_history_is_fabricated_across_gap():
    history = ControlHistory()
    history.append(1., poses(1), poses(1))
    history.append(100., poses(2), poses(2))
    assert [row.stamp for row in history.snapshot()] == [1., 100.]


def test_execution_grippers_survive_wait_and_match_last_fifteen():
    history = ControlHistory()
    for index in range(30):
        history.append(index / 10 + .01, poses(index + 100), poses(index),
                       action_grippers=dict(left=.4, right=.5),
                       state_grippers=dict(left=index / 100, right=.3), state_stamp=index / 10)
    rows = history.snapshot(before=100.)
    assert len(rows) == 15
    for index, row in zip(range(15, 30), rows):
        assert row.action['left'][0] == index + 100
        assert row.state['left'][0] == index
        assert row.state_stamp == index / 10
        assert row.state_grippers['left'] == index / 100
        assert row.action_grippers['left'] == .4
    rows[0].state_grippers['left'] = 99
    assert history.snapshot()[0].state_grippers['left'] == .15
    assert len(history.snapshot(before=1000.)) == 15
    history.clear()
    assert history.snapshot() == ()


def test_postpublication_state_is_recorded_but_not_leaked_into_earlier_observation():
    history = ControlHistory()
    history.append(1., poses(1), poses(1), state_stamp=.99)
    history.append(1.1, poses(2), poses(2), state_stamp=1.105)
    assert len(history.snapshot()) == 2
    assert len(history.snapshot(before=1.103)) == 1
    assert len(history.snapshot(before=1.106)) == 2
