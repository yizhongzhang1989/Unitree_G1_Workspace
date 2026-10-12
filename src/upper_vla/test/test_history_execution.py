
from unittest.mock import Mock
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.clock import Clock
from rclpy.time import Time

import numpy as np
import pytest

from upper_vla import vla_node
from upper_vla.control_history import ControlHistory
from upper_vla.record_observation import ObservationBuffer
from upper_vla.vla_node import VlaBridgeNode
from test_control_history import poses
from test_execution_regression import executor_fixture


def node_with_history():
    node = executor_fixture()
    history = ControlHistory()
    node._history = history
    node.control_measurement_spy = Mock(return_value=(poses(-1.), dict(left=.2, right=.3)))
    patch = pytest.MonkeyPatch()
    patch.setattr(node, '_control_measurement', node.control_measurement_spy)
    node._observations = ObservationBuffer(())
    patch.setattr(node, 'get_clock', Mock(return_value=Mock(
        spec=Clock, now=Mock(return_value=Time(nanoseconds=1_100_000_000)))))
    return node, history


def deliver_state(node, stamp, value=1.):
    message = JointState()
    message.header.stamp = Time(nanoseconds=round(stamp * 1e9)).to_msg()
    message.name, message.position = ['sample'], [float(value)]
    VlaBridgeNode._on_joints(node, message)


def test_history_after_atomic_publication_and_limits():
    node, history = node_with_history()
    assert node._chunk is not None
    node._chunk.poses['left'][0, 0] = 1.
    node._active['right'] = False

    def check_publish(command, grip, generation):
        if history.snapshot():
            pytest.fail('history recorded before publication')
        return VlaBridgeNode._publish_control(node, command, grip, generation)

    node.publish_control_spy.side_effect = check_publish
    VlaBridgeNode._on_tick(node)
    assert history.snapshot() == ()
    deliver_state(node, 1.101)
    row, = history.snapshot()
    assert row.action['left'][0] == .02
    assert row.action['right'][0] == 0.
    assert row.state['left'][0] == -1.
    assert row.state_grippers is not None
    assert row.state_grippers['left'] == .2
    assert row.action_grippers is not None
    assert row.action_grippers['left'] == 0.
    assert row.state_stamp == 1.101 and row.action_stamp == 1.1


@pytest.mark.parametrize('failure', ['measure', 'publish'])
def test_failed_publication_never_records_history(failure):
    node, history = node_with_history()
    if failure == 'measure':
        node.control_measurement_spy.side_effect = RuntimeError('stale')
        VlaBridgeNode._on_tick(node)
        deliver_state(node, 1.101)
        assert node.publish_control_spy.call_count == 1
    else:
        node.publish_control_spy.side_effect = RuntimeError('failed')
        with pytest.raises(RuntimeError):
            VlaBridgeNode._on_tick(node)
    assert history.snapshot() == ()


def test_two_nodes_have_isolated_histories():
    first, first_history = node_with_history()
    _, second_history = node_with_history()
    VlaBridgeNode._on_tick(first)
    deliver_state(first, 1.101)
    assert len(first_history.snapshot()) == 1
    assert second_history.snapshot() == ()


def test_thirty_actual_ticks_keep_last_sixteen_pairs_and_wait_does_not_append():
    node, history = node_with_history()
    node._cartesian_limit_enabled = False
    chunk = node._chunk
    assert chunk is not None
    for index in range(30):
        stamp = index / 10
        node.control_measurement_spy.return_value = (
            poses(index), dict(left=index / 100, right=.3))
        pytest.MonkeyPatch().setattr(node, 'get_clock', Mock(return_value=Mock(
            spec=Clock, now=Mock(return_value=Time(nanoseconds=round((stamp + .01) * 1e9))))))
        for side in ('left', 'right'):
            chunk.poses[side][index] = poses(index + 100)[side]
        VlaBridgeNode._on_tick(node)
        deliver_state(node, stamp + .02)
    assert node._chunk is None
    rows = history.snapshot()
    assert len(rows) == 16
    np.testing.assert_allclose([row.state['left'][0] for row in rows], np.arange(14, 30))
    np.testing.assert_allclose([row.action['left'][0] for row in rows], np.arange(114, 130))
    state_grippers = [row.state_grippers for row in rows]
    assert all(grippers is not None for grippers in state_grippers)
    np.testing.assert_allclose([grippers['left'] for grippers in state_grippers if grippers is not None],
                               np.arange(14, 30) / 100)
    for _ in range(20):
        VlaBridgeNode._on_tick(node)
    assert node.publish_control_spy.call_count == 30
    assert [row.action_stamp for row in history.snapshot()] == [row.action_stamp for row in rows]


def test_state_drains_eligible_actions_without_blocking_publication():
    node, history = node_with_history()
    for stamp in (1.1, 1.2, 1.3):
        pytest.MonkeyPatch().setattr(node, 'get_clock', Mock(return_value=Mock(
            spec=Clock, now=Mock(return_value=Time(nanoseconds=round(stamp * 1e9))))))
        VlaBridgeNode._on_tick(node)
    assert node.publish_control_spy.call_count == 3
    assert len(node._pending_control) == 3
    deliver_state(node, 1.)
    assert history.snapshot() == ()
    node.control_measurement_spy.assert_not_called()
    deliver_state(node, 1.2, 2.)
    assert [row.action_stamp for row in history.snapshot()] == [1.1, 1.2]
    assert [row.state_stamp for row in history.snapshot()] == [1.2, 1.2]
    assert len(node._pending_control) == 1
    node.control_measurement_spy.assert_called_once_with({'sample': 2.})
    deliver_state(node, 100.01, 2.)
    deliver_state(node, 100.02, 3.)
    assert node.control_measurement_spy.call_count == 2
    assert history.snapshot()[-1].state_stamp == 100.01
    assert not node._pending_control


def test_first_state_is_fixed_before_slow_fk(monkeypatch):
    node, history = node_with_history()
    clock = Mock(spec=Time, nanoseconds=1_100_000_000)
    monkeypatch.setattr(node, 'get_clock', Mock(return_value=Mock(spec=Clock, now=Mock(return_value=clock))))
    node._observations = ObservationBuffer(())
    node._observations.add('joints', .99, {'sample': 1}, .99)
    node._model, node._base_frame, node._tip_frames = object(), 'torso_link', {}
    del node._control_measurement

    events = []

    def publish(command, grip, generation):
        events.append('command')
        return VlaBridgeNode._publish_control(node, command, grip, generation)

    node.publish_control_spy.side_effect = publish

    def delayed_fk(*args):
        events.append('fk')
        assert args[-1] == {'sample': 2}
        clock.nanoseconds += 50_000_000
        node._observations.add('joints', 1.15, {'sample': 3}, 1.15)
        assert args[-1] == {'sample': 2}
        return poses(4), {}, dict(left=.2, right=.3)

    monkeypatch.setattr(vla_node, 'measured_state', delayed_fk)
    assert node._publish_control(poses(0), dict(left=0., right=0.), 0)
    deliver_state(node, 1.101, 2.)
    assert events == ['command', 'fk']
    row, = history.snapshot()
    assert row.action_stamp == 1.1
    assert row.state_stamp == 1.101
    assert row.state['left'][0] == 4
    assert row.state_grippers == dict(left=.2, right=.3)
    assert node._running.is_set()


def test_pairing_uses_sample_timestamp_not_callback_clock(monkeypatch):
    node, history = node_with_history()
    monkeypatch.setattr(vla_node.time, 'monotonic', lambda: 10.)
    VlaBridgeNode._on_tick(node)
    monkeypatch.setattr(vla_node.time, 'monotonic', lambda: 9.99)
    deliver_state(node, 1.101)
    assert history.snapshot()[0].state_stamp == 1.101


def test_arms_protection_remains_active_while_waiting():
    node, _ = node_with_history()
    VlaBridgeNode._on_tick(node)
    pytest.MonkeyPatch().setattr(node, '_arms_ready', Mock(return_value='estop'))
    VlaBridgeNode._on_tick(node)
    assert not node._running.is_set()
    assert not node._pending_control
    assert node.publish_control_spy.call_count == 1


def test_stop_during_fk_does_not_restore_cleared_history():
    node, history = node_with_history()

    def measure_and_stop(joints):
        _ = joints
        assert node.publish_control_spy.call_count == 1
        node._stop('test stop during FK')
        return poses(4), dict(left=.2, right=.3)

    node.control_measurement_spy.side_effect = measure_and_stop
    assert VlaBridgeNode._publish_control(node, poses(0), dict(left=0., right=0.), 0)
    deliver_state(node, 1.101)
    assert history.snapshot() == ()
    assert not node._running.is_set()


def test_action_published_during_fk_stays_queued_for_next_state():
    node, history = node_with_history()
    VlaBridgeNode._on_tick(node)

    def measure_and_publish(joints):
        _ = joints
        pytest.MonkeyPatch().setattr(node, 'get_clock', Mock(return_value=Mock(
            spec=Clock, now=Mock(return_value=Time(nanoseconds=1_200_000_000)))))
        VlaBridgeNode._on_tick(node)
        return poses(4), dict(left=.2, right=.3)

    node.control_measurement_spy.side_effect = measure_and_publish
    deliver_state(node, 1.15)
    assert node.publish_control_spy.call_count == 2
    assert [row.action_stamp for row in history.snapshot()] == [1.1]
    assert len(node._pending_control) == 1
    node.control_measurement_spy.side_effect = None
    deliver_state(node, 1.25)
    assert [row.action_stamp for row in history.snapshot()] == [1.1, 1.2]
    assert not node._pending_control


@pytest.mark.parametrize('operation', ['stop', 'reset', 'task', 'home'])
def test_pending_state_cannot_restore_history_after_stop(operation):
    node, history = node_with_history()
    history.append(1., poses(2), poses(-1), dict(left=1., right=1.), dict(left=.2, right=.3), 1.)
    VlaBridgeNode._on_tick(node)
    assert node._pending_control
    if operation == 'task':
        node._task = 'old'
        VlaBridgeNode._on_task(node, String(data='new'))
    elif operation == 'reset':
        node._backend = Mock()
        VlaBridgeNode._on_reset(node, Trigger.Request(), Trigger.Response())
    elif operation == 'home':
        node._base_frame = 'torso_link'
        node._tip_frames = {side: f'{side}_gripper_base' for side in ('left', 'right')}
        VlaBridgeNode._on_home(node, Trigger.Request(), Trigger.Response())
    else:
        node._stop('operator stopped')
    deliver_state(node, 1.101)
    assert history.snapshot() == ()
    assert not node._pending_control
    assert node._running.is_set() is (operation == 'task')
    assert not node._publish_control(poses(0), dict(left=0., right=0.), 0)
    node.control_measurement_spy.assert_not_called()


@pytest.mark.parametrize('stamp', [1., 1.1])
def test_nonincreasing_action_clock_stops_and_clears_episode(stamp):
    node, history = node_with_history()
    VlaBridgeNode._on_tick(node)
    generation = node._generation
    pytest.MonkeyPatch().setattr(node, 'get_clock', Mock(return_value=Mock(
        spec=Clock, now=Mock(return_value=Time(nanoseconds=round(stamp * 1e9))))))
    VlaBridgeNode._on_tick(node)
    assert not node._running.is_set()
    assert node.publish_control_spy.call_count == 2
    assert node._generation > generation
    assert not node._pending_control
    assert history.snapshot() == ()


def test_clock_rollback_during_publication_invalidates_episode():
    node, history = node_with_history()
    stamps = iter([1_100_000_000, 1_000_000_000])
    pytest.MonkeyPatch().setattr(node, 'get_clock', Mock(return_value=Mock(
        spec=Clock, now=Mock(side_effect=lambda: Time(nanoseconds=next(stamps))))))
    VlaBridgeNode._on_tick(node)
    assert not node._running.is_set()
    assert node.publish_control_spy.call_count == 1
    assert not node._pending_control
    assert history.snapshot() == ()


def test_joint_clock_rollback_clears_old_pending_and_allows_new_epoch():
    node, history = node_with_history()
    deliver_state(node, 100.)
    VlaBridgeNode._on_tick(node)
    deliver_state(node, 1.)
    assert not node._running.is_set()
    assert not node._pending_control
    assert history.snapshot() == ()
    node._running.set()
    pytest.MonkeyPatch().setattr(node, 'get_clock', Mock(return_value=Mock(
        spec=Clock, now=Mock(return_value=Time(nanoseconds=1_200_000_000)))))
    assert VlaBridgeNode._publish_control(node, poses(0), dict(left=.2, right=.3), node._generation)
    deliver_state(node, 1.21)
    assert history.snapshot()[0].state_stamp == 1.21
