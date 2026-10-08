# pyright: reportArgumentType=false, reportAttributeAccessIssue=false
# pyright: reportOptionalMemberAccess=false, reportOptionalSubscript=false

from types import SimpleNamespace
from unittest.mock import Mock
from sensor_msgs.msg import JointState
from rclpy.time import Time

import numpy as np
import pytest

from g1_vla_bridge import vla_node
from g1_vla_bridge.control_history import ControlHistory
from g1_vla_bridge.record_observation import ObservationBuffer
from g1_vla_bridge.vla_node import VlaBridgeNode
from test_control_history import poses
from test_execution_regression import executor_fixture


def node_with_history():
    node = executor_fixture()
    node._history = ControlHistory()
    node._control_measurement = Mock(return_value=(poses(-1.), dict(left=.2, right=.3)))
    node._observations = ObservationBuffer(())
    node.get_logger = Mock(return_value=Mock())
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_100_000_000))
    return node


def deliver_state(node, stamp, value=1.):
    message = JointState()
    message.header.stamp = Time(nanoseconds=round(stamp * 1e9)).to_msg()
    message.name, message.position = ['sample'], [float(value)]
    VlaBridgeNode._on_joints(node, message)


def test_history_after_both_publications_and_limits():
    node = node_with_history()
    node._chunk.poses['left'][0, 0] = 1.
    node._active['right'] = False
    def check_publish(message):
        _ = message
        if node._history.snapshot():
            pytest.fail('history recorded before both publications')

    node._publisher.publish.side_effect = check_publish
    VlaBridgeNode._on_tick(node)
    assert node._history.snapshot() == ()
    deliver_state(node, 1.101)
    row, = node._history.snapshot()
    assert row.action['left'][0] == .02
    assert row.action['right'][0] == 0.
    assert row.state['left'][0] == -1.
    assert row.state_grippers['left'] == .2
    assert row.action_grippers['left'] == 0.
    assert row.state_stamp == 1.101 and row.action_stamp == 1.1


@pytest.mark.parametrize('failure', ['measure', 'arms', 'grip'])
def test_failed_publication_never_records_history(failure):
    node = node_with_history()
    if failure == 'measure':
        node._control_measurement.side_effect = RuntimeError('stale')
        VlaBridgeNode._on_tick(node)
        deliver_state(node, 1.101)
        assert node._publisher.publish.call_count == 2
    else:
        node._publisher.publish.side_effect = ([RuntimeError('failed')] if failure == 'arms'
                                               else [None, RuntimeError('failed')])
        with pytest.raises(RuntimeError):
            VlaBridgeNode._on_tick(node)
    assert node._history.snapshot() == ()


def test_two_nodes_have_isolated_histories():
    first, second = node_with_history(), node_with_history()
    VlaBridgeNode._on_tick(first)
    deliver_state(first, 1.101)
    assert len(first._history.snapshot()) == 1
    assert second._history.snapshot() == ()


def test_thirty_actual_ticks_keep_last_sixteen_pairs_and_wait_does_not_append():
    node = node_with_history()
    node._cartesian_limit_enabled = False
    for index in range(30):
        stamp = index / 10
        node._control_measurement.return_value = (
            poses(index), dict(left=index / 100, right=.3))
        node.get_clock = lambda stamp=stamp: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=round((stamp + .01) * 1e9)))
        for side in ('left', 'right'):
            node._chunk.poses[side][index] = poses(index + 100)[side]
        VlaBridgeNode._on_tick(node)
        deliver_state(node, stamp + .02)
    assert node._chunk is None
    rows = node._history.snapshot()
    assert len(rows) == 16
    np.testing.assert_allclose([row.state['left'][0] for row in rows], np.arange(14, 30))
    np.testing.assert_allclose([row.action['left'][0] for row in rows], np.arange(114, 130))
    np.testing.assert_allclose([row.state_grippers['left'] for row in rows], np.arange(14, 30) / 100)
    for _ in range(20):
        VlaBridgeNode._on_tick(node)
    assert node._publisher.publish.call_count == 60
    assert [row.action_stamp for row in node._history.snapshot()] == [row.action_stamp for row in rows]


def test_state_drains_eligible_actions_without_blocking_publication():
    node = node_with_history()
    for stamp in (1.1, 1.2, 1.3):
        node.get_clock = lambda stamp=stamp: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=round(stamp * 1e9)))
        VlaBridgeNode._on_tick(node)
    assert node._publisher.publish.call_count == 6
    assert len(node._pending_control) == 3
    deliver_state(node, 1.)
    assert node._history.snapshot() == ()
    node._control_measurement.assert_not_called()
    deliver_state(node, 1.2, 2.)
    assert [row.action_stamp for row in node._history.snapshot()] == [1.1, 1.2]
    assert [row.state_stamp for row in node._history.snapshot()] == [1.2, 1.2]
    assert len(node._pending_control) == 1
    node._control_measurement.assert_called_once_with({'sample': 2.})
    deliver_state(node, 100.01, 2.)
    deliver_state(node, 100.02, 3.)
    assert node._control_measurement.call_count == 2
    assert node._history.snapshot()[-1].state_stamp == 100.01
    assert not node._pending_control


def test_first_state_is_fixed_before_slow_fk(monkeypatch):
    node = node_with_history()
    clock = SimpleNamespace(nanoseconds=1_100_000_000)
    node.get_clock = lambda: SimpleNamespace(now=lambda: clock)
    node._observations = ObservationBuffer(())
    node._observations.add('joints', .99, {'sample': 1}, .99)
    node._model, node._base_frame, node._tip_frames = object(), 'torso_link', {}
    del node._control_measurement

    events = []

    def publish(message):
        events.append('arms' if len(message.data) == 14 else 'grip')

    node._publisher.publish.side_effect = publish

    def delayed_fk(*args):
        events.append('fk')
        assert args[-1] == {'sample': 2}
        clock.nanoseconds += 50_000_000
        node._observations.add('joints', 1.15, {'sample': 3}, 1.15)
        assert args[-1] == {'sample': 2}
        return poses(4), {}, dict(left=.2, right=.3)

    monkeypatch.setattr(vla_node, 'measured_state', delayed_fk)
    assert VlaBridgeNode._publish_control(node, poses(0), dict(left=0., right=0.), 0)
    deliver_state(node, 1.101, 2.)
    assert events == ['arms', 'grip', 'fk']
    row, = node._history.snapshot()
    assert row.action_stamp == 1.1
    assert row.state_stamp == 1.101
    assert row.state['left'][0] == 4
    assert row.state_grippers == dict(left=.2, right=.3)
    assert node._running.is_set()


def test_pairing_uses_sample_timestamp_not_callback_clock(monkeypatch):
    node = node_with_history()
    monkeypatch.setattr(vla_node.time, 'monotonic', lambda: 10.)
    VlaBridgeNode._on_tick(node)
    monkeypatch.setattr(vla_node.time, 'monotonic', lambda: 9.99)
    deliver_state(node, 1.101)
    assert node._history.snapshot()[0].state_stamp == 1.101


def test_arms_protection_remains_active_while_waiting():
    node = node_with_history()
    VlaBridgeNode._on_tick(node)
    node._arms_ready = lambda: 'estop'
    VlaBridgeNode._on_tick(node)
    assert not node._running.is_set()
    assert not node._pending_control
    assert node._publisher.publish.call_count == 2


def test_stop_during_fk_does_not_restore_cleared_history():
    node = node_with_history()

    def measure_and_stop(joints):
        _ = joints
        assert node._publisher.publish.call_count == 2
        node._stop('test stop during FK')
        return poses(4), dict(left=.2, right=.3)

    node._control_measurement.side_effect = measure_and_stop
    assert VlaBridgeNode._publish_control(node, poses(0), dict(left=0., right=0.), 0)
    deliver_state(node, 1.101)
    assert node._history.snapshot() == ()
    assert not node._running.is_set()


def test_action_published_during_fk_stays_queued_for_next_state():
    node = node_with_history()
    VlaBridgeNode._on_tick(node)

    def measure_and_publish(joints):
        _ = joints
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_200_000_000))
        VlaBridgeNode._on_tick(node)
        return poses(4), dict(left=.2, right=.3)

    node._control_measurement.side_effect = measure_and_publish
    deliver_state(node, 1.15)
    assert node._publisher.publish.call_count == 4
    assert [row.action_stamp for row in node._history.snapshot()] == [1.1]
    assert len(node._pending_control) == 1
    node._control_measurement.side_effect = None
    deliver_state(node, 1.25)
    assert [row.action_stamp for row in node._history.snapshot()] == [1.1, 1.2]
    assert not node._pending_control


@pytest.mark.parametrize('operation', ['stop', 'reset', 'task', 'home'])
def test_pending_state_cannot_restore_history_after_stop(operation):
    node = node_with_history()
    node._history.append(1., poses(2), poses(-1), dict(left=1., right=1.), dict(left=.2, right=.3), 1.)
    VlaBridgeNode._on_tick(node)
    assert node._pending_control
    if operation == 'task':
        node._task = 'old'
        VlaBridgeNode._on_task(node, SimpleNamespace(data='new'))
    elif operation == 'reset':
        node._backend = Mock()
        VlaBridgeNode._on_reset(node, None, SimpleNamespace())
    elif operation == 'home':
        node._base_frame = 'torso_link'
        node._tip_frames = {side: f'{side}_gripper_base' for side in ('left', 'right')}
        VlaBridgeNode._on_home(node, None, SimpleNamespace())
    else:
        node._stop('operator stopped')
    deliver_state(node, 1.101)
    assert node._history.snapshot() == ()
    assert not node._pending_control
    assert node._running.is_set() is (operation == 'task')
    assert not node._publish_control(poses(0), dict(left=0., right=0.), 0)
    node._control_measurement.assert_not_called()


@pytest.mark.parametrize('stamp', [1., 1.1])
def test_nonincreasing_action_clock_stops_and_clears_episode(stamp):
    node = node_with_history()
    VlaBridgeNode._on_tick(node)
    generation = node._generation
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(stamp * 1e9)))
    VlaBridgeNode._on_tick(node)
    assert not node._running.is_set()
    assert node._publisher.publish.call_count == 2
    assert node._generation > generation
    assert not node._pending_control
    assert node._history.snapshot() == ()


def test_clock_rollback_during_publication_invalidates_episode():
    node = node_with_history()
    stamps = iter([1_100_000_000, 1_000_000_000])
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=next(stamps)))
    VlaBridgeNode._on_tick(node)
    assert not node._running.is_set()
    assert node._publisher.publish.call_count == 2
    assert not node._pending_control
    assert node._history.snapshot() == ()


def test_joint_clock_rollback_clears_old_pending_and_allows_new_epoch():
    node = node_with_history()
    deliver_state(node, 100.)
    VlaBridgeNode._on_tick(node)
    deliver_state(node, 1.)
    assert not node._running.is_set()
    assert not node._pending_control
    assert node._history.snapshot() == ()
    node._running.set()
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_200_000_000))
    assert VlaBridgeNode._publish_control(node, poses(0), dict(left=.2, right=.3), node._generation)
    deliver_state(node, 1.21)
    assert node._history.snapshot()[0].state_stamp == 1.21
