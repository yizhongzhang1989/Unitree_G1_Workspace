"""One held measured state drives poses, extrinsics and grippers."""


import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from rclpy.time import Time
from sensor_msgs.msg import Image

from upper_vla import vla_node
from upper_vla.control_history import ControlHistory
from upper_vla.record_observation import ObservationBuffer, record_tool
from upper_vla.vla_backend import ImageSpec, VlaSpec
from upper_vla.vla_node import VlaBridgeNode


@pytest.fixture
def observation_node(monkeypatch):
    monkeypatch.setattr(vla_node.time, 'time', lambda: 12.)
    monkeypatch.setattr(vla_node.time, 'monotonic', lambda: 30.)
    model = record_tool('urdf_fk').RobotModel.from_urdf('''<robot name="test">
      <joint name="slide" type="prismatic"><parent link="torso_link"/>
        <child link="tip"/><axis xyz="1 0 0"/></joint>
      <joint name="camera" type="fixed"><parent link="tip"/>
        <child link="optical"/><origin xyz="0 1 0"/></joint>
    </robot>''')
    buffer = ObservationBuffer(('head',), rate=30.)
    for stamp, pixel, joint in ((11.80, 1, .2), (11.85, 2, .4), (11.95, 3, .9)):
        message = Image()
        message.header.stamp = Time(nanoseconds=int(stamp * 1e9)).to_msg()
        message.height, message.width, message.step = 1, 1, 3
        message.encoding, message.data = 'bgr8', bytes([pixel] * 3)
        buffer.add('head', stamp, message, 29.9)
        if stamp < 11.9:
            joints = {'slide': joint, 'left_eccentric_joint': 1.2, 'right_eccentric_joint': 2.3}
            buffer.add('joints', stamp, joints, 29.9)
    node = VlaBridgeNode.__new__(VlaBridgeNode)
    node._lock = threading.Lock()
    node._observations = buffer
    node._model = model
    node._history = None
    node._generation = 0
    node._spec = VlaSpec(name='observation_test', images=ImageSpec(slots=('head',)))
    node._base_frame = 'torso_link'
    node._tip_frames = {'left': 'tip', 'right': 'tip'}
    node._camera_frames = {'head': 'optical'}
    node._camera_info = {}
    node._task = 'test'
    node._enabled = {'left': True, 'right': True}
    node._grip_command = {'left': 0., 'right': 0.}
    tf_spy = Mock()
    node._tf = tf_spy
    monkeypatch.setattr(node, 'get_clock', Mock(return_value=Mock(now=Mock(return_value=Time(seconds=12)))))
    lookup_spy = Mock(return_value=np.array([.4, 1., 0., 0., 0., 0., 1.]))
    monkeypatch.setattr(node, '_lookup', lookup_spy)
    tf_spy.lookup_transform.return_value = SimpleNamespace(
        header=SimpleNamespace(stamp=Time(nanoseconds=int(11.9 * 1e9)).to_msg()))
    return node, lookup_spy, tf_spy


def test_image_callback_recovers_on_timestamp_rollback(observation_node):
    node, _, _ = observation_node
    callback = VlaBridgeNode._make_image_callback(node, 'head')
    for stamp in (11.98, 11.96):
        message = Image()
        message.header.stamp = Time(nanoseconds=int(stamp * 1e9)).to_msg()
        callback(message)
    latest = node._observations.images['head'][-1]
    assert latest.stamp == pytest.approx(11.96)
    assert len(node._observations.images['head']) == 1
    assert latest.received == 30.


def test_one_held_joint_sample_for_tips_cameras_and_grippers(observation_node):
    node, lookup_spy, tf_spy = observation_node
    observation = VlaBridgeNode._observe(node)
    np.testing.assert_array_equal(observation.images['head'], 2)
    np.testing.assert_allclose(observation.poses['left'], [.4, 0, 0, 0, 0, 0, 1])
    np.testing.assert_allclose(observation.camera_poses['head'][:3, 3], [.4, 1, 0])
    assert observation.grippers == {'left': 1.2, 'right': 2.3}
    assert observation.acquired_monotonic == pytest.approx(29.85)
    lookup_call = lookup_spy.call_args
    assert lookup_call is not None
    assert lookup_call.args[1].nanoseconds == 11_850_000_000
    tf_spy.lookup_transform.assert_called_once()


def test_snapshot_is_immutable_during_decode(observation_node, monkeypatch):
    node, _, _ = observation_node
    decode = vla_node.image_to_bgr

    def replace(message):
        node._observations.add('joints', 11.99, {'slide': 99.}, 30.)
        node._task = 'changed'
        return decode(message)

    monkeypatch.setattr(vla_node, 'image_to_bgr', replace)
    observation = VlaBridgeNode._observe(node)
    assert observation.task == 'test'
    assert observation.poses['left'][0] == .4


def test_missing_joint_never_falls_back_to_tf(observation_node):
    node, lookup_spy, _ = observation_node
    del node._observations.joints[-1].value['slide']
    with pytest.raises(KeyError):
        VlaBridgeNode._observe(node)
    lookup_spy.assert_not_called()


def test_old_observation_and_slow_processing_are_not_rejected(observation_node, monkeypatch):
    node, _, _ = observation_node
    monkeypatch.setattr(node, 'get_clock', Mock(return_value=Mock(now=Mock(return_value=Time(seconds=1000)))))
    clock = SimpleNamespace(now=2000.)
    monkeypatch.setattr(vla_node.time, 'monotonic', lambda: clock.now)
    decode = vla_node.image_to_bgr

    def slow_decode(message):
        clock.now += 100.
        return decode(message)

    monkeypatch.setattr(vla_node, 'image_to_bgr', slow_decode)
    observation = VlaBridgeNode._observe(node)
    assert observation.poses['left'][0] == .4
    timing = node._observation_timing
    assert timing is not None
    assert timing.ages_s['head'] > 900.
    assert clock.now == 2100.


@pytest.mark.parametrize('failure', ['missing', 'description'])
def test_invalid_inputs_fail_closed(observation_node, failure):
    node, _, _ = observation_node
    if failure == 'missing':
        node._observations.reset_camera('head')
    else:
        node._model = None
    with pytest.raises(RuntimeError):
        VlaBridgeNode._observe(node)


def test_observe_copies_execution_history_without_resampling(observation_node, monkeypatch):
    node, _, _ = observation_node
    history = ControlHistory()
    node._history = history
    node._observations.rate = 10.
    for index in range(30):
        measured = {side: np.array([float(index), 0., 0., 0., 0., 0., 1.])
                    for side in ('left', 'right')}
        command = {side: np.array([index + 100., 0., 0., 0., 0., 0., 1.])
                   for side in ('left', 'right')}
        history.append(
            index / 10 + .01, command, measured,
            dict(left=.4, right=.5), dict(left=.2, right=.3), index / 10)
    compute = Mock(wraps=vla_node.measured_state)
    monkeypatch.setattr(vla_node, 'measured_state', compute)
    observation = VlaBridgeNode._observe(node)
    compute.assert_called_once()
    assert observation.poses['left'][0] == .4
    assert len(observation.history) == 16
    state_stamps = []
    for row in observation.history:
        assert row.state_stamp is not None
        state_stamps.append(row.state_stamp)
    np.testing.assert_allclose(state_stamps, np.arange(14, 30) / 10)
    np.testing.assert_allclose([row.action_stamp for row in observation.history],
                               np.arange(14, 30) / 10 + .01)
    assert observation.history[-1].state['left'][0] == 29
    assert observation.history[-1].action['left'][0] == 129
    last = observation.history[-1]
    assert last.state_grippers is not None
    assert last.action_grippers is not None
    assert last.state_grippers['left'] == .2
    assert last.action_grippers['left'] == .4
