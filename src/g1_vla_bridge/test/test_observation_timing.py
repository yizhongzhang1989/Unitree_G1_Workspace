"""Verify observation timing without ROS nodes, inference, or publishers."""

import math
import threading
import time
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, ExtrapolationException

from g1_vla_bridge import vla_node
from g1_vla_bridge.vla_backend import SIDES
from g1_vla_bridge.vla_node import VlaBridgeNode


def image_message(stamp_ns, pixel):
    message = Image()
    message.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
    message.header.frame_id = 'driver_frame'
    message.height, message.width = 1, 1
    message.encoding, message.step = 'bgr8', 3
    message.data = bytes([pixel] * 3)
    return message


@pytest.fixture
def observation_node():
    slots = ('head', 'left_wrist', 'right_wrist')
    node = SimpleNamespace(
        _lock=threading.Lock(),
        _spec=SimpleNamespace(images=SimpleNamespace(slots=slots)),
        _image_timeout=3.0,
        _images={
            slot: (time.monotonic(), image_message(10_500_000_000 + index * 500_000_000,
                                                  index + 1))
            for index, slot in enumerate(slots)
        },
        _camera_info={},
        _camera_frames={slot: f'{slot}_optical' for slot in slots},
        _tip_frames={side: f'{side}_tip' for side in SIDES},
        _base_frame='torso_link',
        _task='test task',
        _grip_command={'left': 0.1, 'right': 0.2},
        _enabled={side: True for side in SIDES},
    )
    buffer = Buffer()
    for child in (*node._camera_frames.values(), *node._tip_frames.values()):
        for seconds, position, angle in ((10, 0.0, 0.0), (12, 2.0, math.pi / 2)):
            transform = TransformStamped()
            transform.header.frame_id = node._base_frame
            transform.header.stamp = Time(seconds=seconds).to_msg()
            transform.child_frame_id = child
            transform.transform.translation.x = position
            transform.transform.rotation.y = math.sin(angle / 2)
            transform.transform.rotation.w = math.cos(angle / 2)
            buffer.set_transform(transform, 'test')
    node._tf = SimpleNamespace(lookup_transform=Mock(wraps=buffer.lookup_transform))
    for name in ('_decode_images', '_lookup', '_measured_pose', '_camera_calibrations'):
        setattr(node, name, MethodType(getattr(VlaBridgeNode, name), node))
    return node


def test_cameras_use_decoded_image_timestamps_but_tips_stay_latest(
        observation_node, monkeypatch):
    node = observation_node
    decode = vla_node.image_to_bgr

    def decode_and_replace_images(message):
        for slot in node._spec.images.slots:
            VlaBridgeNode._make_image_callback(node, slot)(
                image_message(12_000_000_000, 99))
        with node._lock:
            node._grip_command = {'left': 0.3, 'right': 0.4}
        return decode(message)

    monkeypatch.setattr(vla_node, 'image_to_bgr', decode_and_replace_images)
    observation = VlaBridgeNode._observe(node)

    for index, slot in enumerate(node._spec.images.slots):
        np.testing.assert_array_equal(observation.images[slot], index + 1)
        position = 0.5 + index * 0.5
        angle = position * math.pi / 4
        expected = np.array([
            [math.cos(angle), 0.0, math.sin(angle), position],
            [0.0, 1.0, 0.0, 0.0],
            [-math.sin(angle), 0.0, math.cos(angle), 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        np.testing.assert_allclose(observation.camera_poses[slot], expected, atol=1e-12)
        assert node._images[slot][1].data[0] == 99
    for side in SIDES:
        np.testing.assert_allclose(
            observation.poses[side],
            [2.0, 0.0, 0.0, 0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4)],
            atol=1e-12)
    assert [(call.args[1], call.args[2].nanoseconds)
            for call in node._tf.lookup_transform.call_args_list] == [
        ('head_optical', 10_500_000_000),
        ('left_wrist_optical', 11_000_000_000),
        ('right_wrist_optical', 11_500_000_000),
        ('left_tip', 0),
        ('right_tip', 0),
    ]
    assert observation.task == node._task
    assert observation.grippers == node._grip_command
    assert observation.enabled == node._enabled


@pytest.mark.parametrize('stamp_ns', [9_000_000_000, 13_000_000_000])
def test_missing_historical_camera_tf_does_not_fall_back_to_latest(
        observation_node, stamp_ns):
    node = observation_node
    node._images['head'] = (time.monotonic(), image_message(stamp_ns, 1))

    with pytest.raises(ExtrapolationException):
        VlaBridgeNode._observe(node)

    calls = node._tf.lookup_transform.call_args_list
    assert len(calls) == 1
    assert calls[0].args[1] == 'head_optical'
    assert calls[0].args[2].nanoseconds == stamp_ns


@pytest.mark.parametrize('unavailable', ['missing', 'stale'])
def test_observe_rejects_unavailable_images_before_tf(observation_node, unavailable):
    node = observation_node
    if unavailable == 'missing':
        del node._images['head']
    else:
        _, message = node._images['head']
        node._images['head'] = (time.monotonic() - node._image_timeout - 1, message)

    with pytest.raises(RuntimeError, match='head'):
        VlaBridgeNode._observe(node)

    node._tf.lookup_transform.assert_not_called()


def test_decode_without_snapshot_keeps_start_preflight_contract(observation_node):
    frames, reason = observation_node._decode_images()

    assert reason == ''
    assert set(frames) == set(observation_node._spec.images.slots)
    for index, slot in enumerate(observation_node._spec.images.slots):
        np.testing.assert_array_equal(frames[slot], index + 1)
    observation_node._tf.lookup_transform.assert_not_called()


def test_empty_snapshot_does_not_reread_image_cache(observation_node):
    frames, reason = observation_node._decode_images({})

    assert frames == {}
    assert 'head' in reason
    observation_node._tf.lookup_transform.assert_not_called()