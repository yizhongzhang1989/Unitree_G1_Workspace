"""Parity against the real offline exporter, without ROS publishers."""

import importlib.util
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from g1_vla_bridge.record_observation import (
    ObservationBuffer, WristReader, WristTimeline, model_from_description, record_tool,
)


@pytest.fixture
def exporter():
    tools = Path(__file__).resolve().parents[2] / 'record' / 'tools'
    spec = importlib.util.spec_from_file_location('_vla_export_test', tools / 'format/YB/export.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_causal_selection_matches_exporter(exporter):
    buffer = ObservationBuffer(('head', 'left_wrist'))
    stamps = np.array([10., 10.023, 10.077, 10.103, 10.14])
    for index, stamp in enumerate(stamps):
        for slot in ('head', 'left_wrist', 'joints'):
            buffer.add(slot, stamp, index, stamp + .1)
        aligned = buffer.select(stamp + .12, .5)
        grid = np.array([aligned.reference])
        expected_index = exporter.frame_index(stamps[:index + 1], grid, .1)[0]
        values, valid = exporter.hold(stamps[:index + 1], np.arange(index + 1)[:, None], grid, .1)
        assert valid[0]
        assert aligned.images['head'].value == expected_index
        assert aligned.joints.value == values[0, 0]
        assert aligned.joints.stamp <= aligned.reference


def test_wrist_fit_matches_export_on_each_available_prefix():
    timeline = WristTimeline('left_wrist')
    raw = 100. + np.arange(100) / 30 + np.sin(np.arange(100)) * .01
    reader = record_tool('session_reader')
    for index, stamp in enumerate(raw):
        samples = timeline.append(stamp, index, stamp + .03)
        expected = reader.fitted_pts(raw[:index + 1]) - reader.CAMERA_DELAY_S['wrist_left']
        np.testing.assert_allclose([sample.stamp for sample in samples], expected[-32:])
        assert samples[-1].value == index


def test_missing_and_stale_samples_fail_closed():
    buffer = ObservationBuffer(('head',))
    with pytest.raises(RuntimeError, match='missing'):
        buffer.select(10., .5)
    buffer.add('head', 10., 1, 10.)
    buffer.add('joints', 10., {}, 10.)
    with pytest.raises(RuntimeError, match='stale'):
        buffer.select(11., .5)
    buffer.add('head', 9., 99, 11.)
    assert buffer.images['head'][-1].value == 1


def test_ros2_control_joint_entries_are_not_kinematic_joints():
    model = model_from_description('''<robot name="test">
      <joint name="camera" type="fixed"><parent link="base"/><child link="camera"/></joint>
      <ros2_control name="system" type="system"><joint name="camera">
        <state_interface name="position"/></joint></ros2_control>
    </robot>''')
    assert list(model.joints) == ['camera']
    np.testing.assert_array_equal(model.poses('base', 'camera', {})[0], np.eye(4))


def test_head_tf_availability_caps_all_samples():
    buffer = ObservationBuffer(('head',))
    for stamp in (10., 10.05, 10.1):
        for slot in ('head', 'joints'):
            buffer.add(slot, stamp, stamp, stamp)
    selected = buffer.select(10.2, .5, available_until=10.06)
    assert selected.reference <= 10.06
    assert selected.images['head'].value == 10.05
    assert selected.joints.value == 10.05


def test_rolling_fit_and_frame_storage_are_bounded():
    timeline = WristTimeline('right_wrist', capacity=4, fit_frames=12)
    raw = 100. + np.arange(40) / 30.
    for index, stamp in enumerate(raw):
        samples = timeline.append(stamp, index, stamp)
    expected = record_tool('session_reader').fitted_pts(raw[-12:]) - .110
    np.testing.assert_allclose([sample.stamp for sample in samples], expected[-4:])
    assert len(timeline.raw) == 12
    assert [sample.value for sample in samples] == [36, 37, 38, 39]


def test_stream_failure_clears_cached_frames_without_leaking_credentials():
    buffer = ObservationBuffer(('left_wrist',))
    event = threading.Event()
    reader = SimpleNamespace(slot='left_wrist', url='rtsp://secret', buffer=buffer,
                             stop_event=event, error='')

    def fail(*args, **kwargs):
        buffer.add('left_wrist', 10., object(), 10.)
        event.set()
        raise OSError('rtsp://secret')

    reader._av = SimpleNamespace(open=Mock(side_effect=fail))
    WristReader._run(reader)
    assert not buffer.images['left_wrist']
    assert reader.error == 'OSError'
