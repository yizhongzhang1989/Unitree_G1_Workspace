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
        aligned = buffer.select()
        grid = np.array([aligned.reference])
        expected_index = exporter.frame_index(stamps[:index + 1], grid, .1)[0]
        values, valid = exporter.hold(stamps[:index + 1], np.arange(index + 1)[:, None], grid, .1)
        assert valid[0]
        assert aligned.images['head'].value == expected_index
        assert aligned.joints.value == values[0, 0]
        assert aligned.joints.stamp <= aligned.reference


def test_wrist_timestamps_are_fixed_raw_pts_minus_delay():
    timeline = WristTimeline('left_wrist')
    raw = 100. + np.arange(100) / 30 + np.sin(np.arange(100)) * .01
    reader = record_tool('session_reader')
    for index, stamp in enumerate(raw):
        samples = timeline.append(stamp, index, stamp + .03)
        expected = raw[:index + 1] - reader.CAMERA_DELAY_S['wrist_left']
        np.testing.assert_allclose([sample.stamp for sample in samples], expected[-32:])
        assert samples[-1].value == index


def test_missing_samples_fail_but_old_samples_remain_usable():
    buffer = ObservationBuffer(('head',))
    with pytest.raises(RuntimeError, match='missing'):
        buffer.select()
    buffer.add('head', 10., 1, 10.)
    buffer.add('joints', 10., {}, 10.)
    assert buffer.select().reference == 10.
    buffer.add('head', 9., 99, 11.)
    assert buffer.images['head'][-1].value == 99
    assert len(buffer.images['head']) == 1
    assert buffer.origin is None


def test_ros2_control_joint_entries_are_not_kinematic_joints():
    model = model_from_description('''<robot name="test">
      <joint name="camera" type="fixed"><parent link="base"/><child link="camera"/></joint>
      <ros2_control name="system" type="system"><joint name="camera">
        <state_interface name="position"/></joint></ros2_control>
    </robot>''')
    assert list(model.joints) == ['camera']
    np.testing.assert_array_equal(model.poses('base', 'camera', {})[0], np.eye(4))


def test_old_sample_before_reference_has_no_age_limit():
    buffer = ObservationBuffer(('head',))
    buffer.add('head', 1., 'old', 1.)
    buffer.add('head', 100., 'future', 100.)
    buffer.add('joints', 50., {}, 50.)
    selected = buffer.select()
    assert selected.reference == 50.
    assert selected.images['head'].value == 'old'


@pytest.mark.parametrize('limiter', ['grid', 'wrist', 'tf'])
def test_missing_sample_reports_actual_alignment_boundary(limiter):
    buffer = ObservationBuffer(('head', 'left_wrist'))
    buffer.origin = 10.
    buffer.add('head', 10.11, 'head', 10.12)
    buffer.add('left_wrist', 9.95 if limiter == 'wrist' else 10.15, 'wrist', 10.16)
    buffer.add('joints', 10.15, {}, 10.16)
    with pytest.raises(RuntimeError) as failure:
        buffer.select(9.95 if limiter == 'tf' else None)
    message = str(failure.value)
    assert 'observation missing sample before reference: head' in message
    assert 'ranges_relative_to_reference: head=[' in message
    if limiter == 'grid':
        assert 'grid_backoff=0.010s' in message
        assert 'head=[+0.010,+0.010]s/1' in message
    elif limiter == 'wrist':
        assert 'left_wrist=[+0.050,+0.050]s/1' in message
    else:
        assert 'tf_offset=+0.050s' in message


def test_head_tf_availability_caps_all_samples():
    buffer = ObservationBuffer(('head',), rate=30.)
    for stamp in (10., 10.05, 10.1):
        for slot in ('head', 'joints'):
            buffer.add(slot, stamp, stamp, stamp)
    selected = buffer.select(available_until=10.06)
    assert selected.reference <= 10.06
    assert selected.images['head'].value == 10.05
    assert selected.joints.value == 10.05


def test_frame_storage_is_bounded_without_retiming():
    timeline = WristTimeline('right_wrist', capacity=4)
    raw = 100. + np.arange(40) / 30.
    for index, stamp in enumerate(raw):
        samples = timeline.append(stamp, index, stamp)
    expected = raw - .110
    np.testing.assert_allclose([sample.stamp for sample in samples], expected[-4:])
    assert [sample.value for sample in samples] == [36, 37, 38, 39]


@pytest.mark.parametrize('pause_index', [1, 90, 450, 868, 899])
def test_wrist_pause_does_not_shift_frame_timestamps(pause_index):
    timeline = WristTimeline('left_wrist')
    raw = 100. + np.arange(900) / 30.
    raw[pause_index:] += 3.4
    for index, stamp in enumerate(raw):
        samples = timeline.append(float(stamp), index, float(stamp))
    timestamps = np.array([sample.stamp for sample in samples])
    np.testing.assert_allclose(timestamps, raw[-32:] - timeline.delay, atol=1e-12)
    assert np.all(np.diff(timestamps) >= 0.)
    buffer = ObservationBuffer(('head', 'left_wrist'))
    buffer.origin = 100.
    buffer.replace_camera('left_wrist', samples)
    for stamp in raw[-32:]:
        buffer.add('head', float(stamp), 'head', float(stamp))
        buffer.add('joints', float(stamp), {}, float(stamp))
    selected = buffer.select()
    assert selected.images['left_wrist'].stamp <= selected.reference


def test_wrist_timestamp_rollback_starts_new_epoch():
    timeline = WristTimeline('left_wrist')
    timeline.append(100., 'old', 1.)
    samples = timeline.append(1., 'new', 2.)
    assert len(samples) == 1
    assert samples[0].value == 'new'
    assert samples[0].stamp == pytest.approx(.89)


def test_wrist_rollback_resets_observation_grid():
    timeline = WristTimeline('left_wrist')
    buffer = ObservationBuffer(('left_wrist',))
    buffer.replace_camera('left_wrist', timeline.append(100., 'old', 1.))
    buffer.origin = 99.89
    buffer.replace_camera('left_wrist', timeline.append(1., 'new', 2.))
    assert buffer.origin is None
    buffer.add('joints', .89, {}, 2.)
    assert buffer.select().images['left_wrist'].value == 'new'


@pytest.mark.parametrize('slot', ['head', 'joints'])
def test_stream_rollback_recovers_on_next_samples(slot):
    buffer = ObservationBuffer(('head',))
    buffer.add(slot, 100., 'old', 1.)
    assert buffer.add(slot, 1., 'new', 2.)
    assert not buffer.add(slot, 1.1, 'next', 3.)
    queue = buffer.joints if slot == 'joints' else buffer.images[slot]
    assert [sample.value for sample in queue] == ['new', 'next']


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
