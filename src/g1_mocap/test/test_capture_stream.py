"""Strict source behavior without opening a socket."""

import time
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

from g1_mocap.capture_stream import CaptureStream
from g1_mocap.skeleton import BodyFrame
from g1_mocap.stream import MocapStream


def source():
    stream = CaptureStream(SimpleNamespace(), limits=(np.full(29, -3), np.full(29, 3)),
                           log=lambda message: None)
    stream._device = SimpleNamespace(closed=False)
    stream._calibration = object()
    stream.last_valid_arrival = time.monotonic()
    return stream


def frame(stamp=1.0, seq=1, status=1):
    return BodyFrame(stamp, seq, np.zeros((24, 3)), status, 0, np.tile(np.eye(3), (24, 1, 1)))


def result():
    return SimpleNamespace(root_pos=np.array([0.0, 0.0, 0.8]),
                           root_quat=np.array([1.0, 0.0, 0.0, 0.0]), joint_pos=np.zeros(29))


def test_calibration_is_blocked_and_take_rejected():
    stream = source()
    stream.begin()
    with pytest.raises(RuntimeError, match='recalibrating'):
        stream.calibrate()
    assert stream.finish().reason == 'Calibration attempted during recording'


def test_start_requires_fresh_calibration():
    stream = source()
    stream._calibration = None
    with pytest.raises(RuntimeError):
        stream.begin()
    stream._calibration = object()
    stream.last_valid_arrival = time.monotonic() - 1
    with pytest.raises(RuntimeError):
        stream.begin()


@pytest.mark.parametrize('bad', [None, frame(status=2), frame(status=0)])
def test_bad_tracking_is_latched(bad):
    stream = source()
    stream.begin()
    with patch('g1_mocap.capture_stream.parse_body', return_value=bad), \
            patch.object(MocapStream, '_ingest'):
        stream._ingest({})
    assert 'Tracking' in stream.finish().reason


def test_uses_raw_timestamp_and_xyzw():
    stream = source()
    stream.begin()
    stream._capture_frame(999.0, frame(stamp=12.0), result())
    clip = stream.finish()
    assert clip.stamps == [12.0]
    np.testing.assert_array_equal(clip.rows[0][3:7], [0, 0, 0, 1])


def test_missing_sequence_invalidates_take():
    stream = source()
    stream.begin()
    for seq in (1, 3):
        raw = frame(stamp=seq / 90, seq=seq)
        with patch('g1_mocap.capture_stream.parse_body', return_value=raw), \
                patch.object(MocapStream, '_ingest', side_effect=lambda payload:
                             stream._capture_frame(0.0, raw, result())):
            stream._ingest({})
    assert 'missing' in stream.finish().reason


def test_failed_retarget_and_reconnection_rejected():
    stream = source()
    stream.begin()
    with patch('g1_mocap.capture_stream.parse_body', return_value=frame()), \
            patch.object(MocapStream, '_ingest'):
        stream._ingest({})
    assert 'retargeted' in stream.finish().reason
    stream.begin()
    stream._device = object()
    with patch('g1_mocap.capture_stream.parse_body', return_value=frame()), \
            patch.object(MocapStream, '_ingest'):
        stream._ingest({})
    assert 'connection' in stream.finish().reason


def test_source_freezes_at_auto_stop_boundary():
    stream = source()
    stream.begin(duration_limit=60.0)
    for stamp in np.arange(5404) / 90:
        stream._capture_frame(stamp, frame(stamp=stamp), result())
    assert stream.complete
    clip = stream.finish()
    assert clip.stamps[-1] < 60
    assert not clip.reason
    assert clip.resample().shape[0] <= 3000


def test_two_second_auto_stop_is_not_rejected_as_short():
    stream = source()
    stream.begin(duration_limit=2.0)
    for stamp in np.arange(185) / 90:
        stream._capture_frame(stamp, frame(stamp=stamp), result())
    assert stream.complete
    assert len(stream.finish().resample()) >= 100


def test_default_recording_continues_after_thirty_seconds():
    stream = source()
    stream.begin()
    for stamp in np.arange(2791) / 90:
        stream._capture_frame(stamp, frame(stamp=stamp), result())
    assert not stream.complete
    assert stream.clip.stamps[-1] == 31.0
    assert not stream.clip.reason


def test_disconnect_prevents_start_even_with_recent_frame():
    stream = source()
    stream._device = None
    with pytest.raises(RuntimeError, match='connected'):
        stream.begin()


@pytest.mark.parametrize('connected', [False, True])
def test_connection_change_clears_old_calibration_and_samples(connected):
    stream = source()
    stream._raw.extend([frame()] * 20)
    stream.begin()
    stream._note(connected=connected)
    assert not stream.calibrated
    assert stream.recent_frames() == []
    assert stream.last_valid_arrival == float('-inf')
    assert 'connection' in stream.finish().reason


def test_disconnected_calibration_is_rejected():
    stream = source()
    stream._device = None
    stream._raw.extend([frame()] * 20)
    with pytest.raises(RuntimeError, match='connected'):
        stream.calibrate()


@pytest.mark.parametrize('next_stamp', [60.05, 60.2])
def test_irregular_frame_at_sixty_second_boundary(next_stamp):
    stream = source()
    stream.begin()
    for stamp in np.arange(5998) / 100:
        stream._capture_frame(stamp, frame(stamp=stamp), result())
    stream._capture_frame(next_stamp, frame(stamp=next_stamp), result())
    clip = stream.finish()
    if next_stamp == 60.05:
        assert stream.complete
        assert not clip.reason
        assert len(clip.resample()) <= 3000
    else:
        assert clip.reason
