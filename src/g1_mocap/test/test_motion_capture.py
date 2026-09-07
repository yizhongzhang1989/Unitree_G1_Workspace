"""Motion export contract checks without ROS, a headset or robot control."""

import csv

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from g1_mocap.motion_capture import (
    JOINT_NAMES, MotionClip, RejectedMotion, save_motion, validate_ground,
)


def make_clip(rate=90, seconds=3.0):
    clip = MotionClip(np.full(29, -3.0), np.full(29, 3.0))
    for stamp in np.arange(round(seconds * rate) + 1) / rate:
        quat = Rotation.from_rotvec([0.0, 0.0, stamp * 0.5]).as_quat()
        row = np.r_[[stamp * 0.2, 0.0, 0.8], quat, np.full(29, stamp * 0.01)]
        clip.append(stamp, row, 'calibration-1')
    return clip


@pytest.mark.parametrize('rate', [50, 72, 90])
def test_timestamp_resampling_and_xyzw(rate):
    output = make_clip(rate).resample()
    assert output.shape == (151, 36)
    times = np.arange(151) / 50
    np.testing.assert_allclose(output[:, 0], times * 0.2, atol=1e-12)
    np.testing.assert_allclose(output[:, 7], times * 0.01, atol=1e-12)
    expected = Rotation.from_rotvec(np.outer(times * 0.5, [0, 0, 1])).as_quat()
    np.testing.assert_allclose(output[:, 3:7], expected, atol=1e-12)


@pytest.mark.parametrize('defect', ['nan', 'limit', 'gap', 'clock', 'epoch',
                                    'teleport', 'flip', 'norm'])
def test_rejects_entire_take(defect):
    clip = make_clip()
    row = clip.rows[-1].copy()
    stamp, epoch = 3.0 + 1 / 90, 'calibration-1'
    if defect == 'nan':
        row[0] = np.nan
    elif defect == 'limit':
        row[7] = 3.1
    elif defect == 'gap':
        stamp += 0.2
    elif defect == 'clock':
        stamp = 0.0
    elif defect == 'epoch':
        epoch = 'calibration-2'
    elif defect == 'teleport':
        row[0] += 1.0
    elif defect == 'flip':
        row[3:7] = Rotation.from_rotvec([3, 0, 0]).as_quat()
    elif defect == 'norm':
        row[3:7] *= 2
    with pytest.raises(RejectedMotion):
        clip.append(stamp, row, epoch)
    with pytest.raises(RejectedMotion):
        clip.resample()


@pytest.mark.parametrize('rate', [50, 90])
def test_high_joint_speed_is_preserved_and_saved(tmp_path, rate):
    clip = MotionClip(np.full(29, -3.0), np.full(29, 3.0))
    stamps = np.arange(3 * rate + 1) / rate
    angles = 2.0 * np.sin(2.0 * np.pi * 8.0 * stamps)
    assert np.max(np.abs(np.diff(angles)) * rate) > 30.0
    for stamp, angle in zip(stamps, angles):
        row = np.r_[[0.0, 0.0, 0.8], [0.0, 0.0, 0.0, 1.0], np.zeros(29)]
        row[7] = angle
        clip.append(stamp, row, 'calibration-1')
    output = clip.resample()
    expected = np.interp(np.arange(len(output)) / 50, stamps, angles)
    np.testing.assert_allclose(output[:, 7], expected, atol=1e-12)
    assert not clip.reason
    path = save_motion(tmp_path, output, category='dynamic', action='fast')
    saved = np.loadtxt(path, delimiter=',')
    np.testing.assert_allclose(saved, output, atol=1e-10)
    assert np.max(np.abs(np.diff(saved[:, 7])) * 50) > 30.0


def test_quaternion_sign_is_not_a_flip():
    clip = make_clip()
    clip.rows[1::2] = [np.r_[row[:3], -row[3:7], row[7:]] for row in clip.rows[1::2]]
    output = clip.resample()
    assert np.all(np.sum(output[:-1, 3:7] * output[1:, 3:7], axis=1) > 0)


def test_quaternion_sign_with_orthogonal_boundary():
    clip = MotionClip(np.full(29, -3), np.full(29, 3), max_root_angular_speed=200)
    for index in range(101):
        quat = [0, 0, 0, 1] if index < 50 else [1, 0, 0, 0]
        clip.append(index / 50, np.r_[[0, 0, 0.8], quat, np.zeros(29)], 1)
    output = clip.resample()
    assert np.isfinite(output).all()
    np.testing.assert_allclose(np.linalg.norm(output[:, 3:7], axis=1), 1)
    assert np.all(np.sum(output[:-1, 3:7] * output[1:, 3:7], axis=1) >= 0)


def test_duration_limits():
    with pytest.raises(RejectedMotion, match='at least 2'):
        make_clip(seconds=1).resample()
    assert make_clip(rate=50, seconds=60).resample().shape == (3000, 36)
    with pytest.raises(RejectedMotion, match='60 seconds'):
        make_clip(rate=50, seconds=60.1)


def test_ground_check_allows_airborne():
    validate_ground(np.zeros((100, 2)))
    validate_ground(np.full((100, 2), 0.2))
    with pytest.raises(RejectedMotion, match='penetrates'):
        validate_ground(np.full((100, 2), -0.07))


def test_delivery_layout_and_unique_takes(tmp_path):
    rows = make_clip().resample()
    first = save_motion(tmp_path, rows, category='locomotion', action='walk_forward')
    second = save_motion(tmp_path, rows, category='locomotion', action='walk_forward')
    assert first.name == 'locomotion_walk_forward_001.csv'
    assert second.name == 'locomotion_walk_forward_002.csv'
    assert sorted(path.name for path in tmp_path.iterdir()) == ['metadata.csv', 'motions']
    with first.open() as stream:
        parsed = list(csv.reader(stream))
    assert len(parsed) == 151 and all(len(row) == 36 for row in parsed)
    assert all('.' in value for row in parsed for value in row)
    np.testing.assert_allclose(np.loadtxt(first, delimiter=','), rows, atol=1e-10)
    with (tmp_path / 'metadata.csv').open() as stream:
        metadata = list(csv.DictReader(stream))
    assert [entry['fps'] for entry in metadata] == ['50', '50']
    assert metadata[0]['num_frames'] == '151'
    assert float(metadata[0]['duration_seconds']) == 151 / 50
    assert len(JOINT_NAMES) == 29


def test_filename_traversal_rejected(tmp_path):
    with pytest.raises(ValueError):
        save_motion(tmp_path, make_clip().resample(), category='../bad', action='walk')


@pytest.mark.parametrize('failed_sync', [1, 2])
def test_failed_save_leaves_previous_delivery_unchanged(tmp_path, monkeypatch, failed_sync):
    rows = make_clip().resample()
    first = save_motion(tmp_path, rows, category='locomotion', action='walk')
    metadata_before = (tmp_path / 'metadata.csv').read_bytes()
    motion_before = first.read_bytes()
    calls = 0

    def fail_sync(descriptor):
        nonlocal calls
        calls += 1
        if calls == failed_sync:
            raise OSError('simulated disk failure')

    monkeypatch.setattr('g1_mocap.motion_capture.os.fsync', fail_sync)
    with pytest.raises(OSError, match='disk failure'):
        save_motion(tmp_path, rows, category='locomotion', action='walk')
    assert list((tmp_path / 'motions').iterdir()) == [first]
    assert (tmp_path / 'metadata.csv').read_bytes() == metadata_before
    assert first.read_bytes() == motion_before
    assert not list(tmp_path.glob('.capture-*'))
