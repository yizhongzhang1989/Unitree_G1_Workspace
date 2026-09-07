"""Strict, timestamp-based motion export, independent of ROS and hardware."""

from __future__ import annotations

import csv
import fcntl
import io
import os
import re
import tempfile
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


FPS = 50
JOINT_NAMES = tuple(
    f'{side}_{joint}_joint'
    for side in ('left', 'right')
    for joint in ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ankle_pitch', 'ankle_roll')
) + ('waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint') + tuple(
    f'{side}_{joint}_joint'
    for side in ('left', 'right')
    for joint in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
                  'wrist_roll', 'wrist_pitch', 'wrist_yaw')
)
METADATA_FIELDS: tuple[str, ...] = ('file_name', 'action', 'duration_seconds', 'fps', 'num_frames')


class RejectedMotion(ValueError):
    """The entire take must be discarded, not repaired by clamping or padding."""


class MotionClip:
    def __init__(self, lower, upper, *, max_gap=0.1,
                 max_root_speed=8.0, max_root_angular_speed=15.0):
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        if (self.lower.shape != (29,) or self.upper.shape != (29,)
                or not np.isfinite([self.lower, self.upper]).all()
                or np.any(self.lower >= self.upper)):
            raise ValueError('Expected finite limits for exactly 29 joints')
        settings = np.asarray([max_gap, max_root_speed, max_root_angular_speed])
        if not np.isfinite(settings).all() or np.any(settings <= 0):
            raise ValueError('Quality thresholds must be finite and positive')
        self.max_gap = float(max_gap)
        self.max_root_speed = float(max_root_speed)
        self.max_root_angular_speed = float(max_root_angular_speed)
        self.stamps = []
        self.rows = []
        self.epoch = None
        self.reason = ''

    def reject(self, reason):
        if not self.reason:
            self.reason = str(reason)

    def _check_rows(self, rows):
        if rows.ndim != 2 or rows.shape[1] != 36 or not np.isfinite(rows).all():
            raise RejectedMotion('Expected 36 finite values per frame')
        norms = np.linalg.norm(rows[:, 3:7], axis=1)
        if np.any(np.abs(norms - 1.0) > 1e-3):
            raise RejectedMotion('Root quaternion is not unit length')
        joints = rows[:, 7:]
        if np.any(joints < self.lower - 1e-8) or np.any(joints > self.upper + 1e-8):
            raise RejectedMotion('Joint angle exceeds the supplied model limits')

    def _check_steps(self, rows, intervals):
        if np.any(np.linalg.norm(np.diff(rows[:, :3], axis=0), axis=1)
                  / intervals > self.max_root_speed):
            raise RejectedMotion('Root translation jump')
        rotations = Rotation.from_quat(rows[:, 3:7])
        angles = (rotations[:-1].inv() * rotations[1:]).magnitude()
        if np.any(angles / intervals > self.max_root_angular_speed):
            raise RejectedMotion('Root orientation jump')

    def append(self, stamp, row, epoch):
        if self.reason:
            raise RejectedMotion(self.reason)
        try:
            stamp = float(stamp)
            values = np.asarray(row, dtype=float)
            self._check_rows(values[None, :])
            if not np.isfinite(stamp):
                raise RejectedMotion('Non-finite source timestamp')
            if self.stamps:
                interval = stamp - self.stamps[-1]
                if epoch != self.epoch:
                    raise RejectedMotion('Calibration/world coordinate reset')
                if interval <= 0 or interval > self.max_gap:
                    raise RejectedMotion('Source clock reset or tracking gap')
                if stamp - self.stamps[0] > 60.0 + 1e-8:
                    raise RejectedMotion('Take exceeds 60 seconds')
                self._check_steps(np.stack((self.rows[-1], values)), np.array([interval]))
            else:
                self.epoch = epoch
            self.stamps.append(stamp)
            self.rows.append(values.copy())
        except ValueError as exc:
            self.reject(str(exc))
            raise RejectedMotion(self.reason) from exc

    def resample(self):
        if self.reason:
            raise RejectedMotion(self.reason)
        if len(self.stamps) < 2 or self.stamps[-1] - self.stamps[0] < 2.0 - 1e-8:
            raise RejectedMotion('Take must contain at least 2 seconds of continuous tracking')
        stamps = np.asarray(self.stamps) - self.stamps[0]
        count = min(3000, int(np.floor(stamps[-1] * FPS + 1e-7)) + 1)
        times = np.arange(count, dtype=float) / FPS
        times = times[times <= stamps[-1]]
        rows = np.asarray(self.rows)
        output = np.empty((len(times), 36), dtype=float)
        for column in (*range(3), *range(7, 36)):
            output[:, column] = np.interp(times, stamps, rows[:, column])
        output[:, 3:7] = Slerp(stamps, Rotation.from_quat(rows[:, 3:7]))(times).as_quat()
        dots = np.sum(output[:-1, 3:7] * output[1:, 3:7], axis=1)
        output[1:, 3:7] *= np.cumprod(np.where(dots < 0, -1.0, 1.0))[:, None]
        self._check_rows(output)
        self._check_steps(output, np.full(len(output) - 1, 1.0 / FPS))
        return output


def validate_ground(heights, *, penetration=0.06):
    heights = np.asarray(heights, dtype=float)
    if heights.ndim != 2 or heights.shape[1] != 2 or not np.isfinite(heights).all():
        raise RejectedMotion('Invalid model-derived foot heights')
    if np.any(heights < -penetration):
        raise RejectedMotion('Foot collision geometry penetrates the floor')


def validate_label(value):
    if not re.fullmatch(r'[A-Za-z0-9]+(?:_[A-Za-z0-9]+)*', value):
        raise ValueError('Labels must use letters, digits and single underscores')
    return value


def _atomic_text(path, content):
    descriptor, temporary = tempfile.mkstemp(prefix='.capture-', dir=path.parent)
    try:
        with os.fdopen(descriptor, 'w', newline='', encoding='utf-8') as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def save_motion(directory, rows, *, category, action):
    """Write one validated take without overwriting any existing motion."""
    validate_label(category)
    validate_label(action)
    directory = Path(directory).expanduser().resolve()
    motions = directory / 'motions'
    motions.mkdir(parents=True, exist_ok=True)
    directory_fd = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
    try:
        fcntl.flock(directory_fd, fcntl.LOCK_EX)
        metadata = directory / 'metadata.csv'
        previous = []
        if metadata.exists():
            with metadata.open(newline='', encoding='utf-8') as stream:
                reader = csv.DictReader(stream)
                if tuple(reader.fieldnames or ()) != METADATA_FIELDS:
                    raise ValueError('Existing metadata.csv has incompatible columns')
                previous = list(reader)
        take = 1
        reserved = {entry['file_name'] for entry in previous}
        while True:
            name = f'{category}_{action}_{take:03d}.csv'
            path = motions / name
            if not path.exists() and name not in reserved:
                break
            take += 1
        buffer = io.StringIO(newline='')
        np.savetxt(buffer, rows, fmt='%.10f', delimiter=',')
        stream = path.open('x', newline='', encoding='utf-8')
        try:
            with stream:
                stream.write(buffer.getvalue())
                stream.flush()
                os.fsync(stream.fileno())
            buffer = io.StringIO(newline='')
            writer = csv.DictWriter(buffer, fieldnames=METADATA_FIELDS)
            writer.writeheader()
            writer.writerows(previous)
            writer.writerow(dict(file_name=name, action=action,
                                 duration_seconds=f'{len(rows) / FPS:.6f}',
                                 fps=FPS, num_frames=len(rows)))
            _atomic_text(metadata, buffer.getvalue())
        except Exception:
            path.unlink()
            raise
        return path
    finally:
        os.close(directory_fd)
