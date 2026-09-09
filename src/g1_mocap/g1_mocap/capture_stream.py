"""Exclusive capture source; live mocap and tracking behavior stays unchanged."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .motion_capture import FPS, MotionClip, RejectedMotion
from .skeleton import SMPL_JOINTS, STATUS_MESSAGES, STATUS_VALID, ClockAligner, parse_body
from .stream import MocapStream


@dataclass
class SourceClip:
    """Unscaled PICO skeleton samples needed by offline retargeters."""

    calibration: object
    timestamps: list[float] = field(default_factory=list)
    sequences: list[int] = field(default_factory=list)
    positions: list[np.ndarray] = field(default_factory=list)
    orientations: list[np.ndarray] = field(default_factory=list)
    statuses: list[int] = field(default_factory=list)
    messages: list[int] = field(default_factory=list)

    def append(self, frame):
        if frame.rotations is None:
            raise ValueError('Source frame has no joint orientations')
        self.timestamps.append(float(frame.t))
        self.sequences.append(int(frame.seq))
        self.positions.append(np.asarray(frame.positions, dtype=np.float64).copy())
        self.orientations.append(np.asarray(frame.rotations, dtype=np.float64).copy())
        self.statuses.append(int(frame.status))
        self.messages.append(int(frame.message))

    def save(self, path):
        if not self.timestamps:
            raise ValueError('Source trajectory is empty')
        path = Path(path)
        temporary = path.with_name(f'.{path.name}.tmp')
        try:
            with temporary.open('xb') as stream:
                np.savez_compressed(
                    stream,
                    format_version=np.array(1, dtype=np.int64),
                    joint_names=np.asarray(SMPL_JOINTS),
                    timestamps=np.asarray(self.timestamps, dtype=np.float64),
                    sequences=np.asarray(self.sequences, dtype=np.int64),
                    positions=np.asarray(self.positions, dtype=np.float64),
                    orientations=np.asarray(self.orientations, dtype=np.float64),
                    statuses=np.asarray(self.statuses, dtype=np.uint8),
                    messages=np.asarray(self.messages, dtype=np.int32),
                    calibration_scale=np.array(self.calibration.scale, dtype=np.float64),
                    calibration_pelvis_ref_z=np.array(
                        self.calibration.pelvis_ref_z, dtype=np.float64),
                    calibration_stand_height=np.array(
                        self.calibration.stand_height, dtype=np.float64),
                    calibration_pelvis_fix=np.asarray(
                        self.calibration.pelvis_fix, dtype=np.float64),
                    calibration_torso_fix=np.asarray(
                        self.calibration.torso_fix, dtype=np.float64),
                    calibration_joint_bias=np.asarray(
                        self.calibration.joint_bias, dtype=np.float64),
                    calibration_joint_target=np.asarray(
                        self.calibration.joint_target, dtype=np.float64),
                    calibration_arm_hinge_axes=np.asarray(
                        self.calibration.arm_hinge_axes, dtype=np.float64),
                )
                stream.flush()
                import os
                os.fsync(stream.fileno())
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)


class CaptureStream(MocapStream):
    def __init__(self, retargeter, *, limits, **kwargs):
        super().__init__(retargeter, **kwargs)
        self.capture_lock = threading.RLock()
        self.clip = None
        self.source_clip = None
        self.limits = limits
        self.last_valid_arrival = float('-inf')
        self.last_seq = None
        self.last_source_t = None
        self.last_device = None
        self.on_preview = None
        self.complete = False
        self.duration_limit = 60.0
        self.on_frame = self._capture_frame

    def _note(self, **fields):
        with self.capture_lock:
            if 'connected' in fields:
                if self.clip is not None and not self.complete:
                    self.clip.reject('Headset connection changed')
                self._calibration = None
                self.last_valid_arrival = float('-inf')
                with self._raw_lock:
                    self._raw.clear()
                self._clock = ClockAligner()
                self._last_stamp = float('-inf')
            super()._note(**fields)

    def _require_connection(self):
        if self._device is None or self._device.closed:
            raise RuntimeError('Headset must be connected')

    def calibrate(self, *, min_frames=20):
        with self.capture_lock:
            if self.clip is not None:
                self.clip.reject('Calibration attempted during recording')
                raise RuntimeError('Stop/discard the take before recalibrating')
            self._require_connection()
            frames = self.recent_frames()
            stale = time.monotonic() - self.last_valid_arrival > 0.2
            limited = sum(frame.status != STATUS_VALID for frame in frames)
            messages = sum(frame.message != 0 for frame in frames)
            missing_orientations = sum(frame.rotations is None for frame in frames)
            if not frames or stale or limited or messages or missing_orientations:
                stats = self.stats()
                status = {0: 'INVALID', 1: 'VALID', 2: 'LIMITED'}.get(
                    stats.status, str(stats.status))
                message = STATUS_MESSAGES.get(stats.message, str(stats.message))
                raise RuntimeError(
                    'Calibration needs recent fully VALID tracking with orientations: '
                    f'latest={status}({stats.status}), {message}({stats.message}); '
                    f'recent={len(frames)}, non-VALID={limited}, '
                    f'nonzero-message={messages}, missing-orientations={missing_orientations}, '
                    f'fresh-VALID={not stale}')
            return super().calibrate(min_frames=min_frames)

    def begin(self, *, duration_limit=60.0, **quality):
        with self.capture_lock:
            if self.clip is not None:
                raise RuntimeError('A take is already active; stop/discard it first')
            self._require_connection()
            if not self.calibrated or time.monotonic() - self.last_valid_arrival > 0.2:
                raise RuntimeError('Calibrate with fresh VALID tracking before recording')
            if not 2 <= duration_limit <= 60:
                raise ValueError('Duration limit must be between 2 and 60 seconds')
            self.clip = MotionClip(*self.limits, **quality)
            self.source_clip = SourceClip(self.calibration)
            self.duration_limit = float(duration_limit)
            self.complete = False
            self.last_seq = None
            self.last_source_t = None
            self.last_device = self._device

    def finish(self):
        with self.capture_lock:
            if self.clip is None:
                raise RuntimeError('No active take')
            clip, source = self.clip, self.source_clip
            self.clip = self.source_clip = None
            clip.source = source if source.timestamps else None
            return clip

    def seal(self):
        with self.capture_lock:
            if self.clip is not None and not self.complete:
                if time.monotonic() - self.last_valid_arrival > self.clip.max_gap:
                    self.clip.reject('Tracking lost before stop')
                self.complete = True

    def _capture_frame(self, stamped, raw, result):
        if self.clip is not None and not self.complete:
            if self.clip.stamps and raw.t - self.clip.stamps[0] > self.duration_limit:
                interval = raw.t - self.clip.stamps[-1]
                if not np.isfinite(interval) or interval > self.clip.max_gap:
                    self.clip.reject('Source clock reset or tracking gap')
                elif self.clip.stamps[-1] - self.clip.stamps[0] >= 2.0:
                    self.complete = True
            row = np.concatenate((result.root_pos, result.root_quat[[1, 2, 3, 0]],
                                  result.joint_pos))
            if not self.complete:
                try:
                    self.clip.append(raw.t, row, id(self.calibration))
                    self.source_clip.append(raw)
                    endpoint = min(self.duration_limit, 60.0 - 1.0 / FPS)
                    self.complete = self.clip.stamps[-1] - self.clip.stamps[0] >= endpoint
                except RejectedMotion:
                    pass
        if self.on_preview is not None:
            self.on_preview(stamped, raw, result)

    def _ingest(self, payload):
        with self.capture_lock:
            frame = parse_body(payload)
            valid = (frame is not None and frame.status == STATUS_VALID
                     and frame.message == 0 and frame.rotations is not None)
            if valid:
                self.last_valid_arrival = time.monotonic()
            else:
                self.last_valid_arrival = float('-inf')
            clip = self.clip if not self.complete else None
            previous_count = len(clip.stamps) if clip is not None else 0
            if clip is not None:
                if not valid:
                    clip.reject('Tracking invalid/limited or joint orientations missing')
                if self._device is not self.last_device:
                    clip.reject('Headset connection changed')
                if frame is not None:
                    if self.last_seq is not None and (frame.seq - self.last_seq) % (1 << 32) != 1:
                        clip.reject('Source frames missing, duplicated or restarted')
                    if self.last_source_t is not None and frame.t <= self.last_source_t:
                        clip.reject('Headset timestamp reset or reordered')
                    self.last_seq, self.last_source_t = frame.seq, frame.t
            try:
                super()._ingest(payload)
            except Exception as exc:
                if clip is not None:
                    clip.reject(f'Source processing failed: {exc}')
                raise
            if clip is not None and not self.complete and len(clip.stamps) == previous_count:
                clip.reject('Source frame could not be retargeted')
