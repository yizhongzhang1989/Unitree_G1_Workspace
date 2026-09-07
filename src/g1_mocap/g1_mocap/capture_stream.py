"""Exclusive capture source; live mocap and tracking behavior stays unchanged."""

from __future__ import annotations

import threading
import time

import numpy as np

from .motion_capture import FPS, MotionClip, RejectedMotion
from .skeleton import STATUS_VALID, ClockAligner, parse_body
from .stream import MocapStream


class CaptureStream(MocapStream):
    def __init__(self, retargeter, *, limits, **kwargs):
        super().__init__(retargeter, **kwargs)
        self.capture_lock = threading.RLock()
        self.clip = None
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
            if (not frames or time.monotonic() - self.last_valid_arrival > 0.2
                    or any(frame.status != STATUS_VALID or frame.message != 0
                           or frame.rotations is None for frame in frames)):
                raise RuntimeError('Calibration needs recent fully VALID tracking with orientations')
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
            self.duration_limit = float(duration_limit)
            self.complete = False
            self.last_seq = None
            self.last_source_t = None
            self.last_device = self._device

    def finish(self):
        with self.capture_lock:
            if self.clip is None:
                raise RuntimeError('No active take')
            clip, self.clip = self.clip, None
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
