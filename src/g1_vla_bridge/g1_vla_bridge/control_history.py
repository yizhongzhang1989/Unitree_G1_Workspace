"""Ten-Hz paired episode history in the robot base frame."""

from bisect import bisect_left
from dataclasses import dataclass
import threading

import numpy as np


@dataclass(frozen=True)
class ControlStep:
    stamp: float
    action: dict[str, np.ndarray]
    state: dict[str, np.ndarray]
    action_grippers: dict[str, float] | None = None
    state_grippers: dict[str, float] | None = None
    state_stamp: float | None = None

    @property
    def action_stamp(self):
        return self.stamp


def copy_grippers(values):
    if values is None:
        return None
    result = {side: float(values[side]) for side in ('left', 'right')}
    if not all(np.isfinite(value) for value in result.values()):
        raise ValueError('nonfinite gripper history')
    return result


def copy_poses(poses):
    result = {side: np.array(poses[side], dtype=float, copy=True)
              for side in ('left', 'right')}
    if any(pose.shape != (7,) or not np.all(np.isfinite(pose))
           or np.linalg.norm(pose[3:]) < 1e-12 for pose in result.values()):
        raise ValueError('history requires finite bilateral poses with valid quaternions')
    return result


class ControlHistory:
    """Return the latest 15 causal pairs from a ten-Hz episode archive."""

    def __init__(self):
        self._steps = []
        self._ready = []
        self._origin = None
        self._tick = -1
        self._last_stamp = None
        self._lock = threading.Lock()

    def clear(self):
        with self._lock:
            self._steps.clear()
            self._ready.clear()
            self._origin = None
            self._tick = -1
            self._last_stamp = None

    def append(self, stamp, action, state, action_grippers=None, state_grippers=None,
               state_stamp=None):
        if not np.isfinite(stamp):
            raise ValueError('invalid control timestamp')
        if state_stamp is not None and not np.isfinite(state_stamp):
            raise ValueError('invalid measurement timestamp')
        step = ControlStep(float(stamp), copy_poses(action), copy_poses(state),
                           copy_grippers(action_grippers), copy_grippers(state_grippers), state_stamp)
        with self._lock:
            if self._last_stamp is not None and stamp <= self._last_stamp:
                raise ValueError('control timestamps must increase')
            self._last_stamp = stamp
            if self._origin is None:
                self._origin = stamp
            tick = int(np.floor((stamp - self._origin) * 10. + 1e-5))
            if tick <= self._tick:
                return
            self._tick = tick
            self._steps.append(step)
            self._ready.append(max(stamp, state_stamp if state_stamp is not None else stamp,
                                   self._ready[-1] if self._ready else stamp))

    def snapshot(self, before=float('inf')):
        with self._lock:
            stop = bisect_left(self._ready, before)
            steps = self._steps[max(0, stop - 15):stop]
            return tuple(ControlStep(step.stamp, copy_poses(step.action), copy_poses(step.state),
                                     copy_grippers(step.action_grippers), copy_grippers(step.state_grippers),
                                     step.state_stamp)
                         for step in steps)
