"""Software position tracking that emits torque only."""

from collections import deque
from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class TorqueStep:
    torque: np.ndarray
    reference: np.ndarray
    target_error: np.ndarray
    trajectory_complete: bool
    settled: bool


class PoseStabilityWindow:
    """Detect a stationary measured pose from position variation only."""

    def __init__(
        self,
        *,
        duration: float = 0.6,
        position_range_tolerance: float = 0.02,
    ) -> None:
        if duration <= 0.0:
            raise ValueError("duration must be positive")
        if position_range_tolerance < 0.0:
            raise ValueError("position stability tolerance must be non-negative")
        self.duration = float(duration)
        self.position_range_tolerance = float(position_range_tolerance)
        self.reset()

    def reset(self) -> None:
        self._samples = deque()
        self.span = 0.0
        self.max_velocity = float("inf")
        self.max_position_range = float("inf")

    def update(
        self,
        timestamp: float,
        position: Sequence[float],
        velocity: Sequence[float],
    ) -> bool:
        position_array = TorquePoseController._seven(position, "position")
        velocity_array = TorquePoseController._seven(velocity, "velocity")
        now = float(timestamp)
        self._samples.append((now, position_array, velocity_array))
        cutoff = now - self.duration
        while (len(self._samples) > 1 and self._samples[1][0] <= cutoff):
            self._samples.popleft()

        self.span = now - self._samples[0][0]
        positions = np.asarray([sample[1] for sample in self._samples])
        velocities = np.asarray([sample[2] for sample in self._samples])
        self.max_velocity = float(np.max(np.abs(velocities)))
        self.max_position_range = float(np.max(np.ptp(positions, axis=0)))
        return bool(
            self.span >= self.duration and
            self.max_position_range <= self.position_range_tolerance)


class TorquePoseController:
    """Gravity feed-forward plus bounded software PD for one seven-axis arm."""

    def __init__(
        self,
        *,
        stiffness: Sequence[float] = (10.0, 10.0, 8.0, 8.0, 4.0, 4.0, 3.0),
        damping: Sequence[float] = (2, 2, 1.5, 1.5, 1, 1, 0.5),
        torque_slew_rate: Sequence[float] = (30.0,) * 7,
        maximum_speed: float = 0.35,
        position_tolerance: float = 0.04,
        velocity_tolerance: float = 0.05,
        minimum_duration: float = 2.0,
    ) -> None:
        self.stiffness = self._seven(stiffness, "stiffness")
        self.damping = self._seven(damping, "damping")
        self.torque_slew_rate = self._seven(
            torque_slew_rate, "torque_slew_rate")
        if np.any(self.stiffness < 0.0) or np.any(self.damping < 0.0):
            raise ValueError("stiffness and damping must be non-negative")
        if np.any(self.torque_slew_rate <= 0.0):
            raise ValueError("torque slew rates must be positive")
        self.maximum_speed = float(maximum_speed)
        if self.maximum_speed <= 0.0:
            raise ValueError("maximum speed must be positive")
        self.position_tolerance = float(position_tolerance)
        self.velocity_tolerance = float(velocity_tolerance)
        self.minimum_duration = float(minimum_duration)
        self._active = False

    def start(self, timestamp: float, position: Sequence[float],
              target: Sequence[float],
              initial_torque: Sequence[float] = (0.0,) * 7) -> float:
        self._start = self._seven(position, "position")
        self._target = self._seven(target, "target")
        self._start_time = float(timestamp)
        excursion = float(np.max(np.abs(self._target - self._start)))
        self._duration = max(
            self.minimum_duration, excursion / self.maximum_speed)
        self._last_time = self._start_time
        self._last_torque = self._seven(initial_torque, "initial_torque")
        self._active = True
        return self._duration

    def step(
        self,
        timestamp: float,
        position: Sequence[float],
        velocity: Sequence[float],
        gravity_torque: Sequence[float],
    ) -> TorqueStep:
        if not self._active:
            raise RuntimeError("torque pose controller has not been started")
        position_array = self._seven(position, "position")
        velocity_array = self._seven(velocity, "velocity")
        gravity_array = self._seven(gravity_torque, "gravity_torque")

        elapsed = max(0.0, float(timestamp) - self._start_time)
        ratio = min(1.0, elapsed / self._duration)
        smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio)
        reference = self._start + smooth_ratio * (self._target - self._start)
        # 阻抗力矩计算
        raw_torque = gravity_array + self.stiffness * (reference - position_array) - self.damping * velocity_array

        delta_time = max(0.0, float(timestamp) - self._last_time)
        maximum_delta = self.torque_slew_rate * delta_time
        torque = self._last_torque + np.clip(
            raw_torque - self._last_torque, -maximum_delta, maximum_delta)
        self._last_torque = torque
        self._last_time = float(timestamp)
        target_error = self._target - position_array
        trajectory_complete = ratio >= 1.0
        settled = bool(
            trajectory_complete and
            np.max(np.abs(target_error)) <= self.position_tolerance and
            np.max(np.abs(velocity_array)) <= self.velocity_tolerance)
        return TorqueStep(
            torque=torque,
            reference=reference,
            target_error=target_error,
            trajectory_complete=trajectory_complete,
            settled=settled,
        )

    @staticmethod
    def _seven(value: Sequence[float], name: str) -> np.ndarray:
        array = np.asarray(value, dtype=float)
        if array.shape != (7,) or not np.all(np.isfinite(array)):
            raise ValueError("%s must contain seven finite values" % name)
        return array.copy()