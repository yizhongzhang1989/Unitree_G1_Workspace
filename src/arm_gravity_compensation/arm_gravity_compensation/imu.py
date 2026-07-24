"""Stable torso-frame gravity estimation from the head IMU accelerometer."""

from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class GravityEstimate:
    gravity: np.ndarray
    mean_acceleration: np.ndarray
    acceleration_std: np.ndarray
    mean_gyro_norm: float
    sample_count: int
    duration: float


class ImuSampleWindow:
    """Collect one bounded IMU window and reject it when the torso moved."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._times = []
        self._accelerations = []
        self._gyroscopes = []

    def add(
        self,
        timestamp: float,
        acceleration: Sequence[float],
        gyroscope: Sequence[float],
    ) -> None:
        acceleration_array = np.asarray(acceleration, dtype=float)
        gyroscope_array = np.asarray(gyroscope, dtype=float)
        if acceleration_array.shape != (3,) or gyroscope_array.shape != (3,):
            raise ValueError("IMU acceleration and gyroscope must have shape (3,)")
        if not (np.all(np.isfinite(acceleration_array)) and
                np.all(np.isfinite(gyroscope_array))):
            return
        self._times.append(float(timestamp))
        self._accelerations.append(acceleration_array)
        self._gyroscopes.append(gyroscope_array)

    @property
    def sample_count(self) -> int:
        return len(self._times)

    @property
    def duration(self) -> float:
        if len(self._times) < 2:
            return 0.0
        return self._times[-1] - self._times[0]

    def ready(self, minimum_duration: float, minimum_samples: int) -> bool:
        return (self.sample_count >= minimum_samples and
                self.duration >= minimum_duration)

    def estimate(
        self,
        imu_to_torso_rotation: Sequence[Sequence[float]],
        *,
        acceleration_sign: float = -1.0,
        gravity_magnitude: float = 9.81,
        minimum_acceleration_norm: float = 5.0,
        maximum_acceleration_norm: float = 15.0,
        maximum_acceleration_std: float = 0.35,
        maximum_gyro_norm: float = 0.15,
    ) -> GravityEstimate:
        if not self._accelerations:
            raise ValueError("IMU window is empty")
        accelerations = np.asarray(self._accelerations, dtype=float)
        gyroscopes = np.asarray(self._gyroscopes, dtype=float)
        mean_acceleration = np.mean(accelerations, axis=0)
        acceleration_std = np.std(accelerations, axis=0)
        acceleration_norm = float(np.linalg.norm(mean_acceleration))
        mean_gyro_norm = float(np.mean(np.linalg.norm(gyroscopes, axis=1)))
        if not minimum_acceleration_norm <= acceleration_norm <= maximum_acceleration_norm:
            raise ValueError(
                "mean acceleration norm %.3f is outside [%.3f, %.3f]"
                % (acceleration_norm, minimum_acceleration_norm,
                   maximum_acceleration_norm))
        if float(np.max(acceleration_std)) > maximum_acceleration_std:
            raise ValueError(
                "acceleration is not stable (max std %.3f > %.3f)"
                % (float(np.max(acceleration_std)), maximum_acceleration_std))
        if mean_gyro_norm > maximum_gyro_norm:
            raise ValueError(
                "torso is rotating (mean gyro norm %.3f > %.3f)"
                % (mean_gyro_norm, maximum_gyro_norm))

        rotation = np.asarray(imu_to_torso_rotation, dtype=float)
        if rotation.shape != (3, 3):
            raise ValueError("imu_to_torso_rotation must have shape (3, 3)")
        direction = rotation @ mean_acceleration
        gravity = (float(acceleration_sign) * float(gravity_magnitude) *
                   direction / np.linalg.norm(direction))
        return GravityEstimate(
            gravity=gravity,
            mean_acceleration=mean_acceleration,
            acceleration_std=acceleration_std,
            mean_gyro_norm=mean_gyro_norm,
            sample_count=self.sample_count,
            duration=self.duration,
        )