"""Passive pose capture for hand-guided arm calibration."""

from typing import Optional, Sequence

import numpy as np


class PassivePoseCapture:
    """Emit one pose after selected joints move and settle at a new location."""

    def __init__(
        self,
        selected_indices: Sequence[int],
        *,
        movement_threshold: float = 0.08,
        minimum_pose_distance: float = 0.12,
        settled_velocity: float = 0.04,
        settle_duration: float = 0.6,
    ) -> None:
        indices = np.asarray(selected_indices, dtype=int)
        if indices.ndim != 1 or indices.size == 0:
            raise ValueError("at least one selected joint index is required")
        if np.any(indices < 0) or np.any(indices >= 14):
            raise ValueError("selected joint indices must be in [0, 14)")
        self.indices = np.unique(indices)
        self.movement_threshold = float(movement_threshold)
        self.minimum_pose_distance = float(minimum_pose_distance)
        self.settled_velocity = float(settled_velocity)
        self.settle_duration = float(settle_duration)
        self._anchor: Optional[np.ndarray] = None
        self._last_pose: Optional[np.ndarray] = None
        self._moved = False
        self._settled_since: Optional[float] = None

    def reset(self, current_position: Optional[Sequence[float]] = None) -> None:
        self._anchor = (None if current_position is None
                        else self._vector(current_position, "position"))
        self._last_pose = None
        self._moved = False
        self._settled_since = None

    def update(
        self,
        timestamp: float,
        position: Sequence[float],
        velocity: Sequence[float],
    ) -> Optional[np.ndarray]:
        position_array = self._vector(position, "position")
        velocity_array = self._vector(velocity, "velocity")
        selected_position = position_array[self.indices]
        if self._anchor is None:
            self._anchor = position_array.copy()
            return None

        displacement = np.max(np.abs(
            selected_position - self._anchor[self.indices]))
        moving = np.max(np.abs(velocity_array[self.indices])) > self.settled_velocity
        if displacement >= self.movement_threshold or moving:
            self._moved = True
        if moving or not self._moved:
            self._settled_since = None
            return None
        if self._settled_since is None:
            self._settled_since = float(timestamp)
            return None
        if float(timestamp) - self._settled_since < self.settle_duration:
            return None

        if self._last_pose is not None:
            distance = np.linalg.norm(
                selected_position - self._last_pose[self.indices])
            if distance < self.minimum_pose_distance:
                self._anchor = position_array.copy()
                self._moved = False
                self._settled_since = None
                return None
        captured = position_array.copy()
        self._last_pose = captured
        self._anchor = captured.copy()
        self._moved = False
        self._settled_since = None
        return captured

    @staticmethod
    def _vector(value: Sequence[float], name: str) -> np.ndarray:
        array = np.asarray(value, dtype=float)
        if array.shape != (14,) or not np.all(np.isfinite(array)):
            raise ValueError("%s must contain 14 finite values" % name)
        return array