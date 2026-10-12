"""Local sensor snapshot for one task control cycle."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True, slots=True)
class RobotState:
    position: np.ndarray
    velocity: np.ndarray
    orientation: tuple[float, float, float, float]
    angular_velocity: tuple[float, float, float]
