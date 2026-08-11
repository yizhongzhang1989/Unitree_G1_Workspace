"""读取导出的重力表，不依赖 Pinocchio。

标定工作台用 Pinocchio 生成这张表，运行时只需要沿着它走一遍正运动学，所以这里是
纯 numpy 的一份，节点导入它不会把整个刚体动力学库拉进来。
"""

from pathlib import Path
from typing import Dict

import numpy as np
from numpy.typing import ArrayLike
import yaml


def load_gravity_table(path: str) -> Dict[str, object]:
    """Read the exported file, with or without its ROS parameter wrapper."""
    with open(Path(path).expanduser(), "r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if len(document) == 1:
        only = next(iter(document.values()))
        if isinstance(only, dict) and "ros__parameters" in only:
            document = only["ros__parameters"]
    for key in ("imu_to_torso", "left", "right"):
        if key not in document:
            raise ValueError("gravity table is missing %s" % key)
    return document


def axis_rotation(axis: ArrayLike, angle: float) -> np.ndarray:
    """Rodrigues rotation about a unit axis."""
    unit = np.asarray(axis, dtype=float)
    cross = np.array([[0.0, -unit[2], unit[1]],
                      [unit[2], 0.0, -unit[0]],
                      [-unit[1], unit[0], 0.0]])
    return (np.eye(3) + np.sin(angle) * cross +
            (1.0 - np.cos(angle)) * (cross @ cross))


def gravity_from_table(table: Dict[str, object], side: str, q: ArrayLike,
                       gravity: ArrayLike) -> np.ndarray:
    """Joint torques the exported chain produces, without the torque bias."""
    chain = table[side]
    angles = np.asarray(q, dtype=float)
    gravity_vector = np.asarray(gravity, dtype=float)
    masses = np.asarray(chain["mass"], dtype=float)
    count = masses.size

    rotation = np.eye(3)
    translation = np.zeros(3)
    origins = np.zeros((count, 3))
    axes = np.zeros((count, 3))
    moments = np.zeros((count, 3))
    for index in range(count):
        block = slice(3 * index, 3 * index + 3)
        translation = translation + rotation @ np.asarray(
            chain["origin_xyz"][block], dtype=float)
        rotation = rotation @ np.asarray(
            chain["origin_rotation"][9 * index:9 * index + 9],
            dtype=float).reshape(3, 3)
        axis = np.asarray(chain["axis"][block], dtype=float)
        rotation = rotation @ axis_rotation(axis, float(angles[index]))
        origins[index] = translation
        axes[index] = rotation @ axis
        moments[index] = masses[index] * (
            translation + rotation @ np.asarray(chain["com"][block], dtype=float))

    torque = np.zeros(count, dtype=float)
    for index in range(count):
        downstream_mass = float(np.sum(masses[index:]))
        downstream_moment = np.sum(moments[index:], axis=0)
        torque[index] -= axes[index] @ np.cross(
            downstream_moment - downstream_mass * origins[index], gravity_vector)
    return torque


def sensor_orientation_from_table(table: Dict[str, object], side: str,
                                  q: ArrayLike) -> np.ndarray:
    """Rotation from the force sensor frame to the torso frame."""
    chain = table[side]
    if "payload_origin_rotation" not in chain:
        raise ValueError(
            "gravity table predates the force sensor mount; re-export it")
    angles = np.asarray(q, dtype=float)
    rotation = np.eye(3)
    for index in range(len(chain["mass"])):
        block = slice(3 * index, 3 * index + 3)
        rotation = rotation @ np.asarray(
            chain["origin_rotation"][9 * index:9 * index + 9],
            dtype=float).reshape(3, 3)
        rotation = rotation @ axis_rotation(
            np.asarray(chain["axis"][block], dtype=float), float(angles[index]))
    return rotation @ np.asarray(
        chain["payload_origin_rotation"], dtype=float).reshape(3, 3)
