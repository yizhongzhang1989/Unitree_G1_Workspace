"""Shared policy contract and orientation geometry."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class PolicySpec:
    joint_names: tuple[str, ...]
    default_pos: np.ndarray
    action_scale: np.ndarray
    obs_dim: int
    action_dim: int
    hidden_name: str
    hidden_shape: tuple[int, ...]


def projected_gravity(quat_xyzw: Sequence[float]) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError(f'四元数非法: {quat_xyzw}')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        -2.0 * (x * z - w * y),
        -2.0 * (y * z + w * x),
        -(1.0 - 2.0 * (x * x + y * y)),
    ])


def spec_matches(spec: PolicySpec, joint_names: Sequence[str]) -> None:
    expected = tuple(joint_names)
    if spec.joint_names != expected:
        raise ValueError(
            'ONNX 的关节顺序和配置不一致，拒绝启动\n'
            f'  配置: {expected}\n  ONNX: {spec.joint_names}')


def two_stage(elapsed, clear_s, total_s, start, via, goal):
    if elapsed < clear_s:
        return start + elapsed / clear_s * (via - start)
    span = total_s - clear_s
    alpha = 1.0 if span <= 0.0 else min((elapsed - clear_s) / span, 1.0)
    return via + alpha * (goal - via)
