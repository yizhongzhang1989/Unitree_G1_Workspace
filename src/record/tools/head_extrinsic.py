"""Session 级头部外参修订与双 IMU 颈角估计。纯 Python + numpy。"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

OVERRIDE_FILE = 'head_extrinsic_override.json'
MAX_PAIR_GAP_S = 0.1


def read_override(root: str | Path) -> dict | None:
    path = Path(root) / OVERRIDE_FILE
    if not path.is_file():
        return None
    data = json.loads(path.read_text(encoding='utf-8'))
    angle = float(data['head_pitch_rad'])
    mount = data['d435_joint']
    if not math.isfinite(angle) or not all(k in mount for k in ('parent', 'child', 'xyz', 'rpy')):
        raise ValueError(f'{path} 内容无效')
    return data


def unit(vector) -> np.ndarray:
    vector = np.asarray(vector, float)
    length = np.linalg.norm(vector)
    if vector.shape != (3,) or not np.isfinite(vector).all() or length < 1e-6:
        raise ValueError('无效的重力向量')
    return vector / length


def align_vector(source, target) -> np.ndarray:
    """返回把一个单位向量转到另一个单位向量的最短弧旋转。"""
    source, target = unit(source), unit(target)
    cross = np.cross(source, target)
    sine, cosine = np.linalg.norm(cross), float(source @ target)
    if sine < 1e-8:
        if cosine < 0:
            raise ValueError('零位两路重力方向相反')
        return np.eye(3)
    axis = cross / sine
    skew = np.array([[0.0, -axis[2], axis[1]],
                     [axis[2], 0.0, -axis[0]],
                     [-axis[1], axis[0], 0.0]])
    return np.eye(3) + sine * skew + (1.0 - cosine) * (skew @ skew)


def nearest_indices(source: np.ndarray, query: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """每个 query 在有序 source 中的最近下标及绝对时间差。"""
    right = np.searchsorted(source, query, side='left')
    left = np.clip(right - 1, 0, source.size - 1)
    right = np.clip(right, 0, source.size - 1)
    index = np.where(np.abs(source[right] - query) < np.abs(source[left] - query),
                     right, left)
    return index, np.abs(source[index] - query)


def paired_pitches(head, torso, rotation_zero, axis) -> np.ndarray:
    """批量计算配对重力向量的颈角；几何退化的行返回 NaN。"""
    head, torso, axis = np.atleast_2d(head), np.atleast_2d(torso), unit(axis)
    source = (rotation_zero @ (head / np.linalg.norm(head, axis=1)[:, None]).T).T
    target = torso / np.linalg.norm(torso, axis=1)[:, None]
    source -= np.outer(source @ axis, axis)
    target -= np.outer(target @ axis, axis)
    source_norm = np.linalg.norm(source, axis=1)
    target_norm = np.linalg.norm(target, axis=1)
    valid = (source_norm >= 0.1) & (target_norm >= 0.1)
    source[valid] /= source_norm[valid, None]
    target[valid] /= target_norm[valid, None]
    angles = np.full(head.shape[0], np.nan)
    angles[valid] = np.arctan2(
        np.cross(source[valid], target[valid]) @ axis,
        np.einsum('ij,ij->i', source[valid], target[valid]))
    return angles


def estimate_pitch(head_t, head_accel, torso_t, torso_accel,
                   reference: dict, joint, rpy_to_matrix) -> float:
    """时间配对后逐对求相对角，并以圆均值得到一个静态颈角。"""
    if not reference or any(k not in reference for k in ('head_zero', 'torso_zero')):
        raise ValueError('缺少 head_imu_reference')
    head_t, torso_t = np.asarray(head_t), np.asarray(torso_t)
    if head_t.size == 0 or torso_t.size == 0:
        raise ValueError('缺少双 IMU 样本')
    torso_index, gap = nearest_indices(torso_t, head_t)
    head_accel = np.asarray(head_accel, float)
    torso_accel = np.asarray(torso_accel, float)[torso_index]
    head_norm = np.linalg.norm(head_accel, axis=1)
    torso_norm = np.linalg.norm(torso_accel, axis=1)
    valid = gap <= MAX_PAIR_GAP_S
    valid &= np.isfinite(head_accel).all(axis=1) & np.isfinite(torso_accel).all(axis=1)
    valid &= (head_norm > 0.85) & (head_norm < 1.15)
    valid &= (torso_norm > 8.3) & (torso_norm < 11.3)
    if not valid.any():
        raise ValueError('没有时间匹配且模长有效的双 IMU 样本对')

    nominal = rpy_to_matrix([np.pi, 0.05112069379091391, 0.0])
    rotation_zero = align_vector(
        nominal @ unit(reference['head_zero']), reference['torso_zero']) @ nominal
    angles = paired_pitches(head_accel[valid], torso_accel[valid],
                            rotation_zero, joint.axis)
    angles = angles[np.isfinite(angles)]
    if angles.size == 0:
        raise ValueError('没有几何有效的双 IMU 样本对')
    angle = float(np.arctan2(np.sin(angles).mean(), np.cos(angles).mean()))
    if joint.limit is None or not joint.limit[0] <= angle <= joint.limit[1]:
        raise ValueError(f'颈角 {angle:.4f} rad 超出 URDF 限位')
    return angle