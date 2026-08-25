"""4x4 齐次变换的小工具。没装 transforms3d，也不想为几个函数拖进 scipy。"""

from __future__ import annotations

import math

import cv2
import numpy as np


def rt_to_matrix(rotation, translation) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = np.asarray(rotation, float).reshape(3, 3)
    matrix[:3, 3] = np.asarray(translation, float).reshape(3)
    return matrix


def rvec_tvec_to_matrix(rvec, tvec) -> np.ndarray:
    rotation, _ = cv2.Rodrigues(np.asarray(rvec, float).reshape(3, 1))
    return rt_to_matrix(rotation, np.asarray(tvec, float).reshape(3))


def invert(matrix) -> np.ndarray:
    """SE(3) 的逆：转置旋转即可，别调 np.linalg.inv"""
    matrix = np.asarray(matrix, float)
    rotation = matrix[:3, :3]
    out = np.eye(4)
    out[:3, :3] = rotation.T
    out[:3, 3] = -rotation.T @ matrix[:3, 3]
    return out


def quat_to_matrix(quat) -> np.ndarray:
    """四元数按 ROS 的 [x, y, z, w] 顺序"""
    x, y, z, w = (float(v) for v in quat)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError('四元数模长为零')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(matrix) -> np.ndarray:
    """Shepperd 分支法，避开 trace 接近 -1 时的开根号失稳"""
    rotation = np.asarray(matrix, float)[:3, :3]
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / scale
        x = 0.25 * scale
        y = (rotation[0, 1] + rotation[1, 0]) / scale
        z = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / scale
        x = (rotation[0, 1] + rotation[1, 0]) / scale
        y = 0.25 * scale
        z = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / scale
        x = (rotation[0, 2] + rotation[2, 0]) / scale
        y = (rotation[1, 2] + rotation[2, 1]) / scale
        z = 0.25 * scale
    quat = np.array([x, y, z, w])
    return quat / np.linalg.norm(quat)


def matrix_to_rpy(matrix) -> np.ndarray:
    """URDF 的固定轴 roll-pitch-yaw（等价于内旋 ZYX）"""
    rotation = np.asarray(matrix, float)[:3, :3]
    sy = -rotation[2, 0]
    sy = max(-1.0, min(1.0, sy))
    pitch = math.asin(sy)
    if abs(sy) > 1.0 - 1e-9:                      # 万向锁：roll 和 yaw 简并，把 roll 归零
        return np.array([0.0, pitch, math.atan2(-rotation[0, 1], rotation[1, 1])])
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return np.array([roll, pitch, yaw])


def average_transforms(matrices) -> np.ndarray:
    """平移取算术平均，旋转取四元数的 Markley 平均（M M^T 的最大特征向量）。

    直接对旋转矩阵求平均再正交化在样本分散时会偏，四元数逐个相加又受符号影响
    （q 和 -q 同姿态）。特征向量法对符号免疫，也不需要挑参考。
    """
    matrices = [np.asarray(m, float) for m in matrices]
    if not matrices:
        raise ValueError('没有样本可平均')
    quats = np.stack([matrix_to_quat(m) for m in matrices])
    _, vectors = np.linalg.eigh(quats.T @ quats)
    mean_quat = vectors[:, -1]
    if mean_quat[3] < 0:
        mean_quat = -mean_quat
    translation = np.mean([m[:3, 3] for m in matrices], axis=0)
    return rt_to_matrix(quat_to_matrix(mean_quat), translation)


def transform_delta(a, b) -> tuple[float, float]:
    """两个位姿的差异，返回 (旋转角度°, 平移距离 m)"""
    delta = invert(np.asarray(a, float)) @ np.asarray(b, float)
    cos = (float(np.trace(delta[:3, :3])) - 1.0) / 2.0
    angle = math.degrees(math.acos(max(-1.0, min(1.0, cos))))
    return angle, float(np.linalg.norm(delta[:3, 3]))


def perturb(matrix, delta) -> np.ndarray:
    """右乘一个小增量：delta 前 3 个是旋转向量、后 3 个是平移。

    只用于迭代里的局部参数化，所以直接拿平移当平移，不做 se(3) 指数里那个 V 矩阵。
    """
    delta = np.asarray(delta, float)
    step = rt_to_matrix(cv2.Rodrigues(delta[:3])[0], delta[3:])
    return np.asarray(matrix, float) @ step


def log_delta(matrix) -> np.ndarray:
    """把一个（应该接近单位阵的）变换拆成 [旋转向量, 平移]"""
    matrix = np.asarray(matrix, float)
    return np.concatenate([cv2.Rodrigues(matrix[:3, :3])[0].reshape(3),
                           matrix[:3, 3]])


def deviation(deltas) -> dict:
    """一组 (角度°, 距离 m) 差异的 rms 和最大值，距离出口换成 mm"""
    angles = [d[0] for d in deltas]
    distances = [d[1] for d in deltas]
    return {
        'count': len(deltas),
        'angle_rms_deg': float(np.sqrt(np.mean(np.square(angles)))) if angles else 0.0,
        'angle_max_deg': float(max(angles)) if angles else 0.0,
        'trans_rms_mm': float(np.sqrt(np.mean(np.square(distances))) * 1e3) if distances else 0.0,
        'trans_max_mm': float(max(distances) * 1e3) if distances else 0.0,
    }


def spread(matrices, mean=None) -> dict:
    """一组位姿相对均值的离散度，用来判断标定结果稳不稳"""
    matrices = [np.asarray(m, float) for m in matrices]
    reference = average_transforms(matrices) if mean is None else np.asarray(mean, float)
    deltas = [transform_delta(reference, m) for m in matrices]
    return {**deviation(deltas),
            'per_sample': [{'angle_deg': a, 'trans_mm': d * 1e3} for a, d in deltas]}


def matrix_to_dict(matrix) -> dict:
    matrix = np.asarray(matrix, float)
    quat = matrix_to_quat(matrix)
    rpy = matrix_to_rpy(matrix)
    return {
        'translation': [float(v) for v in matrix[:3, 3]],
        'rotation': [float(v) for v in quat],
        'rpy': [float(v) for v in rpy],
    }


def matrix_from_dict(data) -> np.ndarray:
    return rt_to_matrix(quat_to_matrix(data['rotation']), data['translation'])
