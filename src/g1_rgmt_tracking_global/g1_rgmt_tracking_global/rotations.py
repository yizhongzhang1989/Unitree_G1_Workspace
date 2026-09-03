"""四元数与旋转工具。全部按 **wxyz** 顺序，与 MuJoCo / 训练侧一致。

ROS 的 ``geometry_msgs/Quaternion`` 是 xyzw，只在节点边界上转一次，
内部不再混用——两套顺序在同一份代码里出现过一次就会错一次。
"""

from __future__ import annotations

import math

import numpy as np


def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(q))
    if not math.isfinite(n) or n < 1e-9:
        raise ValueError(f'四元数非法: {q}')
    return np.asarray(q, dtype=np.float64) / n


def quat_from_xyzw(q_xyzw) -> np.ndarray:
    """ROS (x, y, z, w) -> 内部 (w, x, y, z)。"""
    x, y, z, w = (float(v) for v in q_xyzw)
    return quat_normalize(np.array([w, x, y, z]))


def quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_to_mat(q: np.ndarray) -> np.ndarray:
    """(w, x, y, z) -> 3x3 旋转矩阵。支持前置批量维。"""
    q = np.asarray(q, dtype=np.float64)
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    mat = np.empty(q.shape[:-1] + (3, 3), dtype=np.float64)
    mat[..., 0, 0] = 1 - 2 * (y * y + z * z)
    mat[..., 0, 1] = 2 * (x * y - w * z)
    mat[..., 0, 2] = 2 * (x * z + w * y)
    mat[..., 1, 0] = 2 * (x * y + w * z)
    mat[..., 1, 1] = 1 - 2 * (x * x + z * z)
    mat[..., 1, 2] = 2 * (y * z - w * x)
    mat[..., 2, 0] = 2 * (x * z - w * y)
    mat[..., 2, 1] = 2 * (y * z + w * x)
    mat[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return mat


def quat_from_mat(mat: np.ndarray) -> np.ndarray:
    """3x3 旋转矩阵 -> (w, x, y, z)。

    按迹分四支，永远从**最大**的那个分量开出平方根：只用 ``w`` 那一支时，
    旋转接近 180 度会让 ``w -> 0``，除以它就把数值噪声放大成几度的姿态误差。
    """
    m = np.asarray(mat, dtype=np.float64)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = np.array([0.25 * s, (m[2, 1] - m[1, 2]) / s,
                      (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s])
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        q = np.array([(m[2, 1] - m[1, 2]) / s, 0.25 * s,
                      (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s])
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        q = np.array([(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s,
                      0.25 * s, (m[1, 2] + m[2, 1]) / s])
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        q = np.array([(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s,
                      (m[1, 2] + m[2, 1]) / s, 0.25 * s])
    return quat_normalize(q)


def rotate_inverse(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """把世界系向量转到 q 所定义的局部系，即 ``R(q)^T @ v``。支持批量。"""
    return np.einsum('...ji,...j->...i', quat_to_mat(q), v)


def yaw_quat(q: np.ndarray) -> np.ndarray:
    """只保留绕 z 的分量。用来把参考动作对齐到机器人当前朝向。"""
    w, x, y, z = (q[..., 0], q[..., 1], q[..., 2], q[..., 3])
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    half = 0.5 * yaw
    zeros = np.zeros_like(half)
    return np.stack([np.cos(half), zeros, zeros, np.sin(half)], axis=-1)


def quat_from_axis(axis: str, angle: float) -> np.ndarray:
    """绕单轴的旋转四元数，用于腰部三轴的正运动学。"""
    half = 0.5 * float(angle)
    s = math.sin(half)
    return {
        'x': np.array([math.cos(half), s, 0.0, 0.0]),
        'y': np.array([math.cos(half), 0.0, s, 0.0]),
        'z': np.array([math.cos(half), 0.0, 0.0, s]),
    }[axis]


def quat_apply(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """把局部系向量转到 q 所在的父系，即 ``R(q) @ v``。是 rotate_inverse 的逆运算。

    参考 key body 的局部化要用它：先取 ``rel = ref_pos_w - robot_pos_w``（世界系），
    再用机器人 anchor 的 yaw 逆四元数把 rel 转进局部系。
    """
    return np.einsum('...ij,...j->...i', quat_to_mat(q), v)


def torso_quat_from_pelvis(pelvis_quat: np.ndarray, waist_yaw: float,
                           waist_roll: float, waist_pitch: float) -> np.ndarray:
    """由盆骨姿态和腰部三轴推出躯干姿态。

    真机 IMU 装在盆骨，而策略的锚刚体是 ``torso_link``。模型里这条链是
    ``pelvis -> Rz(waist_yaw) -> Rx(waist_roll) -> Ry(waist_pitch) -> torso_link``，
    中间三个 body 都没有 ``quat`` 属性（只有平移），所以姿态就是三次纯轴旋转的复合，
    不需要完整的运动学库。
    """
    q = quat_mul(pelvis_quat, quat_from_axis('z', waist_yaw))
    q = quat_mul(q, quat_from_axis('x', waist_roll))
    return quat_mul(q, quat_from_axis('y', waist_pitch))


# waist_roll_link 相对 waist_yaw_link 的固定偏移（MJCF body_pos）。链上另外两段都是 0，
# 所以 torso 原点只被 waist_yaw 转一次，与 roll/pitch 无关。
WAIST_OFFSET = np.array([-0.003964, 0.0, 0.044])


def torso_pos_from_pelvis(pelvis_pos: np.ndarray, pelvis_quat: np.ndarray,
                          waist_yaw: float) -> np.ndarray:
    """由盆骨位姿和腰偏航推出躯干原点的世界位置。

    里程计给的是盆骨（``/dog_odom`` 的 robot_center 姿态逐位等于 pelvis），而观测要的
    anchor 是 ``torso_link``。偏移只有 4.4 cm，但它会**整体平移** key body 通道——
    那 15 维本身量级就是几十厘米，漏掉这一段等于给漂移估计加一个恒定偏置。
    """
    q = quat_mul(quat_normalize(pelvis_quat), quat_from_axis('z', float(waist_yaw)))
    return np.asarray(pelvis_pos, dtype=np.float64) + quat_apply(q, WAIST_OFFSET)
