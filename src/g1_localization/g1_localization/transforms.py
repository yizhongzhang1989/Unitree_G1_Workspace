"""齐次变换与四元数的最小工具集，不依赖 rclpy，便于单测。

约定：四元数一律 ROS 的 `(x, y, z, w)` 顺序；`T` 是 4x4 齐次矩阵，
`T_a_b` 读作「b 在 a 系下的位姿」，于是 `T_a_c = T_a_b @ T_b_c`。
"""

from __future__ import annotations

import math

import numpy as np


def quat_to_mat(q) -> np.ndarray:
    """`(x, y, z, w)` -> 3x3 旋转矩阵。四元数会先归一化。"""
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise ValueError('四元数模长为零')
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def mat_to_quat(m: np.ndarray) -> np.ndarray:
    """3x3 旋转矩阵 -> `(x, y, z, w)`。走 Shepperd 的分支法，避开退化。"""
    m = np.asarray(m, dtype=np.float64)
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        q = np.array([(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
                      (m[1, 0] - m[0, 1]) / s, 0.25 * s])
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        q = np.array([0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s,
                      (m[2, 1] - m[1, 2]) / s])
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        q = np.array([(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s,
                      (m[0, 2] - m[2, 0]) / s])
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        q = np.array([(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s,
                      (m[1, 0] - m[0, 1]) / s])
    n = np.linalg.norm(q)
    if n < 1e-12:
        raise ValueError('旋转矩阵退化，无法取四元数')
    q = q / n
    return q if q[3] >= 0.0 else -q


def make_tf(translation, quat) -> np.ndarray:
    t = np.eye(4, dtype=np.float64)
    t[:3, :3] = quat_to_mat(quat)
    t[:3, 3] = np.asarray(translation, dtype=np.float64)
    return t


def invert(t: np.ndarray) -> np.ndarray:
    """刚体变换求逆。比 `np.linalg.inv` 稳，也更快。"""
    out = np.eye(4, dtype=np.float64)
    r = t[:3, :3]
    out[:3, :3] = r.T
    out[:3, 3] = -r.T @ t[:3, 3]
    return out


def yaw_of(m: np.ndarray) -> float:
    """旋转矩阵的 yaw（绕 z）。pitch 接近 ±90° 时会退化，躯干不会到那儿。"""
    return math.atan2(m[1, 0], m[0, 0])


def rot_z(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def level_frame(t_ref_body: np.ndarray) -> np.ndarray:
    """由一个参考位姿造出「同原点、只保留 yaw」的水平系，返回该系在参考系下的位姿。

    直接拿 `t_ref_body` 当世界原点是不对的：躯干那一刻的 roll/pitch 会被一起冻进去，
    世界系的 z 轴就不再铅垂，之后所有高度和水平距离都是斜的。上游 Point-LIO 已经用
    `gravity_align` 把自己的 z 轴对到了重力，这里只需把 roll/pitch 丢掉、留下 yaw。
    """
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = rot_z(yaw_of(t_ref_body[:3, :3]))
    out[:3, 3] = t_ref_body[:3, 3]
    return out


def rigid_point_velocity(v_ref_a, omega_ref, p_ref_a, p_ref_b) -> np.ndarray:
    """同一刚体上由 a 点速度求 b 点速度：`v_b = v_a + ω × (p_b - p_a)`。

    全部在同一个参考系（通常是世界系）下表达。雷达/IMU 装在头上、离躯干原点 0.4 m 以上，
    转头带来的杠杆速度不可忽略，直接把 IMU 的线速度当躯干的会偏。
    """
    v_ref_a = np.asarray(v_ref_a, dtype=np.float64)
    omega_ref = np.asarray(omega_ref, dtype=np.float64)
    lever = np.asarray(p_ref_b, dtype=np.float64) - np.asarray(p_ref_a, dtype=np.float64)
    return v_ref_a + np.cross(omega_ref, lever)


def body_twist(v_ref_a, omega_a, t_ref_a, t_a_b):
    """换算刚体上另一点的 twist，并把它表达到那一点自己的系里。

    输入是 Point-LIO 那种**混着的** twist：``v_ref_a`` 是 a 点原点的线速度、在参考系下
    表达；``omega_a`` 是角速度、在 a 自己的体系下表达。输出 `(v_b, omega_b)` 两者都在
    b 系，也就是 ``nav_msgs/Odometry`` 对 ``child_frame_id`` 的约定。

    ``t_a_b`` 是常量外参，所以只有杠杆臂那一项，没有相对运动项。
    """
    r_ref_a, r_a_b = t_ref_a[:3, :3], t_a_b[:3, :3]
    omega_a = np.asarray(omega_a, dtype=np.float64)
    v_ref_b = rigid_point_velocity(v_ref_a, r_ref_a @ omega_a,
                                   t_ref_a[:3, 3], (t_ref_a @ t_a_b)[:3, 3])
    return r_a_b.T @ (r_ref_a.T @ v_ref_b), r_a_b.T @ omega_a
