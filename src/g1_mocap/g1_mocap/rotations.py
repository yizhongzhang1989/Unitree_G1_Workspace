"""四元数工具。全部按 **wxyz** 顺序，与 MuJoCo / 训练侧一致。

只留重定向真正用到的两个函数。本包刻意不依赖任何机器人策略包——那边有一份更全的
同名模块，两边逐位同义，但让通用能力反过来依赖策略层是不对的。
"""

from __future__ import annotations

import math

import numpy as np


def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = math.hypot(*q)
    if not math.isfinite(n) or n < 1e-9:
        raise ValueError(f'四元数非法: {q}')
    return np.asarray(q, dtype=np.float64) / n


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


def quat_to_mat(q: np.ndarray) -> np.ndarray:
    """(w, x, y, z) -> 3x3 旋转矩阵。"""
    w, x, y, z = quat_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
