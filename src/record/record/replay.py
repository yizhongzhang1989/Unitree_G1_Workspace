"""封口末端轨迹的纯读取、余弦缓入与时间插值"""

from __future__ import annotations

import math

import numpy as np

ARM_LEN = 14
GRIP_LEN = 2


def load_commands(session) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """读取 base4 + 双臂 pose14 + grip2，返回时间、末端姿态和夹爪"""
    if not session.sealed:
        raise ValueError('session 尚未封口，缺少 DONE')
    t, data = session.table('motion_control_command')
    if data.ndim != 2 or data.shape[1] < 20:
        raise ValueError('motion_control_command 需要至少 20 列末端布局')
    upper = data[:, 4:20]
    valid = np.isfinite(t) & np.isfinite(upper).all(axis=1)
    return t[valid], upper[valid, :ARM_LEN], upper[valid, ARM_LEN:]


def _normalize_quat(quaternion: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion, dtype=float)
    length = float(np.linalg.norm(quaternion))
    if not math.isfinite(length) or length <= 0.0:
        raise ValueError('末端四元数需要有限的非零长度')
    return quaternion / length


def quat_slerp(start: np.ndarray, end: np.ndarray, u: float) -> np.ndarray:
    """单位 xyzw 四元数的最短弧球面插值"""
    first = _normalize_quat(start)
    last = _normalize_quat(end)
    dot = float(np.dot(first, last))
    if dot < 0.0:
        last, dot = -last, -dot
    if dot > 0.9995:
        return _normalize_quat(first + u * (last - first))
    theta = math.acos(min(1.0, dot))
    sin_theta = math.sin(theta)
    return (math.sin((1.0 - u) * theta) / sin_theta) * first + (math.sin(u * theta) / sin_theta) * last


def lerp_arm(start: np.ndarray, end: np.ndarray, u: float) -> np.ndarray:
    """双臂 xyz 线性插值，xyzw 最短弧球面插值"""
    start, end = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
    if start.shape != (ARM_LEN,) or end.shape != (ARM_LEN,):
        raise ValueError('双臂末端姿态需要 14 维')
    result = start + u * (end - start)
    result[3:7] = quat_slerp(start[3:7], end[3:7], u)
    result[10:14] = quat_slerp(start[10:14], end[10:14], u)
    return result


def ramp(start: np.ndarray, end: np.ndarray, seconds: float, hz: float) -> np.ndarray:
    """双臂末端姿态余弦缓入，首帧为第一拍，末帧精确到达目标"""
    n = max(int(round(seconds * hz)), 1)
    us = 0.5 - 0.5 * np.cos(np.linspace(0.0, np.pi, n + 1)[1:])
    return np.array([lerp_arm(start, end, float(u)) for u in us])


class Playback:
    """按经过时间取样一段录制好的轨迹。"""

    def __init__(self, t: np.ndarray, arm: np.ndarray, grip: np.ndarray,
                 t0: float, t1: float, speed: float = 1.0) -> None:
        t = np.asarray(t, dtype=float)
        arm = np.asarray(arm, dtype=float)
        grip = np.asarray(grip, dtype=float)
        if t.ndim != 1 or arm.shape != (t.size, ARM_LEN) or grip.shape != (t.size, GRIP_LEN):
            raise ValueError('轨迹需要时间列、14 维末端姿态和 2 夹爪')
        if not np.isfinite(t).all() or not np.isfinite(arm).all() or not np.isfinite(grip).all():
            raise ValueError('轨迹含无效姿态、夹爪或时间')
        if np.any(np.diff(t) < 0.0):
            raise ValueError('轨迹时间必须递增')
        if not np.isfinite(t0) or not np.isfinite(t1) or t1 <= t0:
            raise ValueError('时间窗口需要有限的 t0 < t1')
        if not np.isfinite(speed) or not 0.05 <= speed <= 2.0:
            raise ValueError(f'速度倍率 {speed} 超出 0.05~2.0')
        m = (t >= t0) & (t <= t1)
        if m.sum() < 2:
            raise ValueError(f'{t0}~{t1} 区间里只有 {int(m.sum())} 条指令，放不了')
        self.t = t[m] - t[m][0]
        self.arm = arm[m].copy()
        for pose in self.arm:
            pose[3:7] = _normalize_quat(pose[3:7])
            pose[10:14] = _normalize_quat(pose[10:14])
        self.grip = grip[m]
        if self.t[-1] <= 0.0:
            raise ValueError('轨迹需要两个不同时刻，放不了')
        self.speed = float(speed)

    @property
    def duration(self) -> float:
        return float(self.t[-1]) / self.speed

    def sample(self, elapsed: float) -> tuple[np.ndarray, np.ndarray, bool]:
        """返回 (双臂末端姿态 14, 夹爪 2, 是否已放完)"""
        want = max(0.0, elapsed) * self.speed
        if want >= self.t[-1]:
            return self.arm[-1], self.grip[-1], True
        index = int(np.searchsorted(self.t, want, side='right')) - 1
        index = max(0, min(index, self.t.size - 2))
        span = self.t[index + 1] - self.t[index]
        u = 0.0 if span <= 0 else float((want - self.t[index]) / span)
        arm = lerp_arm(self.arm[index], self.arm[index + 1], u)
        grip = self.grip[index] + u * (self.grip[index + 1] - self.grip[index])
        return arm, grip, False
