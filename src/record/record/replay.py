"""回放的纯逻辑：从 session 取指令轨迹、生成缓入段、按时间取样。

**不 import rclpy**，方便单测。ROS 那一层在 `data_node.py`。

回放只发**上肢**：`/motion_control/command` 的长度 14（双臂位姿）与长度 2（双夹爪）。
**绝不发长度 4 或 20** —— 那两个会带上 `vx/vy/wz/h`，等于让机器人走路。录的是桌面
操作，回放时机器人应该站着不动。

姿态用 slerp 不用线性插值：四元数线性插值再归一化在大转角时角速度不均匀，缓入段
恰恰可能有大转角（当前位姿距离录制起点很远）。
"""

from __future__ import annotations

import numpy as np

#: `/motion_control/command` 长度 20 的布局
BASE = slice(0, 4)          # vx, vy, wz, h —— 回放不碰
LEFT = slice(4, 11)         # x,y,z,qx,qy,qz,qw
RIGHT = slice(11, 18)
GRIP = slice(18, 20)

ARM_LEN = 14
GRIP_LEN = 2


def load_commands(session) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """返回 (时刻, 双臂位姿 14, 双夹爪 2)。

    时刻用的是录制时的接收时刻 —— 指令是收到那一刻生效的，回放按同样的间隔重发
    就能复现当时的运动。
    """
    t, data = session.table('motion_control_command')
    if data.shape[1] < 20:
        raise ValueError(f'指令表只有 {data.shape[1]} 列，不是 20 列的全量指令')
    return t, np.hstack([data[:, LEFT], data[:, RIGHT]]), data[:, GRIP]


def quat_slerp(a: np.ndarray, b: np.ndarray, u: float) -> np.ndarray:
    """球面线性插值。a、b 为 xyzw。"""
    a = a / (np.linalg.norm(a) or 1.0)
    b = b / (np.linalg.norm(b) or 1.0)
    dot = float(np.dot(a, b))
    if dot < 0.0:                      # 取短弧，否则会绕远路转一圈
        b, dot = -b, -dot
    if dot > 0.9995:                   # 几乎同向，线性插值足够且避免除零
        out = a + u * (b - a)
        return out / (np.linalg.norm(out) or 1.0)
    theta = np.arccos(np.clip(dot, -1.0, 1.0))
    s = np.sin(theta)
    return (np.sin((1 - u) * theta) / s) * a + (np.sin(u * theta) / s) * b


def lerp_arm(start: np.ndarray, end: np.ndarray, u: float) -> np.ndarray:
    """一帧双臂位姿的插值：平移线性、姿态 slerp。"""
    out = np.empty(ARM_LEN)
    for off in (0, 7):
        out[off:off + 3] = start[off:off + 3] + u * (end[off:off + 3] - start[off:off + 3])
        out[off + 3:off + 7] = quat_slerp(start[off + 3:off + 7], end[off + 3:off + 7], u)
    return out


def ramp(start: np.ndarray, end: np.ndarray, seconds: float, hz: float) -> np.ndarray:
    """从当前位姿缓入到录制起点。返回 (n, 14)。

    机器人现在在哪、录制是从哪开始的，两者可能差很远。直接发录制的第一帧，
    出口限速会把它摊成一段全速运动 —— 看着像机器人自己抽了一下。
    """
    n = max(int(round(seconds * hz)), 1)
    # 余弦缓入缓出，两端速度为零
    us = 0.5 - 0.5 * np.cos(np.linspace(0.0, np.pi, n + 1)[1:])
    return np.array([lerp_arm(start, end, float(u)) for u in us])


def pose_from_status(status: dict) -> np.ndarray:
    """从 `~/status` 取当前双臂位姿，拼成缓入起点用的 14 元向量。

    **`limited_pose` 是 `{'left': [7], 'right': [7]}` 的 dict，不是扁平的 14 元列表。**
    按扁平列表写会在 `len(pose) != 14` 上静默失败 —— dict 的长度是 2。
    """
    pose = status.get('limited_pose')
    if not isinstance(pose, dict):
        raise ValueError(f'limited_pose 不是 dict，拿到 {type(pose).__name__}: {pose!r}')
    out = []
    for side in ('left', 'right'):
        v = pose.get(side)
        if not isinstance(v, (list, tuple)) or len(v) != 7:
            raise ValueError(f'limited_pose[{side!r}] 应该是 7 个数，拿到 {v!r}')
        out.extend(float(x) for x in v)
    return np.asarray(out, dtype=float)


class Playback:
    """按经过时间取样一段录制好的轨迹。"""

    def __init__(self, t: np.ndarray, arm: np.ndarray, grip: np.ndarray,
                 t0: float, t1: float, speed: float = 1.0) -> None:
        if not 0.05 <= speed <= 2.0:
            raise ValueError(f'速度倍率 {speed} 超出 0.05~2.0')
        m = (t >= t0) & (t <= t1)
        if m.sum() < 2:
            raise ValueError(f'{t0}~{t1} 区间里只有 {int(m.sum())} 条指令，放不了')
        self.t = t[m] - t[m][0]
        self.arm = arm[m]
        self.grip = grip[m]
        self.speed = float(speed)

    @property
    def duration(self) -> float:
        return float(self.t[-1]) / self.speed

    def sample(self, elapsed: float) -> tuple[np.ndarray, np.ndarray, bool]:
        """返回 (双臂 14, 夹爪 2, 是否已放完)。"""
        want = elapsed * self.speed
        if want >= self.t[-1]:
            return self.arm[-1], self.grip[-1], True
        i = int(np.searchsorted(self.t, want, side='right')) - 1
        i = max(0, min(i, self.t.size - 2))
        span = self.t[i + 1] - self.t[i]
        u = 0.0 if span <= 0 else float((want - self.t[i]) / span)
        arm = lerp_arm(self.arm[i], self.arm[i + 1], u)
        # 夹爪是连续开度，线性插值即可
        grip = self.grip[i] + u * (self.grip[i + 1] - self.grip[i])
        return arm, grip, False
