"""里程计融合：把 10 Hz 的雷达定位和 500 Hz 的 ``/dog_odom`` 合成高频且不漂的躯干世界位姿。

单独用哪一路都不行：

- ``/dog_odom`` 是航位推算，静止 20 s 只漂 0.04~0.07 mm，但**随行走距离无界漂移**；
- 雷达 SLAM 有绝对参照不漂（静止 20 s 最大 8 mm），但只有 **10 Hz**，且 stamp 比
  ``/dog_odom`` 滞后约 34 ms。10 Hz 直接喂 50 Hz 控制环等于每 5 拍才更新一次，
  中间 4 拍全是零阶保持，位置台阶会被策略读成"我在瞬移"。

融合式::

    T_world_torso(t) = T_world_odom(t_k) @ T_odom_torso(t)

左边那项是 odom 的**累积漂移本身**，属于慢变量，可以滤得很狠也不引入动态滞后——
全部动态都由 500 Hz 的快通道承担。这就是"高频 + 有界漂移"的来源。

姿态不参与融合，这个模块也**只输出位置**：roll/pitch 由 IMU 直接给，比任何里程计都准，
anchor 姿态由盆骨 IMU 加腰三轴 FK 算得。内部仍估计 yaw 偏置，因为 odom 的偏航同样会漂，
而它会通过修正量影响位置。
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass

import numpy as np

from .rotations import quat_apply, quat_normalize


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    pos: np.ndarray
    quat: np.ndarray


def _yaw_of(q: np.ndarray) -> float:
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _yaw_quat(yaw: float) -> np.ndarray:
    half = 0.5 * yaw
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)])


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class OdometryFuser:
    """快慢双通道位姿融合。不依赖 ROS，便于离线对拍与单测。

    两路都必须已经归算到 **torso_link**：雷达那路本来就是 ``world -> torso_link``；
    ``/dog_odom`` 给的是盆骨，调用方要先用 ``torso_pos_from_pelvis`` 做一次正运动学。
    """

    def __init__(
        self,
        mode: str = 'fused',
        odom_timeout_s: float = 0.2,
        lidar_timeout_s: float = 1.0,
        buffer_s: float = 2.0,
        correction_tau_s: float = 2.0,
        max_correction_step_m: float = 0.05,
    ) -> None:
        if mode not in ('fused', 'odom_only', 'lidar_only'):
            raise ValueError(f'未知里程计模式: {mode}')
        self._mode = mode
        self._odom_timeout = float(odom_timeout_s)
        self._lidar_timeout = float(lidar_timeout_s)
        self._buffer_s = float(buffer_s)
        self._tau = float(correction_tau_s)
        self._max_step = float(max_correction_step_m)

        self._odom: list[PoseSample] = []
        self._odom_stamps: list[float] = []
        self._last_odom: PoseSample | None = None
        self._last_lidar_stamp: float = -1.0

        # T_world_odom 的 yaw + 平移，慢通道估计
        self._corr_pos = np.zeros(3)
        self._corr_yaw = 0.0
        self._corr_ready = mode != 'fused'

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def correction(self) -> tuple[np.ndarray, float]:
        """当前的漂移修正量，供 status 上报。范数持续增长就是 odom 在漂。"""
        return self._corr_pos.copy(), self._corr_yaw

    def push_odom(self, stamp: float, pos, quat) -> None:
        sample = PoseSample(float(stamp), np.asarray(pos, dtype=np.float64),
                            quat_normalize(quat))
        # 时间戳倒退说明发布端重启或系统时钟跳变，缓存里的历史不再可比，整体丢弃。
        if self._odom_stamps and sample.stamp < self._odom_stamps[-1]:
            self._odom.clear()
            self._odom_stamps.clear()
        self._odom.append(sample)
        self._odom_stamps.append(sample.stamp)
        self._last_odom = sample
        cutoff = sample.stamp - self._buffer_s
        drop = bisect.bisect_left(self._odom_stamps, cutoff)
        if drop > 0:
            del self._odom[:drop]
            del self._odom_stamps[:drop]

    def push_lidar(self, stamp: float, pos, quat) -> bool:
        """并入一帧雷达定位。返回是否被采纳。

        雷达 stamp 滞后约 34 ms，必须拿**同一时刻**的 odom 去解修正量，
        直接和当前 odom 相除会把这 34 ms 的运动算进漂移里。
        """
        if self._mode == 'odom_only':
            return False
        stamp = float(stamp)
        pos = np.asarray(pos, dtype=np.float64)
        quat = quat_normalize(quat)
        self._last_lidar_stamp = stamp

        if self._mode == 'lidar_only':
            self._corr_pos = pos
            self._corr_yaw = _yaw_of(quat)
            self._corr_ready = True
            return True

        paired = self._sample_at(stamp)
        if paired is None:
            return False

        # T_world_odom = T_world_torso(雷达) @ inv(T_odom_torso)，只取 yaw 与平移
        yaw = _wrap(_yaw_of(quat) - _yaw_of(paired.quat))
        target_pos = pos - quat_apply(_yaw_quat(yaw), paired.pos)

        if not self._corr_ready:
            self._corr_pos = target_pos
            self._corr_yaw = yaw
            self._corr_ready = True
            return True

        # 慢通道低通。10 Hz 更新、tau 秒时间常数，单帧跳变还要再钳一道：
        # 雷达在空旷处会退化且协方差恒为 0 给不出预警，只能靠幅度拦。
        alpha = 1.0 - math.exp(-0.1 / max(self._tau, 1e-3))
        delta = target_pos - self._corr_pos
        norm = float(np.linalg.norm(delta))
        if norm > self._max_step:
            delta *= self._max_step / norm
        self._corr_pos = self._corr_pos + alpha * delta
        self._corr_yaw = _wrap(self._corr_yaw + alpha * _wrap(yaw - self._corr_yaw))
        return True

    def _sample_at(self, stamp: float) -> PoseSample | None:
        if not self._odom_stamps:
            return None
        i = bisect.bisect_left(self._odom_stamps, stamp)
        cands = [j for j in (i - 1, i) if 0 <= j < len(self._odom)]
        if not cands:
            return None
        best = min(cands, key=lambda j: abs(self._odom_stamps[j] - stamp))
        # 缓存里没有足够接近的帧就放弃这一帧雷达，宁可不更新也不要配错时刻
        if abs(self._odom_stamps[best] - stamp) > 0.05:
            return None
        return self._odom[best]

    def torso_position(self) -> np.ndarray | None:
        """最新的躯干世界位置。返回 None 表示尚未就绪。

        只给位置：anchor 姿态请用 IMU + 腰三轴 FK，那条路径比里程计准得多。
        """
        if not self._corr_ready:
            return None
        if self._mode == 'lidar_only':
            return self._corr_pos.copy()
        if self._last_odom is None:
            return None
        return self._corr_pos + quat_apply(_yaw_quat(self._corr_yaw), self._last_odom.pos)

    def stale(self, now: float) -> str | None:
        """返回超时原因，None 表示健康。看门狗按它急停。"""
        if self._mode != 'lidar_only':
            if self._last_odom is None:
                return '未收到 /dog_odom'
            if now - self._last_odom.stamp > self._odom_timeout:
                return f'/dog_odom 超时 {now - self._last_odom.stamp:.2f}s'
        if self._mode == 'fused' and not self._corr_ready:
            return '等待首帧雷达定位'
        if self._mode != 'odom_only' and self._last_lidar_stamp >= 0.0:
            gap = now - self._last_lidar_stamp
            if gap > self._lidar_timeout:
                return f'雷达定位超时 {gap:.2f}s'
        return None
