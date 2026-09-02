"""实时动捕的参考动作，和 :class:`~..motion_library.MotionClip` 逐位同构。

``tracking_node`` 拿到的是哪一种，只看 :attr:`streaming`：为真表示这段参考没有终点，
不要按"放完回 IDLE"处理。除此之外接口完全一致——``align`` / ``stand_joint_pos`` /
``anchor_pose_world`` / ``reference_window`` 的语义、量纲、坐标系全部照抄。

两个只有实时源才有的问题：

**未来帧从哪来。** 参考窗口的 ``lookahead_steps`` 最大 +15，也就是要 0.3 s 之后的
参考。实时动捕没有未来，唯一诚实的做法是让播放头**落后**最新帧 ``lead_frames`` 拍，
拿延迟换出前瞻。这就是遥操作的端到端延迟下界，砍不掉。

**两个时钟怎么合。** 头显 72/90 Hz，控制环 50 Hz，没有整数比。播放头按控制环的
整数拍推进（保证参考速度恰好是 50 Hz 网格上的差分，和训练侧同一个网格），再用一个
很慢的速率修正把它锁在"最新帧减 lead"上。直接每拍取最新帧会把 WiFi 抖动变成参考的
速度噪声。
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from g1_mocap.stream import SampleBatch

from .rotations import (
    quat_apply,
    quat_conj,
    quat_mul,
    quat_normalize,
    rotate_inverse,
    yaw_quat,
)

GRAVITY_W = np.array([0.0, 0.0, -1.0])


def _angular_velocity(previous: np.ndarray, current: np.ndarray, dt: float) -> np.ndarray:
    """由相邻两帧姿态求世界系角速度。批量：输入 (N, 4)，输出 (N, 3)。"""
    delta = np.stack([quat_mul(c, quat_conj(p)) for p, c in zip(previous, current)])
    delta = np.where(delta[:, :1] < 0.0, -delta, delta)
    vector = delta[:, 1:]
    sine = np.linalg.norm(vector, axis=-1, keepdims=True)
    angle = 2.0 * np.arctan2(sine[..., 0], np.clip(delta[:, 0], -1.0, 1.0))
    axis = np.where(sine > 1e-12, vector / np.maximum(sine, 1e-12), 0.0)
    return axis * (angle / dt)[:, None]


class MocapClip:
    """把一条动捕数据流包成一段永不结束的参考动作。

    Args:
        source: 数据源。只要求提供 ``span`` / ``sample`` / ``stats`` 三个方法——
            现在传的是 ``g1_mocap`` 的 ``FrameBuffer``（订 ``/mocap/frame``）。
        control_dt: 控制周期，必须与策略契约一致。
        lead_frames: 播放头落后最新帧多少拍。要盖住 ``max(lookahead_steps)`` 再留余量。
        stale_timeout_s: 多久没有新帧就算断流。
        resync_slew: 每拍最多修正多少个控制周期的相位。0.02 表示 2%，即 50 s 追回 1 s。
        hard_resync_s: 相位差超过它就直接跳过去——慢修正追不上的那种断流。
    """

    streaming = True

    def __init__(self, source, *, control_dt: float, lead_frames: int,
                 name: str = 'live', stale_timeout_s: float = 0.3,
                 resync_slew: float = 0.02, hard_resync_s: float = 0.5) -> None:
        self._stream = source
        self._dt = float(control_dt)
        self._lead_s = int(lead_frames) * self._dt
        self.name = name
        self._stale_timeout = float(stale_timeout_s)
        self._slew = float(resync_slew) * self._dt
        self._hard = float(hard_resync_s)

        self._align_quat = np.array([1.0, 0.0, 0.0, 0.0])
        self._align_pos = np.zeros(3)
        self._aligned = False
        self._pending: tuple[np.ndarray, np.ndarray] | None = None
        self._origin: float | None = None
        self._origin_frame = 0
        self._last_play = 0.0

    ##
    # MotionClip 的接口
    ##

    @property
    def num_frames(self) -> int:
        """实时源没有终点。给一个大到跑不完的数，配合 ``streaming`` 一起看。"""
        return 1 << 30

    @property
    def duration_s(self) -> float:
        return float('inf')

    @property
    def aligned(self) -> bool:
        return self._aligned

    def clamp(self, frame: int) -> int:
        return int(max(int(frame), 0))

    def align(self, robot_anchor_pos: np.ndarray, robot_anchor_quat: np.ndarray) -> None:
        """记下要对齐到的机器人位姿。

        真正的对齐推迟到**第一次取参考窗口**那一刻——``~/start`` 之后还有几秒 STAND，
        这期间实时参考一直在往前跑，在 ``~/start`` 就把变换定死等于把那几秒的人体
        运动算成跟踪误差。
        """
        self._pending = (np.asarray(robot_anchor_pos, dtype=np.float64).copy(),
                         np.asarray(robot_anchor_quat, dtype=np.float64).copy())
        self._align_quat = np.array([1.0, 0.0, 0.0, 0.0])
        self._align_pos = np.zeros(3)
        self._origin = None
        self._origin_frame = 0
        self._aligned = True

    def stand_joint_pos(self) -> np.ndarray:
        """STAND 阶段插值的目标：人**此刻**的位形。

        故意用实时值而不是 ``~/start`` 那一刻的快照——插完就直接进 RUNNING，两边取
        同一个姿态才不会在切换那一拍跳一下。代价是 STAND 这几秒人得站着别乱动。
        """
        batch = self._require(np.array([self._latest_playable()]))
        return batch.joint_pos[0]

    def anchor_pose_world(self, frame: int) -> tuple[np.ndarray, np.ndarray]:
        if self._origin is None:
            if self._pending is None:
                raise RuntimeError('参考动作尚未对齐，必须先调用 align()')
            return self._pending[0].copy(), self._pending[1].copy()
        batch = self._require(np.array([self._play_time(frame, advance=False)]))
        return (self._to_world(batch.anchor_pos)[0],
                quat_mul(self._align_quat, quat_normalize(batch.anchor_quat[0])))

    def reference_window(self, frame: int, offsets: np.ndarray,
                         robot_anchor_pos: np.ndarray,
                         robot_anchor_quat: np.ndarray) -> np.ndarray:
        if not self._aligned:
            raise RuntimeError('参考动作尚未对齐，必须先调用 align()')
        play = self._play_time(frame, advance=True)
        offsets = np.asarray(offsets, dtype=np.float64)
        # 训练语料的速度定义是 (x[t+1] - x[t]) / dt；实时源也必须逐拍一致。
        times = np.concatenate([play + offsets * self._dt,
                                play + (offsets + 1.0) * self._dt])
        batch = self._require(times)
        n = len(offsets)
        current, following = batch.at(slice(0, n)), batch.at(slice(n, 2 * n))
        self._ensure_alignment(current, offsets)

        root_quat = current.root_quat
        lin_vel_w = (following.root_pos - current.root_pos) / self._dt
        ang_vel_w = _angular_velocity(root_quat, following.root_quat, self._dt)
        gravity = np.broadcast_to(GRAVITY_W, (n, 3))

        # 后 30 维：先把参考搬进机器人坐标系，再转进机器人 anchor 的 yaw 局部系。
        key_pos_w = self._to_world(current.key_pos.reshape(-1, 3)).reshape(n, -1, 3)
        key_vel_w = quat_apply(
            self._align_quat,
            ((following.key_pos - current.key_pos) / self._dt).reshape(-1, 3)).reshape(n, -1, 3)

        inverse = quat_conj(yaw_quat(quat_normalize(robot_anchor_quat)))
        rel = key_pos_w - np.asarray(robot_anchor_pos, dtype=np.float64)
        return np.concatenate([
            rotate_inverse(root_quat, lin_vel_w),
            rotate_inverse(root_quat, ang_vel_w),
            rotate_inverse(root_quat, gravity),
            current.joint_pos,
            quat_apply(inverse, rel.reshape(-1, 3)).reshape(n, -1),
            quat_apply(inverse, key_vel_w.reshape(-1, 3)).reshape(n, -1),
        ], axis=-1)

    ##
    # 只有实时源才有的
    ##

    def stale(self, now: float) -> str:
        """断流检查。返回空串表示正常，非空就是给 ``~/estop`` 的理由。

        ``now`` 必须和缓冲里的时间轴**同一个时钟域**。数据从 ``/mocap/frame`` 来时
        那是消息的 ``header.stamp``，也就是 ROS 时钟——所以这里故意不给默认值，
        留个默认的 ``time.monotonic()`` 会让两个域差几小时，永远判成断流。
        """
        stats = self._stream.stats()
        if not stats.connected:
            return '动捕链路断开'
        span = self._stream.span()
        if span is None:
            return '动捕缓冲还没攒够两帧'
        gap = now - span[1]
        if gap > self._stale_timeout:
            return f'动捕断流 {gap:.2f}s'
        return ''

    def describe(self) -> str:
        """给 ``~/status`` 用的一行摘要。``lag`` 是播放头落后最新帧多少，应该稳在前瞻跨度上。"""
        stats = self._stream.stats()
        span = self._stream.span()
        lag = '' if span is None or self._origin is None \
            else f' lag={span[1] - self._last_play:.3f}'
        return (f'frames={stats.frames} dropped={stats.dropped} '
                f'link={"up" if stats.connected else "down"} '
                f'body_status={stats.status}{lag}')

    ##
    # 内部
    ##

    def _require(self, times: np.ndarray) -> SampleBatch:
        batch = self._stream.sample(times)
        if batch is None:
            raise RuntimeError('动捕缓冲为空，取不到参考')
        return batch

    def _latest_playable(self) -> float:
        span = self._stream.span()
        if span is None:
            raise RuntimeError('动捕缓冲为空，取不到参考')
        return span[1] - self._lead_s

    def _play_time(self, frame: int, *, advance: bool) -> float:
        target = self._latest_playable()
        if self._origin is None:
            if not advance:
                return target
            self._origin, self._origin_frame = target, int(frame)
            self._last_play = target
            return target
        current = self._origin + (int(frame) - self._origin_frame) * self._dt
        if not advance:
            return current
        error = target - current
        # 慢修正追不上的那种断流（缓冲被清空、头显重连）只能硬跳。
        self._origin += error if abs(error) > self._hard \
            else float(np.clip(error, -self._slew, self._slew))
        self._last_play = self._origin + (int(frame) - self._origin_frame) * self._dt
        return self._last_play

    def _ensure_alignment(self, current: SampleBatch, offsets: np.ndarray) -> None:
        """第一次取窗口时才把参考搬到机器人所在的坐标系，之后整段用同一个变换。"""
        if self._pending is None:
            return
        robot_pos, robot_quat = self._pending
        self._pending = None
        zero = int(np.argmin(np.abs(offsets)))
        anchor_pos = current.anchor_pos[zero]
        anchor_quat = quat_normalize(current.anchor_quat[zero])
        relative = quat_mul(quat_normalize(robot_quat), quat_conj(anchor_quat))
        self._align_quat = yaw_quat(relative)
        self._align_pos = robot_pos - quat_apply(self._align_quat, anchor_pos)
        # 高度不对齐：人的离地高度是动作内容，机器人站立高度的差异由策略自己补。
        self._align_pos[2] = 0.0

    def _to_world(self, positions: np.ndarray) -> np.ndarray:
        return self._align_pos + quat_apply(self._align_quat, positions)


def lead_frames_for(offsets: Sequence[int], *, margin: int = 2) -> int:
    """播放头至少要落后多少拍才能凑出前瞻。"""
    return max(int(max(offsets)), 0) + int(margin)
