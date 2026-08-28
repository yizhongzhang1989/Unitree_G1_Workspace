"""参考动作库：读 NPZ，按时间取帧，装配 RGMT 的参考窗口。不依赖 ROS，可离线单测。

与旧 GMT 部署包的**根本差别**：参考窗口后 30 维要用机器人**当前**的世界位姿来局部化，
所以这段参考必须先被搬进机器人所在的坐标系。旧包只锁偏航（``align_yaw``），
这里必须同时锁平移，见 :meth:`MotionClip.align`。

每个 token 68 维，布局逐位对应训练侧 ``reference_tokens``（契约里的 ``lookahead_layout``）::

    lin_vel_local3, ang_vel_local3, proj_gravity3, joint_pos29,
    key_body_pos_in_robot_anchor_yaw15, key_body_vel_in_robot_anchor_yaw15

前 38 维在**参考自身**根系下，对世界偏航与平移不变；后 30 维在**机器人当前** anchor
的 yaw 局部系下——前者是"该做什么动作"，后者是"你离它差多远"。

NPZ 字段（瘦身格式，由 ``scripts/slim_motion.py`` 生成）::

    fps(1) joint_pos(T,31)
    root_pos(T,3) root_quat(T,4) root_lin_vel(T,3) root_ang_vel(T,3)
    anchor_pos(T,3) anchor_quat(T,4)
    key_pos(T,K,3) key_lin_vel(T,K,3)

也兼容训练格式（``body_pos_w`` 等全量刚体数组），按名字取下标。
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import numpy as np

from .rotations import (
    quat_apply,
    quat_conj,
    quat_mul,
    quat_normalize,
    rotate_inverse,
    yaw_quat,
)

GRAVITY_W = np.array([0.0, 0.0, -1.0])


class MotionClip:
    """一段参考动作。

    Args:
        path: NPZ 路径。
        anchor_index / root_index: 锚（torso_link）与根（pelvis）在全量刚体数组里的下标。
        key_indexes: 5 个 key body 的下标，顺序必须与契约的 ``reference_key_bodies`` 一致。
        policy_joint_ids: 29 个动作关节在 31 轴里的下标。
    """

    def __init__(self, path: str | Path, *, anchor_index: int, root_index: int,
                 key_indexes: Sequence[int], policy_joint_ids: np.ndarray) -> None:
        data = np.load(Path(path))
        self.name = Path(path).stem
        self.fps = float(data['fps'][0]) if 'fps' in data.files else 50.0
        self.joint_pos = np.asarray(data['joint_pos'], dtype=np.float64)
        self._policy_joint_ids = np.asarray(policy_joint_ids, dtype=np.intp)
        keys = np.asarray(key_indexes, dtype=np.intp)

        if 'key_pos' in data.files:
            self._root_pos = np.asarray(data['root_pos'], dtype=np.float64)
            self._root_quat = np.asarray(data['root_quat'], dtype=np.float64)
            self._root_lin = np.asarray(data['root_lin_vel'], dtype=np.float64)
            self._root_ang = np.asarray(data['root_ang_vel'], dtype=np.float64)
            self._anchor_pos = np.asarray(data['anchor_pos'], dtype=np.float64)
            self._anchor_quat = np.asarray(data['anchor_quat'], dtype=np.float64)
            self._key_pos = np.asarray(data['key_pos'], dtype=np.float64)
            self._key_vel = np.asarray(data['key_lin_vel'], dtype=np.float64)
            if self._key_pos.shape[1] != len(keys):
                raise ValueError(
                    f'{path} 的 key body 数 {self._key_pos.shape[1]} 与契约要求的 '
                    f'{len(keys)} 不符——瘦身时的 key body 名单和权重对不上')
        else:
            required = {'body_pos_w', 'body_quat_w', 'body_lin_vel_w', 'body_ang_vel_w'}
            missing = required - set(data.files)
            if missing:
                raise ValueError(f'{path} 既不是瘦身格式，也缺训练格式字段: {sorted(missing)}')
            pos_w = np.asarray(data['body_pos_w'], dtype=np.float64)
            quat_w = np.asarray(data['body_quat_w'], dtype=np.float64)
            lin_w = np.asarray(data['body_lin_vel_w'], dtype=np.float64)
            self._root_pos = pos_w[:, root_index]
            self._root_quat = quat_w[:, root_index]
            self._root_lin = lin_w[:, root_index]
            self._root_ang = np.asarray(data['body_ang_vel_w'], dtype=np.float64)[:, root_index]
            self._anchor_pos = pos_w[:, anchor_index]
            self._anchor_quat = quat_w[:, anchor_index]
            self._key_pos = pos_w[:, keys]
            self._key_vel = lin_w[:, keys]

        self.num_frames = int(self.joint_pos.shape[0])
        if len(self._root_quat) != self.num_frames or len(self._key_pos) != self.num_frames:
            raise ValueError(f'{path} 各字段帧数不一致')

        self._align_quat = np.array([1.0, 0.0, 0.0, 0.0])
        self._align_pos = np.zeros(3)
        self._aligned = False

    @property
    def duration_s(self) -> float:
        return self.num_frames / self.fps

    @property
    def aligned(self) -> bool:
        return self._aligned

    def clamp(self, frame: int) -> int:
        return int(min(max(int(frame), 0), self.num_frames - 1))

    def align(self, robot_anchor_pos: np.ndarray, robot_anchor_quat: np.ndarray) -> None:
        """把参考的首帧锚位姿搬到机器人此刻的位置与朝向。

        必须在 ``~/start`` 那一刻做一次，之后整段动作用同一个变换——**中途重算等于
        把已经产生的跟踪误差抹掉**，那 15 维就永远读作零，退化成开环。

        只锁偏航与平移：俯仰、横滚、离地高度都是动作本身的内容，对齐掉就是篡改参考。
        """
        robot_anchor_pos = np.asarray(robot_anchor_pos, dtype=np.float64)
        q_ref0 = quat_normalize(self._anchor_quat[0])
        rel = quat_mul(quat_normalize(robot_anchor_quat), quat_conj(q_ref0))
        self._align_quat = yaw_quat(rel)
        self._align_pos = robot_anchor_pos - quat_apply(self._align_quat, self._anchor_pos[0])
        # 高度不对齐：参考的离地高度是动作内容，机器人站立高度的差异由策略自己补。
        self._align_pos[2] = 0.0
        self._aligned = True

    def _to_world(self, pos: np.ndarray) -> np.ndarray:
        return self._align_pos + quat_apply(self._align_quat, pos)

    def anchor_pose_world(self, frame: int) -> tuple[np.ndarray, np.ndarray]:
        """参考锚刚体在机器人所在坐标系下的位姿，供 status 上报与偏差诊断。"""
        i = self.clamp(frame)
        return (self._to_world(self._anchor_pos[i]),
                quat_mul(self._align_quat, quat_normalize(self._anchor_quat[i])))

    def stand_joint_pos(self) -> np.ndarray:
        """首帧的 29 轴关节角，STAND 阶段插值到这里。"""
        return self.joint_pos[0][self._policy_joint_ids]

    def reference_window(self, frame: int, offsets: np.ndarray,
                         robot_anchor_pos: np.ndarray,
                         robot_anchor_quat: np.ndarray) -> np.ndarray:
        """装配 (len(offsets), 68) 的参考窗口，展平后就是 ``rg_reference``。

        ``offsets`` 含负值（过去帧），越界两端都钳住——训练侧 ``_window_indexes`` 同样是
        钳位而不是循环，动作开头结尾的窗口会重复首末帧。
        """
        if not self._aligned:
            raise RuntimeError('参考动作尚未对齐，必须先调用 align()')
        idx = np.array([self.clamp(frame + int(o)) for o in offsets], dtype=np.intp)

        root_quat = self._root_quat[idx]
        gravity = np.broadcast_to(GRAVITY_W, (len(idx), 3))
        # 前 38 维全部转到参考**自身**根系，与机器人在哪无关
        lin_vel = rotate_inverse(root_quat, self._root_lin[idx])
        ang_vel = rotate_inverse(root_quat, self._root_ang[idx])
        proj_gravity = rotate_inverse(root_quat, gravity)
        joint_pos = self.joint_pos[idx][:, self._policy_joint_ids]

        # 后 30 维：先把参考搬到机器人坐标系，再转进机器人 anchor 的 yaw 局部系。
        # 速度也要跟着 align 转——整段参考被旋转了，它的速度方向同样要旋转。
        key_pos_w = self._to_world(self._key_pos[idx].reshape(-1, 3)).reshape(len(idx), -1, 3)
        key_vel_w = quat_apply(self._align_quat, self._key_vel[idx].reshape(-1, 3))
        key_vel_w = key_vel_w.reshape(len(idx), -1, 3)

        inv = quat_conj(yaw_quat(quat_normalize(robot_anchor_quat)))
        rel = key_pos_w - np.asarray(robot_anchor_pos, dtype=np.float64)
        key_pos_local = quat_apply(inv, rel.reshape(-1, 3)).reshape(len(idx), -1)
        key_vel_local = quat_apply(inv, key_vel_w.reshape(-1, 3)).reshape(len(idx), -1)

        return np.concatenate(
            [lin_vel, ang_vel, proj_gravity, joint_pos, key_pos_local, key_vel_local],
            axis=-1,
        )


class MotionLibrary:
    """启动时把整个目录读进内存，运行时靠动作名切换。"""

    def __init__(self, directory: str | Path, *, anchor_index: int, root_index: int,
                 key_indexes: Sequence[int], policy_joint_ids: np.ndarray) -> None:
        directory = Path(directory)
        paths = sorted(directory.glob('*.npz'))
        if not paths:
            raise FileNotFoundError(f'{directory} 下没有 NPZ 参考动作')
        self._clips = {
            p.stem: MotionClip(p, anchor_index=anchor_index, root_index=root_index,
                               key_indexes=key_indexes, policy_joint_ids=policy_joint_ids)
            for p in paths
        }

    @property
    def names(self) -> list[str]:
        return sorted(self._clips)

    def get(self, name: str) -> MotionClip:
        if name not in self._clips:
            raise KeyError(f'没有动作 {name!r}，可选: {self.names}')
        return self._clips[name]
