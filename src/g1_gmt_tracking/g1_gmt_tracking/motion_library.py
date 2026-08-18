"""参考动作库：读 NPZ，按时间取帧，算前瞻特征。不依赖 ROS，可离线单测。

NPZ 的字段与训练侧 ``g1_lower_rl/scripts/build_corpus.py`` 的输出一致::

    joint_pos      (T, 31)     全部 31 轴（含两个夹爪偏心轴）
    body_pos_w     (T, B, 3)   世界系刚体位置
    body_quat_w    (T, B, 4)   wxyz
    body_lin_vel_w (T, B, 3)
    body_ang_vel_w (T, B, 3)
    fps            (1,)

刚体维 ``B`` 的顺序是训练时机器人的完整刚体顺序（44 个），而策略只用其中 14 个
（``tracked_body_names``）。这里按名字取下标，不假设它们连续。

**前瞻特征是偏航与平移不变的**：只用离地高度、根系下的重力方向与线/角速度、关节角。
所以同一段动作放在场地哪个位置、朝哪个方向，喂给网络的数字完全一样——不需要把参考
轨迹对齐到机器人。唯一需要对齐的是 ``motion_anchor_ori_b``，它含绝对偏航差，
由 :meth:`MotionClip.align_yaw` 在放第 0 帧那一拍锁定。
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import numpy as np

from g1_gmt_tracking.rotations import (
    quat_conj,
    quat_mul,
    quat_to_mat,
    rotate_inverse,
    yaw_quat,
)

GRAVITY_W = np.array([0.0, 0.0, -1.0])


class MotionClip:
    """一段参考动作。

    Args:
        path: NPZ 路径。
        anchor_index: 锚刚体在 ``body_*`` 数组里的下标（策略用 ``torso_link``）。
        root_index: 根刚体下标，前瞻特征的根位姿取自它（``pelvis``）。
        policy_joint_ids: 29 个动作关节在 31 轴里的下标，前瞻特征只喂这些。
        expected_fps: 控制频率。播放是一个控制拍推进一帧，所以对不上就是变速播放。
    """

    def __init__(self, path: str | Path, *, anchor_index: int, root_index: int,
                 policy_joint_ids: np.ndarray, expected_fps: float) -> None:
        data = np.load(Path(path))
        self.name = Path(path).stem
        self.fps = float(data['fps'][0]) if 'fps' in data.files else 50.0
        self.joint_pos = np.asarray(data['joint_pos'], dtype=np.float64)
        self._policy_joint_ids = np.asarray(policy_joint_ids, dtype=np.intp)

        # 往目录里丢新 NPZ 是常规用法，帧率错了必须当场拦下，不能等上机才发现。
        if abs(self.fps - expected_fps) > 1e-6:
            raise ValueError(
                f'{path} 的 fps={self.fps} 与控制频率 {expected_fps} 不一致，会被变速播放')

        if 'root_quat' in data.files:
            # 瘦身格式：只存根刚体与锚刚体，部署包里用的就是它。
            self._root_pos = np.asarray(data['root_pos'], dtype=np.float64)
            self._root_quat = np.asarray(data['root_quat'], dtype=np.float64)
            self._root_lin = np.asarray(data['root_lin_vel'], dtype=np.float64)
            self._root_ang = np.asarray(data['root_ang_vel'], dtype=np.float64)
            self._anchor_quat = np.asarray(data['anchor_quat'], dtype=np.float64)
        else:
            # 训练格式：全部 44 个刚体，其中 42 个部署端一个字节都不会读。
            required = {'body_pos_w', 'body_quat_w', 'body_lin_vel_w', 'body_ang_vel_w'}
            missing = required - set(data.files)
            if missing:
                raise ValueError(f'{path} 既不是瘦身格式，也缺训练格式字段: {sorted(missing)}')
            self._root_pos = np.asarray(data['body_pos_w'][:, root_index], dtype=np.float64)
            self._root_quat = np.asarray(data['body_quat_w'][:, root_index], dtype=np.float64)
            self._root_lin = np.asarray(data['body_lin_vel_w'][:, root_index], dtype=np.float64)
            self._root_ang = np.asarray(data['body_ang_vel_w'][:, root_index], dtype=np.float64)
            self._anchor_quat = np.asarray(data['body_quat_w'][:, anchor_index], dtype=np.float64)

        self.num_frames = int(self.joint_pos.shape[0])
        if len(self._root_quat) != self.num_frames:
            raise ValueError(f'{path} 各字段帧数不一致')

        # 锚姿态在启动时会被绕 z 旋到机器人当前朝向，先留个单位四元数。
        self._align = np.array([1.0, 0.0, 0.0, 0.0])

    @property
    def duration_s(self) -> float:
        return self.num_frames / self.fps

    def align_yaw(self, robot_anchor_quat: np.ndarray) -> None:
        """把参考动作的初始朝向对齐到机器人当前朝向。

        ``motion_anchor_ori_b`` 是机器人锚姿态与参考锚姿态之差，**含绝对偏航**。
        参考轨迹自己的世界系是录制时定的，和机器人站在场地上的朝向没有关系，
        所以放第 0 帧那一拍必须锁一次偏航差，之后整段动作都用这一个偏移量。

        只取偏航：俯仰和横滚是动作本身的内容，对齐掉就等于篡改了参考。
        """
        self._align = yaw_quat(quat_mul(robot_anchor_quat, quat_conj(self._anchor_quat[0])))

    def anchor_quat(self, frame: int) -> np.ndarray:
        """第 ``frame`` 帧的参考锚姿态，已按启动时锁定的偏航对齐。"""
        return quat_mul(self._align, self._anchor_quat[self.clamp(frame)])

    def clamp(self, frame: int) -> int:
        return int(min(max(frame, 0), self.num_frames - 1))

    def lookahead(self, frame: int, steps: Sequence[int]) -> np.ndarray:
        """前瞻特征，形状 ``(len(steps) * 39,)``。

        每个前瞻帧 39 维 = 高度 1 + 根系重力 3 + 根系线速度 3 + 根系角速度 3 + 关节角 29，
        顺序与训练侧 ``GeneralMotionCommand.command`` 逐位一致。
        超出动作末尾的前瞻帧钳到末帧（训练时也是这样处理的）。
        """
        idx = np.array([self.clamp(frame + int(s)) for s in steps], dtype=np.intp)
        root_quat = self._root_quat[idx]

        height = self._root_pos[idx, 2:3]
        gravity = np.broadcast_to(GRAVITY_W, (len(idx), 3))
        feats = np.concatenate([
            height,
            rotate_inverse(root_quat, gravity),
            rotate_inverse(root_quat, self._root_lin[idx]),
            rotate_inverse(root_quat, self._root_ang[idx]),
            self.joint_pos[idx][:, self._policy_joint_ids],
        ], axis=-1)
        return feats.reshape(-1)

    def anchor_ori_b(self, frame: int, robot_anchor_quat: np.ndarray) -> np.ndarray:
        """``motion_anchor_ori_b``：相对旋转矩阵的前两列，6 维。

        训练侧是 ``matrix_from_quat(q_robot^-1 * q_ref)[..., :2]`` 展平，
        即前两**列**按行优先排成 ``[R00,R01,R10,R11,R20,R21]``。
        """
        rel = quat_mul(quat_conj(robot_anchor_quat), self.anchor_quat(frame))
        return quat_to_mat(rel)[:, :2].reshape(-1)


def resolve_indices(all_body_names: Sequence[str], anchor_body_name: str,
                    root_body_name: str, obs_joint_names: Sequence[str],
                    action_joint_names: Sequence[str], control_dt: float) -> dict:
    """由 ONNX 契约算出 :class:`MotionClip` 的全部构造参数。名字对不上直接抛。

    刚体下标查的是 ``all_body_names``（机器人全部刚体），因为 NPZ 的刚体维就是它；
    ``tracked_body_names`` 只是训练时参与奖励的子集，拿它的下标去索引 NPZ 会取错刚体。
    """
    bodies = list(all_body_names)
    for name in (anchor_body_name, root_body_name):
        if name not in bodies:
            raise ValueError(f'刚体 {name} 不在 all_body_names 里')
    obs = list(obs_joint_names)
    try:
        policy_ids = np.array([obs.index(n) for n in action_joint_names], dtype=np.intp)
    except ValueError as exc:
        raise ValueError(f'动作关节不在观测关节名单里: {exc}') from exc
    return {
        'anchor_index': bodies.index(anchor_body_name),
        'root_index': bodies.index(root_body_name),
        'policy_joint_ids': policy_ids,
        'expected_fps': 1.0 / control_dt,
    }
