"""下肢策略的观测装配与推理，不依赖 ROS，可离线单测。

观测布局必须和训练侧 ``src/tasks/lower_body/lower_body_env_cfg.py`` 的
``actor`` 组逐项对齐，顺序也一样（顺序写死在观测向量里，错一位就是错一个策略）：

===  ==================  ====  ==========================================
序   项                   维度  实机来源
===  ==================  ====  ==========================================
0    base_ang_vel          3   盆骨 IMU 角速度（盆骨系）
1    projected_gravity     3   盆骨 IMU 姿态四元数投影的重力方向
2    command_twist         3   [vx, vy, wz]
3    command_height        1   [h]
4    phase                 2   [sin, cos]，指令接近零时置零
5    joint_pos            15   q_meas - q_default
6    joint_vel            15   dq_meas
7    actions              15   上一拍策略原始输出
===  ==================  ====  ==========================================

动作到关节目标：``q_target = q_default + action_scale * action``，和训练侧
``JointPositionActionCfg(use_default_offset=True)`` 一致。

**目标位置故意不按关节行程裁剪**。底层是 PD，``tau = kp*(q_target - q) - kd*dq``，
关节靠到限位时想要力就只能靠把目标顶到行程外。mjlab 建 <position> 执行器时
显式设了 ``ctrllimited=False`` + ``inheritrange=0``，注释写得很直白：“clamping ctrl to
the joint range would produce zero force when the joint is at its limit”。CPU MuJoCo 闭环
重跑实测：1.0 m/s 前进时 **18.6% 的拍**目标位置在硬行程之外，``waist_roll`` 能到
−0.763 rad（行程只有 ±0.520）。按行程裁剪 = 把平衡最吃力的那几个关节掍掉。

真正裁的是 MuJoCo 自己那份 informational ``ctrlrange``，即
``jnt_range ± effort_limit / stiffness``：越过它力矩已经饱和，再大的目标也换不来额外的力，
所以裁在这里物理上是空操作（上述五个场景里越界率 **0%**），只用来拦真正跑飞的输出。

关节顺序、默认位姿、动作缩放全部从 ONNX 的 metadata 里读，不在这里抄一遍：
抄一遍就多一个会和权重不同步的副本。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np

# 训练侧 actor 组的项名与维度，用来校验 ONNX 是不是这个任务导出的。
EXPECTED_OBS_TERMS: tuple[tuple[str, int], ...] = (
    ('base_ang_vel', 3),
    ('projected_gravity', 3),
    ('command_twist', 3),
    ('command_height', 1),
    ('phase', 2),
    ('joint_pos', 15),
    ('joint_vel', 15),
    ('actions', 15),
)

GAIT_PERIOD_S = 0.6
"""训练侧 ``phase`` 观测的步态周期。"""

STAND_COMMAND_NORM = 0.1
"""``|[vx, vy, wz]|`` 低于此值时 phase 置零，与训练侧 ``mdp.phase`` 一致。"""


@dataclass(frozen=True)
class PolicySpec:
    """从 ONNX metadata 读出的策略契约。"""

    joint_names: tuple[str, ...]
    """策略控制的关节，顺序即动作/观测里的顺序。"""
    default_pos: np.ndarray
    """这些关节的默认位姿，动作的偏置。"""
    action_scale: np.ndarray
    """动作缩放。"""
    obs_dim: int
    action_dim: int


def _metadata(session) -> dict[str, str]:
    return dict(session.get_modelmeta().custom_metadata_map)


def load_policy(path: str):
    """加载 ONNX 并解析出 :class:`PolicySpec`。

    归一化层已经在导出时折进图里（``obs_normalization=True``），所以这里喂原始
    观测即可，不要再自己减均值。
    """
    import onnxruntime

    options = onnxruntime.SessionOptions()
    # 50 Hz 的 57->512->256->128->15 MLP 是微秒级负载，多线程只会引入调度抖动。
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    session = onnxruntime.InferenceSession(
        path, options, providers=['CPUExecutionProvider'])

    meta = _metadata(session)
    missing = {'joint_names', 'default_joint_pos', 'action_scale',
               'observation_names'} - meta.keys()
    if missing:
        raise ValueError(f'{path} 缺少 metadata: {sorted(missing)}')

    obs_terms = tuple(meta['observation_names'].split(','))
    expected = tuple(name for name, _ in EXPECTED_OBS_TERMS)
    if obs_terms != expected:
        raise ValueError(
            f'观测项不匹配，这个 ONNX 不是下肢任务导出的\n'
            f'  期望: {expected}\n  实际: {obs_terms}')

    names = meta['joint_names'].split(',')
    defaults = [float(v) for v in meta['default_joint_pos'].split(',')]
    scales = np.asarray(
        [float(v) for v in meta['action_scale'].split(',')], dtype=np.float64)
    if len(names) != len(defaults):
        raise ValueError('joint_names 与 default_joint_pos 长度不一致')

    # 训练侧动作项按模型关节顺序解析，且下肢关节正好排在最前，所以前 N 个就是策略
    # 关节。下面 spec_matches() 会拿它和期望列表逐个比对，写错了不会静默通过。
    count = len(scales)
    spec = PolicySpec(
        joint_names=tuple(names[:count]),
        default_pos=np.asarray(defaults[:count], dtype=np.float64),
        action_scale=scales,
        obs_dim=int(session.get_inputs()[0].shape[-1]),
        action_dim=count,
    )

    expected_obs = sum(dim for _, dim in EXPECTED_OBS_TERMS)
    if spec.obs_dim != expected_obs or spec.action_dim != 15:
        raise ValueError(
            f'维度不匹配: obs {spec.obs_dim} (期望 {expected_obs}), '
            f'action {spec.action_dim} (期望 15)')
    return session, spec


def projected_gravity(quat_xyzw: Sequence[float]) -> np.ndarray:
    """世界系 ``[0, 0, -1]`` 投影到机体系，等价于训练侧的 ``projected_gravity_b``。

    Args:
        quat_xyzw: 盆骨姿态四元数，ROS 的 (x, y, z, w) 顺序。
    """
    x, y, z, w = (float(v) for v in quat_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError(f'四元数非法: {quat_xyzw}')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    # g_b = R(q)^T @ [0, 0, -1] = -(R 的第三行)
    return np.array([
        -2.0 * (x * z - w * y),
        -2.0 * (y * z + w * x),
        -(1.0 - 2.0 * (x * x + y * y)),
    ])


def gait_phase(elapsed_s: float, command: Sequence[float],
               period_s: float = GAIT_PERIOD_S) -> np.ndarray:
    """步态相位 ``[sin, cos]``，站立指令下置零。"""
    if float(np.linalg.norm(np.asarray(command[:3], dtype=np.float64))) < STAND_COMMAND_NORM:
        return np.zeros(2)
    angle = 2.0 * math.pi * ((elapsed_s % period_s) / period_s)
    return np.array([math.sin(angle), math.cos(angle)])


class LocomotionPolicy:
    """一拍策略推理。持有上一拍动作和相位计时器，所以是有状态的。"""

    def __init__(self, session, spec: PolicySpec, *,
                 control_dt: float,
                 target_lower: np.ndarray,
                 target_upper: np.ndarray,
                 gait_period_s: float = GAIT_PERIOD_S) -> None:
        if target_lower.shape != (spec.action_dim,) or target_upper.shape != (spec.action_dim,):
            raise ValueError('目标边界长度必须等于动作维度')
        if np.any(target_lower >= target_upper):
            raise ValueError('目标边界上下界颠倒')
        self._session = session
        self._spec = spec
        self._input = session.get_inputs()[0].name
        self._control_dt = float(control_dt)
        self._lower = target_lower
        self._upper = target_upper
        self._gait_period_s = float(gait_period_s)
        self._obs = np.zeros((1, spec.obs_dim), dtype=np.float32)
        self._last_action = np.zeros(spec.action_dim)
        self._steps = 0

    def reset(self) -> None:
        """回到刚复位的状态：上一拍动作清零，相位从 0 重新计时。"""
        self._last_action[:] = 0.0
        self._steps = 0

    def observe(self, *, joint_pos: np.ndarray, joint_vel: np.ndarray,
                ang_vel: Sequence[float], quat_xyzw: Sequence[float],
                command: Sequence[float]) -> np.ndarray:
        """按训练顺序拼观测。command 是 ``[vx, vy, wz, height]``。"""
        parts = (
            np.asarray(ang_vel, dtype=np.float64),
            projected_gravity(quat_xyzw),
            np.asarray(command[:3], dtype=np.float64),
            np.asarray(command[3:4], dtype=np.float64),
            gait_phase(self._steps * self._control_dt, command, self._gait_period_s),
            joint_pos - self._spec.default_pos,
            joint_vel,
            self._last_action,
        )
        obs = np.concatenate(parts)
        if obs.shape != (self._spec.obs_dim,):
            raise ValueError(f'观测维度 {obs.shape} != {self._spec.obs_dim}')
        return obs

    def step(self, *, joint_pos: np.ndarray, joint_vel: np.ndarray,
             ang_vel: Sequence[float], quat_xyzw: Sequence[float],
             command: Sequence[float]) -> np.ndarray:
        """跑一拍，返回 15 个关节目标位置。

        目标可以、也应该落在关节行程之外（见模块文档）；只裁到力矩已饱和的
        ``ctrlrange``。

        Raises:
            ValueError: 输入或网络输出出现非有限值。
        """
        obs = self.observe(joint_pos=joint_pos, joint_vel=joint_vel,
                           ang_vel=ang_vel, quat_xyzw=quat_xyzw, command=command)
        if not np.all(np.isfinite(obs)):
            raise ValueError('观测里有非有限值')

        self._obs[0, :] = obs
        action = np.asarray(
            self._session.run(None, {self._input: self._obs})[0], dtype=np.float64).reshape(-1)
        if action.shape != (self._spec.action_dim,) or not np.all(np.isfinite(action)):
            raise ValueError(f'策略输出非法: {action}')

        # 先存原始动作再裁剪：下一拍的 ``actions`` 观测在训练里就是未裁剪的原始输出。
        self._last_action[:] = action
        self._steps += 1
        target = self._spec.default_pos + self._spec.action_scale * action
        return np.clip(target, self._lower, self._upper)


def spec_matches(spec: PolicySpec, joint_names: Sequence[str]) -> None:
    """校验 ONNX 里的关节顺序和配置文件声明的一致，不一致直接抛。

    这是实机上最容易出人命的一类错误：权重换了、关节顺序变了，而配置没跟着改，
    结果左右腿指令互换。宁可起不来也不要跑错。
    """
    expected = tuple(joint_names)
    if spec.joint_names != expected:
        raise ValueError(
            'ONNX 的关节顺序和配置不一致，拒绝启动\n'
            f'  配置: {expected}\n  ONNX: {spec.joint_names}')
