"""下肢 GRU 策略的观测装配与推理，不依赖 ROS，可离线单测。

**观测布局不写死在代码里，而是照着 ONNX metadata 的 ``observation_names`` 逐项装配。**
换策略 = 换 `config/policy.onnx`，不用改代码，也不用改这个文件里的常量。这样做的直接
理由是：站立策略和原来的速度跟踪策略观测项不同（前者没有指令、关节取全身 29 轴，后者
有指令和步态相位、关节只取下肢 15 轴），而两者要能随时互换。

目前支持的项，遇到不认识的名字直接拒绝加载：

===================  ======================  ==========================================
项                    维度                    实机来源
===================  ======================  ==========================================
``base_ang_vel``      3                       盆骨 IMU 角速度（盆骨系）
``projected_gravity`` 3                       盆骨 IMU 姿态四元数投影的重力方向
``command_twist``     3                       ``~/command`` 的 ``[vx, vy, wz]``
``command_height``    1                       ``~/command`` 的 ``[h]``
``phase``             2                       ``[sin, cos]``，站立指令下置零
``joint_pos``         len(观测关节)            ``q_meas - q_default``
``joint_vel``         len(观测关节)            ``dq_meas``
``actions``           len(动作关节)            上一拍策略原始输出
===================  ======================  ==========================================

于是两套契约长这样：

* 速度跟踪（``config/policy.onnx`` 里原来那个）：57 维 = 3+3+3+1+2+15+15+15，
  观测关节 = 动作关节 = 下肢 15 轴。
* 站立（``G1-Gloria-Stand``）：79 维 = 3+3+29+29+15，**没有指令项**，
  观测关节 29 轴（下肢 15 + 手臂 14），动作关节仍是下肢 15 轴。

``~/command`` 无论哪套都照收、照限速、照在 ``~/state`` 里回显；只是站立那套的
``observation_names`` 里没有指令项，于是它压根不会被拼进观测向量。节点侧的调用代码
两套完全一样，这正是"能随时退回原策略"的前提。

**站立策略的观测含手臂 14 轴，但不含两个夹爪轴。** 手臂由 VR IK 自顾自地驱动，它摆到
哪里直接决定质心落在哪里，训练侧的事件会把手臂在整个可达范围内摆起来，所以这 14 维在
训练分布里是有方差的，喂实测值进去安全。夹爪不同：它在训练里恒为 0，观测归一化学到的
标准差接近零，真机上一开合就会被除成巨值盖掉平衡信号——实测夹爪偏 0.2 rad 造成的动作
扰动是左膝偏同样角度的 5 倍。所以它只以物理扰动的形式进入。

共 42 维。**没有 ``actions`` 项**——GRU 的隐状态本来就记得自己上一拍输出了什么，再从
输入喂回去等于给策略接了一条显式正反馈通路。部署侧因此也不再维护“上一拍动作”这个状态。

隐状态（``h_in`` / ``h_out``，形状 (1,1,32)）逐拍回喂，并在 :meth:`LocomotionPolicy.reset`
里清零——训练里它就是每个 episode 开头归零的，不清就会把上一次急停前的状态带进下一次接管。

动作到关节目标：``q_target = q_default + action_scale * action``，和训练侧
``JointPositionActionCfg(use_default_offset=True)`` 一致。

**目标位置故意不按关节行程裁剪**。底层是 PD，``tau = kp*(q_target - q) - kd*dq``，
关节靠到限位时想要力就只能靠把目标顶到行程外。mjlab 建 <position> 执行器时
显式设了 ``ctrllimited=False`` + ``inheritrange=0``，注释写得很直白：“clamping ctrl to
the joint range would produce zero force when the joint is at its limit”。CPU MuJoCo 闭环
重跑实测：1.0 m/s 前进时 **18.6% 的拍**目标位置在硬行程之外，``waist_roll`` 能到
−0.763 rad（行程只有 ±0.520）。按行程裁剪 = 把平衡最吃力的那几个关节掐掉。

真正裁的是 MuJoCo 自己那份 informational ``ctrlrange``，即
``jnt_range ± effort_limit / stiffness``：越过它力矩已经饱和，再大的目标也换不来额外的力，
所以裁在这里物理上是空操作（上述五个场景里越界率 **0%**），只用来拦真正跑飞的输出。

关节顺序、默认位姿、动作缩放全部从 metadata 里读，不在这里抄一遍。``joint_names`` 是
模型的**全部 31 轴**，拿它截前 N 个去猜子集只在"下肢正好排在最前"时才对；新导出的
ONNX 另写了权威名单 ``action_joint_names`` 和 ``obs_joint_pos_joint_names`` /
``obs_joint_vel_joint_names``，有就用，没有才退回截断（老权重就是这么读的）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np

# 不带关节的观测项及其固定维度。带关节的三项维度由 metadata 的名单长度决定。
FIXED_OBS_TERM_DIMS: dict[str, int] = {
    'base_ang_vel': 3,
    'projected_gravity': 3,
    'command_twist': 3,
    'command_height': 1,
    'phase': 2,
}

COMMAND_OBS_TERMS = frozenset({'command_twist', 'command_height', 'phase'})
"""需要 ``~/command`` 才能装配的项。站立策略一个都不含。"""

GAIT_PERIOD_S = 0.6
"""训练侧 ``phase`` 观测的步态周期。"""

STAND_COMMAND_NORM = 0.1
"""``|[vx, vy, wz]|`` 低于此值时 phase 置零，与训练侧 ``mdp.phase`` 一致。"""


@dataclass(frozen=True)
class PolicySpec:
    """从 ONNX metadata 读出的策略契约。

    观测关节和动作关节可以是**两个不同的集合**：站立策略看全身 29 轴、只驱动下肢 15 轴；
    速度跟踪策略两者相同。
    """

    obs_terms: tuple[str, ...]
    """观测项的名字与顺序，直接来自 metadata，是装配观测的唯一依据。"""
    action_joint_names: tuple[str, ...]
    """策略驱动的关节，顺序即动作向量的顺序。"""
    action_default_pos: np.ndarray
    """这些关节的默认位姿，动作的偏置。"""
    action_scale: np.ndarray
    """动作缩放。"""
    obs_joint_names: tuple[str, ...]
    """进观测的关节，顺序即 ``joint_pos``/``joint_vel`` 两段的顺序。"""
    obs_default_pos: np.ndarray
    """这些关节的默认位姿，``joint_pos`` 观测的减数。"""
    obs_dim: int
    action_dim: int
    hidden_name: str
    """隐状态输入名，训练侧导出为 ``h_in``；前馈网络的权重为空串。"""
    hidden_shape: tuple[int, ...]
    """隐状态张量形状，当前 GRU 权重为 ``(1, 1, 32)``。"""

    @property
    def uses_command(self) -> bool:
        """这个策略是否真的把 ``~/command`` 拼进了观测。"""
        return bool(COMMAND_OBS_TERMS & set(self.obs_terms))


def _metadata(session) -> dict[str, str]:
    return dict(session.get_modelmeta().custom_metadata_map)


def _named_defaults(table: dict[str, float], names: Sequence[str],
                    field: str) -> np.ndarray:
    missing = [name for name in names if name not in table]
    if missing:
        raise ValueError(f'{field} 里的关节不在 joint_names 里: {missing}')
    return np.asarray([table[name] for name in names], dtype=np.float64)


def load_policy(path: str):
    """加载 ONNX 并解析出 :class:`PolicySpec`。

    归一化层已经在导出时折进图里（``obs_normalization=True``），所以这里喂原始
    观测即可，不要再自己减均值。
    """
    import onnxruntime

    options = onnxruntime.SessionOptions()
    # 50 Hz 下这几个 MLP 都是微秒级负载，多线程只会引入调度抖动。
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    session = onnxruntime.InferenceSession(
        path, options, providers=['CPUExecutionProvider'])

    meta = _metadata(session)
    missing = {'joint_names', 'default_joint_pos', 'action_scale',
               'observation_names', 'action_joint_names'} - meta.keys()
    if missing:
        raise ValueError(f'{path} 缺少 metadata: {sorted(missing)}')

    names = meta['joint_names'].split(',')
    defaults = [float(v) for v in meta['default_joint_pos'].split(',')]
    if len(names) != len(defaults):
        raise ValueError('joint_names 与 default_joint_pos 长度不一致')
    table = dict(zip(names, defaults))

    scales = np.asarray(
        [float(v) for v in meta['action_scale'].split(',')], dtype=np.float64)
    # 老权重没有权威名单。它们的下肢关节正好排在模型最前，截断才是对的；新权重一律
    # 显式声明，不再依赖这个巧合。
    action_joints = tuple(
        meta['action_joint_names'].split(',') if 'action_joint_names' in meta
        else names[:len(scales)])
    if len(action_joints) != len(scales):
        raise ValueError('action_joint_names 与 action_scale 长度不一致')

    if 'obs_joint_pos_joint_names' in meta:
        obs_joints = tuple(meta['obs_joint_pos_joint_names'].split(','))
        vel_joints = tuple(meta.get('obs_joint_vel_joint_names', '').split(','))
        if obs_joints != vel_joints:
            raise ValueError('joint_pos 与 joint_vel 观测的关节集合不一致')
    else:
        obs_joints = action_joints  # 老契约：观测关节就是动作关节。

    obs_terms = tuple(meta['observation_names'].split(','))
    dims = dict(FIXED_OBS_TERM_DIMS)
    dims['joint_pos'] = dims['joint_vel'] = len(obs_joints)
    dims['actions'] = len(action_joints)
    unknown = [name for name in obs_terms if name not in dims]
    if unknown:
        raise ValueError(
            f'metadata 里有本层不认识的观测项: {unknown}\n'
            f'  已知: {sorted(dims)}\n'
            f'  新增项必须同时在 LocomotionPolicy.observe() 里给出装配方式')

    # 循环策略多一个隐状态输入；前馈策略只有观测一个输入，两套都要能加载。
    inputs = session.get_inputs()
    if len(inputs) > 2:
        raise ValueError(
            f'期望一个或两个输入（obs [+ 隐状态]），实际 {len(inputs)} 个: '
            f'{[i.name for i in inputs]}')
    hidden = inputs[1] if len(inputs) == 2 else None

    spec = PolicySpec(
        obs_terms=obs_terms,
        action_joint_names=action_joints,
        action_default_pos=_named_defaults(table, action_joints, 'action_joint_names'),
        action_scale=scales,
        obs_joint_names=obs_joints,
        obs_default_pos=_named_defaults(table, obs_joints, 'obs_joint_names'),
        obs_dim=int(inputs[0].shape[-1]),
        action_dim=len(action_joints),
        hidden_name=hidden.name if hidden is not None else '',
        hidden_shape=tuple(int(d) for d in hidden.shape) if hidden is not None else (),
    )

    expected_obs = sum(dims[name] for name in obs_terms)
    if spec.obs_dim != expected_obs:
        raise ValueError(
            f'观测维度不匹配: 权重要 {spec.obs_dim}，按 metadata 的项与关节名单算出 '
            f'{expected_obs}\n  项: {obs_terms}\n'
            f'  观测关节 {len(obs_joints)} 个，动作关节 {len(action_joints)} 个')
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
    """一拍策略推理。持有隐状态和相位计时器，所以是有状态的。"""

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
        self._steps = 0
        self._recurrent = bool(spec.hidden_name)
        self._hidden = np.zeros(spec.hidden_shape, dtype=np.float32)
        self._last_action = np.zeros(spec.action_dim)

    def reset(self) -> None:
        """回到刚复位的状态：隐状态与上一拍动作清零，相位从 0 重新计时。

        隐状态必须在这里清——训练里它就是每个 episode 开头归零的。不清的话，
        上一次跑飞/急停时的状态会被带进下一次接管。
        """
        self._steps = 0
        self._hidden[:] = 0.0
        self._last_action[:] = 0.0

    def observe(self, *, joint_pos: np.ndarray, joint_vel: np.ndarray,
                ang_vel: Sequence[float], quat_xyzw: Sequence[float],
                command: Sequence[float]) -> np.ndarray:
        """按 ``spec.obs_terms`` 声明的项与顺序拼观测。

        Args:
            joint_pos: 实测关节位置，顺序必须是 ``spec.obs_joint_names``。
            joint_vel: 实测关节速度，同上。
            command: ``[vx, vy, wz, height]``。策略不含指令项时原样收下但不使用。
        """
        parts = []
        for name in self._spec.obs_terms:
            if name == 'base_ang_vel':
                parts.append(np.asarray(ang_vel, dtype=np.float64))
            elif name == 'projected_gravity':
                parts.append(projected_gravity(quat_xyzw))
            elif name == 'command_twist':
                parts.append(np.asarray(command[:3], dtype=np.float64))
            elif name == 'command_height':
                parts.append(np.asarray(command[3:4], dtype=np.float64))
            elif name == 'phase':
                parts.append(gait_phase(
                    self._steps * self._control_dt, command, self._gait_period_s))
            elif name == 'joint_pos':
                parts.append(np.asarray(joint_pos, dtype=np.float64)
                             - self._spec.obs_default_pos)
            elif name == 'joint_vel':
                parts.append(np.asarray(joint_vel, dtype=np.float64))
            else:  # 'actions'，load_policy() 已经挡掉了其它名字
                parts.append(self._last_action)
        obs = np.concatenate(parts)
        if obs.shape != (self._spec.obs_dim,):
            raise ValueError(f'观测维度 {obs.shape} != {self._spec.obs_dim}')
        return obs

    def step(self, *, joint_pos: np.ndarray, joint_vel: np.ndarray,
             ang_vel: Sequence[float], quat_xyzw: Sequence[float],
             command: Sequence[float]) -> np.ndarray:
        """跑一拍，返回各动作关节的目标位置。

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
        feed = {self._input: self._obs}
        if self._recurrent:
            feed[self._spec.hidden_name] = self._hidden
        outputs = self._session.run(None, feed)
        action = np.asarray(outputs[0], dtype=np.float64).reshape(-1)
        if action.shape != (self._spec.action_dim,) or not np.all(np.isfinite(action)):
            raise ValueError(f'策略输出非法: {action}')
        if self._recurrent:
            new_hidden = np.asarray(outputs[1], dtype=np.float32)
            if not np.all(np.isfinite(new_hidden)):
                raise ValueError('隐状态出现非有限值')
            self._hidden = new_hidden

        self._last_action = action
        self._steps += 1
        target = self._spec.action_default_pos + self._spec.action_scale * action
        return np.clip(target, self._lower, self._upper)


def spec_matches(spec: PolicySpec, action_joints: Sequence[str],
                 obs_joints: Sequence[str]) -> None:
    """校验 ONNX 里的关节顺序和配置文件声明的一致，不一致直接抛。

    这是实机上最容易出人命的一类错误：权重换了、关节顺序变了，而配置没跟着改，
    结果左右腿指令互换。宁可起不来也不要跑错。观测顺序同理：手臂那 14 轴排错位置，
    策略会按一个不存在的质心去配平衡。

    Args:
        obs_joints: 期望的观测关节。策略只看动作关节时，传和 ``action_joints``
            一样的列表即可——老契约就是这种情况。
    """
    for field, actual, expected in (
            ('动作', spec.action_joint_names, tuple(action_joints)),
            ('观测', spec.obs_joint_names, tuple(obs_joints))):
        if actual != expected:
            raise ValueError(
                f'ONNX 的{field}关节顺序和配置不一致，拒绝启动\n'
                f'  配置: {expected}\n  ONNX: {actual}')
