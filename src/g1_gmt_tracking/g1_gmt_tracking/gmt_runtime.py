"""GMT 策略的观测装配与推理，不依赖 ROS，可离线单测。

观测布局必须和训练侧 ``g1_gmt`` 的 ``actor`` 组逐项对齐，顺序也一样：

===  ====================  =====  ==================================================
序   项                     维度   实机来源
===  ====================  =====  ==================================================
0    command                 390   参考动作的前瞻特征，10 个前瞻帧 × 39
1    motion_anchor_ori_b       6   参考锚姿态相对躯干姿态的旋转矩阵前两列
2    base_ang_vel        3 × 5   盆骨 IMU 角速度，最近 5 拍
3    joint_pos          31 × 5   q_meas - q_default，**31 轴**（含两个夹爪偏心轴）
4    joint_vel          31 × 5   dq_meas，31 轴
5    actions            29 × 5   上一拍策略原始输出
===  ====================  =====  ==================================================

历史按**时序**排（旧 -> 新），与 mjlab ``CircularBuffer.buffer`` 的
"chronologically ordered, oldest to newest" 一致。

**观测 31 轴、动作 29 轴，两者不是同一个名单。** 夹爪偏心轴进观测但不由策略驱动，
所以真机上夹爪的编码器读数必须照喂——缺了它观测会错位 2 维，整段全废。

动作到关节目标：``q_target = q_default + action_scale * action``，与训练侧
``JointPositionActionCfg(use_default_offset=True)`` 一致。目标故意不按关节行程裁剪，
理由与 ``g1_motion_control/policy_runtime.py`` 完全相同（底层是 PD，靠到限位还想要力
就得把目标顶到行程外），只裁到力矩已饱和的 ``ctrlrange``。
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

from g1_gmt_tracking.motion_library import MotionClip

EXPECTED_OBS_TERMS: tuple[str, ...] = (
    'command', 'motion_anchor_ori_b', 'base_ang_vel',
    'joint_pos', 'joint_vel', 'actions',
)

CONTRACT_JSON = 'policy_contract.json'

# ONNX 把它序列化成 "-inf;inf,..."，和 JSON 的嵌套列表没有可比形式。部署端也不读。
_UNCOMPARABLE = frozenset({'observation_terms_clip'})


@dataclass(frozen=True)
class GmtSpec:
    """从 ONNX metadata 读出的策略契约。"""

    obs_joint_names: tuple[str, ...]
    """观测里 joint_pos / joint_vel 覆盖的 31 轴，顺序即观测里的顺序。"""
    action_joint_names: tuple[str, ...]
    """策略驱动的 29 轴，顺序即动作里的顺序。"""
    default_joint_pos: np.ndarray
    """31 轴的默认位姿。"""
    action_scale: np.ndarray
    """29 轴的动作缩放。"""
    all_body_names: tuple[str, ...]
    anchor_body_name: str
    root_body_name: str
    lookahead_steps: tuple[int, ...]
    history_length: int
    control_dt: float
    obs_dim: int
    action_dim: int


def _csv(meta: dict[str, str], key: str) -> list[str]:
    raw = meta[key]
    return json.loads(raw) if raw.lstrip().startswith('[') else raw.split(',')


def _agrees(mine: list[str], theirs) -> bool:
    """两边是不是同一个值。ONNX metadata 的数值只存到 3 位小数，只能比到这个精度。"""
    if isinstance(theirs, str):
        return mine == [theirs]
    if len(mine) != len(theirs):
        return False
    try:
        return all(abs(float(a) - float(b)) <= 5e-4 for a, b in zip(mine, theirs))
    except (TypeError, ValueError):
        return mine == [str(v) for v in theirs]


def _contract(session, policy_path: str) -> dict[str, str]:
    """策略契约 = ONNX metadata + 同目录 ``policy_contract.json`` 补缺。

    导出脚本只把训练框架自己那套键写进了 ONNX，部署端还要的 ``lookahead_steps`` /
    ``anchor_body_name`` / ``all_body_names`` 只落在 JSON 里，所以这两个文件必须一起拷。
    两边都有的键逐个比对，对不上就是 JSON 和权重不是同一次导出的产物；通过之后取 JSON
    那一份，因为 ``action_scale`` 这种直接乘进关节目标的量要用全精度。
    """
    meta = dict(session.get_modelmeta().custom_metadata_map)
    sidecar = Path(policy_path).with_name(CONTRACT_JSON)
    if not sidecar.is_file():
        return meta
    extra = json.loads(sidecar.read_text(encoding='utf-8'))
    for key, value in extra.items():
        if key in _UNCOMPARABLE:
            continue
        if key in meta and not _agrees(_csv(meta, key), value):
            raise ValueError(f'{sidecar} 的 {key} 和权重对不上，两者不是同一次导出的')
        meta[key] = value if isinstance(value, str) else json.dumps(value)
    return meta


def _obs_joint_key(meta: dict[str, str]) -> str:
    """观测关节名单的键。

    训练侧后来把这一个键拆成了 joint_pos / joint_vel 两份（站立类任务两者可以不同）。
    动作跟踪任务里两者恒等，取 joint_pos 那份即可；旧策略只有合并的键，一并兼容。
    """
    for key in ('obs_joint_pos_joint_names', 'obs_joint_names'):
        if key in meta:
            return key
    raise ValueError(
        '缺少观测关节名单：既没有 obs_joint_pos_joint_names 也没有 obs_joint_names')


def load_policy(path: str, root_body_name: str = 'pelvis'):
    """加载 ONNX 并解析出 :class:`GmtSpec`。

    归一化层导出时已折进图里（``obs_normalization=True``），这里喂原始观测即可。
    """
    import onnxruntime

    options = onnxruntime.SessionOptions()
    # 50 Hz 下这个规模的网络是微秒级负载，多线程只会引入调度抖动。
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    session = onnxruntime.InferenceSession(
        path, options, providers=['CPUExecutionProvider'])

    meta = _contract(session, path)
    obs_joint_key = _obs_joint_key(meta)
    required = {'observation_names', 'observation_terms_history_length',
                obs_joint_key, 'action_joint_names', 'default_joint_pos',
                'action_scale', 'all_body_names', 'anchor_body_name',
                'lookahead_steps'}
    missing = required - meta.keys()
    if missing:
        raise ValueError(
            f'{path} 缺少 metadata: {sorted(missing)}；'
            f'同目录下的 {CONTRACT_JSON} 也没能补上，导出时要把它一起拷过来')

    terms = tuple(_csv(meta, 'observation_names'))
    if terms != EXPECTED_OBS_TERMS:
        raise ValueError(
            '观测项不匹配，这个 ONNX 不是 GMT 任务导出的\n'
            f'  期望: {EXPECTED_OBS_TERMS}\n  实际: {terms}')

    # metadata 里数值一律按浮点格式化（"5.000"），先过 float 再取整。
    history = [int(float(v)) for v in _csv(meta, 'observation_terms_history_length')]
    # 前两项无历史，后四项共用同一个窗口长度；不一致说明训练配置改过，别猜。
    if history[:2] != [0, 0] or len(set(history[2:])) != 1:
        raise ValueError(f'历史长度布局异常: {history}')

    spec = GmtSpec(
        obs_joint_names=tuple(_csv(meta, obs_joint_key)),
        action_joint_names=tuple(_csv(meta, 'action_joint_names')),
        default_joint_pos=np.asarray(
            [float(v) for v in _csv(meta, 'default_joint_pos')], dtype=np.float64),
        action_scale=np.asarray(
            [float(v) for v in _csv(meta, 'action_scale')], dtype=np.float64),
        all_body_names=tuple(_csv(meta, 'all_body_names')),
        anchor_body_name=meta['anchor_body_name'],
        root_body_name=root_body_name,
        lookahead_steps=tuple(int(float(v)) for v in _csv(meta, 'lookahead_steps')),
        history_length=int(history[2]),
        control_dt=float(meta.get('control_dt', 0.02)),
        obs_dim=int(session.get_inputs()[0].shape[-1]),
        action_dim=len(_csv(meta, 'action_scale')),
    )

    n_obs_j, n_act_j, k, h = (len(spec.obs_joint_names), spec.action_dim,
                              len(spec.lookahead_steps), spec.history_length)
    expected = k * 39 + 6 + h * (3 + 2 * n_obs_j + n_act_j)
    if spec.obs_dim != expected:
        raise ValueError(f'观测维度 {spec.obs_dim} != 按契约算出的 {expected}')
    if len(spec.default_joint_pos) != n_obs_j:
        raise ValueError('default_joint_pos 与 obs_joint_names 长度不一致')
    return session, spec


class GmtPolicy:
    """一拍策略推理。持有历史窗口、上一拍动作和播放进度，所以是有状态的。"""

    def __init__(self, session, spec: GmtSpec, *,
                 target_lower: np.ndarray, target_upper: np.ndarray) -> None:
        if target_lower.shape != (spec.action_dim,) or target_upper.shape != (spec.action_dim,):
            raise ValueError('目标边界长度必须等于动作维度')
        if np.any(target_lower >= target_upper):
            raise ValueError('目标边界上下界颠倒')
        self._session = session
        self._spec = spec
        self._input = session.get_inputs()[0].name
        # RNN 策略把隐状态当额外输入/输出暴露出来（GRU 是 h_in/h_out）。纯 MLP 策略没有，
        # 两者都要支持，所以按图的实际输入个数判定，不硬编码。
        self._state_names = tuple(i.name for i in session.get_inputs()[1:])
        self._state_shapes = tuple(
            tuple(d if isinstance(d, int) else 1 for d in i.shape)
            for i in session.get_inputs()[1:])
        self._state = [np.zeros(s, dtype=np.float32) for s in self._state_shapes]
        self._lower = target_lower
        self._upper = target_upper

        obs_names = list(spec.obs_joint_names)
        self.action_slots = np.array(
            [obs_names.index(n) for n in spec.action_joint_names], dtype=np.intp)
        """29 个动作关节在 31 轴观测名单里的下标。"""
        self._action_default = spec.default_joint_pos[self.action_slots]

        h, n_obs_j = spec.history_length, len(spec.obs_joint_names)
        self._hist_ang_vel = np.zeros((h, 3))
        self._hist_joint_pos = np.zeros((h, n_obs_j))
        self._hist_joint_vel = np.zeros((h, n_obs_j))
        self._hist_actions = np.zeros((h, spec.action_dim))
        self._obs = np.zeros((1, spec.obs_dim), dtype=np.float32)
        self.frame = 0
        self._primed = False

    @property
    def spec(self) -> GmtSpec:
        return self._spec

    def reset(self) -> None:
        """回到刚接管的状态：历史清空、动作清零、参考动作从第 0 帧重放。"""
        for buf in (self._hist_ang_vel, self._hist_joint_pos,
                    self._hist_joint_vel, self._hist_actions):
            buf[:] = 0.0
        # 不清隐状态的话，重新接管时策略还带着上一段的记忆。
        # 重建而不是就地清零：上一拍的隐状态直接来自 ONNX 输出，可能是只读的。
        self._state = [np.zeros(s, dtype=np.float32) for s in self._state_shapes]
        self.frame = 0
        self._primed = False

    @staticmethod
    def _push(buf: np.ndarray, value: np.ndarray) -> None:
        buf[:-1] = buf[1:]
        buf[-1] = value

    def observe(self, *, clip: MotionClip, joint_pos: np.ndarray,
                joint_vel: np.ndarray, ang_vel: Sequence[float],
                anchor_quat: np.ndarray) -> np.ndarray:
        """按训练顺序拼观测。

        Args:
            clip: 当前参考动作，播放进度由 :attr:`frame` 决定。
            joint_pos: 31 轴实测角，顺序同 ``spec.obs_joint_names``。
            joint_vel: 31 轴实测角速度。
            ang_vel: 盆骨 IMU 角速度（盆骨系）。
            anchor_quat: 躯干姿态四元数 wxyz（由盆骨 IMU 与腰三轴推出）。
        """
        spec = self._spec
        rel_pos = np.asarray(joint_pos, dtype=np.float64) - spec.default_joint_pos
        vel = np.asarray(joint_vel, dtype=np.float64)
        omega = np.asarray(ang_vel, dtype=np.float64)

        # 首拍把窗口填满，否则前 4 拍会喂给网络一串它训练时从没见过的零。
        if not self._primed:
            self._hist_ang_vel[:] = omega
            self._hist_joint_pos[:] = rel_pos
            self._hist_joint_vel[:] = vel
            self._primed = True
        else:
            self._push(self._hist_ang_vel, omega)
            self._push(self._hist_joint_pos, rel_pos)
            self._push(self._hist_joint_vel, vel)

        obs = np.concatenate([
            clip.lookahead(self.frame, spec.lookahead_steps),
            clip.anchor_ori_b(self.frame, anchor_quat),
            self._hist_ang_vel.reshape(-1),
            self._hist_joint_pos.reshape(-1),
            self._hist_joint_vel.reshape(-1),
            self._hist_actions.reshape(-1),
        ])
        if obs.shape != (spec.obs_dim,):
            raise ValueError(f'观测维度 {obs.shape} != {spec.obs_dim}')
        return obs

    def step(self, *, clip: MotionClip, joint_pos: np.ndarray,
             joint_vel: np.ndarray, ang_vel: Sequence[float],
             anchor_quat: np.ndarray) -> np.ndarray:
        """跑一拍，返回 29 个关节目标位置，并把参考动作推进一帧。

        Raises:
            ValueError: 输入或网络输出出现非有限值。
        """
        obs = self.observe(clip=clip, joint_pos=joint_pos, joint_vel=joint_vel,
                           ang_vel=ang_vel, anchor_quat=anchor_quat)
        if not np.all(np.isfinite(obs)):
            raise ValueError('观测里有非有限值')

        self._obs[0, :] = obs
        feed = {self._input: self._obs}
        feed.update(zip(self._state_names, self._state))
        outputs = self._session.run(None, feed)
        if self._state_names:
            # 隐状态必须回写，否则 RNN 退化成无记忆网络且不会报错。
            self._state = [np.asarray(o, dtype=np.float32)
                           for o in outputs[1:1 + len(self._state_names)]]
        action = np.asarray(outputs[0], dtype=np.float64).reshape(-1)
        if action.shape != (self._spec.action_dim,) or not np.all(np.isfinite(action)):
            raise ValueError(f'策略输出非法: {action}')

        # 先存原始动作再裁剪：下一拍的 ``actions`` 观测在训练里就是未裁剪的原始输出。
        self._push(self._hist_actions, action)
        self.frame += 1

        target = self._action_default + self._spec.action_scale * action
        return np.clip(target, self._lower, self._upper)


def spec_matches(spec: GmtSpec, obs_joints: Sequence[str],
                 action_joints: Sequence[str]) -> None:
    """校验 ONNX 里的关节顺序和配置声明一致，不一致直接抛。

    实机上最容易出人命的一类错误：权重换了、关节顺序变了，而配置没跟着改，
    结果左右腿指令互换。宁可起不来也不要跑错。
    """
    if spec.obs_joint_names != tuple(obs_joints):
        raise ValueError('ONNX 的观测关节顺序和配置不一致，拒绝启动\n'
                         f'  配置: {tuple(obs_joints)}\n  ONNX: {spec.obs_joint_names}')
    if spec.action_joint_names != tuple(action_joints):
        raise ValueError('ONNX 的动作关节顺序和配置不一致，拒绝启动\n'
                         f'  配置: {tuple(action_joints)}\n  ONNX: {spec.action_joint_names}')


def resolve_policy_path(path: str) -> str:
    """支持 ``package://<pkg>/<相对路径>`` 写法。"""
    prefix = 'package://'
    if not path.startswith(prefix):
        return path
    from ament_index_python.packages import get_package_share_directory
    pkg, _, rest = path[len(prefix):].partition('/')
    return str(Path(get_package_share_directory(pkg)) / rest)
