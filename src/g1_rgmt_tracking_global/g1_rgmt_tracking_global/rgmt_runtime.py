"""RGMT 策略运行时：契约解析、观测装配、ONNX 推理。不依赖 ROS，可离线单测。

与旧 GMT 部署包（``gmt_runtime.py``）的三处结构性差异：

1. **6 个命名输入**而不是单一扁平向量，且没有 RNN 隐状态（RGMT 用注意力记时序）。
2. **历史 H=10**（旧的是 5），且参考窗口是 **21 个 token × 68 维**、含过去帧。
3. 参考窗口后 30 维依赖机器人的**世界位姿**——这是本包叫 ``_global`` 的原因，
   旧包完全回避了根节点位置。

契约一律从 ONNX 的 ``custom_metadata_map`` 读，不打开任何 JSON：权重换了元数据跟着换，
而旁边那份 ``policy_contract.json`` 只是同一内容的可读副本，供人工核对。
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import onnxruntime

from .rotations import rotate_inverse

EXPECTED_OBS_TERMS = (
    'projected_gravity',
    'base_ang_vel',
    'joint_pos',
    'joint_vel',
    'actions',
)

# ONNX 输入名与 rg_* 观测组一一对应，顺序由 rgmt_model.py 的 input_names 决定。
PROP_INPUTS = (
    'rg_projected_gravity',
    'rg_base_ang_vel',
    'rg_joint_pos',
    'rg_joint_vel',
    'rg_actions',
)
REFERENCE_INPUT = 'rg_reference'

GRAVITY_W = np.array([0.0, 0.0, -1.0])


def resolve_policy_path(path: str) -> str:
    prefix = 'package://'
    if not path.startswith(prefix):
        return path
    from ament_index_python.packages import get_package_share_directory

    pkg, _, rest = path[len(prefix):].partition('/')
    return str(Path(get_package_share_directory(pkg)) / rest)


def _csv(meta: dict[str, str], key: str) -> list[str]:
    raw = meta[key]
    if raw.lstrip().startswith('['):
        return [str(v) for v in json.loads(raw.replace("'", '"'))]
    return [v.strip() for v in raw.split(',') if v.strip()]


def _floats(meta: dict[str, str], key: str) -> np.ndarray:
    return np.array([float(v) for v in _csv(meta, key)], dtype=np.float64)


@dataclass(frozen=True)
class RgmtSpec:
    obs_joint_names: tuple[str, ...]
    action_joint_names: tuple[str, ...]
    all_body_names: tuple[str, ...]
    anchor_body_name: str
    reference_key_bodies: tuple[str, ...]
    default_joint_pos: np.ndarray
    action_scale: np.ndarray
    window_offsets: np.ndarray
    token_dim: int
    token_layout: tuple[tuple[str, int], ...]
    history_len: int
    control_dt: float

    @property
    def key_pos_offset(self) -> int:
        """key_body_pos 段在 token 里的起始下标。钳位漂移量要用。"""
        cursor = 0
        for name, width in self.token_layout:
            if name.startswith('key_body_pos'):
                return cursor
            cursor += width
        raise ValueError('token 布局里没有 key_body_pos 段')


def load_spec(session: onnxruntime.InferenceSession) -> RgmtSpec:
    meta = dict(session.get_modelmeta().custom_metadata_map)

    terms = tuple(_csv(meta, 'observation_names'))
    if terms != EXPECTED_OBS_TERMS:
        raise ValueError(
            f'观测项不匹配，这个 ONNX 不是 RGMT 导出的。\n'
            f'  期望 {EXPECTED_OBS_TERMS}\n  实际 {terms}\n'
            f'  若是 GRU/MLP 权重请用 g1_gmt_tracking 部署。')

    history = [int(float(v)) for v in _csv(meta, 'observation_terms_history_length')]
    if len(set(history)) != 1:
        raise ValueError(f'RGMT 要求所有本体观测同历史长度，实际 {history}')
    history_len = history[0]

    layout: list[tuple[str, int]] = []
    for chunk in _csv(meta, 'lookahead_layout'):
        digits = len(chunk) - len(chunk.rstrip('0123456789'))
        if digits == 0:
            raise ValueError(f'layout 段 {chunk!r} 没有维度后缀')
        layout.append((chunk[:-digits], int(chunk[-digits:])))
    token_dim = int(float(meta['lookahead_feature_dim']))
    if sum(w for _, w in layout) != token_dim:
        raise ValueError(
            f'layout 各段之和 {sum(w for _, w in layout)} != feature_dim {token_dim}。'
            f'部署端按 layout 排布输入，对不上就是静默错位。')

    spec = RgmtSpec(
        obs_joint_names=tuple(_csv(meta, 'obs_joint_pos_joint_names')),
        action_joint_names=tuple(_csv(meta, 'action_joint_names')),
        all_body_names=tuple(_csv(meta, 'all_body_names')),
        anchor_body_name=meta['anchor_body_name'].strip(),
        reference_key_bodies=tuple(_csv(meta, 'reference_key_bodies')),
        default_joint_pos=_floats(meta, 'default_joint_pos'),
        action_scale=_floats(meta, 'action_scale'),
        window_offsets=np.array([int(float(v)) for v in _csv(meta, 'lookahead_steps')],
                                dtype=np.int64),
        token_dim=token_dim,
        token_layout=tuple(layout),
        history_len=history_len,
        control_dt=float(meta['control_dt']),
    )

    n_obs, n_act = len(spec.obs_joint_names), len(spec.action_joint_names)
    expect = {
        'rg_projected_gravity': 3 * history_len,
        'rg_base_ang_vel': 3 * history_len,
        'rg_joint_pos': n_obs * history_len,
        'rg_joint_vel': n_obs * history_len,
        'rg_actions': n_act * history_len,
        REFERENCE_INPUT: token_dim * len(spec.window_offsets),
    }
    actual = {i.name: int(i.shape[-1]) for i in session.get_inputs()}
    if set(actual) != set(expect):
        raise ValueError(f'ONNX 输入名不匹配。\n  期望 {sorted(expect)}\n  实际 {sorted(actual)}')
    for name, want in expect.items():
        if actual[name] != want:
            raise ValueError(f'输入 {name} 维度 {actual[name]} != 按契约算出的 {want}')
    if len(spec.default_joint_pos) != n_obs:
        raise ValueError(
            f'default_joint_pos 长度 {len(spec.default_joint_pos)} != 观测关节数 {n_obs}')
    if len(spec.action_scale) != n_act:
        raise ValueError(f'action_scale 长度 {len(spec.action_scale)} != 动作关节数 {n_act}')
    return spec


def spec_matches(spec: RgmtSpec, obs_joints: list[str], action_joints: list[str],
                 key_bodies: list[str]) -> None:
    """权重与配置的一致性检查。

    实机上最容易出人命的一类错误：换了权重、关节顺序变了而配置没跟着改，
    结果左右腿指令互换。宁可起不来也不要跑错。
    """
    if spec.obs_joint_names != tuple(obs_joints):
        raise ValueError(
            f'ONNX 的观测关节顺序和配置不一致，拒绝启动\n'
            f'  ONNX: {spec.obs_joint_names}\n  配置: {tuple(obs_joints)}')
    if spec.action_joint_names != tuple(action_joints):
        raise ValueError(
            f'ONNX 的动作关节顺序和配置不一致，拒绝启动\n'
            f'  ONNX: {spec.action_joint_names}\n  配置: {tuple(action_joints)}')
    if spec.reference_key_bodies != tuple(key_bodies):
        raise ValueError(
            f'ONNX 的 key body 名单和配置不一致，拒绝启动\n'
            f'  ONNX: {spec.reference_key_bodies}\n  配置: {tuple(key_bodies)}')


class RgmtPolicy:
    """观测缓冲 + ONNX 推理 + 残差动作还原。

    Args:
        path: ONNX 路径，支持 ``package://``。
        target_limits: (29,) 上下限，用 MuJoCo 的 ctrlrange 而非关节行程——
            底层是 PD，关节靠到行程边上还想出力就只能把目标顶到行程之外。
        max_anchor_offset_m: 参考锚点相对机器人的偏移上限。里程计跑飞时的兜底，见 ``step``。
    """

    def __init__(self, path: str, *, target_lower: np.ndarray, target_upper: np.ndarray,
                 max_anchor_offset_m: float = 0.3) -> None:
        options = onnxruntime.SessionOptions()
        # 50 Hz 下这个规模的网络是微秒级负载，多线程只会引入调度抖动。
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self._session = onnxruntime.InferenceSession(
            resolve_policy_path(path), options, providers=['CPUExecutionProvider'])
        self._spec = load_spec(self._session)
        spec = self._spec

        h, n_obs, n_act = spec.history_len, len(spec.obs_joint_names), len(spec.action_joint_names)
        self._hist = {
            'rg_projected_gravity': np.zeros((h, 3)),
            'rg_base_ang_vel': np.zeros((h, 3)),
            'rg_joint_pos': np.zeros((h, n_obs)),
            'rg_joint_vel': np.zeros((h, n_obs)),
            'rg_actions': np.zeros((h, n_act)),
        }
        self._last_action = np.zeros(n_act)
        self._primed = False
        self.frame = 0
        self._lower = np.asarray(target_lower, dtype=np.float64)
        self._upper = np.asarray(target_upper, dtype=np.float64)
        self._max_offset = float(max_anchor_offset_m)
        self._clamped = False

    @property
    def spec(self) -> RgmtSpec:
        return self._spec

    @property
    def anchor_clamped(self) -> bool:
        """上一拍是否触发了漂移钳位。持续为真说明里程计已经不可信。"""
        return self._clamped

    def action_slots(self) -> np.ndarray:
        """29 个动作关节在 31 轴观测名单里的下标。两套名单顺序不同，不能切片。"""
        obs = list(self._spec.obs_joint_names)
        return np.array([obs.index(n) for n in self._spec.action_joint_names], dtype=np.intp)

    def reset(self) -> None:
        for buf in self._hist.values():
            buf[:] = 0.0
        self._last_action[:] = 0.0
        self._primed = False
        self._clamped = False
        self.frame = 0

    @staticmethod
    def _push(buf: np.ndarray, value: np.ndarray) -> None:
        buf[:-1] = buf[1:]
        buf[-1] = value

    def _clamp_anchor(self, window: np.ndarray) -> None:
        """把参考锚点相对机器人的偏移钳到上限内。

        key body 的第一个就是 anchor 自己，所以那 3 维是**纯漂移量**。里程计失效时它会
        无界增长，而失效模式是正反馈：策略以为自己漂了 -> 往回纠 -> 真的走偏 -> 读数更大。
        钳住之后最坏退化成开环，不会主动把机器人推倒。其余 key body 不能钳——它们含
        "手脚相对躯干"的正常偏移，量级本来就有几十厘米。
        """
        off = self._spec.key_pos_offset
        seg = window[:, off:off + 3]
        norm = np.linalg.norm(seg, axis=-1, keepdims=True)
        over = norm > self._max_offset
        self._clamped = bool(np.any(over))
        if self._clamped:
            window[:, off:off + 3] = np.where(
                over, seg * (self._max_offset / np.maximum(norm, 1e-9)), seg)

    def step(self, *, joint_pos: np.ndarray, joint_vel: np.ndarray, ang_vel: np.ndarray,
             base_quat: np.ndarray, clip, robot_anchor_pos: np.ndarray,
             robot_anchor_quat: np.ndarray) -> np.ndarray:
        """跑一拍，返回 29 维关节位置目标。

        **两个刚体绝不能混**：

        * ``base_quat`` / ``ang_vel`` 挂 **pelvis**。训练侧 ``projected_gravity_b`` 算的是
          ``quat_apply_inverse(root_link_quat_w, gravity_vec_w)``，``root_link`` 就是自由
          关节所在的 pelvis，而真机 IMU 正好装在盆骨——这一路直通，**不要做腰部 FK**。
        * ``robot_anchor_*`` 挂 **torso_link**（anchor），用于 key body 局部化。

        两者只差 4.4 cm 平移和腰三轴转角，很容易被“顺手统一”，而取错不报错、只是全废。

        重力向量是**归一化**的 ``[0,0,-1]``（mjlab entity.py 里写死），不是 -9.81。
        """
        spec = self._spec
        rel_pos = np.asarray(joint_pos, dtype=np.float64) - spec.default_joint_pos
        vel = np.asarray(joint_vel, dtype=np.float64)
        omega = np.asarray(ang_vel, dtype=np.float64)
        gravity = rotate_inverse(np.asarray(base_quat, dtype=np.float64), GRAVITY_W)

        values = {
            'rg_projected_gravity': gravity,
            'rg_base_ang_vel': omega,
            'rg_joint_pos': rel_pos,
            'rg_joint_vel': vel,
            'rg_actions': self._last_action,
        }
        if not self._primed:
            # 首拍把窗口填满，否则前 H-1 拍会喂给网络一串它训练时从没见过的零。
            for name, value in values.items():
                self._hist[name][:] = value
            self._primed = True
        else:
            for name, value in values.items():
                self._push(self._hist[name], value)

        window = clip.reference_window(self.frame, spec.window_offsets,
                                       robot_anchor_pos, robot_anchor_quat)
        if window.shape != (len(spec.window_offsets), spec.token_dim):
            raise RuntimeError(f'参考窗口形状 {window.shape} 与契约不符')
        self._clamp_anchor(window)

        feed = {name: buf.reshape(1, -1).astype(np.float32) for name, buf in self._hist.items()}
        feed[REFERENCE_INPUT] = window.reshape(1, -1).astype(np.float32)
        for name, array in feed.items():
            if not np.all(np.isfinite(array)):
                raise RuntimeError(f'观测 {name} 出现非有限值')

        action = np.asarray(self._session.run(None, feed)[0], dtype=np.float64).reshape(-1)
        if action.shape != self._last_action.shape or not np.all(np.isfinite(action)):
            raise RuntimeError(f'策略输出异常: shape={action.shape}')

        self._last_action[:] = action
        self.frame += 1
        default = spec.default_joint_pos[self.action_slots()]
        return np.clip(default + spec.action_scale * action, self._lower, self._upper)
