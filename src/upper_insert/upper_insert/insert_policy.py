"""插孔任务 ONNX 策略 (config/policy.onnx) 的观测装配、15ms 力觉滤波/力臂转换与 50Hz GRU 推理，不依赖 ROS，可离线单测"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import TYPE_CHECKING, Any, Sequence

import numpy as np

from upper_insert import package_file

if TYPE_CHECKING:
    from onnxruntime import InferenceSession

WRENCH_ORDERS = ('force_torque', 'torque_force')

DEFAULT_TRANSLATION_LIMIT_M = 0.001
DEFAULT_ROTATION_LIMIT_RAD = 0.5 * math.pi / 180.0
DEFAULT_FILTER_SECONDS = 0.015
DEFAULT_FORCE_LIMIT_N = 30.0
DEFAULT_MOMENT_LIMIT_NM = 2.0


DEFAULT_POLICY_ONNX = str(package_file('config/policy.onnx'))
DEFAULT_CONTRACT_JSON = str(package_file('config/contract.json'))


def canonical_quat_wxyz(
    quat: Sequence[float] | np.ndarray,
    *,
    input_order: str = 'wxyz',
) -> np.ndarray:
    """将四元数转为规范化 wxyz 单位四元数（首个绝对值 > 1e-8 的分量为正）；拒绝非有限或零范数输入"""
    q = np.asarray(quat, dtype=np.float64).reshape(4)
    if not np.all(np.isfinite(q)):
        raise ValueError('四元数包含非有限值')
    if input_order.lower() == 'xyzw':
        q = np.array([q[3], q[0], q[1], q[2]], dtype=np.float64)
    elif input_order.lower() != 'wxyz':
        raise ValueError(f"input_order 必须为 'wxyz' 或 'xyzw'，收到 {input_order!r}")
    norm = float(np.linalg.norm(q))
    if norm < 1e-6:
        raise ValueError('四元数范数过小或为零')
    q = q / norm
    for val in q:
        if abs(float(val)) > 1e-8:
            if val < 0.0:
                q = -q
            break
    return q


def format_wrench(
    force: Sequence[float] | np.ndarray,
    torque: Sequence[float] | np.ndarray,
    order: str = 'force_torque',
) -> np.ndarray:
    """按约定顺序拼接六维力与力矩向量"""
    f_arr = np.asarray(force, dtype=np.float64).reshape(3)
    t_arr = np.asarray(torque, dtype=np.float64).reshape(3)
    if order == 'force_torque':
        return np.concatenate([f_arr, t_arr])
    if order == 'torque_force':
        return np.concatenate([t_arr, f_arr])
    raise ValueError(f'wrench_order 必须为 {WRENCH_ORDERS} 之一，收到 {order!r}')


def shift_wrench_to_tip(
    force_s: Sequence[float] | np.ndarray,
    moment_s: Sequence[float] | np.ndarray,
    r_st_t: Sequence[float] | np.ndarray,
    r_ts: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """将力传感器参考点 S 处的六维力/力矩旋转并平移至插头名义尖端 T (按 DEPLOYMENT_ONNX.md 3.3):
    F_T = R_TS @ F_S
    M_T = R_TS @ M_S - cross(r_ST_T, F_T)
    其中 r_ST_T 为从力传感器参考点 S 指向插头名义末端 T、在 T 系下表达的位移向量 (m)
    """
    f_s = np.asarray(force_s, dtype=np.float64).reshape(3)
    m_s = np.asarray(moment_s, dtype=np.float64).reshape(3)
    r_vec = np.asarray(r_st_t, dtype=np.float64).reshape(3)
    if r_ts is None:
        f_t = f_s.copy()
        m_rot = m_s.copy()
    else:
        rot = np.asarray(r_ts, dtype=np.float64).reshape(3, 3)
        f_t = rot @ f_s
        m_rot = rot @ m_s
    m_t = m_rot - np.cross(r_vec, f_t)
    return f_t, m_t


class WrenchProcessor:
    """力/力矩低通滤波 (tau=0.015s) 与末端力臂平移及安全阈值监测 (DEPLOYMENT_ONNX.md 第 3、6 节)"""

    def __init__(
        self,
        *,
        tau_s: float = DEFAULT_FILTER_SECONDS,
        force_limit_n: float = DEFAULT_FORCE_LIMIT_N,
        moment_limit_nm: float = DEFAULT_MOMENT_LIMIT_NM,
    ) -> None:
        self.tau_s = float(tau_s)
        self.force_limit_n = float(force_limit_n)
        self.moment_limit_nm = float(moment_limit_nm)
        self.filtered_s = np.zeros(6, dtype=np.float64)
        self.raw_s = np.zeros(6, dtype=np.float64)
        self._last_stamp: float | None = None

    def reset(self) -> None:
        """在确认安全的无接触状态下重置滤波器状态"""
        self.filtered_s.fill(0.0)
        self.raw_s.fill(0.0)
        self._last_stamp = None

    def update_sample(
        self,
        force_s: Sequence[float] | np.ndarray,
        moment_s: Sequence[float] | np.ndarray,
        stamp_s: float | None = None,
        dt_s: float | None = None,
    ) -> np.ndarray:
        """高频样本到达时在传感器坐标系 S 下执行一阶指数低通滤波"""
        f_s = np.asarray(force_s, dtype=np.float64).reshape(3)
        m_s = np.asarray(moment_s, dtype=np.float64).reshape(3)
        w_s = np.concatenate([f_s, m_s])
        if not np.all(np.isfinite(w_s)):
            return self.filtered_s.copy()
        self.raw_s = w_s

        if dt_s is None:
            if stamp_s is not None and self._last_stamp is not None:
                dt = max(1e-5, min(0.1, float(stamp_s - self._last_stamp)))
            else:
                dt = 0.001
        else:
            dt = max(1e-5, float(dt_s))
        if stamp_s is not None:
            self._last_stamp = float(stamp_s)

        if self.tau_s <= 1e-6:
            self.filtered_s = w_s.copy()
        else:
            alpha = 1.0 - math.exp(-dt / self.tau_s)
            self.filtered_s = self.filtered_s + alpha * (w_s - self.filtered_s)
        return self.filtered_s.copy()

    def tip_wrench(
        self,
        r_st_t: Sequence[float] | np.ndarray,
        r_ts: np.ndarray | None = None,
        *,
        use_filtered: bool = True,
    ) -> tuple[np.ndarray, np.ndarray]:
        src = self.filtered_s if use_filtered else self.raw_s
        return shift_wrench_to_tip(src[:3], src[3:6], r_st_t, r_ts)

    def check_safety(
        self,
        r_st_t: Sequence[float] | np.ndarray,
        r_ts: np.ndarray | None = None,
    ) -> tuple[bool, str, float, float]:
        """监测未滤波原始补偿末端力与力矩范数是否超限"""
        f_raw_t, m_raw_t = self.tip_wrench(r_st_t, r_ts, use_filtered=False)
        f_norm = float(np.linalg.norm(f_raw_t))
        m_norm = float(np.linalg.norm(m_raw_t))
        if f_norm > self.force_limit_n:
            return False, f'末端接触力超限: {f_norm:.1f} N > {self.force_limit_n:.1f} N', f_norm, m_norm
        if m_norm > self.moment_limit_nm:
            return False, f'末端力矩超限: {m_norm:.2f} N·m > {self.moment_limit_nm:.2f} N·m', f_norm, m_norm
        return True, '', f_norm, m_norm


def build_observation(
    force: Sequence[float] | np.ndarray,
    torque: Sequence[float] | np.ndarray,
    delta_pos: Sequence[float] | np.ndarray,
    delta_quat: Sequence[float] | np.ndarray,
    *,
    wrench_order: str = 'force_torque',
    quat_order: str = 'wxyz',
) -> np.ndarray:
    """按 DEPLOYMENT_ONNX.md 第 2 节规范组装 13 维策略输入向量:
    obs[0:3]   = p_goal_H - p_tip_H (固定孔系 H 下的平移差距, m)
    obs[3:7]   = canonical(inverse(q_HT) * q_goal_H) (规范化 wxyz 单位四元数)
    obs[7:10]  = F_T (滤波并重力补偿后在名义末端系 T 下的力, N)
    obs[10:13] = M_T (滤波并重力补偿后对名义末端 T 取矩并在 T 系下表达的力矩, N·m)
    """
    wrench = format_wrench(force, torque, order=wrench_order)
    d_pos = np.asarray(delta_pos, dtype=np.float64).reshape(3)
    if not (np.all(np.isfinite(d_pos)) and np.all(np.isfinite(wrench))):
        raise ValueError('ONNX 策略输入包含非有限值')
    q_wxyz = canonical_quat_wxyz(delta_quat, input_order=quat_order)
    obs = np.concatenate([d_pos, q_wxyz, wrench])
    return np.ascontiguousarray(obs, dtype=np.float32)


class InsertPolicy:
    """加载并执行 50Hz config/policy.onnx 策略 (含两层 GRU 隐状态管理与 contract.json 参数加载)"""

    def __init__(
        self,
        model_path: str = DEFAULT_POLICY_ONNX,
        *,
        contract_path: str = DEFAULT_CONTRACT_JSON,
        wrench_order: str = 'force_torque',
        action_scale: float = 1.0,
        max_step_m: float = DEFAULT_TRANSLATION_LIMIT_M,
        max_step_rad: float = DEFAULT_ROTATION_LIMIT_RAD,
    ) -> None:
        if wrench_order not in WRENCH_ORDERS:
            raise ValueError(f'wrench_order 必须为 {WRENCH_ORDERS} 之一')
        self.model_path = model_path.strip()
        self.contract_path = contract_path.strip()
        self.wrench_order = wrench_order
        self.action_scale = float(action_scale)
        self.translation_limit_m = float(max_step_m)
        self.rotation_limit_rad = float(max_step_rad)
        self.filter_seconds = DEFAULT_FILTER_SECONDS
        self.force_limit_n = DEFAULT_FORCE_LIMIT_N
        self.moment_limit_nm = DEFAULT_MOMENT_LIMIT_NM
        self.contract: dict[str, Any] = {}

        self._session: InferenceSession | None = None
        self._obs_dim = 13
        self._action_dim = 6
        self._hidden = np.zeros((2, 1, 128), dtype=np.float32)

        if self.contract_path and os.path.isfile(self.contract_path):
            self._load_contract(self.contract_path)
        if self.model_path:
            self.load(self.model_path)

    def _load_contract(self, path: str) -> None:
        self.contract = json.loads(Path(path).read_text(encoding='utf-8'))
        for attr, key in (
            ('translation_limit_m', 'translation_limit_m'),
            ('rotation_limit_rad', 'rotation_component_limit_rad'),
            ('filter_seconds', 'filter_seconds'),
            ('force_limit_n', 'force_limit_n'),
            ('moment_limit_nm', 'moment_limit_nm'),
        ):
            if key in self.contract:
                setattr(self, attr, float(self.contract[key]))

    @property
    def loaded(self) -> bool:
        return self._session is not None

    @property
    def obs_dim(self) -> int:
        return self._obs_dim

    @property
    def action_dim(self) -> int:
        return self._action_dim

    @property
    def hidden_state(self) -> np.ndarray:
        return self._hidden.copy()

    def load(self, path: str) -> None:
        if not os.path.isfile(path):
            raise FileNotFoundError(f'未找到 ONNX 模型文件: {path}')
        import onnxruntime

        contract_candidate = Path(path).with_name('contract.json')
        if contract_candidate.is_file():
            self._load_contract(str(contract_candidate))

        options = onnxruntime.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self._session = onnxruntime.InferenceSession(
            path, sess_options=options, providers=['CPUExecutionProvider']
        )
        self._hidden.fill(0.0)
        self.model_path = path

    def reset(self) -> None:
        self._hidden.fill(0.0)

    def infer_raw(self, obs: np.ndarray) -> np.ndarray:
        """输入原始 SI 观测，更新 GRU 状态，返回未限幅的动作均值；不采样或使用 std"""
        obs_c = np.ascontiguousarray(obs, dtype=np.float32).reshape(1, self._obs_dim)
        if not np.all(np.isfinite(obs_c)):
            raise ValueError('ONNX 策略输入包含非有限值')
        if self._session is None:
            return np.zeros(self._action_dim, dtype=np.float32)

        raw_actions, next_h = self._session.run(
            ['actions', 'h_out'],
            {'obs': obs_c, 'h_in': np.ascontiguousarray(self._hidden, dtype=np.float32)},
        )
        if not (np.all(np.isfinite(raw_actions)) and np.all(np.isfinite(next_h))):
            raise RuntimeError('ONNX 策略输出或隐状态包含非有限值')
        self._hidden = np.ascontiguousarray(next_h, dtype=np.float32)
        return np.asarray(raw_actions, dtype=np.float32).reshape(-1)

    def step(
        self,
        force: Sequence[float] | np.ndarray,
        torque: Sequence[float] | np.ndarray,
        delta_pos: Sequence[float] | np.ndarray,
        delta_quat: Sequence[float] | np.ndarray,
        *,
        quat_order: str = 'wxyz',
    ) -> np.ndarray:
        """单拍 50Hz 推理，返回孔系 H 下的 6D 物理步长 [dp_H(3, m), dr_H(3, rad)]"""
        obs = build_observation(
            force,
            torque,
            delta_pos,
            delta_quat,
            wrench_order=self.wrench_order,
            quat_order=quat_order,
        )
        if self._session is None:
            return np.zeros(6, dtype=np.float64)

        actions = self.infer_raw(obs).astype(np.float64)
        out = np.zeros(6, dtype=np.float64)
        out[:3] = actions[:3] * self.translation_limit_m * self.action_scale
        out[3:6] = actions[3:6] * self.rotation_limit_rad * self.action_scale
        return out
