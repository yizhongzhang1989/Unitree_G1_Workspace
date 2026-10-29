"""使用主相机 ID1 观测与图像时刻发布的 TF 标定工具空间变换与力臂"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pinocchio as pin
import yaml

from upper_insert.vision import (
    DEFAULT_POS_JSON,
    MARKER_LENGTH_M,
    R_GRIPPER_FROM_ID1,
    VisionPoseEstimator,
    _PLUG_TO_HOLE_BASIS,
    load_pos_config,
)

# 夹爪抓取插头时的名义旋转 R_S_T = R_G_T = R_GRIPPER_FROM_ID1 @ _PLUG_TO_HOLE_BASIS
NOMINAL_R_S_T = R_GRIPPER_FROM_ID1 @ _PLUG_TO_HOLE_BASIS
NOMINAL_R_TS = NOMINAL_R_S_T.T

# URDF 中 kwr57b_link -> gripper_base 固定偏移 [0, 0.003, 0.053] m
KWR57_TO_GRIPPER_XYZ = np.array([0.0, 0.003, 0.053], dtype=np.float64)

# 名义抓取下插头末端在 gripper_base 下约 [0, 0, 0.1475] m（距 KWR57 link 原点约 200.5 mm）
NOMINAL_P_G_T = np.array([0.0, 0.0, 0.1475], dtype=np.float64)


def _default_tool_calibration_path() -> Path:
    here = Path(__file__).resolve()
    for root in (here.parents[3], here.parents[2], Path('/workspace')):
        candidate = root / 'src' / 'upper_insert' / 'config' / 'tool_calibration.json'
        if candidate.parent.is_dir():
            return candidate
    return here.parent.parent / 'config' / 'tool_calibration.json'


DEFAULT_TOOL_CALIBRATION_JSON = str(_default_tool_calibration_path())


def _orthonormalize_rotation(mat: Any) -> np.ndarray:
    arr = np.asarray(mat, dtype=np.float64).reshape(3, 3)
    if not np.all(np.isfinite(arr)):
        raise ValueError('工具标定旋转矩阵包含非有限值')
    u, _, vt = np.linalg.svd(arr)
    det = 1.0 if float(np.linalg.det(u @ vt)) >= 0.0 else -1.0
    return u @ np.diag([1.0, 1.0, det]) @ vt


def _find_workspace_file(
    rel_candidates: list[str], pkg_share: tuple[str, str] | None = None
) -> Path | None:
    here = Path(__file__).resolve()
    for root in (here.parents[3], here.parents[2], Path('/workspace')):
        for rel in rel_candidates:
            p = root / rel
            if p.is_file():
                return p
    if pkg_share is not None:
        try:
            from ament_index_python.packages import get_package_share_directory
            p = Path(get_package_share_directory(pkg_share[0])) / pkg_share[1]
            if p.is_file():
                return p
        except Exception:
            pass
    return None


@dataclass
class FtSensorCalib:
    force_bias: np.ndarray
    torque_bias: np.ndarray
    tool_mass: float
    tool_com: np.ndarray
    rotation: np.ndarray
    polarity: float
    measurement_origin: np.ndarray
    frame: str


def load_ft_sensor_calibrations() -> dict[str, FtSensorCalib]:
    """从 arm_gravity_compensation/config/ft_calibration.yaml 加载左右臂 KWR57 标定参数"""
    path = _find_workspace_file(
        ['src/arm_gravity_compensation/config/ft_calibration.yaml'],
        ('arm_gravity_compensation', 'config/ft_calibration.yaml'),
    )
    out: dict[str, FtSensorCalib] = {}
    raw = (yaml.safe_load(path.read_text(encoding='utf-8')) or {}) if path else {}
    params = raw.get('ft_wrench_compensator', {}).get('ros__parameters', raw)
    for side in ('left', 'right'):
        entry = params.get(side, {})
        out[side] = FtSensorCalib(
            force_bias=np.asarray(entry.get('force_bias', [0.0, 0.0, 0.0]), dtype=np.float64).reshape(3),
            torque_bias=np.asarray(entry.get('torque_bias', [0.0, 0.0, 0.0]), dtype=np.float64).reshape(3),
            tool_mass=float(entry.get('tool_mass', 0.80)),
            tool_com=np.asarray(entry.get('tool_com', [0.0, 0.0, 0.10]), dtype=np.float64).reshape(3),
            rotation=np.asarray(entry.get('rotation', np.eye(3).ravel()), dtype=np.float64).reshape(3, 3),
            polarity=float(entry.get('polarity', 1.0)),
            measurement_origin=np.asarray(
                entry.get('measurement_origin', [0.0, 0.0, 0.053]), dtype=np.float64
            ).reshape(3),
            frame=str(entry.get('frame', f'{side}_kwr57b_link')),
        )
    return out


@dataclass
class ToolCalibrationState:
    """单臂末端插头尖端 T 与力传感器 S / 夹爪基座 G 的标定状态"""

    arm: str
    source: str
    r_ST_T: np.ndarray
    R_TS: np.ndarray
    p_G_T: np.ndarray
    R_G_T: np.ndarray
    p_kwr_T: np.ndarray
    p_torso_T: np.ndarray | None = None
    p_torso_S: np.ndarray | None = None
    camera_tf_time_offset_s: float | None = None

    @property
    def distance_flange_mm(self) -> float:
        return float(np.linalg.norm(self.p_G_T) * 1000.0)

    @property
    def distance_kwr_link_mm(self) -> float:
        return float(np.linalg.norm(self.r_ST_T) * 1000.0)

    def to_dict(self) -> dict[str, Any]:
        return {
            'arm': self.arm,
            'source': self.source,
            'camera_tf_time_offset_s': self.camera_tf_time_offset_s,
            'r_ST_T_m': [round(float(v), 5) for v in self.r_ST_T],
            'r_ST_T_mm': [round(float(v * 1000.0), 2) for v in self.r_ST_T],
            'lever_arm_norm_mm': round(self.distance_kwr_link_mm, 2),
            'flange_to_tip_norm_mm': round(self.distance_flange_mm, 2),
            'p_G_T_m': [round(float(v), 5) for v in self.p_G_T],
            'p_G_T_mm': [round(float(v * 1000.0), 2) for v in self.p_G_T],
            'R_G_T': [[round(float(x), 8) for x in row] for row in self.R_G_T],
            'R_TS': [[round(float(x), 8) for x in row] for row in self.R_TS],
            'p_torso_T_m': (
                [round(float(v), 4) for v in self.p_torso_T]
                if self.p_torso_T is not None
                else None
            ),
            'p_torso_S_m': (
                [round(float(v), 4) for v in self.p_torso_S]
                if self.p_torso_S is not None
                else None
            ),
        }


def make_nominal_tool_calibration(
    arm: str,
    tip_in_gripper_m: np.ndarray | None = None,
) -> ToolCalibrationState:
    """根据 URDF (kwr57b_link -> gripper_base) 与夹爪/插头几何构造默认标定值"""
    p_g_t = (
        NOMINAL_P_G_T.copy()
        if tip_in_gripper_m is None
        else np.asarray(tip_in_gripper_m, dtype=np.float64).reshape(3)
    )
    r_g_t = NOMINAL_R_S_T.copy()
    r_ts = NOMINAL_R_TS.copy()
    p_kwr_t = KWR57_TO_GRIPPER_XYZ + p_g_t
    r_st_t = r_ts @ p_kwr_t
    return ToolCalibrationState(
        arm=arm,
        source='urdf_nominal',
        r_ST_T=r_st_t,
        R_TS=r_ts,
        p_G_T=p_g_t,
        R_G_T=r_g_t,
        p_kwr_T=p_kwr_t,
    )


class RobotToolCalibrator:
    """结合头部主摄像机 (D435i) ID1 视觉定位与 URDF 正运动学标定插头末端到力传感器的距离与力臂"""

    ARM_JOINT_NAMES: dict[str, tuple[str, ...]] = {
        'left': (
            'left_shoulder_pitch_joint',
            'left_shoulder_roll_joint',
            'left_shoulder_yaw_joint',
            'left_elbow_joint',
            'left_wrist_roll_joint',
            'left_wrist_pitch_joint',
            'left_wrist_yaw_joint',
        ),
        'right': (
            'right_shoulder_pitch_joint',
            'right_shoulder_roll_joint',
            'right_shoulder_yaw_joint',
            'right_elbow_joint',
            'right_wrist_roll_joint',
            'right_wrist_pitch_joint',
            'right_wrist_yaw_joint',
        ),
    }

    def __init__(
        self,
        pos_json: str = DEFAULT_POS_JSON,
        calibration_json: str | Path | None = DEFAULT_TOOL_CALIBRATION_JSON,
    ) -> None:
        _, self.plug_mm = load_pos_config(pos_json)
        self.calibration_path: Path | None = (
            Path(calibration_json) if calibration_json else None
        )
        self.ft_calibs: dict[str, FtSensorCalib] = {}
        self.reload_ft_calibrations()

        self.T_id1_tip = np.eye(4, dtype=np.float64)
        self.T_id1_tip[:3, :3] = _PLUG_TO_HOLE_BASIS.copy()
        self.T_id1_tip[:3, 3] = self.plug_mm * 0.001
        self.last_head_poses: dict[int, tuple[np.ndarray, np.ndarray]] = {}

        self.model: Any = None
        self.data: Any = None
        self.torso_id: int = -1
        self._load_urdf()
        self.calibrations: dict[str, ToolCalibrationState] = {
            side: make_nominal_tool_calibration(side) for side in ('left', 'right')
        }
        self.load_saved_calibrations()

    def load_saved_calibrations(self) -> None:
        path = self.calibration_path
        if path is None:
            return
        if not path.is_file():
            fallback = _find_workspace_file(
                ['src/upper_insert/config/tool_calibration.json'],
                ('upper_insert', 'config/tool_calibration.json'),
            )
            if fallback is None or not fallback.is_file():
                return
            path = fallback
        try:
            raw = json.loads(path.read_text(encoding='utf-8'))
        except Exception:
            return
        if not isinstance(raw, dict):
            return
        for side in ('left', 'right'):
            entry = raw.get(side)
            if not isinstance(entry, dict) or entry.get('source') != 'head_camera_tf':
                continue
            try:
                r_st_t = np.asarray(entry['r_ST_T_m'], dtype=np.float64).reshape(3)
                r_ts = _orthonormalize_rotation(entry['R_TS'])
                p_g_t = np.asarray(entry['p_G_T_m'], dtype=np.float64).reshape(3)
                r_g_t = _orthonormalize_rotation(entry['R_G_T'])
                p_kwr_t = np.asarray(
                    entry.get('p_kwr_T_m', r_ts.T @ r_st_t), dtype=np.float64
                ).reshape(3)
                if not (
                    np.all(np.isfinite(r_st_t))
                    and np.all(np.isfinite(p_g_t))
                    and np.all(np.isfinite(p_kwr_t))
                ):
                    continue
                p_torso_t = (
                    np.asarray(entry['p_torso_T_m'], dtype=np.float64).reshape(3)
                    if entry.get('p_torso_T_m') is not None
                    else None
                )
                p_torso_s = (
                    np.asarray(entry['p_torso_S_m'], dtype=np.float64).reshape(3)
                    if entry.get('p_torso_S_m') is not None
                    else None
                )
                offset = entry.get('camera_tf_time_offset_s')
                offset_val = float(offset) if offset is not None and math.isfinite(float(offset)) else None
                self.calibrations[side] = ToolCalibrationState(
                    arm=side,
                    source='head_camera_tf',
                    r_ST_T=r_st_t,
                    R_TS=r_ts,
                    p_G_T=p_g_t,
                    R_G_T=r_g_t,
                    p_kwr_T=p_kwr_t,
                    p_torso_T=p_torso_t,
                    p_torso_S=p_torso_s,
                    camera_tf_time_offset_s=offset_val,
                )
            except Exception:
                continue

    def save_calibrations(self) -> None:
        if self.calibration_path is None:
            return
        payload: dict[str, Any] = {}
        if self.calibration_path.is_file():
            try:
                existing = json.loads(self.calibration_path.read_text(encoding='utf-8'))
                if isinstance(existing, dict):
                    payload.update(existing)
            except Exception:
                pass
        for side, state in self.calibrations.items():
            if state.source != 'head_camera_tf':
                continue
            payload[side] = {
                'arm': side,
                'source': state.source,
                'camera_tf_time_offset_s': state.camera_tf_time_offset_s,
                'r_ST_T_m': [round(float(v), 8) for v in state.r_ST_T],
                'R_TS': [[round(float(x), 8) for x in row] for row in state.R_TS],
                'p_G_T_m': [round(float(v), 8) for v in state.p_G_T],
                'R_G_T': [[round(float(x), 8) for x in row] for row in state.R_G_T],
                'p_kwr_T_m': [round(float(v), 8) for v in state.p_kwr_T],
                'p_torso_T_m': (
                    [round(float(v), 6) for v in state.p_torso_T]
                    if state.p_torso_T is not None
                    else None
                ),
                'p_torso_S_m': (
                    [round(float(v), 6) for v in state.p_torso_S]
                    if state.p_torso_S is not None
                    else None
                ),
            }
        self.calibration_path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = self.calibration_path.with_suffix('.json.tmp')
        tmp_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        tmp_path.replace(self.calibration_path)

    def reload_ft_calibrations(self) -> dict[str, FtSensorCalib]:
        self.ft_calibs = load_ft_sensor_calibrations()
        return self.ft_calibs

    def _q7_to_joint_dict(
        self, arm: str, q_arm7: dict[str, float] | list[float] | np.ndarray
    ) -> dict[str, float]:
        if isinstance(q_arm7, dict):
            return q_arm7
        arr = np.asarray(q_arm7, dtype=np.float64).reshape(-1)
        names = self.ARM_JOINT_NAMES[arm]
        return dict(zip(names, map(float, arr)))

    def get_calibration(self, arm: str) -> dict[str, Any]:
        return self.calibrations[arm].to_dict()

    def reset_nominal(self, arm: str) -> dict[str, Any]:
        self.calibrations[arm] = make_nominal_tool_calibration(arm)
        return self.calibrations[arm].to_dict()

    def compute_arm_fk(
        self, arm: str, q_arm7: dict[str, float] | list[float] | np.ndarray
    ) -> dict[str, np.ndarray]:
        jdict = self._q7_to_joint_dict(arm, q_arm7)
        frames = self.arm_frames_in_torso(arm, jdict)
        return {
            'T_torso_S': frames['T_torso_kwr'],
            'T_torso_G': frames['T_torso_gb'],
        }

    def compute_live_fk(
        self, arm: str, q_arm7: dict[str, float] | list[float] | np.ndarray
    ) -> dict[str, Any]:
        cal = self.calibrations[arm]
        out = cal.to_dict()
        try:
            fk = self.compute_arm_fk(arm, q_arm7)
            T_torso_S = fk['T_torso_S']
            T_torso_G = fk['T_torso_G']
            T_G_T = np.eye(4, dtype=np.float64)
            T_G_T[:3, :3] = cal.R_G_T
            T_G_T[:3, 3] = cal.p_G_T
            T_torso_T = T_torso_G @ T_G_T
            out['p_torso_S_m'] = [round(float(v), 4) for v in T_torso_S[:3, 3]]
            out['p_torso_T_m'] = [round(float(v), 4) for v in T_torso_T[:3, 3]]
        except Exception:
            out['p_torso_S_m'] = None
            out['p_torso_T_m'] = None
        ft_c = self.ft_calibs.get(arm)
        out['tool_mass_kg'] = round(float(ft_c.tool_mass), 3) if ft_c is not None else 0.0
        return out

    def _load_urdf(self) -> None:
        urdf_file = _find_workspace_file(
            ['src/unitree_g1_description/model/final.urdf'],
            ('unitree_g1_description', 'model/final.urdf'),
        )
        if urdf_file is not None:
            self.model = pin.buildModelFromUrdf(str(urdf_file))
            self.data = self.model.createData()
            self.torso_id = self.model.getFrameId('torso_link')

    def arm_frames_in_torso(
        self, arm: str, joint_positions: dict[str, float]
    ) -> dict[str, np.ndarray]:
        """根据实测关节角与 URDF 正运动学计算力传感器与夹爪在 torso_link 下的 4x4 位姿"""
        if self.model is None or self.data is None:
            raise RuntimeError('URDF 模型未加载')
        q = pin.neutral(self.model)
        for j_name, val in joint_positions.items():
            if self.model.existJointName(j_name):
                jid = self.model.getJointId(j_name)
                idx_q = self.model.joints[jid].idx_q
                if self.model.joints[jid].nq == 1 and math.isfinite(float(val)):
                    q[idx_q] = float(val)

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        torso_inverse = self.data.oMf[self.torso_id].inverse()
        return {
            key: (torso_inverse * self.data.oMf[self.model.getFrameId(link)]).homogeneous.copy()
            for key, link in (('T_torso_kwr', f'{arm}_kwr57b_link'),
                              ('T_torso_gb', f'{arm}_gripper_base'))
        }

    @staticmethod
    def detect_marker_multiscale(
        frame: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        target_ids: tuple[int, ...] = (1,),
        marker_length_m: float = MARKER_LENGTH_M,
    ) -> dict[int, tuple[np.ndarray, np.ndarray]]:
        """全图及分块标准检测；标定不使用遮挡补码。"""
        est = VisionPoseEstimator(
            camera_matrix, dist_coeffs, marker_length_m=marker_length_m,
            target_ids=target_ids, allow_recovery=False, standard_scales=(1.0, 0.5, 0.25),
        )
        _, poses = est.estimate_poses(frame, draw=False)
        if all(m_id in poses for m_id in target_ids):
            return poses

        h_img, w_img = frame.shape[:2]

        tile_h, tile_w = 440, 460
        ys = list(range(0, max(1, h_img - tile_h + 1), 280))
        if h_img > tile_h and ys[-1] != h_img - tile_h:
            ys.append(h_img - tile_h)
        xs = list(range(0, max(1, w_img - tile_w + 1), 320))
        if w_img > tile_w and xs[-1] != w_img - tile_w:
            xs.append(w_img - tile_w)

        est.standard_scales = (1.0,)
        for y0 in ys:
            for x0 in xs:
                tile_k = np.array(camera_matrix, dtype=np.float64, copy=True)
                tile_k[:2, 2] -= [x0, y0]
                est.camera_matrix = tile_k
                _, local_poses = est.estimate_poses(
                    frame[y0:y0 + tile_h, x0:x0 + tile_w], draw=False,
                )
                for m_id, pose in local_poses.items():
                    poses.setdefault(m_id, pose)
                if all(m_id in poses for m_id in target_ids):
                    return poses
        return poses

    def calibrate_from_head_camera(
        self,
        arm: str,
        *,
        frame_bgr: np.ndarray,
        head_K: np.ndarray,
        head_dist: np.ndarray,
        T_torso_camera: np.ndarray,
        T_torso_gripper: np.ndarray,
        T_torso_sensor: np.ndarray,
        camera_tf_time_offset_s: float = 0.0,
    ) -> dict[str, Any]:
        """手臂 TF 对齐图像；相机仅在头部静止假设下允许有界时间差。"""
        self.last_head_poses = {}
        if not np.isfinite([T_torso_camera, T_torso_gripper, T_torso_sensor]).all():
            raise ValueError('工具标定 TF 含非有限数据')
        if not np.isfinite(head_K).all() or head_K[0, 0] <= 0.0 or head_K[1, 1] <= 0.0:
            raise ValueError('主相机 CameraInfo 内参无效')
        poses_head = self.detect_marker_multiscale(
            frame_bgr, head_K, head_dist, target_ids=(1,))
        self.last_head_poses = poses_head
        if 1 not in poses_head:
            raise ValueError('主相机未检测到 ID1；标定需要正面完整的标准码，不使用遮挡恢复')
        rvec, tvec = poses_head[1]

        marker = pin.SE3(cv2.Rodrigues(rvec)[0], tvec.reshape(3))
        tip = pin.SE3(T_torso_camera) * marker * pin.SE3(self.T_id1_tip)
        gripper_tip = pin.SE3(T_torso_gripper).inverse() * tip
        sensor_tip = pin.SE3(T_torso_sensor).inverse() * tip
        r_ts = sensor_tip.rotation.T.copy()
        state = ToolCalibrationState(
            arm=arm, source='head_camera_tf',
            r_ST_T=r_ts @ sensor_tip.translation, R_TS=r_ts,
            p_G_T=gripper_tip.translation.copy(), R_G_T=gripper_tip.rotation.copy(),
            p_kwr_T=sensor_tip.translation.copy(), p_torso_T=tip.translation.copy(),
            p_torso_S=T_torso_sensor[:3, 3].copy(),
            camera_tf_time_offset_s=camera_tf_time_offset_s,
        )
        self.calibrations[arm] = state
        self.save_calibrations()
        return state.to_dict()
