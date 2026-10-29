"""6DoF 视觉定位与机械臂插孔策略输入计算核心模块."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Any, Callable, cast

import cv2
import numpy as np

from upper_insert import package_file

# 3.5cm 贴纸（含四周 2.5mm 白边）内部黑色方框外沿物理边长：3.0 cm = 0.030 m
MARKER_LENGTH_M = 0.030
ARUCO_DICT_ID = cv2.aruco.DICT_4X4_50
TARGET_MARKER_IDS = (0, 1)


DEFAULT_CAMERA_CONFIG = str(package_file("config/camera_config.json"))
DEFAULT_POS_JSON = str(package_file("config/pos.json"))

_DST_60 = np.array([[0.0, 0.0], [60.0, 0.0], [60.0, 60.0], [0.0, 60.0]], dtype=np.float32)
_CORNERS_AFFINE = np.array(
    [[0.0, 0.0, 1.0], [60.0, 0.0, 1.0], [60.0, 60.0, 1.0], [0.0, 60.0, 1.0]], dtype=np.float32
)
_S_VALS = np.linspace(-3.5, 3.5, 29, dtype=np.float64)
_T_VALS = np.linspace(0.12, 0.88, 28, dtype=np.float64)
_OFFSETS_9 = np.arange(-4, 5, dtype=np.int32)
_BORDER_MASK = np.ones((6, 6), dtype=bool)
_BORDER_MASK[1:5, 1:5] = False

# 当插头完全对准插孔时（插头 -Y1 对齐插孔 -Z0，且插头 +Z1 对齐插孔 +X0），
# ID1 的三个坐标轴 [X1, Y1, Z1] 分别对应 ID0 的 [+Y0, +Z0, +X0]：
_PLUG_TO_HOLE_BASIS = np.array(
    [[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]], dtype=np.float64
)

# 夹爪抓取插头时 ID1 坐标系到 gripper_base 坐标系的旋转矩阵：
# +X1 -> +X_gripper（夹爪张合方向），-Y1 -> +Z_gripper（夹爪前伸/插头插入方向），+Z1 -> +Y_gripper
R_GRIPPER_FROM_ID1 = np.array(
    [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]], dtype=np.float64
)


def draw_aruco_marker(aruco_dict: Any, marker_id: int, side_pixels: int) -> np.ndarray:
    """兼容 OpenCV 4.5 (drawMarker) 与 OpenCV 4.7+ (generateImageMarker)."""
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(aruco_dict, marker_id, side_pixels)
    if hasattr(aruco_dict, "generateImageMarker"):
        return aruco_dict.generateImageMarker(marker_id, side_pixels)
    if hasattr(cv2.aruco, "drawMarker"):
        return cv2.aruco.drawMarker(aruco_dict, marker_id, side_pixels)
    return aruco_dict.drawMarker(marker_id, side_pixels)


def resolve_camera_index(camera_index: int = 0, camera_name: str = "Azure Kinect") -> int:
    """在 Linux V4L2 下根据设备名自动定位 Azure Kinect 4K Camera 的真实 video 节点索引."""
    if os.name == "nt":
        return int(camera_index)
    v4l_root = Path("/sys/class/video4linux")
    if not v4l_root.is_dir():
        return int(camera_index)

    keywords = []
    if "azure kinect" in camera_name.lower() or "kinect" in camera_name.lower():
        keywords.append("azure kinect")
    elif camera_name and camera_name.lower() != "camera":
        keywords.append(camera_name.split("(")[0].strip().lower())

    def _read_text(p: Path) -> str:
        try:
            return p.read_text(encoding="utf-8").strip()
        except OSError:
            return ""

    if camera_index >= 0:
        cur_dir = v4l_root / f"video{camera_index}"
        cur_name = _read_text(cur_dir / "name").lower()
        if cur_dir.exists() and (not keywords or any(k in cur_name for k in keywords)):
            return int(camera_index)

    entries: list[tuple[int, str, str]] = []
    for entry in v4l_root.iterdir():
        if not entry.name.startswith("video"):
            continue
        suffix = entry.name[5:]
        if not suffix.isdigit():
            continue
        idx = int(suffix)
        name = _read_text(entry / "name")
        v4l_idx = _read_text(entry / "index")
        entries.append((idx, name, v4l_idx))
    entries.sort(key=lambda item: item[0])

    for kw in keywords or ["azure kinect"]:
        for idx, name, v4l_idx in entries:
            if kw in name.lower() and v4l_idx in ("0", ""):
                return idx
        for idx, name, _ in entries:
            if kw in name.lower():
                return idx

    return max(0, int(camera_index))


# ============================================================================
# 1. 配置文件加载 (camera_config.json & pos.json)
# ============================================================================


def load_camera_config(config_path: str = DEFAULT_CAMERA_CONFIG) -> dict[str, Any]:
    """从 camera_config.json 加载相机内参矩阵、畸变系数与视频流采集参数."""
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"未找到相机内参配置文件: {config_path}")
    with open(config_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    cam_block = data.get("camera", data)
    intr = cam_block.get("intrinsics", cam_block)

    width = int(cam_block.get("width", 1280))
    height = int(cam_block.get("height", 720))
    fps = int(cam_block.get("fps", 30))
    camera_name = str(cam_block.get("camera_name", "Azure Kinect"))
    raw_index = int(cam_block.get("camera_index", 6))
    camera_index = resolve_camera_index(raw_index, camera_name)

    calib_w = int(intr.get("calib_width", width))
    calib_h = int(intr.get("calib_height", height))
    sx = float(width) / float(calib_w) if calib_w > 0 else 1.0
    sy = float(height) / float(calib_h) if calib_h > 0 else 1.0

    if "camera_matrix" in cam_block:
        camera_matrix = np.asarray(cam_block["camera_matrix"], dtype=np.float64).reshape(3, 3).copy()
    else:
        fx = float(intr["fx"])
        fy = float(intr.get("fy", fx))
        cx = float(intr.get("cx", calib_w / 2.0))
        cy = float(intr.get("cy", calib_h / 2.0))
        camera_matrix = np.array(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64
        )

    camera_matrix[0, 0] *= sx
    camera_matrix[0, 2] *= sx
    camera_matrix[1, 1] *= sy
    camera_matrix[1, 2] *= sy

    raw_dist = cam_block.get("dist_coeffs", [0.0, 0.0, 0.0, 0.0, 0.0])
    dist_coeffs = np.asarray(raw_dist, dtype=np.float64).reshape(-1, 1)

    return {
        "camera_name": camera_name,
        "camera_index": camera_index,
        "width": width,
        "height": height,
        "fps": fps,
        "pixel_format": cam_block.get("pixel_format", "MJPG"),
        "camera_matrix": camera_matrix,
        "dist_coeffs": dist_coeffs,
    }


def load_pos_config(json_path: str = DEFAULT_POS_JSON) -> tuple[dict[str, np.ndarray], np.ndarray]:
    """加载 pos.json 中的孔位与插头局部坐标（单位: mm）."""
    if not os.path.exists(json_path):
        raise FileNotFoundError(f"未找到点位配置文件: {json_path}")
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    holes_mm = {
        name: np.asarray(coords, dtype=np.float64).reshape(3)
        for name, coords in data.get("holes", {}).items()
    }
    plug_mm = np.asarray(data.get("plug", [0.0, 0.0, 0.0]), dtype=np.float64).reshape(3)
    return holes_mm, plug_mm


# ============================================================================
# 2. 四元数转换与插头-插孔相对位姿 (Delta) 计算
# ============================================================================


def _to_rot_mat(rvec_or_R: np.ndarray) -> np.ndarray:
    arr = np.asarray(rvec_or_R, dtype=np.float64)
    return arr if arr.shape == (3, 3) else cv2.Rodrigues(arr)[0]


def _canonicalize_quat(q_xyzw: np.ndarray, order: str = "xyzw") -> np.ndarray:
    norm = float(np.linalg.norm(q_xyzw))
    q = (q_xyzw / norm) if norm > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    q_wxyz = np.array([q[3], q[0], q[1], q[2]], dtype=np.float64)
    for comp in q_wxyz:
        if abs(float(comp)) > 1e-8:
            if comp < 0.0:
                q = -q
                q_wxyz = -q_wxyz
            break
    if order.lower() == "wxyz":
        return q_wxyz
    return q


def rotation_matrix_to_quat(R: np.ndarray, order: str = "xyzw") -> np.ndarray:
    """将 3x3 旋转矩阵转换为单位四元数 (w >= 0)，支持 'xyzw' 或 'wxyz' 顺序."""
    m = np.asarray(R, dtype=np.float64).reshape(3, 3)
    tr = float(m[0, 0] + m[1, 1] + m[2, 2])
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q_xyzw = np.array(
            [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s],
            dtype=np.float64,
        )
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        q_xyzw = np.array(
            [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s],
            dtype=np.float64,
        )
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        q_xyzw = np.array(
            [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s],
            dtype=np.float64,
        )
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        q_xyzw = np.array(
            [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s],
            dtype=np.float64,
        )
    return _canonicalize_quat(q_xyzw, order)


def compute_plug_hole_delta(
    rvec0: np.ndarray,
    tvec0: np.ndarray,
    rvec1: np.ndarray,
    tvec1: np.ndarray,
    holes_mm: dict[str, np.ndarray] | None = None,
    plug_mm: np.ndarray | None = None,
    target_hole: str | None = None,
    quat_order: str = "xyzw",
    angle_thresh_deg: float = 25.0,
    insert_depth_mm: float | None = None,
) -> dict[str, Any]:
    """计算插头 (ID 1) 到目标插孔 (ID 0) 的平移差与旋转四元数差，供机械臂插入控制策略直接使用."""
    if holes_mm is None or plug_mm is None:
        default_holes, default_plug = load_pos_config()
        if holes_mm is None:
            holes_mm = default_holes
        if plug_mm is None:
            plug_mm = default_plug

    R0, R1 = _to_rot_mat(rvec0), _to_rot_mat(rvec1)
    t0 = np.asarray(tvec0, dtype=np.float64).reshape(3)
    t1 = np.asarray(tvec1, dtype=np.float64).reshape(3)

    plug_cam_m = R1 @ (plug_mm * 0.001) + t1
    plug_in_id0_mm = (R0.T @ (plug_cam_m - t0)) * 1000.0

    nearest_hole = min(holes_mm, key=lambda k: float(np.linalg.norm(plug_in_id0_mm - holes_mm[k])))
    selected_hole = nearest_hole if (not target_hole or target_hole == "auto") else target_hole
    if selected_hole not in holes_mm:
        raise KeyError(f"未知的插孔名称 '{selected_hole}'，可选值: {list(holes_mm.keys())}")

    hole_in_id0_mm = holes_mm[selected_hole].copy()
    if insert_depth_mm is not None and math.isfinite(float(insert_depth_mm)):
        hole_in_id0_mm[2] = -float(insert_depth_mm)
    eff_depth_mm = float(-hole_in_id0_mm[2])
    cur_depth_mm = float(-plug_in_id0_mm[2])

    delta_id0_mm = hole_in_id0_mm - plug_in_id0_mm
    delta_pos = delta_id0_mm * 0.001
    R_HT = R0.T @ R1 @ _PLUG_TO_HOLE_BASIS
    delta_id1_m = _PLUG_TO_HOLE_BASIS @ R_HT.T @ delta_pos
    delta_rot_id1 = _PLUG_TO_HOLE_BASIS @ R_HT.T @ _PLUG_TO_HOLE_BASIS.T
    delta_quat = rotation_matrix_to_quat(R_HT.T, order=quat_order)
    delta_quat_wxyz = rotation_matrix_to_quat(R_HT.T, order="wxyz")

    axis_angle_deg = float(np.degrees(np.arccos(np.clip(float(R_HT[2, 2]), -1.0, 1.0))))
    cos_sim = float(np.clip(float(R_HT[0, 0]), -1.0, 1.0))
    yaw_angle_deg = float(np.degrees(np.arccos(cos_sim)))
    distance_mm = float(np.linalg.norm(delta_id0_mm))
    lateral_err_mm = float(np.hypot(delta_id0_mm[0], delta_id0_mm[1]))

    return {
        "policy_obs": np.concatenate([delta_pos, delta_quat]).astype(np.float64),
        "delta_pos": delta_pos,
        "delta_quat": delta_quat,
        "delta_quat_wxyz": delta_quat_wxyz,
        "delta_rot_id1": delta_rot_id1,
        "R_HT": R_HT,
        "delta_id0_m": delta_pos.copy(),
        "delta_id1_m": delta_id1_m,
        "target_hole": selected_hole,
        "nearest_hole": nearest_hole,
        "distance_mm": distance_mm,
        "lateral_err_mm": lateral_err_mm,
        "depth_err_mm": float(eff_depth_mm - cur_depth_mm),
        "insert_depth_mm": eff_depth_mm,
        "current_depth_mm": cur_depth_mm,
        "plug_in_id0_mm": plug_in_id0_mm,
        "hole_in_id0_mm": hole_in_id0_mm,
        "axis_angle_deg": axis_angle_deg,
        "yaw_angle_deg": yaw_angle_deg,
        "cos_sim": cos_sim,
        "orientation_ok": yaw_angle_deg <= angle_thresh_deg,
    }


# ============================================================================
# 3. 抗遮挡多分辨率自适应标定码检测与 6DoF 位姿解算
# ============================================================================


def _build_gt_templates(target_ids=TARGET_MARKER_IDS) -> dict[int, list[np.ndarray]]:
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    templates: dict[int, list[np.ndarray]] = {}
    for m_id in target_ids:
        base = (draw_aruco_marker(aruco_dict, m_id, 6) > 127).astype(np.uint8)
        templates[m_id] = [np.rot90(base, -r) for r in range(4)]
    return templates


GT_TEMPLATES = _build_gt_templates()


def _build_transition_tables(templates: dict[int, list[np.ndarray]]) -> dict[int, tuple]:
    tables: dict[int, tuple] = {}
    for m_id, rots in templates.items():
        gt = rots[0]
        h_list, v_list = [], []
        for r in range(6):
            for c in range(5):
                if gt[r, c] != gt[r, c + 1]:
                    sign = 1.0 if gt[r, c + 1] > gt[r, c] else -1.0
                    xc = (c + 1) * 10
                    for dy in (3, 5, 7):
                        h_list.append((xc, r * 10 + dy, sign))
        for r in range(5):
            for c in range(6):
                if gt[r, c] != gt[r + 1, c]:
                    sign = 1.0 if gt[r + 1, c] > gt[r, c] else -1.0
                    yc = (r + 1) * 10
                    for dx in (3, 5, 7):
                        v_list.append((c * 10 + dx, yc, sign))
        tables[m_id] = (
            np.array([x for x, _, _ in h_list], dtype=np.int32),
            np.array([y for _, y, _ in h_list], dtype=np.int32),
            np.array([s for _, _, s in h_list], dtype=np.float32),
            np.array([x for x, _, _ in v_list], dtype=np.int32),
            np.array([y for _, y, _ in v_list], dtype=np.int32),
            np.array([s for _, _, s in v_list], dtype=np.float32),
        )
    return tables


TRANS_TABLES = _build_transition_tables(GT_TEMPLATES)


def get_marker_object_points(marker_length: float = MARKER_LENGTH_M) -> np.ndarray:
    """返回单个标定码四个角点在标定码自身坐标系下的 3D 坐标（单位：米，Z=0）."""
    half = marker_length / 2.0
    return np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=np.float64,
    )


def _intersect_lines(
    a1: np.ndarray, a2: np.ndarray, b1: np.ndarray, b2: np.ndarray
) -> np.ndarray | None:
    da = a2 - a1
    db = b2 - b1
    dap = np.array([-da[1], da[0]], dtype=np.float64)
    denom = float(np.dot(dap, db))
    if abs(denom) < 1e-6:
        return None
    dp = a1 - b1
    return b1 + (float(np.dot(dap, dp)) / denom) * db


def _native_vision():
    try:
        from upper_insert import _vision_native
    except ImportError as error:
        raise RuntimeError(
            "Native vision extension unavailable; build upper_insert with colcon"
        ) from error
    return _vision_native


def _bgr_to_sat(bgr: np.ndarray) -> np.ndarray:
    """精确融合 OpenCV 饱和度与 V/115 暗部加权，省去无用的色相计算."""
    if bgr.strides[0] < 0 or bgr.strides[1:] != (3, 1):
        bgr = np.ascontiguousarray(bgr)
    result = np.empty(bgr.shape[:2], dtype=np.uint8)
    _native_vision().saturation(bgr, result)
    return result


def _order_quad_clockwise(pts: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts, dtype=np.float32).reshape(4, 2)
    center = pts.mean(axis=0)
    angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    ordered = pts[np.argsort(angles)]
    start_idx = int(np.argmin(ordered.sum(axis=1)))
    return np.roll(ordered, -start_idx, axis=0).astype(np.float32)


def _verify_and_decode_occluded_quad(
    gray: np.ndarray,
    hsv_sat: np.ndarray,
    quad: np.ndarray,
    missing_ids: list[int],
    min_valid_cells: int = 12,
    min_inner_cells: int = 4,
    min_total_acc: float = 0.88,
    min_inner_acc: float = 0.82,
    require_rot0: bool = False,
) -> tuple[int, np.ndarray, float] | None:
    """将候选四边形透视矫正为 60x60 网格，掩膜剔除遮挡单元格后与标准模板匹配解码."""
    quad = (
        np.asarray(quad, dtype=np.float32).reshape(4, 2)
        if require_rot0
        else _order_quad_clockwise(quad)
    )

    sides = [float(np.linalg.norm(quad[(i + 1) % 4] - quad[i])) for i in range(4)]
    s_min, s_max = min(sides), max(sides)
    if s_min < 13.0 or s_max / max(1.0, s_min) > 3.6:
        return None
    if (
        max(sides[0], sides[2]) / max(1.0, min(sides[0], sides[2])) > 1.55
        or max(sides[1], sides[3]) / max(1.0, min(sides[1], sides[3])) > 1.55
    ):
        return None
    diag1 = float(np.linalg.norm(quad[2] - quad[0]))
    diag2 = float(np.linalg.norm(quad[3] - quad[1]))
    if max(diag1, diag2) / max(1.0, min(diag1, diag2)) > 2.2:
        return None

    h_img, w_img = gray.shape[:2]
    x0 = max(0, int(np.floor(quad[:, 0].min())) - 2)
    y0 = max(0, int(np.floor(quad[:, 1].min())) - 2)
    x1 = min(w_img, int(np.ceil(quad[:, 0].max())) + 3)
    y1 = min(h_img, int(np.ceil(quad[:, 1].max())) + 3)
    if x1 - x0 < 16 or y1 - y0 < 16:
        return None

    quad_local = quad - np.array([[x0, y0]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(quad_local, _DST_60)
    warped_gray = cv2.warpPerspective(gray[y0:y1, x0:x1], M, (60, 60))
    warped_sat = cv2.warpPerspective(hsv_sat[y0:y1, x0:x1], M, (60, 60))

    pg = warped_gray.reshape(6, 10, 6, 10)[:, 2:8, :, 2:8].transpose(0, 2, 1, 3).reshape(6, 6, 36)
    ps = warped_sat.reshape(6, 10, 6, 10)[:, 2:8, :, 2:8].transpose(0, 2, 1, 3).reshape(6, 6, 36)

    cell_gray = np.partition(pg, 18, axis=2)[:, :, 18].astype(np.float32)
    cell_sat = np.partition(ps, 18, axis=2)[:, :, 18].astype(np.float32)
    cell_std = np.std(pg.astype(np.float32), axis=2)

    valid_mask = (cell_sat < 145.0) & (cell_std < 52.0)
    if int(np.count_nonzero(valid_mask)) < min_valid_cells:
        return None

    sv = np.sort(cell_gray[valid_mask])
    n_sv = len(sv) - 1
    p15 = float(sv[(15 * n_sv) // 100])
    p85 = float(sv[(85 * n_sv) // 100])
    if p85 - p15 < 32.0:
        return None

    thresh = 0.5 * (p15 + p85)
    confident_mask = valid_mask & (np.abs(cell_gray - thresh) > 10.0)
    n_total = int(np.count_nonzero(confident_mask))
    if n_total < min_valid_cells:
        return None

    inner_conf = confident_mask[1:5, 1:5]
    n_inner = int(np.count_nonzero(inner_conf))
    if n_inner < min_inner_cells:
        return None

    border_conf = confident_mask & _BORDER_MASK
    n_border = int(np.count_nonzero(border_conf))
    if n_border < 6:
        return None

    bits = (cell_gray > thresh).astype(np.uint8)
    if float(np.count_nonzero(bits[border_conf] == 0)) / n_border < 0.85:
        return None

    bits_inner = bits[1:5, 1:5]
    best_match: tuple[int, int] | None = None
    best_score = -1.0
    rot_range = (0,) if require_rot0 else (0, 1, 2, 3)

    for m_id in missing_ids:
        rots = GT_TEMPLATES[m_id]
        for rot in rot_range:
            gt = rots[rot]
            total_acc = float(np.count_nonzero(bits[confident_mask] == gt[confident_mask])) / n_total
            inner_acc = (
                float(np.count_nonzero(bits_inner[inner_conf] == gt[1:5, 1:5][inner_conf]))
                / n_inner
            )
            if total_acc >= min_total_acc and inner_acc >= min_inner_acc:
                size_bonus = 0.15 * min(1.0, float(np.mean(sides)) / 120.0)
                score = inner_acc * 1.2 + total_acc + 0.015 * n_inner + size_bonus
                if score > best_score:
                    best_score = score
                    best_match = (m_id, rot)

    if best_match is None:
        return None

    matched_id, matched_rot = best_match
    aligned_quad = np.roll(quad, -matched_rot, axis=0).astype(np.float32)
    return matched_id, aligned_quad, best_score


def _fit_robust_1d(x: np.ndarray, y: np.ndarray, target: np.ndarray) -> np.ndarray:
    A = np.column_stack([x, y, np.ones_like(x)])
    p, _, _, _ = np.linalg.lstsq(A, target, rcond=None)
    for _ in range(3):
        r = np.abs(A @ p - target)
        w = np.where(r < 1.0, 1.0, 1.0 / (r + 1e-3))
        Aw = A * w[:, None]
        tw = target * w
        p, _, _, _ = np.linalg.lstsq(Aw, tw, rcond=None)
    return p


def _refine_quad_edges_subpix(
    gray: np.ndarray,
    hsv_sat: np.ndarray | None,
    quad: np.ndarray,
    m_id: int | None = None,
    res_scale: float = 1.0,
) -> np.ndarray:
    """向量化边缘法向梯度 + 内部比特网格跳变联合亚像素精修."""
    h, w = gray.shape[:2]
    pad = max(12, int(round(12.0 * res_scale)))
    x0 = max(0, int(np.floor(quad[:, 0].min())) - pad)
    y0 = max(0, int(np.floor(quad[:, 1].min())) - pad)
    x1 = min(w, int(np.ceil(quad[:, 0].max())) + pad + 1)
    y1 = min(h, int(np.ceil(quad[:, 1].max())) + pad + 1)
    if x1 - x0 < 16 or y1 - y0 < 16:
        return quad

    offset = np.array([x0, y0], dtype=np.float32)
    quad_local = (quad - offset[None, :]).astype(np.float32)
    g_roi = gray[y0:y1, x0:x1]
    s_roi = np.zeros_like(g_roi) if hsv_sat is None else hsv_sat[y0:y1, x0:x1]
    g_roi_f = g_roi.astype(np.float32)
    s_roi_f = s_roi.astype(np.float32)
    rh, rw = g_roi.shape[:2]

    def _fit_outer_edges(q_loc: np.ndarray) -> list[tuple[np.ndarray, np.ndarray] | None]:
        q_d = q_loc.astype(np.float64)
        c_loc = q_d.mean(axis=0)
        fitted_lines: list[tuple[np.ndarray, np.ndarray] | None] = [None] * 4
        samples: list[tuple[int, np.ndarray, np.ndarray]] = []
        maps_x: list[np.ndarray] = []
        maps_y: list[np.ndarray] = []
        for i in range(4):
            p1 = q_d[i]
            p2 = q_d[(i + 1) % 4]
            edge_vec = p2 - p1
            edge_len = float(np.linalg.norm(edge_vec))
            if edge_len < 12.0:
                continue
            t_dir = edge_vec / edge_len
            n_out = np.array([t_dir[1], -t_dir[0]], dtype=np.float64)
            if np.dot(0.5 * (p1 + p2) - c_loc, n_out) < 0:
                n_out = -n_out

            base_pts = p1[None, :] + (_T_VALS[:, None] * edge_len) * t_dir[None, :]
            ix = np.round(base_pts[:, 0]).astype(np.int32)
            iy = np.round(base_pts[:, 1]).astype(np.int32)
            in_bounds = (ix >= 5) & (ix < rw - 5) & (iy >= 5) & (iy < rh - 5)
            valid_idx = np.where(in_bounds)[0]
            valid_idx = valid_idx[s_roi[iy[valid_idx], ix[valid_idx]] <= 145]

            if len(valid_idx) >= 6:
                valid_base = base_pts[valid_idx]
                sample_pts = valid_base[:, None, :] + _S_VALS[None, :, None] * n_out[None, None, :]
                samples.append((i, valid_base, n_out))
                maps_x.append(sample_pts[:, :, 0].astype(np.float32))
                maps_y.append(sample_pts[:, :, 1].astype(np.float32))
        if not samples:
            return fitted_lines

        map_x = np.concatenate(maps_x, axis=0)
        map_y = np.concatenate(maps_y, axis=0)
        profs_g = cv2.remap(g_roi_f, map_x, map_y, cv2.INTER_LINEAR)
        profs_s = cv2.remap(s_roi_f, map_x, map_y, cv2.INTER_LINEAR)
        grads_mid = (profs_g[:, 6:] - profs_g[:, :-6]) * (1.0 / 6.0)
        sub_grads = grads_mid[:, 1:-1]
        best_rel = np.argmax(sub_grads, axis=1)
        row_idx = np.arange(len(map_x))
        best_grad = sub_grads[row_idx, best_rel]
        peak_idx = best_rel + 4

        in_idx = np.maximum(0, peak_idx - 4)
        out_idx = np.minimum(len(_S_VALS) - 1, peak_idx + 4)
        in_g = profs_g[row_idx, in_idx]
        out_g = profs_g[row_idx, out_idx]
        out_s = profs_s[row_idx, out_idx]
        contrast = out_g - in_g
        good_edge = (
            (best_grad > np.maximum(4.0, 0.08 * contrast))
            & (in_g < 0.80 * out_g)
            & (contrast > np.maximum(22.0, 0.18 * out_g))
            & (out_s < 150.0)
        )
        g_prev = grads_mid[row_idx, best_rel]
        g_curr = grads_mid[row_idx, best_rel + 1]
        g_next = grads_mid[row_idx, best_rel + 2]
        denom = g_prev - 2.0 * g_curr + g_next
        safe_denom = np.where(np.abs(denom) > 1e-5, denom, 1.0)
        sub_off = np.where(
            np.abs(denom) > 1e-5,
            np.clip(0.5 * (g_prev - g_next) / safe_denom, -0.5, 0.5),
            0.0,
        )
        best_s = _S_VALS[peak_idx] + sub_off * 0.25

        start = 0
        for i, valid_base, n_out in samples:
            end = start + len(valid_base)
            good = good_edge[start:end]
            best_s_edge = best_s[start:end]
            start = end
            # Batched profiles still require six good points on each individual edge.
            if int(np.count_nonzero(good)) < 6:
                continue
            edge_pts = (
                valid_base[good] + best_s_edge[good, None] * n_out[None, :]
            ).astype(np.float32)
            vx, vy, lx0, ly0 = cv2.fitLine(
                edge_pts, cv2.DIST_HUBER, 0, 0.01, 0.01
            ).flatten()
            p_start = np.array([lx0, ly0], dtype=np.float64)
            p_end = np.array([lx0 + vx * 100.0, ly0 + vy * 100.0], dtype=np.float64)
            fitted_lines[i] = (p_start, p_end)
        return fitted_lines

    lines = _fit_outer_edges(quad_local)

    if any(line is None for line in lines) and m_id is not None and m_id in TRANS_TABLES:
        hx, hy, hsign, vx_t, vy_t, vsign = TRANS_TABLES[m_id]
        H_c2i = cv2.getPerspectiveTransform(_DST_60, quad_local)
        M_i2c = cv2.getPerspectiveTransform(quad_local, _DST_60)
        warped_g = cv2.warpPerspective(g_roi, M_i2c, (60, 60)).astype(np.float32)
        warped_s = cv2.warpPerspective(s_roi, M_i2c, (60, 60))
        wg_flat = np.sort(warped_g.ravel())
        min_inner_grad = max(
            6.0,
            0.12
            * float(
                wg_flat[(85 * (len(wg_flat) - 1)) // 100]
                - wg_flat[(15 * (len(wg_flat) - 1)) // 100]
            ),
        )

        def _eval_trans(
            profs: np.ndarray, s_max: np.ndarray, signs: np.ndarray
        ) -> tuple[np.ndarray, np.ndarray]:
            grads = 0.5 * (profs[:, 2:] - profs[:, :-2]) * signs[:, None]
            best_idx = np.argmax(grads, axis=1)
            best_val = grads[np.arange(len(signs)), best_idx]
            return best_idx, (s_max <= 75) & (best_val > min_inner_grad)

        h_x_idx = hx[:, None] + _OFFSETS_9[None, :]
        h_s_max = np.maximum(
            warped_s[hy[:, None] - 1, h_x_idx],
            np.maximum(warped_s[hy[:, None], h_x_idx], warped_s[hy[:, None] + 1, h_x_idx]),
        ).max(axis=1)
        h_best_idx, h_ok = _eval_trans(warped_g[hy[:, None], h_x_idx], h_s_max, hsign)

        v_y_idx = vy_t[:, None] + _OFFSETS_9[None, :]
        v_s_max = np.maximum(
            warped_s[v_y_idx, vx_t[:, None] - 1],
            np.maximum(warped_s[v_y_idx, vx_t[:, None]], warped_s[v_y_idx, vx_t[:, None] + 1]),
        ).max(axis=1)
        v_best_idx, v_ok = _eval_trans(warped_g[v_y_idx, vx_t[:, None]], v_s_max, vsign)

        if int(np.count_nonzero(h_ok)) >= 5 and int(np.count_nonzero(v_ok)) >= 5:
            pu = _fit_robust_1d(
                hx[h_ok].astype(np.float32),
                hy[h_ok].astype(np.float32),
                (hx[h_ok] - 3 + h_best_idx[h_ok]).astype(np.float32),
            )
            pv = _fit_robust_1d(
                vx_t[v_ok].astype(np.float32),
                vy_t[v_ok].astype(np.float32),
                (vy_t[v_ok] - 3 + v_best_idx[v_ok]).astype(np.float32),
            )
            corr_corners = (
                np.column_stack([_CORNERS_AFFINE @ pu, _CORNERS_AFFINE @ pv])
                .reshape(1, 4, 2)
                .astype(np.float32)
            )
            cand = cv2.perspectiveTransform(corr_corners, H_c2i).reshape(4, 2)
            if np.max(np.linalg.norm(cand - quad_local, axis=1)) < 12.0 * max(1.0, res_scale):
                quad_local = cand.astype(np.float32)
                lines = _fit_outer_edges(quad_local)

    quad_d = quad_local.astype(np.float64)
    final_lines: list[tuple[np.ndarray, np.ndarray]] = [
        line_i if line_i is not None else (quad_d[i], quad_d[(i + 1) % 4])
        for i, line_i in enumerate(lines)
    ]
    refined_local = quad_local.copy()
    max_shift = 8.0 * max(1.0, res_scale)
    for i in range(4):
        l_prev, l_curr = final_lines[(i - 1) % 4], final_lines[i]
        inter = _intersect_lines(l_prev[0], l_prev[1], l_curr[0], l_curr[1])
        if inter is not None and np.linalg.norm(inter - quad_d[i]) < max_shift:
            refined_local[i] = inter.astype(np.float32)
    return refined_local + offset[None, :]


def _recover_occluded_markers_single_frame(
    gray: np.ndarray,
    hsv_sat: np.ndarray | None,
    found_corners: dict[int, np.ndarray],
    target_ids: tuple[int, ...] = TARGET_MARKER_IDS,
    frame: np.ndarray | None = None,
    pass_idx: int | None = None,
) -> None:
    candidates = _native_vision().recover(
        gray, hsv_sat, frame, found_corners, target_ids, GT_TEMPLATES,
        -1 if pass_idx is None else pass_idx,
    )
    res_scale = max(0.5, float(min(gray.shape)) / 720.0)
    for marker, quad, x0, y0, x1, y1 in candidates:
        local_g = gray[y0:y1, x0:x1]
        if hsv_sat is not None:
            local_s = hsv_sat[y0:y1, x0:x1]
        elif frame is not None:
            local_s = _bgr_to_sat(frame[y0:y1, x0:x1])
        else:
            local_s = np.zeros_like(local_g)
        offset = np.array([[x0, y0]], dtype=np.float32)
        refined = _refine_quad_edges_subpix(
            local_g, local_s, np.asarray(quad, dtype=np.float32) - offset,
            m_id=marker, res_scale=res_scale,
        )
        found_corners[marker] = (refined + offset).reshape(1, 4, 2).astype(np.float32)


class VisionPoseEstimator:
    """面向实时视频流与单帧图像的 6DoF 视觉位姿估计器."""

    def __init__(
        self,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        marker_length_m: float = MARKER_LENGTH_M,
        target_ids: tuple[int, ...] = TARGET_MARKER_IDS,
        max_coast_frames: int = 30,
        *,
        allow_recovery: bool = True,
        standard_scales: tuple[float, ...] = (1.0,),
    ):
        self.camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1, 1)
        self.marker_length_m = float(marker_length_m)
        self.target_ids = tuple(target_ids)
        self.max_coast_frames = max_coast_frames
        self.allow_recovery = allow_recovery
        self.standard_scales = standard_scales
        self.color_preprocessor: Callable[
            [np.ndarray], tuple[np.ndarray, np.ndarray]
        ] | None = None

        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
        params = (
            cv2.aruco.DetectorParameters_create()
            if hasattr(cv2.aruco, "DetectorParameters_create")
            else cv2.aruco.DetectorParameters()
        )
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        if hasattr(cv2.aruco, "ArucoDetector"):
            self._detect_standard = cv2.aruco.ArucoDetector(aruco_dict, params).detectMarkers
        else:
            self._detect_standard = lambda gray: cv2.aruco.detectMarkers(
                gray, aruco_dict, parameters=params
            )

        coords = np.linspace(6.0, 54.0, 7, dtype=np.float32)
        gx, gy = np.meshgrid(coords, coords)
        self.canonical_grid = np.stack([gx.flatten(), gy.flatten()], axis=-1).reshape(-1, 1, 2)
        self.canonical_corners = _DST_60.reshape(4, 1, 2)
        self.state: dict[int, dict[str, Any]] = {}
        self.last_valid_normals: dict[int, np.ndarray] = {}
        self.missing_streak: int = 0
        self.frame_count: int = 0

    def _try_fast_roi_reacquire(
        self,
        frame: np.ndarray,
        gray: np.ndarray,
        found_corners: dict[int, np.ndarray],
        hsv_sat: np.ndarray | None = None,
        target_ids: tuple[int, ...] | None = None,
    ):
        if frame.ndim == 2:
            return
        h_img, w_img = gray.shape[:2]
        res_scale = max(0.5, float(min(h_img, w_img)) / 720.0)

        for m_id in (self.target_ids if target_ids is None else target_ids):
            if m_id in found_corners or m_id not in self.state:
                continue
            st = self.state[m_id]
            if st["coast"] >= self.max_coast_frames or st["gray"].shape != gray.shape:
                continue

            prev_q = st["corners"]
            cx, cy = float(prev_q[:, 0].mean()), float(prev_q[:, 1].mean())
            span = float(max(np.ptp(prev_q[:, 0]), np.ptp(prev_q[:, 1]), 50.0 * res_scale))
            half_roi = int(max(95.0 * res_scale, span * 1.4))
            rx0 = max(0, int(round(cx)) - half_roi)
            ry0 = max(0, int(round(cy)) - half_roi)
            rx1 = min(w_img, int(round(cx)) + half_roi)
            ry1 = min(h_img, int(round(cy)) + half_roi)
            if rx1 - rx0 < 40 or ry1 - ry0 < 40:
                st["coast"] += 1
                continue

            roi_gray = gray[ry0:ry1, rx0:rx1]
            roi_sat = (hsv_sat[ry0:ry1, rx0:rx1] if hsv_sat is not None
                       else _bgr_to_sat(frame[ry0:ry1, rx0:rx1]))
            offset_xy = np.array([[rx0, ry0]], dtype=np.float32)
            q_local = prev_q - offset_xy

            if st["coast"] <= 2:
                q_cand = _refine_quad_edges_subpix(
                    roi_gray, roi_sat, q_local, m_id=m_id, res_scale=res_scale
                )
                verified = _verify_and_decode_occluded_quad(
                    roi_gray, roi_sat, q_cand, [m_id], require_rot0=True
                )
                if verified is not None:
                    _, aligned_local, _ = verified
                    aligned_full = (aligned_local + offset_xy).astype(np.float32)
                    found_corners[m_id] = aligned_full.reshape(1, 4, 2)
                    st["gray"] = gray
                    st["corners"] = aligned_full
                    st["coast"] = 0
                    continue

            prev_roi_gray = st["gray"][ry0:ry1, rx0:rx1]
            H_prev = cv2.getPerspectiveTransform(self.canonical_corners.reshape(4, 2), q_local)
            prev_grid_pts = cv2.perspectiveTransform(self.canonical_grid, H_prev)
            win_sz = max(21, int(round(21.0 * res_scale)) | 1)
            lk_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 18, 0.03)
            next_pts, status1, _ = cv2.calcOpticalFlowPyrLK(
                prev_roi_gray,
                roi_gray,
                prev_grid_pts,
                cast(Any, None),
                winSize=(win_sz, win_sz),
                maxLevel=3,
                criteria=lk_crit,
            )
            if next_pts is None or status1 is None:
                st["coast"] += 8
                continue
            back_pts, status2, _ = cv2.calcOpticalFlowPyrLK(
                roi_gray,
                prev_roi_gray,
                next_pts,
                cast(Any, None),
                winSize=(win_sz, win_sz),
                maxLevel=3,
                criteria=lk_crit,
            )
            if back_pts is None or status2 is None:
                st["coast"] += 8
                continue

            fb_err = np.linalg.norm((prev_grid_pts - back_pts).reshape(-1, 2), axis=1)
            good_mask = (
                (status1.flatten() == 1) & (status2.flatten() == 1) & (fb_err < 1.8 * res_scale)
            )

            rh, rw = roi_gray.shape[:2]
            pts_2d = next_pts.reshape(-1, 2)
            px = np.round(pts_2d[:, 0]).astype(np.int32)
            py = np.round(pts_2d[:, 1]).astype(np.int32)
            in_b = (px >= 2) & (px < rw - 2) & (py >= 2) & (py < rh - 2)
            good_mask &= in_b
            good_mask[in_b] &= roi_sat[py[in_b], px[in_b]] <= 145

            if int(np.count_nonzero(good_mask)) < 8:
                st["coast"] += 10
                continue

            src_good = self.canonical_grid.reshape(-1, 2)[good_mask]
            dst_good = pts_2d[good_mask]
            H_curr, _ = cv2.findHomography(src_good, dst_good, cv2.RANSAC, 2.5 * res_scale)
            if H_curr is None:
                st["coast"] += 5
                continue

            pred_local = cv2.perspectiveTransform(self.canonical_corners, H_curr).reshape(4, 2)
            verified = _verify_and_decode_occluded_quad(
                roi_gray,
                roi_sat,
                pred_local,
                [m_id],
                min_valid_cells=10,
                min_inner_cells=3,
                min_total_acc=0.82,
                min_inner_acc=0.75,
                require_rot0=True,
            )
            if verified is not None:
                _, aligned_local, _ = verified
                refined_local = _refine_quad_edges_subpix(
                    roi_gray, roi_sat, aligned_local, m_id=m_id, res_scale=res_scale
                )
                refined_full = (refined_local + offset_xy).astype(np.float32)
                found_corners[m_id] = refined_full.reshape(1, 4, 2)
                st["gray"] = gray
                st["corners"] = refined_full
                st["coast"] += 1
            else:
                st["coast"] += 5

    def _ingest_standard_detections(
        self,
        gray_full: np.ndarray,
        gray_patch: np.ndarray,
        sat_patch: np.ndarray | None,
        frame_patch: np.ndarray,
        corners: tuple[np.ndarray, ...] | list[np.ndarray],
        ids: np.ndarray | None,
        rejected: tuple[np.ndarray, ...] | list[np.ndarray],
        found_corners: dict[int, np.ndarray],
        target_set: set[int],
        req_tuple: tuple[int, ...],
        res_scale: float,
        *,
        scale: float = 1.0,
        offset: np.ndarray | None = None,
    ) -> None:
        if ids is not None:
            for corner, marker in zip(corners, ids):
                marker_id = int(np.asarray(marker).item())
                if marker_id not in target_set or marker_id in found_corners:
                    continue
                c_full = corner.reshape(4, 2).astype(np.float32) / scale
                if offset is not None:
                    c_full = c_full + offset
                c_ref = _refine_quad_edges_subpix(
                    gray_full, None, c_full, m_id=marker_id, res_scale=res_scale
                ).reshape(1, 4, 2)
                found_corners[marker_id] = c_ref
                self.state[marker_id] = {
                    "gray": gray_full,
                    "corners": c_ref.reshape(4, 2).astype(np.float32),
                    "coast": 0,
                }
        still_missing = [m for m in req_tuple if m not in found_corners]
        if not still_missing or not rejected:
            return
        sat_ref = (
            sat_patch
            if sat_patch is not None
            else (_bgr_to_sat(frame_patch) if frame_patch.ndim == 3 else np.zeros_like(gray_patch))
        )
        for rej_quad in rejected:
            q_patch = rej_quad.reshape(4, 2).astype(np.float32) / scale
            sides_q = [float(np.linalg.norm(q_patch[(i + 1) % 4] - q_patch[i])) for i in range(4)]
            if min(sides_q) < 28.0 * res_scale or max(sides_q) > 260.0 * res_scale:
                continue
            c_q = q_patch.mean(axis=0, keepdims=True)
            for shrink in (30.0 / 35.0, 1.0):
                q_in = c_q + (q_patch - c_q) * shrink
                dec = _verify_and_decode_occluded_quad(gray_patch, sat_ref, q_in, still_missing)
                if dec is not None:
                    m_id, aligned_q, _ = dec
                    q_full = aligned_q + offset if offset is not None else aligned_q
                    c_ref = _refine_quad_edges_subpix(
                        gray_full, None, q_full, m_id=m_id, res_scale=res_scale
                    ).reshape(1, 4, 2)
                    found_corners[m_id] = c_ref
                    self.state[m_id] = {
                        "gray": gray_full,
                        "corners": c_ref.reshape(4, 2).astype(np.float32),
                        "coast": 0,
                    }
                    still_missing = [m for m in req_tuple if m not in found_corners]
                    break
            if not still_missing:
                break

    def detect_markers(
        self,
        frame: np.ndarray,
        *,
        required_ids: tuple[int, ...] | None = None,
    ) -> tuple[tuple[np.ndarray, ...], np.ndarray | None]:
        hsv_sat = None
        if frame.ndim == 3 and self.color_preprocessor is not None:
            gray, hsv_sat = self.color_preprocessor(frame)
        else:
            gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        target_set = set(self.target_ids)
        req_tuple = self.target_ids if required_ids is None else tuple(required_ids)
        req_set = set(req_tuple)
        found_corners: dict[int, np.ndarray] = {}
        self.frame_count += 1

        h_img, w_img = gray.shape[:2]
        res_scale = max(0.5, float(min(h_img, w_img)) / 720.0)

        # 快速路径：当所需标记在最近帧中持续跟踪时，优先在局部 ROI 内执行检测与重捕获
        active_tracked = [
            self.state[m_id]["corners"]
            for m_id in req_tuple
            if m_id in self.state
            and self.state[m_id]["coast"] <= 4
            and self.state[m_id]["gray"].shape == gray.shape
        ]
        if active_tracked and len(active_tracked) == len(req_set):
            all_pts = np.vstack(active_tracked)
            pad = int(round(130.0 * res_scale))
            rx0 = max(0, int(np.floor(all_pts[:, 0].min())) - pad)
            ry0 = max(0, int(np.floor(all_pts[:, 1].min())) - pad)
            rx1 = min(w_img, int(np.ceil(all_pts[:, 0].max())) + pad)
            ry1 = min(h_img, int(np.ceil(all_pts[:, 1].max())) + pad)
            if (rx1 - rx0) * (ry1 - ry0) < 0.65 * (w_img * h_img):
                gray_roi = gray[ry0:ry1, rx0:rx1]
                roi_corners, roi_ids, roi_rejected = self._detect_standard(gray_roi)
                self._ingest_standard_detections(
                    gray,
                    gray_roi,
                    hsv_sat[ry0:ry1, rx0:rx1] if hsv_sat is not None else None,
                    frame[ry0:ry1, rx0:rx1] if frame.ndim == 3 else gray_roi,
                    roi_corners,
                    roi_ids,
                    roi_rejected,
                    found_corners,
                    target_set,
                    req_tuple,
                    res_scale,
                    offset=np.array([[rx0, ry0]], dtype=np.float32),
                )
                if (
                    not req_set.issubset(found_corners.keys())
                    and self.allow_recovery
                    and frame.ndim == 3
                ):
                    self._try_fast_roi_reacquire(
                        frame, gray, found_corners, hsv_sat, target_ids=req_tuple
                    )

        if not req_set.issubset(found_corners.keys()):
            for scale in self.standard_scales:
                sample = (
                    gray if scale == 1.0
                    else cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                )
                corners, ids, rejected = self._detect_standard(sample)
                self._ingest_standard_detections(
                    gray,
                    gray,
                    hsv_sat,
                    frame,
                    corners,
                    ids,
                    rejected,
                    found_corners,
                    target_set,
                    req_tuple,
                    res_scale,
                    scale=scale,
                )
                if req_set.issubset(found_corners.keys()):
                    break

        if self.allow_recovery and frame.ndim == 3 and not req_set.issubset(found_corners.keys()):
            self._try_fast_roi_reacquire(frame, gray, found_corners, hsv_sat, target_ids=req_tuple)
            if req_set.issubset(found_corners.keys()):
                self.missing_streak = 0
                sorted_ids = sorted(found_corners.keys())
                return (
                    tuple(found_corners[i] for i in sorted_ids),
                    np.array([[i] for i in sorted_ids], dtype=np.int32),
                )
        stage0_ids = set(found_corners.keys())

        pass_idx = None if self.missing_streak < 2 else self.frame_count
        if self.allow_recovery and not req_set.issubset(found_corners.keys()):
            _recover_occluded_markers_single_frame(
                gray, hsv_sat, found_corners, req_tuple, frame=frame, pass_idx=pass_idx
            )

        if req_set.issubset(found_corners.keys()):
            self.missing_streak = 0
        else:
            self.missing_streak += 1

        for k, v in found_corners.items():
            if k not in stage0_ids:
                self.state[k] = {
                    "gray": gray,
                    "corners": v.reshape(4, 2).astype(np.float32),
                    "coast": 0,
                }
        for m_id, st in list(self.state.items()):
            if m_id not in found_corners:
                st["coast"] += 1

        if not found_corners:
            return (), None

        sorted_ids = sorted(found_corners.keys())
        return (
            tuple(found_corners[i] for i in sorted_ids),
            np.array([[i] for i in sorted_ids], dtype=np.int32),
        )

    def estimate_poses(
        self,
        frame: np.ndarray,
        draw: bool = False,
        marker_length_m: float | None = None,
        *,
        required_ids: tuple[int, ...] | None = None,
    ) -> tuple[np.ndarray | None, dict[int, tuple[np.ndarray, np.ndarray]]]:
        """检测并解算 6DoF 位姿。当 draw=False 时不复制图像、不执行绘图."""
        m_len = self.marker_length_m if marker_length_m is None else float(marker_length_m)
        corners, ids = (
            self.detect_markers(frame)
            if required_ids is None
            else self.detect_markers(frame, required_ids=required_ids)
        )
        vis_frame = frame.copy() if draw else None
        poses: dict[int, tuple[np.ndarray, np.ndarray]] = {}

        if ids is not None and len(ids) > 0:
            if vis_frame is not None:
                cv2.aruco.drawDetectedMarkers(vis_frame, corners, ids)
            obj_pts = get_marker_object_points(m_len)

            for corner, m_id_arr in zip(corners, ids):
                m_id = int(np.asarray(m_id_arr).item())
                img_pts = corner.reshape(4, 2).astype(np.float64)
                if not np.all(np.isfinite(img_pts)):
                    self.state.pop(m_id, None)
                    continue
                q_f32 = img_pts.astype(np.float32).reshape(4, 1, 2)
                if not cv2.isContourConvex(q_f32) or abs(cv2.contourArea(q_f32)) < 36.0:
                    self.state.pop(m_id, None)
                    continue
                ret, rvecs, tvecs, reproj_errs = cv2.solvePnPGeneric(
                    obj_pts,
                    img_pts,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
                if not ret or len(rvecs) == 0:
                    self.state.pop(m_id, None)
                    continue

                rvec, tvec = rvecs[0], tvecs[0]
                R0, _ = cv2.Rodrigues(rvec)
                n0 = R0[:, 2]

                if len(rvecs) > 1 and reproj_errs is not None and len(reproj_errs) > 1:
                    err0 = float(np.asarray(reproj_errs[0]).item())
                    err1 = float(np.asarray(reproj_errs[1]).item())
                    R1, _ = cv2.Rodrigues(rvecs[1])
                    n1 = R1[:, 2]
                    if err1 < err0 * 0.65 and n1[2] < -0.15:
                        rvec, tvec, n0 = rvecs[1], tvecs[1], n1
                    elif m_id in self.last_valid_normals and err1 < err0 * 1.45 and n1[2] < -0.15:
                        prev_n = self.last_valid_normals[m_id]
                        if float(np.dot(n1, prev_n)) > float(np.dot(n0, prev_n)) + 0.12:
                            rvec, tvec, n0 = rvecs[1], tvecs[1], n1

                if not (np.all(np.isfinite(rvec)) and np.all(np.isfinite(tvec))):
                    self.state.pop(m_id, None)
                    continue
                tz = float(tvec.reshape(3)[2])
                if not (0.06 <= tz <= 3.5) or float(np.linalg.norm(tvec)) > 4.0 or n0[2] >= -0.05:
                    self.state.pop(m_id, None)
                    continue
                proj_pts, _ = cv2.projectPoints(
                    obj_pts, rvec, tvec, self.camera_matrix, self.dist_coeffs
                )
                proj_pts = proj_pts.reshape(4, 2)
                side_px = float(
                    np.mean(np.linalg.norm(np.roll(img_pts, -1, axis=0) - img_pts, axis=1))
                )
                if (
                    not np.all(np.isfinite(proj_pts))
                    or float(np.max(np.linalg.norm(proj_pts - img_pts, axis=1)))
                    > max(8.0, 0.35 * side_px)
                ):
                    self.state.pop(m_id, None)
                    continue

                if n0[2] < -0.1:
                    self.last_valid_normals[m_id] = n0.copy()

                poses[m_id] = (rvec, tvec)

                if vis_frame is not None:
                    cv2.drawFrameAxes(
                        vis_frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, m_len * 0.75
                    )
                    tx, ty, tz = tvec.flatten() * 100.0
                    dist_cm = float(np.linalg.norm([tx, ty, tz]))
                    c_pt = img_pts[0].astype(int)
                    label = f"ID:{m_id} ({tx:.1f},{ty:.1f},{tz:.1f})cm d={dist_cm:.1f}cm"
                    cv2.putText(
                        vis_frame,
                        label,
                        (c_pt[0], max(25, c_pt[1] - 12)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )

        return vis_frame, poses


# ============================================================================
# 4. 异步相机读取器 & 机械臂插孔策略跟踪器
# ============================================================================


class AsyncCameraReader:
    """后台非阻塞相机读取器，始终缓存最新一帧图像."""

    def __init__(
        self,
        camera_index: int = 6,
        width: int = 1280,
        height: int = 720,
        fps: int = 30,
        camera_name: str = "Azure Kinect",
        decode_rate_hz: float | None = None,
        pixel_format: str = "MJPG",
    ):
        if decode_rate_hz is not None and (
            not np.isfinite(decode_rate_hz) or decode_rate_hz <= 0.0
        ):
            raise ValueError("decode_rate_hz must be finite and positive")
        if pixel_format not in ("MJPG", "YUYV"):
            raise ValueError("pixel_format must be MJPG or YUYV")
        self._decode_period_s = (
            1.0 / decode_rate_hz
            if decode_rate_hz is not None and decode_rate_hz < fps else 0.0
        )
        self._requested_index = camera_index
        self.width = width
        self.height = height
        self.fps = fps
        self.camera_name = camera_name
        self.pixel_format = pixel_format
        self.cap = None
        self.running = False
        self.lock = threading.Lock()
        self.latest_frame: np.ndarray | None = None
        self.frame_seq = 0
        self.frame_stamp_s = 0.0
        self.camera_index = resolve_camera_index(camera_index, camera_name)

        cap, frame = self._open_and_configure(self.camera_index)
        if cap is not None and frame is not None:
            self.cap = cap
            self.running = True
            self.frame_stamp_s = time.monotonic()
            self.latest_frame = frame
            self.frame_seq = 1
            self.thread = threading.Thread(target=self._loop, daemon=True)
            self.thread.start()

    def _open_and_configure(
        self, index: int
    ) -> tuple[cv2.VideoCapture | None, np.ndarray | None]:
        backends = (
            (cv2.CAP_DSHOW, cv2.CAP_ANY)
            if os.name == "nt"
            else (cv2.CAP_V4L2, cv2.CAP_ANY)
        )
        cap = None
        for api_pref in backends:
            candidate = cv2.VideoCapture(index, api_pref)
            if candidate.isOpened():
                cap = candidate
                break
        if cap is None:
            return None, None
        try:
            fourcc = cv2.VideoWriter_fourcc(*self.pixel_format)
            for prop, value in (
                (cv2.CAP_PROP_FOURCC, fourcc),
                (cv2.CAP_PROP_FRAME_WIDTH, self.width),
                (cv2.CAP_PROP_FRAME_HEIGHT, self.height),
                (cv2.CAP_PROP_FPS, self.fps),
            ):
                if not cap.set(prop, value):
                    raise RuntimeError(f"Camera rejected property {prop}={value}")
            actual = tuple(cap.get(prop) for prop in (
                cv2.CAP_PROP_FOURCC, cv2.CAP_PROP_FRAME_WIDTH,
                cv2.CAP_PROP_FRAME_HEIGHT, cv2.CAP_PROP_FPS,
            ))
            if actual[:3] != (fourcc, self.width, self.height) or not np.isclose(
                actual[3], self.fps, rtol=0.0, atol=0.5
            ):
                raise RuntimeError(
                    f"Camera did not negotiate {self.pixel_format} "
                    f"{self.width}x{self.height}@{self.fps}: {actual}"
                )
            # Keep one YUYV buffer queued for DMA while the other is converted.
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 2 if self.pixel_format == "YUYV" else 1)
            ret, frame = cap.read()
            if not ret or frame is None or frame.shape != (self.height, self.width, 3):
                raise RuntimeError("Camera did not return the configured BGR frame")
            return cap, frame
        except (RuntimeError, ValueError, cv2.error):
            cap.release()
            raise

    def _try_reconnect(self) -> bool:
        if self.cap is not None:
            try:
                self.cap.release()
            except cv2.error:
                pass
            self.cap = None
        self.camera_index = resolve_camera_index(self._requested_index, self.camera_name)
        try:
            cap, frame = self._open_and_configure(self.camera_index)
        except (RuntimeError, ValueError, cv2.error):
            return False
        if cap is None or frame is None:
            return False
        stamp = time.monotonic()
        self.cap = cap
        with self.lock:
            self.latest_frame = frame
            self.frame_seq += 1
            self.frame_stamp_s = stamp
        return True

    def _loop(self):
        next_decode = self.frame_stamp_s + self._decode_period_s
        failures = 0
        while self.running:
            if self.cap is None or not self.cap.grab():
                if not self.running:
                    break
                failures += 1
                if failures >= 30:
                    if self._try_reconnect():
                        failures = 0
                        next_decode = self.frame_stamp_s + self._decode_period_s
                    else:
                        time.sleep(0.2)
                else:
                    time.sleep(0.002)
                continue
            failures = 0
            grabbed = time.monotonic()
            if self._decode_period_s:
                if grabbed < next_decode:
                    continue
                next_decode += self._decode_period_s * (
                    int((grabbed - next_decode) / self._decode_period_s) + 1
                )
            ret, frame = self.cap.retrieve()
            if ret and frame is not None:
                stamp = time.monotonic()
                with self.lock:
                    self.latest_frame = frame
                    self.frame_seq += 1
                    self.frame_stamp_s = stamp
            else:
                time.sleep(0.002)

    def read_latest_timed(self) -> tuple[bool, np.ndarray | None, int, float]:
        """Timestamp is camera-read completion, not hardware exposure time."""
        with self.lock:
            if self.latest_frame is None:
                return False, None, 0, 0.0
            return True, self.latest_frame, self.frame_seq, self.frame_stamp_s

    def release(self):
        self.running = False
        if hasattr(self, "thread") and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        if self.cap is not None:
            self.cap.release()


class PlugHoleTracker:
    """面向机械臂插入策略的高层跟踪器，管理相机配置、孔位表与位姿估计器."""

    def __init__(
        self,
        pos_json: str = DEFAULT_POS_JSON,
        camera_config: str = DEFAULT_CAMERA_CONFIG,
        target_hole: str | None = None,
        marker_length_m: float = MARKER_LENGTH_M,
        open_camera: bool = False,
        camera_index: int | None = None,
        camera_decode_rate_hz: float | None = None,
    ):
        self.cam_cfg = load_camera_config(camera_config)
        self.holes_mm, self.plug_mm = load_pos_config(pos_json)
        self.target_hole = target_hole
        self.camera_decode_rate_hz = camera_decode_rate_hz

        self.estimator = VisionPoseEstimator(
            self.cam_cfg["camera_matrix"],
            self.cam_cfg["dist_coeffs"],
            marker_length_m=marker_length_m,
        )
        self.camera_reader: AsyncCameraReader | None = None
        if open_camera:
            self.start_camera(camera_index)

    def start_camera(self, camera_index: int | None = None) -> bool:
        idx = (
            self.cam_cfg["camera_index"]
            if camera_index is None or camera_index < 0
            else camera_index
        )
        self.camera_reader = AsyncCameraReader(
            idx,
            width=self.cam_cfg["width"],
            height=self.cam_cfg["height"],
            fps=self.cam_cfg["fps"],
            camera_name=self.cam_cfg.get("camera_name", "Azure Kinect"),
            decode_rate_hz=self.camera_decode_rate_hz,
            pixel_format=self.cam_cfg["pixel_format"],
        )
        return self.camera_reader.running

    def stop_camera(self):
        if self.camera_reader is not None:
            self.camera_reader.release()
            self.camera_reader = None
