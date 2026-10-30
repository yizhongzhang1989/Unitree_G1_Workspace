"""基于 ArUco 标定码 (ID 0: 孔板, ID 1: 插头) 与 pos.json 的插孔/插头点位可视化标注模块."""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from upper_insert.vision.vision_locator import (
    MARKER_LENGTH_M,
    compute_plug_hole_delta,
    get_marker_object_points,
)


def _draw_text_with_bg(
    img: np.ndarray,
    text: str,
    org: tuple[int, int],
    font_scale: float = 0.42,
    text_color: tuple[int, int, int] = (255, 255, 255),
    bg_color: tuple[int, int, int] = (20, 20, 20),
    thickness: int = 1,
):
    """绘制带半透明深色底框的清晰文字标签."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    x, y = int(org[0]), int(org[1])
    h_img, w_img = img.shape[:2]
    x = max(2, min(w_img - tw - 4, x))
    y = max(th + 4, min(h_img - baseline - 4, y))

    x0, y0 = x - 2, y - th - 2
    x1, y1 = x + tw + 2, y + baseline + 2
    roi = img[y0:y1, x0:x1]
    if roi.size > 0:
        bg = np.full_like(roi, bg_color, dtype=np.uint8)
        cv2.addWeighted(bg, 0.65, roi, 0.35, 0, roi)
    cv2.putText(img, text, (x, y), font, font_scale, text_color, thickness, cv2.LINE_AA)
    return (x0, y0, x1, y1)


def _draw_topdown_minimap(
    vis: np.ndarray,
    holes_mm: dict[str, np.ndarray],
    plug_in_id0_mm: np.ndarray | None,
    nearest_hole: str | None,
    orientation_ok: bool | None,
):
    """在画面右上角绘制 ID 0 孔板坐标系 (俯视图: 横轴 +Y 向右, 纵轴 +X 向下) 的 2D 雷达分布图."""
    h_img, w_img = vis.shape[:2]
    panel_w, panel_h = 280, 280
    x0 = w_img - panel_w - 14
    y0 = 14
    x1, y1 = x0 + panel_w, y0 + panel_h
    if x0 < 0 or y1 >= h_img:
        return

    roi = vis[y0:y1, x0:x1]
    bg = np.full_like(roi, (22, 26, 22), dtype=np.uint8)
    cv2.addWeighted(bg, 0.82, roi, 0.18, 0, roi)
    cv2.rectangle(vis, (x0, y0), (x1, y1), (90, 140, 90), 1, cv2.LINE_AA)

    cv2.putText(
        vis,
        "ID0 Top-Down Map (+Y right, +X down)",
        (x0 + 10, y0 + 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (180, 240, 180),
        1,
        cv2.LINE_AA,
    )

    cx = x0 + panel_w // 2
    cy = y0 + panel_h // 2 + 10
    scale = 2.6

    def mm_to_px(x_mm: float, y_mm: float) -> tuple[int, int]:
        u = int(round(cx + y_mm * scale))
        v = int(round(cy + x_mm * scale))
        return u, v

    m_half_outer = 17.5
    m_half_inner = 15.0
    cv2.rectangle(
        vis,
        mm_to_px(-m_half_outer, -m_half_outer),
        mm_to_px(m_half_outer, m_half_outer),
        (60, 65, 60),
        1,
        cv2.LINE_AA,
    )
    cv2.rectangle(
        vis,
        mm_to_px(-m_half_inner, -m_half_inner),
        mm_to_px(m_half_inner, m_half_inner),
        (85, 85, 85),
        1,
        cv2.LINE_AA,
    )
    cv2.line(vis, (cx - 115, cy), (cx + 115, cy), (50, 60, 50), 1, cv2.LINE_AA)
    cv2.line(vis, (cx, cy - 110), (cx, cy + 110), (50, 60, 50), 1, cv2.LINE_AA)
    cv2.arrowedLine(vis, (cx, cy), (cx, cy + 42), (0, 140, 255), 2, cv2.LINE_AA, tipLength=0.22)
    cv2.arrowedLine(vis, (cx, cy), (cx + 42, cy), (0, 220, 0), 2, cv2.LINE_AA, tipLength=0.22)
    cv2.putText(
        vis, "+X", (cx + 4, cy + 44), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 160, 255), 1, cv2.LINE_AA
    )
    cv2.putText(
        vis, "+Y", (cx + 44, cy - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 220, 0), 1, cv2.LINE_AA
    )

    for name, h_mm in holes_mm.items():
        hu, hv = mm_to_px(float(h_mm[0]), float(h_mm[1]))
        is_near = name == nearest_hole
        col = (0, 255, 0) if is_near else (255, 220, 0)
        rad = 6 if is_near else 4
        cv2.circle(vis, (hu, hv), rad, col, -1 if not is_near else 2, cv2.LINE_AA)
        dy_lbl = -6 if "tight" in name else (10 if "loose" in name else -4)
        dx_lbl = 6 if h_mm[1] >= 0 else -max(28, len(name) * 5)
        cv2.putText(
            vis,
            name,
            (hu + dx_lbl, hv + dy_lbl),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.31,
            (120, 255, 140) if is_near else (210, 220, 220),
            1,
            cv2.LINE_AA,
        )

    if plug_in_id0_mm is not None:
        pu, pv = mm_to_px(float(plug_in_id0_mm[0]), float(plug_in_id0_mm[1]))
        pu = max(x0 + 8, min(x1 - 8, pu))
        pv = max(y0 + 28, min(y1 - 8, pv))
        p_col = (0, 255, 180) if orientation_ok else (255, 80, 255)
        cv2.circle(vis, (pu, pv), 7, p_col, 2, cv2.LINE_AA)
        cv2.drawMarker(vis, (pu, pv), (255, 0, 255), cv2.MARKER_CROSS, 14, 2, cv2.LINE_AA)
        cv2.putText(
            vis,
            "PLUG",
            (pu + 7, pv + 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (255, 140, 255),
            1,
            cv2.LINE_AA,
        )


def _valid_uv(uv: np.ndarray, w_img: int, h_img: int, margin: float = 2.0) -> bool:
    arr = np.asarray(uv, dtype=np.float64)
    if not np.all(np.isfinite(arr)):
        return False
    pts = arr.reshape(-1, 2)
    lim_x = max(w_img, 1) * margin
    lim_y = max(h_img, 1) * margin
    return bool(
        np.all(pts[:, 0] >= -lim_x)
        and np.all(pts[:, 0] <= w_img + lim_x)
        and np.all(pts[:, 1] >= -lim_y)
        and np.all(pts[:, 1] <= h_img + lim_y)
    )


def annotate_plug_and_holes(
    frame: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    holes_mm: dict[str, np.ndarray],
    plug_mm: np.ndarray,
    marker_length: float = MARKER_LENGTH_M,
    angle_thresh_deg: float = 25.0,
    target_hole: str | None = None,
    insert_depth_mm: float | None = None,
    estimator: Any = None,
    *,
    poses: dict[int, tuple[np.ndarray, np.ndarray]] | None = None,
) -> tuple[np.ndarray, dict[str, Any]]:
    """将 pos.json 中的所有插孔点位、插头中心点位、方向校验与策略 delta 输入标注在图像上."""
    h_img, w_img = frame.shape[:2]
    if poses is not None:
        vis = frame.copy()
        obj_pts = get_marker_object_points(marker_length)
        axis_chk = np.vstack([obj_pts, np.eye(3, dtype=np.float64) * (marker_length * 0.75)])
        for marker_id, (rvec, tvec) in poses.items():
            if not (np.all(np.isfinite(rvec)) and np.all(np.isfinite(tvec))):
                continue
            if float(np.asarray(tvec).reshape(3)[2]) <= 0.05:
                continue
            chk_uv, _ = cv2.projectPoints(axis_chk, rvec, tvec, camera_matrix, dist_coeffs)
            if not _valid_uv(chk_uv, w_img, h_img):
                continue
            corners = chk_uv[:4]
            cv2.aruco.drawDetectedMarkers(
                vis, [corners.reshape(1, 4, 2).astype(np.float32)],
                np.array([[marker_id]], dtype=np.int32),
            )
            cv2.drawFrameAxes(vis, camera_matrix, dist_coeffs, rvec, tvec, marker_length * 0.75)
    elif estimator is not None:
        vis, poses = estimator.estimate_poses(frame, draw=True, marker_length_m=marker_length)
        assert vis is not None
    else:
        vis, poses = frame.copy(), {}

    info: dict[str, Any] = {
        "poses": poses,
        "holes_cam_mm": {},
        "holes_uv": {},
        "plug_cam_mm": None,
        "plug_in_id0_mm": None,
        "plug_uv": None,
        "orientation_ok": None,
        "angle_deg": None,
        "axis_angle_deg": None,
        "cos_sim": None,
        "nearest_hole": None,
        "nearest_dist_mm": None,
        "delta_pos": None,
        "delta_quat": None,
        "policy_obs": None,
        "delta": None,
    }

    if 0 in poses:
        rvec0, tvec0 = poses[0]
        R0, _ = cv2.Rodrigues(rvec0)
        hole_names = list(holes_mm.keys())
        pts_local_m = np.array([holes_mm[k] * 0.001 for k in hole_names], dtype=np.float64)
        pts_cam_mm = (pts_local_m @ R0.T + tvec0.reshape(1, 3)) * 1000.0
        uv_pts, _ = cv2.projectPoints(pts_local_m, rvec0, tvec0, camera_matrix, dist_coeffs)
        uv_pts = uv_pts.reshape(-1, 2)

        for name, p_cam_mm, uv in zip(hole_names, pts_cam_mm, uv_pts):
            if p_cam_mm[2] > 50.0 and _valid_uv(uv, w_img, h_img):
                info["holes_cam_mm"][name] = p_cam_mm
                info["holes_uv"][name] = (float(uv[0]), float(uv[1]))

        axis_pts_m = np.array([[0.0, 0.0, 0.0], [0.045, 0.0, 0.0]], dtype=np.float64)
        axis_cam_z = (axis_pts_m @ R0.T + tvec0.reshape(1, 3))[:, 2]
        axis_uv, _ = cv2.projectPoints(axis_pts_m, rvec0, tvec0, camera_matrix, dist_coeffs)
        if np.all(axis_cam_z > 0.05) and _valid_uv(axis_uv, w_img, h_img):
            p0_uv = (int(round(float(axis_uv[0, 0, 0]))), int(round(float(axis_uv[0, 0, 1]))))
            px_uv = (int(round(float(axis_uv[1, 0, 0]))), int(round(float(axis_uv[1, 0, 1]))))
            cv2.arrowedLine(vis, p0_uv, px_uv, (0, 140, 255), 3, cv2.LINE_AA, tipLength=0.22)
            _draw_text_with_bg(
                vis, "ID0.+X", (px_uv[0] + 4, px_uv[1]), 0.45, (0, 200, 255), (20, 20, 20), 1
            )

    if 1 in poses:
        rvec1, tvec1 = poses[1]
        R1, _ = cv2.Rodrigues(rvec1)
        plug_local_m = (plug_mm * 0.001).reshape(1, 3).astype(np.float64)
        info["plug_cam_mm"] = (R1 @ plug_local_m.reshape(3, 1) + tvec1).flatten() * 1000.0

        plug_z_tip_m = plug_local_m.reshape(3) + np.array([0.0, 0.0, 0.035], dtype=np.float64)
        pts1_m = np.vstack(
            [np.zeros((1, 3), dtype=np.float64), plug_local_m, plug_z_tip_m.reshape(1, 3)]
        )
        pts1_cam_z = (pts1_m @ R1.T + tvec1.reshape(1, 3))[:, 2]
        uv1, _ = cv2.projectPoints(pts1_m, rvec1, tvec1, camera_matrix, dist_coeffs)
        uv1 = uv1.reshape(-1, 2)
        if np.all(pts1_cam_z > 0.05) and _valid_uv(uv1, w_img, h_img):
            origin1_uv = (int(round(float(uv1[0, 0]))), int(round(float(uv1[0, 1]))))
            plug_uv = (int(round(float(uv1[1, 0]))), int(round(float(uv1[1, 1]))))
            plug_z_uv = (int(round(float(uv1[2, 0]))), int(round(float(uv1[2, 1]))))
            info["plug_uv"] = (float(uv1[1, 0]), float(uv1[1, 1]))

            cv2.line(vis, origin1_uv, plug_uv, (255, 0, 255), 2, cv2.LINE_AA)
            cv2.arrowedLine(vis, plug_uv, plug_z_uv, (255, 180, 0), 3, cv2.LINE_AA, tipLength=0.25)
            _draw_text_with_bg(
                vis, "ID1.+Z", (plug_z_uv[0] + 4, plug_z_uv[1]), 0.45, (255, 220, 0), (20, 20, 20), 1
            )

    delta: dict[str, Any] | None = None
    if 0 in poses and 1 in poses:
        rvec0, tvec0 = poses[0]
        rvec1, tvec1 = poses[1]
        delta = compute_plug_hole_delta(
            rvec0,
            tvec0,
            rvec1,
            tvec1,
            holes_mm=holes_mm,
            plug_mm=plug_mm,
            target_hole=target_hole,
            angle_thresh_deg=angle_thresh_deg,
            insert_depth_mm=insert_depth_mm,
        )
        info.update(
            delta=delta, angle_deg=delta["yaw_angle_deg"],
            nearest_hole=delta["target_hole"], nearest_dist_mm=delta["distance_mm"],
            **{key: delta[key] for key in (
                "orientation_ok", "axis_angle_deg", "cos_sim", "plug_in_id0_mm",
                "delta_pos", "delta_quat", "policy_obs",
            )},
        )

    if info["holes_uv"]:
        nearest_name = info["nearest_hole"]
        all_u = [uv[0] for uv in info["holes_uv"].values()]
        all_v = [uv[1] for uv in info["holes_uv"].values()]
        min_u, max_u = min(all_u), max(all_u)
        mid_u = 0.5 * (min_u + max_u)
        mid_v = 0.5 * (min(all_v) + max(all_v))

        left_items = sorted(
            [(name, uv) for name, uv in info["holes_uv"].items() if uv[0] < mid_u],
            key=lambda kv: (kv[1][1], kv[0]),
        )
        right_items = sorted(
            [(name, uv) for name, uv in info["holes_uv"].items() if uv[0] >= mid_u],
            key=lambda kv: (kv[1][1], kv[0]),
        )

        step_y = 20
        for side, items in (("left", left_items), ("right", right_items)):
            n_items = len(items)
            if n_items == 0:
                continue
            start_y = int(round(mid_v - 0.5 * (n_items - 1) * step_y))
            for idx_item, (name, (u, v)) in enumerate(items):
                pt_x, pt_y = int(round(u)), int(round(v))
                is_nearest = (name == nearest_name) and (info["nearest_dist_mm"] is not None)
                text_color = (0, 255, 120) if is_nearest else (235, 245, 255)
                bg_color = (0, 75, 0) if is_nearest else (25, 25, 25)
                line_color = (0, 255, 100) if is_nearest else (160, 200, 220)

                label_y = start_y + idx_item * step_y
                font = cv2.FONT_HERSHEY_SIMPLEX
                (tw, _), _ = cv2.getTextSize(name, font, 0.39, 1)
                if side == "left":
                    anchor_x = int(round(min_u)) - 28
                    label_x = anchor_x - tw - 4
                else:
                    anchor_x = int(round(max_u)) + 28
                    label_x = anchor_x + 4

                cv2.line(vis, (pt_x, pt_y), (anchor_x, label_y - 4), line_color, 1, cv2.LINE_AA)
                _draw_text_with_bg(vis, name, (label_x, label_y), 0.39, text_color, bg_color, 1)

        for name, (u, v) in info["holes_uv"].items():
            pt = (int(round(u)), int(round(v)))
            is_nearest = (name == nearest_name) and (info["nearest_dist_mm"] is not None)
            if is_nearest:
                cv2.circle(vis, pt, 7, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.circle(vis, pt, 3, (0, 255, 255), -1, cv2.LINE_AA)
            else:
                cv2.circle(vis, pt, 4, (255, 220, 0), -1, cv2.LINE_AA)
                cv2.circle(vis, pt, 5, (20, 20, 20), 1, cv2.LINE_AA)

    if info["plug_uv"] is not None:
        pu, pv = int(round(info["plug_uv"][0])), int(round(info["plug_uv"][1]))
        if info["nearest_hole"] and info["nearest_hole"] in info["holes_uv"]:
            hu, hv = info["holes_uv"][info["nearest_hole"]]
            cv2.line(vis, (pu, pv), (int(round(hu)), int(round(hv))), (0, 255, 180), 2, cv2.LINE_AA)

        cv2.circle(vis, (pu, pv), 7, (255, 0, 255), 2, cv2.LINE_AA)
        cv2.circle(vis, (pu, pv), 3, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.drawMarker(vis, (pu, pv), (255, 0, 255), cv2.MARKER_CROSS, 16, 1, cv2.LINE_AA)

        if info["plug_in_id0_mm"] is not None:
            px, py, pz = info["plug_in_id0_mm"]
            plug_label = f"PLUG ({px:.1f},{py:.1f},{pz:.1f})mm"
        else:
            px, py, pz = info["plug_cam_mm"]
            plug_label = f"PLUG cam({px:.0f},{py:.0f},{pz:.0f})mm"
        holes_max_v = (
            int(round(max(uv[1] for uv in info["holes_uv"].values()))) if info["holes_uv"] else pv
        )
        plug_lbl_y = max(pv + 46, holes_max_v + 44)
        plug_lbl_x = pu - 40
        cv2.line(vis, (pu, pv), (pu, plug_lbl_y - 14), (255, 80, 255), 1, cv2.LINE_AA)
        _draw_text_with_bg(
            vis, plug_label, (plug_lbl_x, plug_lbl_y), 0.44, (255, 160, 255), (45, 0, 45), 1
        )

    if 0 in poses:
        _draw_topdown_minimap(
            vis,
            holes_mm,
            info["plug_in_id0_mm"],
            info["nearest_hole"],
            info["orientation_ok"],
        )

    if delta is not None:
        is_ok = delta["orientation_ok"]
        ang = delta["yaw_angle_deg"]
        axis_ang = delta["axis_angle_deg"]
        status_str = "ALIGNED" if is_ok else "MISALIGNED"
        status_color = (0, 255, 0) if is_ok else (0, 60, 255)
        bg_col = (0, 55, 0) if is_ok else (0, 0, 80)
        _draw_text_with_bg(
            vis,
            f"Plug Dir: {status_str} | Axis(-Y1->-Z0)={axis_ang:.1f}deg | Yaw(+Z1->+X0)={ang:.1f}deg",
            (18, 34),
            0.60,
            status_color,
            bg_col,
            2,
        )

        px, py, pz = delta["plug_in_id0_mm"]
        nh = delta["target_hole"]
        nd = delta["distance_mm"]
        _draw_text_with_bg(
            vis,
            f"Plug in ID0: [{px:.1f}, {py:.1f}, {pz:.1f}] mm | Target Hole: {nh} (dist={nd:.1f} mm)",
            (18, 66),
            0.56,
            (0, 235, 255),
            (25, 25, 25),
            2,
        )

        dp = delta["delta_pos"]
        dq = delta["delta_quat"]
        _draw_text_with_bg(
            vis,
            f"Policy Delta -> dPos(m): [{dp[0]:+.4f}, {dp[1]:+.4f}, {dp[2]:+.4f}]"
            f" | dQuat(xyzw): [{dq[0]:+.3f}, {dq[1]:+.3f}, {dq[2]:+.3f}, {dq[3]:+.3f}]",
            (18, 98),
            0.50,
            (255, 210, 120),
            (25, 25, 25),
            1,
        )
    else:
        missing = [i for i in (0, 1) if i not in poses]
        _draw_text_with_bg(
            vis,
            f"Waiting for both markers to check alignment (missing ID: {missing})",
            (18, 34),
            0.60,
            (0, 200, 255),
            (25, 25, 25),
            2,
        )

    return vis, info
