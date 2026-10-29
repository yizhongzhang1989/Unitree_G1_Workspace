"""600 DPI 3.5cm ArUco 标定码与 A4 打印排版生成脚本."""

from __future__ import annotations

import os

import cv2
import numpy as np

from upper_insert.vision.vision_locator import draw_aruco_marker

EXACT_DPI = 600
TOTAL_CARD_CM = 3.5
BLACK_MARKER_CM = 3.0
QUIET_ZONE_CM = (TOTAL_CARD_CM - BLACK_MARKER_CM) / 2.0


def cm_to_px(cm: float, dpi: int = EXACT_DPI) -> int:
    return int(round(cm * dpi / 2.54))


def generate_single_marker(
    aruco_dict,
    marker_id: int,
    marker_size_cm: float = BLACK_MARKER_CM,
    quiet_zone_cm: float = QUIET_ZONE_CM,
    dpi: int = EXACT_DPI,
) -> np.ndarray:
    """生成单个带白边的 ArUco 标定码贴纸 BGR 图像."""
    total_cm = marker_size_cm + 2.0 * quiet_zone_cm
    total_px = cm_to_px(total_cm, dpi)
    marker_px = cm_to_px(marker_size_cm, dpi)
    border_px = (total_px - marker_px) // 2
    marker_px = total_px - 2 * border_px

    marker_img = draw_aruco_marker(aruco_dict, marker_id, marker_px)

    canvas = np.full((total_px, total_px), 255, dtype=np.uint8)
    canvas[border_px:border_px + marker_px, border_px:border_px + marker_px] = marker_img
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)


def generate_a4_sheet(
    aruco_dict,
    marker_ids=(0, 1),
    marker_size_cm: float = BLACK_MARKER_CM,
    quiet_zone_cm: float = QUIET_ZONE_CM,
    dpi: int = EXACT_DPI,
) -> np.ndarray:
    """生成满版可裁剪 A4 打印页 (21.0 cm × 29.7 cm @ 600 DPI)."""
    a4_w_px = cm_to_px(21.0, dpi)
    a4_h_px = cm_to_px(29.7, dpi)
    sheet = np.full((a4_h_px, a4_w_px, 3), 255, dtype=np.uint8)

    ruler_x0 = cm_to_px(1.25, dpi)
    ruler_y0 = cm_to_px(1.25, dpi)
    ruler_cm = 10
    ruler_len_px = cm_to_px(ruler_cm, dpi)

    total_cm_val = marker_size_cm + 2 * quiet_zone_cm
    cv2.putText(
        sheet,
        f"1:1 Scale (100% size) | Card={total_cm_val:.1f}x{total_cm_val:.1f}cm "
        f"(border {quiet_zone_cm * 10:.1f}mm, black {marker_size_cm:.1f}cm)",
        (ruler_x0, cm_to_px(0.85, dpi)),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.2,
        (0, 0, 0),
        2,
        cv2.LINE_AA,
    )
    cv2.line(sheet, (ruler_x0, ruler_y0), (ruler_x0 + ruler_len_px, ruler_y0), (0, 0, 0), 3)

    for mm in range(0, ruler_cm * 10 + 1):
        x = ruler_x0 + cm_to_px(mm / 10.0, dpi)
        is_highlight = mm in (
            int(round(marker_size_cm * 10)),
            int(round((marker_size_cm + 2 * quiet_zone_cm) * 10)),
        )
        if mm % 10 == 0:
            tick = cm_to_px(0.32, dpi)
            w = 3
            cv2.putText(
                sheet,
                f"{mm // 10}",
                (x - cm_to_px(0.10, dpi), ruler_y0 + tick + cm_to_px(0.28, dpi)),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.1,
                (0, 0, 0),
                2,
                cv2.LINE_AA,
            )
        elif mm % 5 == 0:
            tick = cm_to_px(0.22, dpi)
            w = 2
        else:
            tick = cm_to_px(0.12, dpi)
            w = 1

        color = (0, 0, 200) if is_highlight else (0, 0, 0)
        cv2.line(sheet, (x, ruler_y0), (x, ruler_y0 + tick), color, w)

    total_card_cm = marker_size_cm + 2.0 * quiet_zone_cm
    card_px = cm_to_px(total_card_cm, dpi)

    cols = 5
    rows = 6
    col_gap_cm = 0.25
    row_gap_cm = 0.42

    grid_w_cm = cols * total_card_cm + (cols - 1) * col_gap_cm
    start_x_cm = (21.0 - grid_w_cm) / 2.0
    start_y_cm = 2.35

    marker_cards = {
        m_id: generate_single_marker(aruco_dict, m_id, marker_size_cm, quiet_zone_cm, dpi)
        for m_id in marker_ids
    }

    dash_len = cm_to_px(0.12, dpi)
    for r in range(rows):
        m_id = marker_ids[0] if r < (rows // 2) else marker_ids[-1]
        card_img = marker_cards[m_id]
        card_y0 = cm_to_px(start_y_cm + r * (total_card_cm + row_gap_cm), dpi)
        card_y1 = card_y0 + card_px

        for c in range(cols):
            card_x0 = cm_to_px(start_x_cm + c * (total_card_cm + col_gap_cm), dpi)
            card_x1 = card_x0 + card_px

            sheet[card_y0:card_y1, card_x0:card_x1] = card_img

            for x in range(card_x0, card_x1, dash_len * 2):
                for y in (card_y0, card_y1):
                    cv2.line(sheet, (x, y), (min(x + dash_len, card_x1), y), (140, 140, 140), 2)
            for y in range(card_y0, card_y1, dash_len * 2):
                for x in (card_x0, card_x1):
                    cv2.line(sheet, (x, y), (x, min(y + dash_len, card_y1)), (140, 140, 140), 2)

            cv2.putText(
                sheet,
                f"ID {m_id}",
                (card_x0 + cm_to_px(1.35, dpi), card_y1 + cm_to_px(0.30, dpi)),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.1,
                (40, 40, 40),
                2,
                cv2.LINE_AA,
            )

    return sheet


def main():
    out_dir = os.getcwd()
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_size_cm = BLACK_MARKER_CM
    quiet_zone_cm = QUIET_ZONE_CM
    total_card_cm = marker_size_cm + 2.0 * quiet_zone_cm

    for m_id in (0, 1):
        single_img = generate_single_marker(aruco_dict, m_id)
        single_path = os.path.join(
            out_dir, f"aruco_4x4_50_id{m_id}_{total_card_cm:.1f}cm_600dpi.png"
        )
        cv2.imwrite(single_path, single_img)
        print(f"已生成单张贴纸图像: {single_path}")

    sheet = generate_a4_sheet(aruco_dict)
    png_path = os.path.join(out_dir, "aruco_markers_3.5cm_A4_print.png")
    cv2.imwrite(png_path, sheet)
    print(f"已生成 A4 打印图像: {png_path}")


if __name__ == "__main__":
    main()
