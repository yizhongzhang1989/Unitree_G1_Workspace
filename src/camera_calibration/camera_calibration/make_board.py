"""按 config/board.yaml 生成一张可打印的 ChArUco 板。

主要用途是核对手上的实物：打出来（或屏幕上）和实物比格数、比朝向、量方格边长。
打印时务必关掉「适应页面 / 缩放」，然后拿尺子量一格核对 —— 尺寸填错的话标定照样
跑完，出来的内参看着也正常，只有涉及米的量（外参平移）会整体差一个比例。
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import yaml
from ament_index_python.packages import get_package_share_directory

from camera_calibration.board import Board

MM_PER_INCH = 25.4


def main() -> None:
    share = Path(get_package_share_directory('camera_calibration')) / 'config'
    parser = argparse.ArgumentParser(description='生成 ChArUco 标定板图')
    parser.add_argument('--config', default=str(share / 'board.yaml'))
    parser.add_argument('--out', default='/tmp/charuco_board.png')
    parser.add_argument('--dpi', type=float, default=300.0)
    parser.add_argument('--margin-squares', type=float, default=0.5)
    args = parser.parse_args()

    config = yaml.safe_load(Path(args.config).expanduser().read_text(encoding='utf-8'))
    board = Board.from_config(config)
    image = board.draw(args.dpi / MM_PER_INCH * 1000.0, args.margin_squares)
    cv2.imwrite(args.out, image)

    described = board.describe()
    print(f'{args.out}  {image.shape[1]}x{image.shape[0]} px @ {args.dpi:g} dpi')
    print(f"  {described['squares_x']}x{described['squares_y']} 格 · "
          f"方格 {described['square_size'] * 1000:g} mm · "
          f"marker {described['marker_size'] * 1000:g} mm · {described['dictionary']}")
    print(f"  板面 {described['width_m'] * 1000:g} x {described['height_m'] * 1000:g} mm · "
          f"{described['corner_count']} 个内角点 · "
          f"{described['squares_x'] * described['squares_y'] // 2} 个 marker")
    print('  打印时关掉缩放，打完量一格核对边长')
