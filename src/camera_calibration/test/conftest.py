import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from camera_calibration import transforms  # noqa: E402
from camera_calibration.board import Board  # noqa: E402

BOARD_CONFIG = {
    'board': {'squares_x': 9, 'squares_y': 12, 'square_size': 0.030,
              'marker_size': 0.0225, 'dictionary': 'DICT_5X5_100'},
    'probe': {'dictionaries': ['DICT_5X5_50', 'DICT_5X5_100', 'DICT_4X4_250'],
              'orientations': [[9, 12], [12, 9]]},
}


@pytest.fixture(scope='session')
def board():
    return Board.from_config(BOARD_CONFIG)


@pytest.fixture(scope='session')
def flat(board):
    return board.draw(4000.0)


def front_pose(board, rvec, translation):
    """板坐标系 +z 指向观察者，所以正面视角要先绕 x 轴翻 180°"""
    rotation = (cv2.Rodrigues(np.array([np.pi, 0.0, 0.0]))[0]
                @ cv2.Rodrigues(np.asarray(rvec, float))[0])
    centre = board.object_points_all.mean(axis=0)
    return transforms.rt_to_matrix(
        rotation, np.asarray(translation, float) - rotation @ centre)


def render(board, flat, pose, camera_matrix, distortion, size):
    """把平板图 warp 成"从 pose 拍到"的样子。

    板面米 -> 平板像素 的映射从平板图自身检测拟合，不手推，这样 draw() 和
    chessboardCorners 的排布约定万一不一致会当场 assert 掉。
    """
    detection = board.detect(flat)
    assert detection.count == board.corner_count
    affine, _ = cv2.estimateAffine2D(
        board.object_points(detection.ids)[:, :2].astype(np.float32),
        detection.corners.reshape(-1, 2).astype(np.float32))

    obj = np.ascontiguousarray(board.object_points_all)
    rvec = cv2.Rodrigues(pose[:3, :3])[0]
    projected = cv2.projectPoints(obj, rvec, pose[:3, 3], camera_matrix,
                                  distortion)[0].reshape(-1, 2)
    flat_px = cv2.transform(
        np.ascontiguousarray(obj[:, :2]).reshape(-1, 1, 2), affine).reshape(-1, 2)
    matrix, _ = cv2.findHomography(flat_px, projected, 0)
    assert np.linalg.det(matrix[:2, :2]) > 0, '渲染成了镜像，ArUco 解不出来'
    scene = cv2.warpPerspective(flat, matrix, size, borderValue=255)
    return cv2.cvtColor(scene, cv2.COLOR_GRAY2BGR), projected
