import numpy as np
import pytest

from camera_calibration import transforms
from camera_calibration.board import Board, Detection, probe
from conftest import BOARD_CONFIG, front_pose, render

K = np.array([[900.0, 0, 640.0], [0, 900.0, 360.0], [0, 0, 1.0]])
D = np.zeros(5)
SIZE = (1280, 720)


def test_geometry(board):
    described = board.describe()
    assert described['corner_count'] == 88
    assert described['width_m'] == pytest.approx(0.27)
    assert described['height_m'] == pytest.approx(0.36)


def test_dictionary_capacity_is_checked():
    # 9x12 = 108 格要 54 个 marker，DICT_5X5_50 装不下，必须当场报错而不是静默出错
    with pytest.raises(ValueError, match='54'):
        Board(9, 12, 0.030, 0.0225, 'DICT_5X5_50')


def test_marker_must_be_smaller_than_square():
    with pytest.raises(ValueError):
        Board(9, 12, 0.030, 0.030, 'DICT_5X5_100')


def test_detect_and_pose(board, flat):
    truth = front_pose(board, [0.25, -0.35, 0.12], [0.0, 0.0, 0.55])
    scene, expected = render(board, flat, truth, K, D, SIZE)

    detection = board.detect(scene)
    assert detection.count == board.corner_count
    assert detection.marker_count == 54
    assert detection.coverage() > 0.1

    error = np.linalg.norm(
        detection.corners.reshape(-1, 2) - expected[detection.ids.reshape(-1)], axis=1)
    assert error.mean() < 1.5

    pose = board.estimate_pose(detection, K, D)
    angle, distance = transforms.transform_delta(truth, pose)
    assert angle < 1.0
    assert distance < 3e-3


def test_pose_needs_enough_corners(board):
    empty = Detection(np.zeros((0, 1, 2), np.float32), np.zeros((0, 1), np.int32),
                      SIZE)
    assert board.estimate_pose(empty, K, D) is None


def test_probe_picks_the_right_orientation(board, flat):
    truth = front_pose(board, [0.15, -0.20, 0.05], [0.0, 0.0, 0.6])
    scene, _ = render(board, flat, truth, K, D, SIZE)

    results = [r for r in probe(scene, BOARD_CONFIG) if r.get('corners')]
    best = results[0]
    assert (best['squares_x'], best['squares_y']) == (9, 12)
    assert best['residual_px'] < 2.0

    # 朝向猜错时角点数一样多，只有共面残差能把它区分出来
    wrong = [r for r in results if (r['squares_x'], r['squares_y']) == (12, 9)][0]
    assert wrong['residual_px'] > 10 * best['residual_px']


def test_detection_json_roundtrip(board, flat):
    detection = board.detect(flat)
    back = Detection.from_json(detection.to_json())
    assert np.allclose(back.corners, detection.corners)
    assert np.array_equal(back.ids, detection.ids)
    assert back.size == detection.size
