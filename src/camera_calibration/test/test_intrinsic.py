import cv2
import numpy as np

from camera_calibration import intrinsic
from camera_calibration.board import Detection
from conftest import front_pose

K = np.array([[820.0, 0, 636.0], [0, 818.0, 358.0], [0, 0, 1.0]])
D = np.array([-0.152, 0.041, 0.0011, -0.0007, 0.0])
SIZE = (1280, 720)


def synth(board, pose, size=SIZE, noise=0.0, seed=0):
    """直接投影出角点，跳过渲染。标定算法只吃角点，渲染只会拖慢测试。"""
    rvec = cv2.Rodrigues(pose[:3, :3])[0]
    points = cv2.projectPoints(np.ascontiguousarray(board.object_points_all), rvec,
                               pose[:3, 3], K, D)[0].reshape(-1, 2)
    if noise:
        points = points + np.random.default_rng(seed).normal(0, noise, points.shape)
    inside = ((points[:, 0] > 2) & (points[:, 0] < size[0] - 2)
              & (points[:, 1] > 2) & (points[:, 1] < size[1] - 2))
    ids = np.nonzero(inside)[0].astype(np.int32)
    return Detection(points[inside].astype(np.float32).reshape(-1, 1, 2),
                     ids.reshape(-1, 1), size, marker_count=len(ids) // 2)


def sweep(board, count=24, seed=1):
    """姿态要在画面里铺开：畸变系数全靠边角的点约束，都堆在中间标不出来"""
    rng = np.random.default_rng(seed)
    views = []
    for index in range(count):
        rvec = rng.uniform(-0.5, 0.5, 3)
        distance = rng.uniform(0.40, 0.90)
        offset = rng.uniform(-0.24, 0.24, 2) * distance
        pose = front_pose(board, rvec, [offset[0], offset[1], distance])
        views.append({'name': f'{index:04d}', 'detection': synth(board, pose)})
    return views


def test_recovers_known_intrinsics(board):
    result = intrinsic.calibrate(board, sweep(board))
    assert result['ok'], result.get('reason')
    matrix = np.asarray(result['camera_matrix']).reshape(3, 3)
    assert np.allclose(matrix[:2, :2], K[:2, :2], rtol=0.01, atol=1.0)
    assert np.allclose(matrix[:2, 2], K[:2, 2], atol=2.0)
    assert np.allclose(result['distortion_coefficients'][:2], D[:2], atol=0.02)
    assert result['rms'] < 0.2
    assert result['coverage'] > 0.4
    assert len(result['per_image']) == 24


def test_rejects_too_few_views(board):
    result = intrinsic.calibrate(board, sweep(board, count=3))
    assert not result['ok']
    assert '至少' in result['reason']


def test_rejects_mixed_resolutions(board):
    views = sweep(board)
    views[0]['detection'].size = (640, 360)
    result = intrinsic.calibrate(board, views)
    assert not result['ok']
    assert '分辨率不一致' in result['reason']


def test_outliers_flags_the_bad_view(board):
    views = sweep(board)
    bad = views[5]['detection']
    # 必须是随机扰动：整体平移会被该视图自己的 rvec/tvec 吸掉，重投影误差不变
    noise = np.random.default_rng(7).normal(0, 3.0, bad.corners.shape)
    bad.corners = (bad.corners + noise).astype(np.float32)
    result = intrinsic.calibrate(board, views)
    assert '0005' in intrinsic.outliers(result)


def test_fov_matches_pinhole_geometry(board):
    result = intrinsic.calibrate(board, sweep(board))
    expected = np.degrees(2 * np.arctan(SIZE[0] / (2 * K[0, 0])))
    assert abs(result['fov_deg'][0] - expected) < 1.0


def test_relate_profiles_detects_scale(board):
    high = intrinsic.calibrate(board, sweep(board))
    low = dict(high)
    low['width'], low['height'] = 640, 360
    matrix = np.asarray(high['camera_matrix'], float).reshape(3, 3) * 0.5
    matrix[2, 2] = 1.0
    low['camera_matrix'] = matrix.reshape(9).tolist()

    relation = intrinsic.relate_profiles(low, high)
    assert relation['kind'] == 'scale'
    assert relation['convertible']


def test_relate_profiles_detects_crop(board):
    high = intrinsic.calibrate(board, sweep(board))
    # 裁剪：fx 一点不变，只有主点跟着裁掉的边平移
    low = dict(high)
    low['width'], low['height'] = 960, 720
    matrix = np.asarray(high['camera_matrix'], float).reshape(3, 3).copy()
    matrix[0, 2] -= (1280 - 960) / 2
    low['camera_matrix'] = matrix.reshape(9).tolist()

    relation = intrinsic.relate_profiles(low, high)
    assert relation['kind'] == 'crop'
    assert not relation['convertible']
