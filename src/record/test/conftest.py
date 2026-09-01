"""共用夹具。"""

import pytest

#: 一份形状与 `camera_calibration/config/calibration.yaml` 相同的最小标定。
#: 数值不必是真的，但**结构必须真** —— 快照按分辨率精确匹配内参，腕相机的两个
#: link 也只由 `create: true` 那两条现建。
CALIBRATION = {
    'version': 1,
    'intrinsics': {
        'camera_left': [
            {'width': 1920, 'height': 1080,
             'camera_matrix': [1288.0, 0.0, 962.8, 0.0, 1286.4, 539.2, 0.0, 0.0, 1.0],
             'distortion_model': 'plumb_bob',
             'distortion_coefficients': [-0.37, 0.14, 0.0006, 0.001, -0.031]},
            {'width': 640, 'height': 360,
             'camera_matrix': [429.6, 0.0, 319.3, 0.0, 429.2, 173.7, 0.0, 0.0, 1.0],
             'distortion_model': 'plumb_bob',
             'distortion_coefficients': [-0.37, 0.15, 0.003, 0.002, -0.032]},
        ],
        'camera_right': [
            {'width': 1920, 'height': 1080,
             'camera_matrix': [1282.3, 0.0, 962.7, 0.0, 1280.6, 566.6, 0.0, 0.0, 1.0],
             'distortion_model': 'plumb_bob',
             'distortion_coefficients': [-0.37, 0.14, -0.0003, 0.0008, -0.028]},
        ],
    },
    'urdf_overrides': {
        'd435_joint': {
            'parent': 'torso_link', 'child': 'd435_link',
            'xyz': [0.0708667, 0.0112327, 0.4236048],
            'rpy': [0.025366, 1.0747681, 0.0237102],
        },
        'left_camera_optical_joint': {
            'parent': 'left_camera_mount_link', 'child': 'camera_left', 'create': True,
            'xyz': [0.0044787, 0.051485, 0.0538411],
            'rpy': [-0.2018536, 0.0113029, 3.1085119],
        },
        'right_camera_optical_joint': {
            'parent': 'right_camera_mount_link', 'child': 'camera_right', 'create': True,
            'xyz': [-0.0079276, 0.0403146, 0.0479877],
            'rpy': [-0.225763, -0.0402273, -3.0978914],
        },
    },
}


@pytest.fixture(scope='session')
def calibration() -> dict:
    """别就地改它 —— 整个测试会话共用一份。"""
    return CALIBRATION
