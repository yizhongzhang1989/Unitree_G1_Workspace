"""由相机位姿自动解坐标系参数：数学部分。"""

import numpy as np
import pytest

from g1_vla_bridge.transforms import (
    invert_pose,
    mat_to_rpy,
    pose_matrix,
    rpy_to_mat,
    solve_base_frame,
    solve_origin_position,
)
from g1_vla_bridge.vla_backend import FrameSpec

RPY_CASES = ([0.0, 0.0, 0.0], [0.1, -0.2, 0.3], [1.2, 0.4, -2.5],
             [-0.7, 1.0, 0.0], [0.0, 0.0, np.pi])


@pytest.mark.parametrize('rpy', RPY_CASES)
def test_rpy_roundtrip(rpy):
    assert np.allclose(mat_to_rpy(rpy_to_mat(rpy)), rpy, atol=1e-12)


def test_rpy_gimbal_lock():
    """|pitch|=90° 时 roll/yaw 简并，只要能还原出同一个矩阵就算对。"""
    for pitch in (np.pi / 2, -np.pi / 2):
        m = rpy_to_mat([0.4, pitch, -0.9])
        assert np.allclose(rpy_to_mat(mat_to_rpy(m)), m, atol=1e-9)


def test_invert_pose():
    m = pose_matrix([0.1, 0.2, 0.3, 0.9], [1.0, -2.0, 0.5])
    assert np.allclose(invert_pose(m) @ m, np.eye(4), atol=1e-12)


def test_solve_base_frame_identity():
    """两台相机位姿完全一样时，模型系就等于 base_frame。"""
    cam = pose_matrix([0.1, 0.2, 0.3, 0.9], [0.05, 0.03, 0.43])
    offset, rpy = solve_base_frame(cam, cam)
    assert np.allclose(offset, 0.0, atol=1e-12)
    assert np.allclose(rpy, 0.0, atol=1e-12)


def test_solve_base_frame_recovers_known_transform():
    """构造一个已知的 T_model<-base，看能不能原样解回来。"""
    truth_rpy = [0.05, -0.62, 0.11]
    truth_offset = np.array([0.37, -0.04, 0.54])
    model_from_base = np.eye(4)
    model_from_base[:3, :3] = rpy_to_mat(truth_rpy)
    model_from_base[:3, 3] = truth_offset

    base_cam = pose_matrix([-0.658, 0.662, -0.253, 0.254], [0.057, 0.033, 0.430])
    model_cam = model_from_base @ base_cam          # 同一台相机在模型系里的位姿
    offset, rpy = solve_base_frame(model_cam, base_cam)
    assert np.allclose(offset, truth_offset, atol=1e-12)
    assert np.allclose(rpy, truth_rpy, atol=1e-12)


def test_origin_position_puts_our_camera_on_theirs():
    """只挪原点：我方相机在模型系里的**位置**必须落到训练相机那个点上。"""
    base_cam = pose_matrix([-0.658, 0.662, -0.253, 0.254], [0.057, 0.033, 0.430])
    model_cam = pose_matrix([0.0, 0.0, 0.0, 1.0], [0.269, -0.018, 0.888])

    frame = FrameSpec(origin_in_base=solve_origin_position(model_cam, base_cam)).transform()
    assert np.allclose(frame.base_to_model(base_cam)[:3, 3], model_cam[:3, 3], atol=1e-12)


def test_origin_position_keeps_the_frame_level():
    """模型系不能被掰斜——重力方向得留在原处，否则末端 state 会跟着歪。"""
    base_cam = pose_matrix([-0.658, 0.662, -0.253, 0.254], [0.057, 0.033, 0.430])
    model_cam = pose_matrix([0.0, 0.0, 0.0, 1.0], [0.269, -0.018, 0.888])

    frame = FrameSpec(origin_in_base=solve_origin_position(model_cam, base_cam)).transform()
    assert np.allclose(frame.base_rot, np.eye(3), atol=1e-12)
    # 而完整解会把它掰斜，两者不是一回事。
    _, rpy = solve_base_frame(model_cam, base_cam)
    assert np.linalg.norm(rpy) > 1.0
