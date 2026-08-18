"""模型系 <-> base_frame 的换算，用真实标定数据钉住符号。"""

import numpy as np
import pytest

from g1_vla_bridge.transforms import (
    FrameTransform,
    mat_to_quat,
    pose_matrix,
    quat_angle,
    rpy_to_mat,
)

# 2026-08-14 那套已被推翻的标定值（见 README），这里只当**测符号用的真实量级数据**，
# 别拿去当现役参数——现役的在 backends/a2d_omnipicker.py 的 SPEC 里。
CALIB = dict(base_offset=[0.3704, 0.0, 0.5427],
             tool_rotation_rpy=[0.0, 0.0, np.pi],
             tool_offset=[0.0, 0.0, -0.0281])

# VLA 侧给的 joint7 4x4（模型系）。
VLA_LEFT = np.array([
    [-0.983890956, -0.037213603, 0.174853467, 0.601039566],
    [-0.177980610, 0.112113026, -0.977626499, 0.301644861],
    [0.016777653, -0.992998397, -0.116930292, 0.723674160],
    [0.0, 0.0, 0.0, 1.0]])
# 同姿态下我方 gripper_base 的 URDF 正解（torso_link）。
OURS_LEFT_POS = np.array([0.2354, 0.2740, 0.1797])


def test_rpy_to_mat_z180():
    assert np.allclose(rpy_to_mat([0.0, 0.0, np.pi]), np.diag([-1.0, -1.0, 1.0]), atol=1e-12)


def test_rpy_to_mat_is_rotation():
    for rpy in ([0.1, -0.2, 0.3], [np.pi / 2, 0, 0], [0, np.pi / 3, -np.pi / 4]):
        r = rpy_to_mat(rpy)
        assert np.allclose(r @ r.T, np.eye(3), atol=1e-12)
        assert np.isclose(np.linalg.det(r), 1.0)


@pytest.mark.parametrize('kwargs', [
    {},
    CALIB,
    dict(CALIB, base_rotation_rpy=[0.1, -0.84, 0.3]),   # 带旋转的完整 SE3
])
def test_roundtrip(kwargs):
    frame = FrameTransform(**kwargs)
    rng = np.random.default_rng(0)
    for _ in range(50):
        quat = rng.normal(size=4)
        pose = np.concatenate([rng.uniform(-0.5, 0.5, 3), quat / np.linalg.norm(quat)])
        back = frame.from_model(*frame.to_model(pose))
        assert np.allclose(back[:3], pose[:3], atol=1e-12)
        # quat_angle 走 acos，0 附近的双精度地板就有 ~2e-8 rad，别卡更死。
        assert quat_angle(back[3:], pose[3:]) < 1e-6


def test_identity_when_unconfigured():
    frame = FrameTransform()
    pose = np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0])
    trans, rot = frame.to_model(pose)
    assert np.allclose(trans, pose[:3])
    assert np.allclose(rot, np.eye(3))


def test_calibration_lands_on_measured_pose():
    """把 VLA 的 joint7 位姿换回 torso_link，应落在我方正解上。"""
    frame = FrameTransform(**CALIB)
    pose = frame.from_model(VLA_LEFT[:3, 3], VLA_LEFT[:3, :3])
    # 位置对到 3 mm 以内；关节角只是「类似的姿态」，姿态残差约 7.5°。
    assert np.linalg.norm(pose[:3] - OURS_LEFT_POS) < 3e-3


def test_tool_rotation_is_actually_applied():
    """漏掉绕 Z 的 180° 会让姿态整个反过来——这是最容易犯又最致命的错。"""
    frame = FrameTransform(**CALIB)
    naive = FrameTransform(base_offset=CALIB['base_offset'], tool_offset=CALIB['tool_offset'])
    a = frame.from_model(VLA_LEFT[:3, 3], VLA_LEFT[:3, :3])
    b = naive.from_model(VLA_LEFT[:3, 3], VLA_LEFT[:3, :3])
    assert quat_angle(a[3:], b[3:]) > np.radians(179.0)


def test_base_offset_direction():
    """state 往模型系发时是「加」偏置，不是减。"""
    frame = FrameTransform(base_offset=[0.3704, 0.0, 0.5427])
    trans, _ = frame.to_model([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    assert np.allclose(trans, [0.3704, 0.0, 0.5427])


def test_base_rotation_actually_rotates():
    """绕 Z 转 90°时，+X 上的点应当变成 +Y。"""
    frame = FrameTransform(base_rotation_rpy=[0.0, 0.0, np.pi / 2])
    trans, _ = frame.to_model([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    assert np.allclose(trans, [0.0, 1.0, 0.0], atol=1e-12)


def test_from_model_orthonormalizes():
    frame = FrameTransform(**CALIB)
    noisy = VLA_LEFT[:3, :3] + 1e-3
    pose = frame.from_model(VLA_LEFT[:3, 3], noisy)
    assert np.isclose(np.linalg.norm(pose[3:]), 1.0)
    assert quat_angle(pose[3:], mat_to_quat(noisy @ np.diag([-1.0, -1.0, 1.0]))) < 1e-9


def test_base_to_model_carries_rotation():
    """相机外参必须整体搬进模型系：只挪平移会把身体俯仰丢掉。"""
    frame = FrameTransform(base_offset=[0.1, -0.2, 0.5],
                           base_rotation_rpy=[0.0, 0.3, 0.0],
                           tool_rotation_rpy=[0.0, 0.0, np.pi],
                           tool_offset=[0.0, 0.0, -0.0281])
    camera = pose_matrix(mat_to_quat(rpy_to_mat([0.1, 0.8, -0.2])), [0.06, 0.03, 0.43])
    out = frame.base_to_model(camera)

    expected = np.eye(4)
    expected[:3, :3] = frame.base_rot @ camera[:3, :3]
    expected[:3, 3] = frame.base_rot @ camera[:3, 3] + frame.base_offset
    assert np.allclose(out, expected)
    # tool_* 是末端专用的，相机不能沾。
    assert not np.allclose(out[:3, :3], frame.base_rot @ camera[:3, :3] @ frame.tool_rot)
    # 俯仰确实进去了：base_rot 非单位时旋转必须变。
    assert quat_angle(mat_to_quat(out[:3, :3]), mat_to_quat(camera[:3, :3])) > 0.2
