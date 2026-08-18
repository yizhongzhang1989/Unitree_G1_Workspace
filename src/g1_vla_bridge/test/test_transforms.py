"""旋转表示互转的往返一致性。"""

import numpy as np
import pytest

from g1_vla_bridge.transforms import (
    mat_to_quat,
    orthonormalize,
    pose_matrix,
    quat_angle,
    quat_slerp,
    quat_to_mat,
)

# 覆盖 Shepperd 的四个分支：trace>0 和三个对角元各自最大的情况。
_ANGLES = (0.0, 0.3, 1.2, np.pi / 2, np.pi - 1e-3, np.pi)
_AXES = ((1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 0), (1, 1, 1), (-1, 2, -3))


def _axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    return np.array([*(axis * np.sin(angle / 2.0)), np.cos(angle / 2.0)])


@pytest.mark.parametrize('axis', _AXES)
@pytest.mark.parametrize('angle', _ANGLES)
def test_quat_mat_roundtrip(axis, angle):
    quat = _axis_angle(axis, angle)
    back = mat_to_quat(quat_to_mat(quat))
    # 四元数双覆盖，q 和 -q 是同一个姿态。
    assert quat_angle(quat, back) < 1e-9
    assert np.allclose(quat_to_mat(back), quat_to_mat(quat), atol=1e-12)


def test_mat_to_quat_is_unit():
    for axis in _AXES:
        for angle in _ANGLES:
            quat = mat_to_quat(quat_to_mat(_axis_angle(axis, angle)))
            assert np.isclose(np.linalg.norm(quat), 1.0)


def test_orthonormalize_fixes_noise():
    rot = quat_to_mat(_axis_angle((1, 2, 3), 0.7))
    noisy = rot + np.random.default_rng(0).normal(scale=1e-3, size=(3, 3))
    fixed = orthonormalize(noisy)
    assert np.allclose(fixed @ fixed.T, np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(fixed), 1.0)
    assert quat_angle(mat_to_quat(fixed), mat_to_quat(rot)) < 5e-3


def test_orthonormalize_rejects_reflection():
    # det=-1 的输入必须被投影回 det=+1，不能原样吐出去。
    fixed = orthonormalize(np.diag([1.0, 1.0, -1.0]))
    assert np.isclose(np.linalg.det(fixed), 1.0)


def test_slerp_clamps_step():
    a = _axis_angle((0, 0, 1), 0.0)
    b = _axis_angle((0, 0, 1), 1.0)
    limit = 0.1
    stepped = quat_slerp(a, b, limit / quat_angle(a, b))
    assert np.isclose(quat_angle(a, stepped), limit, atol=1e-9)


def test_slerp_takes_short_arc():
    # 传进来的 b 取了负号（同一姿态的另一半覆盖），不能绕远路。
    a = _axis_angle((0, 1, 0), 0.0)
    b = -_axis_angle((0, 1, 0), 0.4)
    assert np.isclose(quat_angle(a, quat_slerp(a, b, 1.0)), 0.4, atol=1e-9)


def test_pose_matrix():
    quat = _axis_angle((0, 0, 1), np.pi / 2)
    out = pose_matrix(quat, [1.0, 2.0, 3.0])
    assert out.shape == (4, 4)
    assert np.allclose(out[3], [0, 0, 0, 1])
    assert np.allclose(out[:3, 3], [1.0, 2.0, 3.0])
    assert np.allclose(out[:3, :3] @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-12)


def test_invalid_inputs():
    with pytest.raises(ValueError):
        quat_to_mat([0.0, 0.0, 0.0, 0.0])
    with pytest.raises(ValueError):
        orthonormalize([[np.nan] * 3] * 3)
