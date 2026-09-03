"""`g1_localization.transforms` 的数学检查。

这些函数没有 ROS 依赖，坏了也不会在启动时报错，只会让世界坐标悄悄歪掉，
所以逐条钉死：往返一致、与已知闭式解吻合、退化输入按约定报错。
"""

import math

import numpy as np
import pytest

from g1_localization.transforms import (
    body_twist,
    invert,
    level_frame,
    ground_level_frame,
    make_tf,
    mat_to_quat,
    quat_to_mat,
    rigid_point_velocity,
    rot_z,
    yaw_of,
)


def test_ground_level_frame_keeps_heading_and_places_ground_at_zero():
    t_ref_body = make_tf([2.0, -1.0, 0.82], [0.12, -0.08, 0.31, 0.938])
    t_world_ref = invert(ground_level_frame(t_ref_body, ground_z=0.07))
    body_world = t_world_ref @ t_ref_body
    ground_world = t_world_ref @ np.array([2.0, -1.0, 0.07, 1.0])

    assert np.allclose(ground_world[:3], 0.0, atol=1e-12)
    assert body_world[2, 3] == pytest.approx(0.75)
    assert np.allclose(t_world_ref[:3, :3] @ np.array([0.0, 0.0, 1.0]),
                       [0.0, 0.0, 1.0], atol=1e-12)

RNG = np.random.default_rng(20260825)


def random_quat(n=1):
    q = RNG.normal(size=(n, 4))
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def test_quat_to_mat_is_orthonormal_and_right_handed():
    for q in random_quat(50):
        m = quat_to_mat(q)
        assert np.allclose(m @ m.T, np.eye(3), atol=1e-12)
        assert np.isclose(np.linalg.det(m), 1.0, atol=1e-12)


def test_quat_mat_roundtrip():
    for q in random_quat(200):
        back = mat_to_quat(quat_to_mat(q))
        # q 与 -q 是同一个旋转，比矩阵而不是比分量。
        assert np.allclose(quat_to_mat(back), quat_to_mat(q), atol=1e-10)


def test_mat_to_quat_covers_all_four_branches():
    """Shepperd 分支法按迹和对角线最大项分四支，每支都要走到。"""
    cases = [
        np.eye(3),                                   # trace > 0
        quat_to_mat([1.0, 0.0, 0.0, 0.0]),           # 绕 x 转 180°，m00 最大
        quat_to_mat([0.0, 1.0, 0.0, 0.0]),           # 绕 y 转 180°，m11 最大
        quat_to_mat([0.0, 0.0, 1.0, 0.0]),           # 绕 z 转 180°，m22 最大
    ]
    for m in cases:
        assert np.allclose(quat_to_mat(mat_to_quat(m)), m, atol=1e-12)


def test_quat_to_mat_normalizes_input():
    scaled = quat_to_mat([0.0, 0.0, 0.0, 7.0])
    assert np.allclose(scaled, np.eye(3), atol=1e-12)


def test_quat_to_mat_rejects_zero():
    with pytest.raises(ValueError):
        quat_to_mat([0.0, 0.0, 0.0, 0.0])


def test_known_rotation():
    """绕 z 转 90°：x 轴应当转到 y 轴。"""
    m = quat_to_mat([0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)])
    assert np.allclose(m @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-12)


def test_invert_matches_linalg_and_composes_to_identity():
    for q in random_quat(30):
        t = make_tf(RNG.normal(size=3) * 3.0, q)
        assert np.allclose(invert(t), np.linalg.inv(t), atol=1e-10)
        assert np.allclose(invert(t) @ t, np.eye(4), atol=1e-12)


def test_compose_is_associative_in_the_expected_order():
    """`T_a_c = T_a_b @ T_b_c`：点从 c 系变到 a 系应当与分两步一致。"""
    t_a_b = make_tf([1.0, 2.0, 3.0], random_quat()[0])
    t_b_c = make_tf([-0.5, 0.25, 4.0], random_quat()[0])
    p_c = np.array([0.3, -0.7, 1.1, 1.0])
    assert np.allclose((t_a_b @ t_b_c) @ p_c, t_a_b @ (t_b_c @ p_c), atol=1e-12)


@pytest.mark.parametrize('yaw', [0.0, 0.3, -1.2, math.pi - 0.01])
def test_yaw_of_rot_z_roundtrip(yaw):
    assert np.isclose(yaw_of(rot_z(yaw)), yaw, atol=1e-12)


def test_yaw_of_ignores_roll_and_pitch():
    """有 roll/pitch 时 yaw_of 取的仍是绕 z 的那一份。"""
    yaw = 0.6
    pitch = np.array([[math.cos(0.2), 0.0, math.sin(0.2)],
                      [0.0, 1.0, 0.0],
                      [-math.sin(0.2), 0.0, math.cos(0.2)]])
    assert np.isclose(yaw_of(rot_z(yaw) @ pitch), yaw, atol=1e-9)


def test_level_frame_zeroes_translation_and_yaw_but_keeps_tilt():
    """这是世界原点的定义：设原点那一刻的位姿换算到世界系后只剩 roll/pitch。"""
    tilt = quat_to_mat([math.sin(0.05), 0.0, 0.0, math.cos(0.05)])   # 绕 x 倾 5.7°
    t_ref_body = np.eye(4)
    t_ref_body[:3, :3] = rot_z(1.1) @ tilt
    t_ref_body[:3, 3] = [4.0, -2.0, 0.75]

    t_world_ref = invert(level_frame(t_ref_body))
    at_origin = t_world_ref @ t_ref_body

    assert np.allclose(at_origin[:3, 3], 0.0, atol=1e-12)
    assert np.isclose(yaw_of(at_origin[:3, :3]), 0.0, atol=1e-12)
    # 倾角必须留着，否则世界系的 z 轴就不铅垂了。
    assert np.allclose(at_origin[:3, :3], tilt, atol=1e-12)


def test_level_frame_keeps_z_axis_vertical():
    """世界系的 z 轴与参考系的 z 轴同向 —— 上游 gravity_align 已把它对到重力。"""
    t_ref_body = np.eye(4)
    t_ref_body[:3, :3] = quat_to_mat(random_quat()[0])
    t_ref_body[:3, 3] = RNG.normal(size=3)
    assert np.allclose(level_frame(t_ref_body)[:3, 2], [0.0, 0.0, 1.0], atol=1e-12)


def test_rigid_point_velocity_zero_lever_is_identity():
    v = np.array([0.1, -0.2, 0.3])
    assert np.allclose(rigid_point_velocity(v, [1.0, 2.0, 3.0], [1, 1, 1], [1, 1, 1]), v)


def test_rigid_point_velocity_pure_rotation():
    """绕 z 以 ω 转，距轴 r 处的线速度是 ω·r，方向切向。"""
    omega = np.array([0.0, 0.0, 2.0])
    v = rigid_point_velocity([0.0, 0.0, 0.0], omega, [0.0, 0.0, 0.0], [0.5, 0.0, 0.0])
    assert np.allclose(v, [0.0, 1.0, 0.0], atol=1e-12)


def test_rigid_point_velocity_lever_matters_at_head_offset():
    """雷达离躯干原点 0.428 m，转头时的杠杆速度不可忽略 —— 这是节点里补它的理由。"""
    omega = np.array([0.0, 0.5, 0.0])       # 0.5 rad/s 的点头
    v_imu = np.array([0.0, 0.0, 0.0])
    v_torso = rigid_point_velocity(v_imu, omega, [0.0, 0.0, 0.428], [0.0, 0.0, 0.0])
    assert np.isclose(np.linalg.norm(v_torso), 0.5 * 0.428, atol=1e-12)
    assert np.linalg.norm(v_torso) > 0.2


def test_make_tf_places_translation_in_last_column():
    t = make_tf([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])
    assert np.allclose(t[:3, 3], [1.0, 2.0, 3.0])
    assert np.allclose(t[3, :], [0.0, 0.0, 0.0, 1.0])


# -- body_twist --------------------------------------------------------------
#
# 这是整个包里最容易错又最难看出来的一段：三个参考系混在一起，写反了不会报错，
# 只会让录进数据集的躯干速度悄悄偏掉。拿有限差分当独立的第二实现钉死。


def _skew(w):
    return np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])


def _screw(t0: np.ndarray, v_ref: np.ndarray, omega_ref: np.ndarray, dt: float):
    """t0 处以恒定 `omega_ref`（参考系下）和 `v_ref` 运动，dt 后的位姿。"""
    ang = float(np.linalg.norm(omega_ref)) * dt
    if abs(ang) < 1e-15:
        dr = np.eye(3)
    else:
        k = _skew(omega_ref / np.linalg.norm(omega_ref))
        dr = np.eye(3) + math.sin(ang) * k + (1.0 - math.cos(ang)) * (k @ k)
    out = np.eye(4)
    out[:3, :3] = dr @ t0[:3, :3]
    out[:3, 3] = t0[:3, 3] + v_ref * dt
    return out


def _twist_by_finite_difference(t_ref_a, t_a_b, v_ref_a, omega_a, h=1e-6):
    omega_ref = t_ref_a[:3, :3] @ omega_a
    tb = [_screw(t_ref_a, v_ref_a, omega_ref, d) @ t_a_b for d in (-h, 0.0, h)]
    r = tb[1][:3, :3]
    v = r.T @ ((tb[2][:3, 3] - tb[0][:3, 3]) / (2 * h))
    w_hat = r.T @ ((tb[2][:3, :3] - tb[0][:3, :3]) / (2 * h))
    return v, np.array([w_hat[2, 1], w_hat[0, 2], w_hat[1, 0]])


def test_body_twist_matches_finite_difference():
    for _ in range(100):
        t_a_b = make_tf(RNG.normal(size=3) * 0.4, random_quat()[0])
        t_ref_a = make_tf(RNG.normal(size=3) * 2.0, random_quat()[0])
        v_ref_a = RNG.normal(size=3) * 0.8
        omega_a = RNG.normal(size=3) * 1.5

        v, w = body_twist(v_ref_a, omega_a, t_ref_a, t_a_b)
        v_fd, w_fd = _twist_by_finite_difference(t_ref_a, t_a_b, v_ref_a, omega_a)
        assert np.allclose(v, v_fd, atol=1e-7)
        assert np.allclose(w, w_fd, atol=1e-7)


def test_body_twist_angular_only_changes_frame():
    """角速度只换表达系，不受平移和杠杆臂影响。"""
    t_a_b = make_tf([0.1, -0.4, 0.9], random_quat()[0])
    t_ref_a = make_tf(RNG.normal(size=3), random_quat()[0])
    omega_a = np.array([0.3, -1.1, 0.7])
    _, w = body_twist(RNG.normal(size=3), omega_a, t_ref_a, t_a_b)
    assert np.allclose(w, t_a_b[:3, :3].T @ omega_a, atol=1e-12)


def test_body_twist_identity_extrinsic_is_pure_frame_change():
    """外参为单位阵时没有杠杆臂，只剩「把线速度转进体系」。"""
    t_ref_a = make_tf([1.0, 2.0, 3.0], random_quat()[0])
    v_ref_a = np.array([0.2, -0.5, 0.1])
    v, w = body_twist(v_ref_a, [0.4, 0.2, -0.3], t_ref_a, np.eye(4))
    assert np.allclose(v, t_ref_a[:3, :3].T @ v_ref_a, atol=1e-12)
    assert np.allclose(w, [0.4, 0.2, -0.3], atol=1e-12)


def test_body_twist_lever_dominates_when_head_turns():
    """静止但转头：躯干原点该有速度，量级就是 ω x 杠杆臂。"""
    t_a_b = np.eye(4)
    t_a_b[:3, 3] = [0.0, 0.0, -0.3914]          # 躯干原点在 IMU 系下
    v, _ = body_twist([0.0, 0.0, 0.0], [0.0, 0.5, 0.0], np.eye(4), t_a_b)
    assert np.isclose(np.linalg.norm(v), 0.5 * 0.3914, atol=1e-12)
