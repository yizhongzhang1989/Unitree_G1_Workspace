"""arm_ik 的离线校验。不需要真机，也不需要 ROS。

这里只测和实机安全直接相关的四件事：缩减模型确实只剩 14 轴且 q 顺序可信、
正逆运动学互为反解、够不着时不抛异常且解仍在 URDF 限位内、单侧求解不动另一侧。
"""

from pathlib import Path

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from g1_lower_body_policy.arm_ik import ArmIK

ARM_JOINTS = [f'{side}_{joint}_joint'
              for side in ('left', 'right')
              for joint in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw',
                            'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')]
TIP_FRAMES = {'left': 'left_gripper_base', 'right': 'right_gripper_base'}


@pytest.fixture(scope='module')
def ik():
    urdf = (Path(get_package_share_directory('unitree_g1_description'))
            / 'model' / 'final.urdf')
    return ArmIK(urdf.read_text(encoding='utf-8'), ARM_JOINTS, TIP_FRAMES)


def test_reduced_model_keeps_only_the_arms(ik):
    assert ik.model.nq == 14
    # 调用方按 joint_names 反查 31 轴槽位，这个顺序必须是模型自己报的。
    assert sorted(ik.joint_names) == sorted(ARM_JOINTS)
    assert np.all(ik.lower < ik.upper)


def test_missing_joint_or_frame_fails_at_construction(ik):
    urdf = (Path(get_package_share_directory('unitree_g1_description'))
            / 'model' / 'final.urdf').read_text(encoding='utf-8')
    with pytest.raises(ValueError):
        ArmIK(urdf, ARM_JOINTS + ['no_such_joint'], TIP_FRAMES)
    with pytest.raises(ValueError):
        ArmIK(urdf, ARM_JOINTS, {'left': 'no_such_frame'})


def test_fk_then_ik_recovers_the_pose(ik):
    """实机是 50 Hz 小步跟随：种子离解很近，必须收敛到毫米内。"""
    rng = np.random.default_rng(0)
    for _ in range(50):
        truth = np.clip(rng.normal(0.0, 0.4, 14), ik.lower, ik.upper)
        targets = ik.fk(truth)
        seed = np.clip(truth + rng.normal(0.0, 0.05, 14), ik.lower, ik.upper)
        q, pos_err, ori_err, _ = ik.solve(seed, targets)
        assert pos_err < 2e-3 and ori_err < 1e-2
        assert np.all(np.isfinite(q))


def test_unreachable_target_is_best_effort_not_an_exception(ik):
    """够不着不能报错：上肢的问题不该把正在平衡的下肢一起拖下水。"""
    q, pos_err, _, iters = ik.solve(
        np.zeros(14), {'right': [1.5, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0]})
    assert iters == 10 and pos_err > 0.5
    assert np.all(np.isfinite(q))
    assert np.all(q >= ik.lower - 1e-9) and np.all(q <= ik.upper + 1e-9)


def test_solving_one_side_leaves_the_other_untouched(ik):
    """长度 7 的指令只该动右臂——左臂的列必须原封不动等于种子。"""
    seed = np.full(14, 0.1)
    q, _, _, _ = ik.solve(seed, {'right': ik.fk(np.full(14, 0.3))['right']})
    left = [i for i, name in enumerate(ik.joint_names) if name.startswith('left')]
    right = [i for i, name in enumerate(ik.joint_names) if name.startswith('right')]
    assert np.allclose(q[left], seed[left])
    assert not np.allclose(q[right], seed[right])


def test_seed_outside_limits_is_clamped(ik):
    q, _, _, _ = ik.solve(np.full(14, 1e3), {})
    assert np.allclose(q, ik.upper)


def test_redundant_dof_does_not_drift_along_a_closed_path(ik):
    """7 自由度、任务 6 维，多出来的那一维必须被钉住。

    没有零空间姿态偏置时，让右手画一个 6 cm 的圆就能让 shoulder_yaw 一路漂到
    -1.23 rad，然后在某一帧被迫重构、单帧跳 0.23 rad。上肢不限速，这一下就是电机
    全权限的弹动。这里复刻节点的调用方式（上一帧的解当种子，每帧 solve 一次）。
    """
    rest = {'left_shoulder_roll_joint': 0.25, 'left_elbow_joint': 0.5,
            'right_shoulder_roll_joint': -0.25, 'right_elbow_joint': 0.5}
    solver = ArmIK((Path(get_package_share_directory('unitree_g1_description'))
                    / 'model' / 'final.urdf').read_text(encoding='utf-8'),
                   ARM_JOINTS, TIP_FRAMES, rest_posture=rest)
    start = np.array([0.2, -0.3, 0, 0.8, 0, 0, 0, 0.2, 0.3, 0, 0.8, 0, 0, 0])
    center = solver.fk(start)['right']
    right = [i for i, name in enumerate(solver.joint_names) if name.startswith('right')]

    q, track = start.copy(), []
    for step in range(200):
        angle = 2.0 * np.pi * step / 200.0
        goal = center.copy()
        goal[1] += 0.06 * np.cos(angle) - 0.06
        goal[2] += 0.06 * np.sin(angle)
        q, pos_err, _, _ = solver.solve(q, {'right': goal})
        track.append(q[right].copy())

    jump = np.abs(np.diff(np.asarray(track), axis=0)).max()
    drift = np.abs(np.asarray(track)[-1] - start[right]).max()
    assert jump < 0.10, f'单帧关节跃变 {jump:.4f} rad 过大，零空间约束可能失效'
    assert drift < 0.35, f'绕回原点后位形漂移 {drift:.4f} rad，冗余自由度在游走'
    assert pos_err < 2e-3


def test_unreachable_target_settles_instead_of_chattering(ik):
    """够不着的目标保持不动时，解必须收敛到定点，不能持续抖。

    纯最小范数 DLS 在这里会残留 1.4e-2 rad 的循环抖动，50 Hz 下就是肉眼可见的嗡动。
    """
    goal = ik.fk(np.zeros(14))['right'].copy()
    goal[0] += 0.35
    q, track = np.zeros(14), []
    for _ in range(60):
        q, _, _, _ = ik.solve(q, {'right': goal})
        track.append(q.copy())
    assert np.abs(np.diff(np.asarray(track)[20:], axis=0)).max() < 1e-6
