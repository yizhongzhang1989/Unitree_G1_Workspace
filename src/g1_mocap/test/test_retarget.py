"""SMPL -> G1 重定向的单测。不需要头显，也不需要网络。

核心是 :func:`skeleton_from_pose` 这个**闭环**：拿 G1 自己的 FK 造一副"人体"骨架，
再让重定向去解它。解出来的关节角应该回到造它时用的那一组——这条过不了，实机上
只会看到一个动作幅度对不上、但哪儿都不报错的机器人。

闭环量的是**解算数学本身**。它量不到的是人机骨架定义的差异（人的肩髋中心固定在
躯干上，G1 的等效中心会随姿态微动），那一项由
:func:`test_fixed_joint_centers_degrade_gracefully` 单独兜住。
"""

from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from g1_mocap.kinematics import G1Kinematics
from g1_mocap.retarget import (
    ANKLE_LINKS,
    ARMS,
    LEGS,
    RetargetCalibration,
    Retargeter,
)
from g1_mocap.rotations import quat_from_mat, quat_to_mat
from g1_mocap.skeleton import JOINT_INDEX, SMPL_JOINTS, BodyFrame

URDF = str(Path(__file__).resolve().parents[2] / 'unitree_g1_description' / 'model'
           / 'g1_description' / 'g1_29dof_mode_15.urdf')

ACTION_JOINTS = (
    'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
    'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
    'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
    'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
    'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint',
    'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint',
    'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint',
    'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint',
    'right_wrist_yaw_joint',
)
KEY_BODIES = ('torso_link', 'left_wrist_yaw_link', 'right_wrist_yaw_link',
              'left_ankle_roll_link', 'right_ankle_roll_link')
DEFAULT_Q = np.array([-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
                      0.0, 0.0, 0.0, 0.35, 0.25, 0.0, 0.87, 0.0, 0.0, 0.0,
                      0.35, -0.25, 0.0, 0.87, 0.0, 0.0, 0.0])
SLOT = {name: i for i, name in enumerate(ACTION_JOINTS)}
# 绕肢体自身轴的自转从单个方向向量里解不出来，这四轴恒为 0，闭环里也别去转它们。
UNOBSERVABLE = ('left_ankle_roll_joint', 'right_ankle_roll_joint',
                'left_wrist_roll_joint', 'right_wrist_roll_joint')


@pytest.fixture(scope='module')
def kin() -> G1Kinematics:
    return G1Kinematics(URDF, ACTION_JOINTS)


@pytest.fixture(scope='module')
def retargeter(kin: G1Kinematics) -> Retargeter:
    return Retargeter(kin, key_bodies=KEY_BODIES, anchor_body='torso_link',
                      default_joint_pos=DEFAULT_Q)


def rot(axis: str, angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return {'x': np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
            'y': np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            'z': np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def skeleton_from_pose(kin: G1Kinematics, q29: np.ndarray, *, pelvis_pos: np.ndarray,
                       pelvis_rot: np.ndarray, frozen_centers: bool = False) -> np.ndarray:
    """用 G1 的 FK 造一副 24 关节骨架，摆位与 SMPL 的语义一一对应。

    脚尖和手尖是 SMPL 里有、G1 的 29 轴模型里没有的两个点，按末端 link 的 +x 轴外推——
    重定向那边正是把它们当作末端 link 的 x 轴来解的。

    ``frozen_centers`` 把肩髋中心钉在零位（相对父刚体固定），模拟真人的骨架：SMPL 的
    ``*_SHOULDER`` / ``*_HIP`` 不会随肢体姿态移动，而 G1 的 ``*_roll_link`` 会。
    """
    kin.key_body_pos(q29, ('pelvis',))
    rest = {}
    if frozen_centers:
        for spec in list(LEGS.values()) + list(ARMS.values()):
            parent = kin.frame_pos(spec.root_body)
            parent_rot = kin.frame_rot(spec.root_body)
            rest[spec.kin_links[0]] = (parent_rot.T @ (kin.frame_pos(spec.kin_links[0])
                                                       - parent), spec.root_body)
        kin.key_body_pos(np.zeros(len(q29)), ('pelvis',))
        rest = {link: kin.frame_rot(root).T @ (kin.frame_pos(link) - kin.frame_pos(root))
                for link, (_, root) in rest.items()}
        kin.key_body_pos(q29, ('pelvis',))

    def place(link: str, offset: np.ndarray | None = None) -> np.ndarray:
        local = kin.frame_pos(link)
        if offset is not None:
            local = local + kin.frame_rot(link) @ offset
        return pelvis_pos + pelvis_rot @ local

    def center(spec) -> np.ndarray:
        if not frozen_centers:
            return place(spec.kin_links[0])
        root = kin.frame_pos(spec.root_body)
        return pelvis_pos + pelvis_rot @ (root + kin.frame_rot(spec.root_body)
                                          @ rest[spec.kin_links[0]])

    torso_rot = pelvis_rot @ kin.frame_rot('torso_link')
    points = {
        'PELVIS': place('pelvis'),
        'SPINE1': place('pelvis') + pelvis_rot @ np.array([0.0, 0.0, 0.10]),
        'SPINE2': place('pelvis') + pelvis_rot @ np.array([0.0, 0.0, 0.02]),
        'SPINE3': place('torso_link'),
        'NECK': place('torso_link') + torso_rot @ np.array([0.0, 0.0, 0.20]),
        'HEAD': place('torso_link') + torso_rot @ np.array([0.0, 0.0, 0.35]),
        'LEFT_COLLAR': place('left_shoulder_pitch_link'),
        'RIGHT_COLLAR': place('right_shoulder_pitch_link'),
    }
    for side, spec in LEGS.items():
        prefix = 'LEFT' if side == 'left' else 'RIGHT'
        points[f'{prefix}_HIP'] = center(spec)
        points[f'{prefix}_KNEE'] = place(spec.kin_links[1])
        points[f'{prefix}_ANKLE'] = place(spec.kin_links[2])
        points[f'{prefix}_FOOT'] = place(spec.tip_link, np.array([0.12, 0.0, 0.0]))
    for side, spec in ARMS.items():
        prefix = 'LEFT' if side == 'left' else 'RIGHT'
        points[f'{prefix}_SHOULDER'] = center(spec)
        points[f'{prefix}_ELBOW'] = place(spec.kin_links[1])
        points[f'{prefix}_WRIST'] = place(spec.kin_links[2])
        points[f'{prefix}_HAND'] = place(spec.tip_link, np.array([0.08, 0.0, 0.0]))
    return np.array([points[name] for name in SMPL_JOINTS])


def identity_calibration(pelvis_z: float) -> RetargetCalibration:
    """不做任何人机对齐，把缩放、姿态偏置、零位映射三路都摘掉。

    闭环要量的是解算数学本身，标定那一层得先拿开。
    """
    return replace(RetargetCalibration.identity(pelvis_z), pelvis_ref_z=pelvis_z)


def round_trip(kin, retargeter, q29, *, pelvis_rot=None, frozen_centers=False):
    pelvis_rot = np.eye(3) if pelvis_rot is None else pelvis_rot
    pelvis_pos = np.array([0.4, -0.2, 0.78])
    positions = skeleton_from_pose(kin, q29, pelvis_pos=pelvis_pos, pelvis_rot=pelvis_rot,
                                   frozen_centers=frozen_centers)
    frame = BodyFrame(t=0.0, seq=0, positions=positions, status=1, message=0)
    return retargeter.solve(frame, identity_calibration(pelvis_pos[2])), pelvis_pos, pelvis_rot


def random_pose(kin, seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    lower, upper = kin.limits()
    q = rng.uniform(np.maximum(lower, -1.0), np.minimum(upper, 1.0))
    for name in UNOBSERVABLE:
        q[SLOT[name]] = 0.0
    return q


##
# 零位常量
##

def test_zero_pose_round_trips(kin, retargeter):
    """零位是最基本的一档：解出来必须全接近 0，不然零位常量就取错了。

    残差 0.4 度是 G1 大腿本身的侧倾——它让"三点夹角"和"绕膝轴转过的角"差了这么多，
    只用位置的话消不掉。
    """
    result, _, _ = round_trip(kin, retargeter, np.zeros(29))
    assert np.max(np.abs(result.joint_pos)) < 0.01


def test_default_pose_round_trips(kin, retargeter):
    result, _, _ = round_trip(kin, retargeter, DEFAULT_Q)
    error = np.abs(result.joint_pos - DEFAULT_Q)
    assert np.max(error) < 0.03, dict(zip(ACTION_JOINTS, np.round(error, 4)))


def test_elbow_zero_bend_is_not_zero(kin, retargeter):
    """G1 零位不是人的立正：大臂垂下、小臂前伸，肘的零位弯角接近 80 度。

    照着"人伸直手臂 = 关节角 0"写死，整条手臂会差 80 度，而且不会报任何错。
    """
    hinge = retargeter._arms['left'].hinge
    assert math.degrees(hinge.bend0) == pytest.approx(79.42, abs=0.1)
    assert hinge.slope < 0  # 关节角变大 = 手臂伸直
    assert retargeter._legs['left'].hinge.bend0 == pytest.approx(0.0, abs=1e-3)


def test_ball_offsets_capture_the_tilted_axes(retargeter):
    """髋 roll 轴前倾 10 度、肩 roll 轴外倾 16 度，这两个偏置漏掉就是静默错角。"""
    assert math.degrees(retargeter._legs['left'].ball.offsets[0]) == pytest.approx(
        -10.02, abs=0.05)
    assert math.degrees(retargeter._arms['left'].ball.offsets[1]) == pytest.approx(
        -16.0, abs=0.05)
    assert math.degrees(retargeter._legs['left'].hinge.placement_offset) == pytest.approx(
        10.02, abs=0.05)


##
# 闭环精度
##

@pytest.mark.parametrize('seed', range(8))
def test_random_pose_round_trips(kin, retargeter, seed):
    """随机位形。容差按自由度分档，因为这套解法有两处**结构性**近似：

    * 髋/肩当成理想球窝，实际三根轴之间隔着几厘米——偏航那一路最受影响；
    * 腕的三个轴串联跨了 8.9 cm，而人只有一个腕点，pitch/yaw 只能近似。
    """
    q = random_pose(kin, seed)
    result, _, _ = round_trip(kin, retargeter, q, pelvis_rot=rot('z', 0.7) @ rot('y', 0.15))
    error = np.abs(result.joint_pos - q)
    report = dict(zip(ACTION_JOINTS, np.round(error, 4)))

    def worst(*names: str) -> float:
        return float(np.max([error[SLOT[n]] for n in names]))

    assert worst(*UNOBSERVABLE) == 0.0, report
    assert worst('left_knee_joint', 'right_knee_joint',
                 'left_elbow_joint', 'right_elbow_joint') < 0.25, report
    assert worst('waist_roll_joint', 'waist_pitch_joint') < 0.10, report
    assert worst('left_hip_pitch_joint', 'left_hip_roll_joint',
                 'right_hip_pitch_joint', 'right_hip_roll_joint',
                 'left_shoulder_pitch_joint', 'left_shoulder_roll_joint',
                 'right_shoulder_pitch_joint', 'right_shoulder_roll_joint') < 0.30, report
    assert worst('left_hip_yaw_joint', 'right_hip_yaw_joint', 'waist_yaw_joint',
                 'left_shoulder_yaw_joint', 'right_shoulder_yaw_joint') < 0.35, report
    assert worst('left_wrist_pitch_joint', 'left_wrist_yaw_joint',
                 'right_wrist_pitch_joint', 'right_wrist_yaw_joint') < 0.60, report


def test_round_trip_error_distribution(kin, retargeter):
    """整体分布。只看单个位形的最坏值容易被某一次的奇异姿态带偏。"""
    errors = []
    for seed in range(40):
        q = random_pose(kin, seed)
        result, _, _ = round_trip(kin, retargeter, q,
                                  pelvis_rot=rot('z', 0.7) @ rot('y', 0.15))
        errors.append(np.abs(result.joint_pos - q))
    errors = np.array(errors)
    assert np.percentile(errors, 50) < 0.04
    assert np.percentile(errors, 95) < 0.30


def test_fixed_joint_centers_degrade_gracefully(kin, retargeter):
    """真人的肩髋中心不随肢体动，G1 的等效中心会——这一项闭环量不到，单独兜住。

    数值明显比自洽闭环差，是**结构差异**不是 bug。真要压下去只能上数值 IK，
    那是另一个量级的复杂度和另一类失效模式（不收敛）。
    """
    errors = []
    for seed in range(20):
        q = random_pose(kin, seed)
        result, _, _ = round_trip(kin, retargeter, q, pelvis_rot=rot('z', 0.4),
                                  frozen_centers=True)
        errors.append(np.abs(result.joint_pos - q))
    errors = np.array(errors)
    assert np.percentile(errors, 50) < 0.20
    assert errors.max() < 1.6


##
# 根位姿与 key body
##

def test_root_pose_is_recovered(kin, retargeter):
    """根位姿必须原样还原：它决定参考落在世界的哪儿，错了就是整段参考平移/转向。"""
    pelvis_rot = rot('z', -0.9) @ rot('y', 0.2) @ rot('x', -0.1)
    result, pelvis_pos, _ = round_trip(kin, retargeter, DEFAULT_Q, pelvis_rot=pelvis_rot)
    assert np.allclose(result.root_pos, pelvis_pos, atol=1e-9)
    assert np.max(np.abs(quat_from_mat(pelvis_rot) - result.root_quat)) < 1e-6


def test_key_bodies_come_from_g1_fk(kin, retargeter):
    """key body 必须是 G1 的 FK 算出来的，不能直接拿人的关节点顶替。

    人机肢长不同，拿人的点当 key body 会有系统偏差，而且和输出的关节角互相矛盾——
    策略读到的参考就成了它做不到的姿态。
    """
    result, pelvis_pos, pelvis_rot = round_trip(kin, retargeter, DEFAULT_Q)
    expected = pelvis_pos + kin.key_body_pos(result.joint_pos, KEY_BODIES) @ pelvis_rot.T
    assert np.allclose(result.key_pos, expected, atol=1e-9)
    assert np.allclose(result.anchor_pos, expected[0], atol=1e-9)


##
# 校准
##

def test_calibration_scales_by_leg_length(kin, retargeter):
    """人比 G1 大一圈时，缩放要把位移收回来，站立高度锚到 G1 自己的高度。"""
    pelvis_pos = np.array([0.0, 0.0, 0.78])
    positions = skeleton_from_pose(kin, np.zeros(29), pelvis_pos=pelvis_pos,
                                   pelvis_rot=np.eye(3))
    frames = [BodyFrame(t=0.0, seq=0, positions=positions * 1.6, status=1, message=0)] * 30
    calibration = retargeter.calibrate(frames)
    assert calibration.scale == pytest.approx(1.0 / 1.6, rel=1e-6)

    result = retargeter.solve(frames[0], calibration)
    assert result.root_pos[2] == pytest.approx(retargeter.stand_height, abs=1e-9)


def test_calibration_rejects_a_collapsed_skeleton(retargeter):
    positions = np.zeros((len(SMPL_JOINTS), 3))
    frames = [BodyFrame(t=0.0, seq=0, positions=positions, status=1, message=0)] * 30
    with pytest.raises(ValueError, match='站立姿态'):
        retargeter.calibrate(frames)


def test_ankle_bias_cancels_a_toe_down_skeleton(kin, retargeter):
    """SMPL 的 *_FOOT 是脚趾根部，站直时本就朝前下方；不扣掉就是一直勾着脚。"""
    pelvis_pos = np.array([0.0, 0.0, 0.78])
    positions = skeleton_from_pose(kin, np.zeros(29), pelvis_pos=pelvis_pos,
                                   pelvis_rot=np.eye(3))
    for name in ('LEFT_FOOT', 'RIGHT_FOOT'):
        positions[JOINT_INDEX[name]] += np.array([0.0, 0.0, -0.05])
    frames = [BodyFrame(t=0.0, seq=0, positions=positions, status=1, message=0)] * 30

    naive = retargeter.solve(frames[0], identity_calibration(pelvis_pos[2]))
    assert abs(naive.joint_pos[SLOT['left_ankle_pitch_joint']]) > 0.2

    # 标定之后站立位形整体落到 default 上，勾脚那一份偏置跟着被吸掉。
    calibrated = retargeter.solve(frames[0], retargeter.calibrate(frames))
    assert calibrated.joint_pos[SLOT['left_ankle_pitch_joint']] == pytest.approx(
        DEFAULT_Q[SLOT['left_ankle_pitch_joint']], abs=1e-6)


def test_calibration_maps_the_stand_pose_onto_the_default(kin, retargeter):
    """校准帧解出来的位形必须恰好落在 G1 的 default 上。

    这是站立零位映射的全部意义：人站直时策略读到的参考就是它最熟的那个姿态，
    而不是一个大腿后摆 20 度、两腿外撑 35 度的、训练集里一次都没出现过的位形。
    """
    pelvis_pos = np.array([0.0, 0.0, 0.78])
    # 正是实机量到的那两项偏置：盆骨前倾（弄歪 SPINE1）+ 脚外八。
    positions = skeleton_from_pose(kin, np.zeros(29), pelvis_pos=pelvis_pos,
                                   pelvis_rot=np.eye(3))
    positions[JOINT_INDEX['SPINE1']] += np.array([0.04, 0.0, 0.0])
    for name, sign in (('LEFT_FOOT', 1.0), ('RIGHT_FOOT', -1.0)):
        positions[JOINT_INDEX[name]] += np.array([-0.02, sign * 0.05, -0.03])
    frames = [BodyFrame(t=0.0, seq=0, positions=positions, status=1, message=0)] * 30

    calibration = retargeter.calibrate(frames)
    result = retargeter.solve(frames[0], calibration)
    assert np.allclose(result.joint_pos, DEFAULT_Q, atol=1e-6)


def test_pose_fix_uprights_a_tilted_pelvis(kin, retargeter):
    """盆骨前倾会污染 ``projected_gravity``，光改关节角修不掉。

    修完之后校准帧的根朝向只剩偏航，俯仰与横滚归零。
    """
    pelvis_pos = np.array([0.0, 0.0, 0.78])
    positions = skeleton_from_pose(kin, np.zeros(29), pelvis_pos=pelvis_pos,
                                   pelvis_rot=rot('z', 0.6))
    positions[JOINT_INDEX['SPINE1']] += rot('z', 0.6) @ np.array([0.05, 0.0, 0.0])
    frames = [BodyFrame(t=0.0, seq=0, positions=positions, status=1, message=0)] * 30

    tilted = retargeter.solve(frames[0], identity_calibration(pelvis_pos[2]))
    gravity = quat_to_mat(tilted.root_quat).T @ np.array([0.0, 0.0, -1.0])
    assert abs(gravity[0]) > 0.2  # 前倾把重力甩到了 x 上

    fixed = retargeter.solve(frames[0], retargeter.calibrate(frames))
    gravity = quat_to_mat(fixed.root_quat).T @ np.array([0.0, 0.0, -1.0])
    assert np.allclose(gravity, [0.0, 0.0, -1.0], atol=1e-6)


##
# 边界
##

def test_output_is_clipped_to_joint_limits(kin, retargeter):
    """人能做到的角度 G1 未必做得到（比如完全伸直手臂），越界必须裁住。"""
    lower, upper = kin.limits()
    for seed in range(10):
        q = random_pose(kin, seed)
        result, _, _ = round_trip(kin, retargeter, q)
        assert np.all(result.joint_pos >= lower - 1e-9)
        assert np.all(result.joint_pos <= upper + 1e-9)


def test_straight_limbs_do_not_flip_the_hinge_axis(kin, retargeter):
    """腿伸直时膝轴的叉乘退化，兜底轴取错会让髋偏航凭空跳 90 度。"""
    for knee in (0.0, 0.005, 0.02, 0.05, 0.2):
        q = np.zeros(29)
        q[SLOT['left_knee_joint']] = knee
        q[SLOT['right_knee_joint']] = knee
        result, _, _ = round_trip(kin, retargeter, q)
        assert abs(result.joint_pos[SLOT['left_hip_yaw_joint']]) < 0.05, knee


def test_ankle_links_exist_in_the_model(kin):
    """名单打错字不会报错，只会让站立高度悄悄取到别的刚体上。"""
    for link in ANKLE_LINKS + KEY_BODIES:
        assert np.all(np.isfinite(kin.frame_pos(link)))


def test_wrong_joint_names_are_rejected():
    with pytest.raises(ValueError, match='没有这些关节'):
        G1Kinematics(URDF, ('left_hip_pitch_joint', 'nonexistent_joint'))
