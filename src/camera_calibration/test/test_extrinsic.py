import cv2
import numpy as np
import pytest

from camera_calibration import extrinsic, transforms


def pose(rvec, translation):
    return transforms.rt_to_matrix(cv2.Rodrigues(np.asarray(rvec, float))[0],
                                   np.asarray(translation, float))


T_BASE_REF = pose([0.0, 0.83, 0.0], [0.058, 0.018, 0.430])
T_BASE_BOARD = pose([2.9, 0.2, 0.1], [0.55, -0.05, 0.15])
T_LINK_CAM = pose([0.10, -1.55, 0.02], [0.031, -0.008, 0.046])

ARM_POSES = [
    pose([0.20, 0.10, -0.30], [0.34, 0.19, 0.24]),
    pose([-0.35, 0.42, 0.18], [0.39, 0.13, 0.31]),
    pose([0.55, -0.24, 0.40], [0.30, 0.25, 0.19]),
    pose([-0.15, -0.50, -0.22], [0.42, 0.16, 0.28]),
    pose([0.31, 0.62, 0.11], [0.36, 0.21, 0.22]),
    pose([-0.48, 0.05, 0.35], [0.33, 0.11, 0.33]),
]


def build(arm_poses=None, base_ref=None, link_cam=None):
    base_ref = T_BASE_REF if base_ref is None else base_ref
    link_cam = T_LINK_CAM if link_cam is None else link_cam
    samples = []
    for arm in (arm_poses or ARM_POSES):
        samples.append({
            'T_base_link': arm,
            'T_base_ref': base_ref,
            'T_ref_board': transforms.invert(base_ref) @ T_BASE_BOARD,
            'T_cam_board': transforms.invert(link_cam) @ transforms.invert(arm)
            @ T_BASE_BOARD,
        })
    return samples


def test_reference_method_recovers_the_transform():
    result = extrinsic.solve_from_reference(build())
    assert result['ok']
    angle, distance = transforms.transform_delta(
        T_LINK_CAM, np.asarray(result['matrix']))
    assert angle < 1e-6
    assert distance < 1e-9
    assert result['spread']['angle_max_deg'] < 1e-6
    assert result['consistency']['trans_max_mm'] < 1e-6


def test_handeye_recovers_the_same_transform():
    result = extrinsic.solve_handeye(build())
    assert result['ok'], result.get('reason')
    angle, distance = transforms.transform_delta(
        T_LINK_CAM, np.asarray(result['matrix']))
    assert angle < 1e-6
    assert distance < 1e-9
    assert set(result['per_method']) == {'tsai', 'park', 'horaud', 'daniilidis'}


def test_handeye_needs_enough_poses():
    result = extrinsic.solve_handeye(build(ARM_POSES[:2]))
    assert not result['ok']
    assert '姿态' in result['reason']


def test_handeye_rejects_parallel_rotation_axes():
    # 只绕同一根轴转，AX=XB 的旋转部分欠定；不拦住的话会安静地给出一个错解
    poses = [pose([0.0, 0.0, angle], [0.35, 0.18, 0.25])
             for angle in (0.0, 0.3, 0.6, 0.9)]
    result = extrinsic.solve_handeye(build(poses))
    assert not result['ok']
    assert '平行' in result['reason']


def test_wrong_reference_extrinsic_shows_up_in_the_cross_check():
    """头部外参偏 3°，参考相机法会跟着偏，AX=XB 不受影响 —— 这就是交叉验证的意义"""
    biased = T_BASE_REF @ pose([0.0, np.deg2rad(3.0), 0.0], [0.0, 0.0, 0.0])
    samples = build()
    # 只换 TF 报的值。T_ref_board 是相机实拍出来的，仍然对应真实光学系 ——
    # 两边都换的话偏差自己抵消了，就测不出东西
    for sample in samples:
        sample['T_base_ref'] = biased

    reference = extrinsic.solve_from_reference(samples)
    handeye = extrinsic.solve_handeye(samples)

    # AX=XB 不碰头部外参，仍然精确
    assert transforms.transform_delta(
        T_LINK_CAM, np.asarray(handeye['matrix']))[0] < 1e-6

    # 参考相机法整体偏了，量级就是头部的 3°。没有恰好 3° 是因为每个手臂姿态
    # 把这 3° 折到了不同的轴上，取均值会把它摊薄一点
    assert 2.0 < extrinsic.compare(reference['matrix'], T_LINK_CAM)['angle_deg'] < 3.1

    # 各姿态解出来的结果互相对不上 —— 光看参考相机法自己就能闻到味道，
    # 前提是手臂姿态拉得开；只拍一张的话这个信号根本不存在
    assert reference['spread']['angle_max_deg'] > 0.5

    bias = extrinsic.reference_bias(samples, np.asarray(handeye['matrix']))
    assert bias['delta']['angle_deg'] == pytest.approx(3.0, abs=1e-3)
    assert bias['spread']['angle_max_deg'] < 1e-6


def test_board_consistency_grows_with_a_wrong_transform():
    samples = build()
    wrong = T_LINK_CAM @ pose([0.0, 0.0, np.deg2rad(2.0)], [0.0, 0.0, 0.0])
    good = extrinsic.board_consistency(samples, T_LINK_CAM)
    bad = extrinsic.board_consistency(samples, wrong)
    assert good['trans_rms_mm'] < 1e-6
    assert bad['trans_rms_mm'] > 1.0


def test_board_consistency_is_pairwise_not_a_spread():
    """板挪过时，外参完全正确也该是 0 —— 逐组和头部配对比，不是看各组之间的离散。

    实机上踩过：用离散度当指标，最小二乘残差 3.2 mm 却报出 73 mm。
    """
    moving = build_moving_board()
    assert extrinsic.board_consistency(moving, T_LINK_CAM)['trans_rms_mm'] < 1e-6

    # 各组反推的板位姿本身确实差得很远（板真的挪了），但那不是外参的误差
    poses = [np.asarray(s['T_base_link']) @ T_LINK_CAM @ np.asarray(s['T_cam_board'])
             for s in moving]
    assert transforms.spread(poses)['trans_rms_mm'] > 50


def test_board_consistency_uses_the_corrected_head():
    """头部偏了时，拿 TF 原值比会有残差，拿修正后的比应该归零"""
    biased = T_BASE_REF @ pose([0.0, np.deg2rad(4.0), 0.0], [0, 0, 0])
    samples = build_moving_board()
    for sample in samples:
        sample['T_base_ref'] = biased
    correction = transforms.invert(biased) @ T_BASE_REF

    assert extrinsic.board_consistency(samples, T_LINK_CAM)['trans_rms_mm'] > 5
    assert extrinsic.board_consistency(
        samples, T_LINK_CAM, correction)['trans_rms_mm'] < 1e-6


def test_empty_input():
    assert not extrinsic.solve_from_reference([])['ok']


# ---------- 板每组都挪的情形 ----------

def build_moving_board(arm_poses=None, base_ref=None, link_cam=None, seed=3):
    """每组都把板挪到别处。头部每组重测一次，所以参考相机法和联合最小二乘仍然成立。"""
    base_ref = T_BASE_REF if base_ref is None else base_ref
    link_cam = T_LINK_CAM if link_cam is None else link_cam
    rng = np.random.default_rng(seed)
    samples = []
    for arm in (arm_poses or ARM_POSES):
        board = pose(rng.uniform(-0.6, 0.6, 3) + np.array([2.9, 0.2, 0.1]),
                     rng.uniform(-0.15, 0.15, 3) + np.array([0.55, -0.05, 0.15]))
        samples.append({
            'T_base_link': arm,
            'T_base_ref': base_ref,
            'T_ref_board': transforms.invert(base_ref) @ board,
            'T_cam_board': transforms.invert(link_cam) @ transforms.invert(arm) @ board,
        })
    return samples


def test_board_stability_detects_a_moved_board():
    assert extrinsic.board_stability(build())['fixed']

    moved = extrinsic.board_stability(build_moving_board())
    assert not moved['fixed']
    assert moved['trans_max_mm'] > extrinsic.BOARD_FIXED_TRANS_MM


def test_joint_least_squares_works_with_a_moving_board():
    """板每组都挪 —— AX=XB 直接欠定，联合最小二乘照样精确"""
    samples = build_moving_board()
    result = extrinsic.solve_joint(samples)
    assert result['ok']
    angle, distance = transforms.transform_delta(T_LINK_CAM,
                                                 np.asarray(result['matrix']))
    assert angle < 1e-4
    assert distance < 1e-6
    assert result['residual_mm'] < 1e-3
    assert result['reference_correction']['angle_deg'] < 1e-4


def test_joint_recovers_a_biased_head_even_with_a_moving_board():
    """头部外参偏 3°：板动了没法靠 AX=XB 交叉验证，但联合解能把偏差顶出来"""
    biased = T_BASE_REF @ pose([0.0, np.deg2rad(3.0), 0.0], [0.0, 0.0, 0.0])
    samples = build_moving_board()
    for sample in samples:                       # 只换 TF 报的值，测量仍对应真实光学系
        sample['T_base_ref'] = biased

    result = extrinsic.solve_joint(samples)
    assert result['ok'] and result['well_posed']
    assert transforms.transform_delta(
        T_LINK_CAM, np.asarray(result['matrix']))[0] < 1e-2
    # 修正量把 biased 拉回真值：biased @ correction == T_BASE_REF
    correction = transforms.matrix_from_dict(result['reference_correction'])
    assert transforms.transform_delta(biased @ correction, T_BASE_REF)[0] < 1e-2
    assert result['reference_correction']['angle_deg'] == pytest.approx(3.0, abs=1e-2)


def test_joint_matches_averaging_when_only_solving_x():
    """只估 X 时，位姿空间等权最小二乘数学上就是在求均值 —— 别宣称它更准。

    联合解真正多出来的能力是能把头部修正量一起估（见上一条测试）。
    """
    rng = np.random.default_rng(11)
    samples = build_moving_board(seed=5)
    for sample in samples:
        sample['T_cam_board'] = transforms.perturb(
            sample['T_cam_board'], rng.normal(0, 0.004, 6))

    averaged = extrinsic.solve_from_reference(samples)
    joint = extrinsic.solve_joint(samples, refine_reference=False)
    delta = transforms.transform_delta(np.asarray(averaged['matrix']),
                                       np.asarray(joint['matrix']))
    assert delta[0] < 0.5 and delta[1] < 1e-3


def test_joint_needs_two_poses_for_the_head_correction():
    result = extrinsic.solve_joint(build_moving_board(ARM_POSES[:1]))
    assert not result['ok']
    assert '2 组' in result['reason']


def test_head_error_does_not_leak_into_x_when_delta_h_is_free():
    """头部外参偏 5°：把 ΔH 放开之后，X 完全不受影响。

    代数上：ΔH = Cᵢ·X·Dᵢ 对每组都成立，两组相减后 T_base_ref 被约掉，
    剩下 (T_base_link_j⁻¹ T_base_link_i)·X = X·(...)，一个不含头部外参的 AX=XB。
    所以「头不准」和「解不出腕相机外参」是两回事。
    """
    biased = T_BASE_REF @ pose([0.0, np.deg2rad(5.0), 0.0], [0.0, 0.0, 0.0])
    samples = build_moving_board()
    for sample in samples:
        sample['T_base_ref'] = biased

    def error(matrix):
        return transforms.transform_delta(T_LINK_CAM, np.asarray(matrix))[0]

    reference = error(extrinsic.solve_from_reference(samples)['matrix'])
    trusting = error(extrinsic.solve_joint(samples, refine_reference=False)['matrix'])
    free = error(extrinsic.solve_joint(samples, refine_reference=True)['matrix'])

    # 相信头部的两条路都被那 5° 带偏
    assert reference > 1.0
    assert trusting > 1.0
    # 放开 ΔH 之后精确还原
    assert free < 1e-2
    assert free < reference / 100


def test_joint_flags_parallel_rotation_axes_as_ill_posed():
    """只绕一根轴转的话，12 个未知量撑不起来 —— 得报出来而不是给个错解"""
    poses = [pose([0.0, 0.0, angle], [0.35, 0.18, 0.25])
             for angle in (0.0, 0.3, 0.6, 0.9)]
    result = extrinsic.solve_joint(build_moving_board(poses))

    assert result['ok']
    assert not result['well_posed']
    assert result['condition'] > 1e4


# ---------- 三个相机一起解 ----------

T_RIGHT_CAM = pose([-0.08, 1.60, -0.05], [0.028, 0.011, 0.043])
RIGHT_ARM = [pose([0.30, -0.20, 0.25], [0.33, -0.20, 0.26]),
             pose([-0.40, 0.35, -0.15], [0.38, -0.14, 0.32]),
             pose([0.50, 0.28, -0.35], [0.31, -0.24, 0.20]),
             pose([-0.22, -0.45, 0.30], [0.41, -0.17, 0.29]),
             pose([0.35, 0.55, 0.12], [0.35, -0.21, 0.23]),
             pose([-0.50, 0.08, -0.30], [0.34, -0.12, 0.31])]


def build_groups(head_bias_deg=0.0, moving=True):
    """左右两只手各一组样本，共用同一个（可能偏了的）头部外参"""
    biased = T_BASE_REF @ pose([0.0, np.deg2rad(head_bias_deg), 0.0], [0, 0, 0])
    groups = {}
    for name, arms, cam, seed in (('camera_left', ARM_POSES, T_LINK_CAM, 3),
                                  ('camera_right', RIGHT_ARM, T_RIGHT_CAM, 9)):
        rows = (build_moving_board(arms, link_cam=cam, seed=seed) if moving
                else build(arms, link_cam=cam))
        for row in rows:                    # TF 报的是偏了的，测量仍来自真实光学系
            row['T_base_ref'] = biased
        groups[name] = rows
    return groups


@pytest.mark.parametrize('moving', [True, False])
def test_solve_all_recovers_both_arms_and_the_head(moving):
    """板动不动都行：M ʰ 是实测量不是自由参数，所以头部外参始终可辨识"""
    groups = build_groups(head_bias_deg=4.0, moving=moving)
    result = extrinsic.solve_all(groups)

    assert result['ok'] and result['well_posed']
    assert transforms.transform_delta(
        T_LINK_CAM, np.asarray(result['cameras']['camera_left']['matrix']))[0] < 1e-2
    assert transforms.transform_delta(
        T_RIGHT_CAM, np.asarray(result['cameras']['camera_right']['matrix']))[0] < 1e-2
    assert result['reference_correction']['angle_deg'] == pytest.approx(4.0, abs=1e-2)
    # 修正后的绝对头部外参应当等于真值
    assert transforms.transform_delta(
        transforms.matrix_from_dict(result['reference_absolute']), T_BASE_REF)[0] < 1e-2


def test_solve_all_gives_one_head_answer_instead_of_two():
    """分开解会得到两个不一样的 ΔH，一起解只有一个"""
    groups = build_groups(head_bias_deg=4.0)
    for rows in groups.values():            # 加噪声，让两侧各自被带偏一点
        rng = np.random.default_rng(4)
        for row in rows:
            row['T_cam_board'] = transforms.perturb(row['T_cam_board'],
                                                    rng.normal(0, 0.003, 6))

    separate = [extrinsic.solve_joint(rows)['reference_correction']['angle_deg']
                for rows in groups.values()]
    assert abs(separate[0] - separate[1]) > 1e-6      # 两个值确实不一样

    combined = extrinsic.solve_all(groups)['reference_correction']['angle_deg']
    assert min(separate) - 0.5 <= combined <= max(separate) + 0.5


def test_solve_all_uses_poses_where_only_one_arm_sees_the_board():
    """只有一只手看得见板的组也能给 ΔH 出力"""
    groups = build_groups(head_bias_deg=4.0)
    groups['camera_right'] = groups['camera_right'][:2]

    result = extrinsic.solve_all(groups)
    assert result['ok']
    assert result['cameras']['camera_right']['samples'] == 2
    assert transforms.transform_delta(
        T_RIGHT_CAM, np.asarray(result['cameras']['camera_right']['matrix']))[0] < 1e-2


def test_solve_all_needs_samples():
    assert not extrinsic.solve_all({})['ok']
    assert not extrinsic.solve_all({'camera_left': []})['ok']
