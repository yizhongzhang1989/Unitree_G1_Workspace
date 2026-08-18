"""arm_ik 的离线校验。不需要真机，也不需要 ROS。

这里只测和实机安全直接相关的四件事：缩减模型确实只剩 14 轴且 q 顺序可信、
正逆运动学互为反解、够不着时不抛异常且解仍在 URDF 限位内、单侧求解不动另一侧。
"""

import math
from pathlib import Path

import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from g1_motion_control.arm_ik import ArmIK

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


def _config():
    share = Path(get_package_share_directory('g1_motion_control'))
    return yaml.safe_load((share / 'config' / 'motion_control.yaml')
                          .read_text(encoding='utf-8'))['/motion_control']['ros__parameters']


def STAND_POSTURE(ik):
    """STAND 结束时手臂停的位形（passive_targets 的手臂 14 位），也是接管的起点。"""
    config = _config()
    by_name = dict(zip(config['arm_joints'], config['passive_targets'][:14]))
    return np.array([by_name[name] for name in ik.joint_names])


def _configured_ik(null_gain=None, null_target=None):
    """按 config/motion_control.yaml 里实际发布的参数建，用来卡住部署值本身。

    库默认值和实际跑的那一套不是一回事，只测默认值等于没测。
    """
    config = _config()
    urdf = (Path(get_package_share_directory('unitree_g1_description'))
            / 'model' / 'final.urdf')
    arms = config['arm_joints']
    if null_gain is None:
        null_gain = {name: gain
                     for name, gain in zip(arms, config['ik_null_gain']) if gain}
    if null_target is None:
        null_target = {name: target
                       for name, target in zip(arms, config.get('ik_null_target', []))
                       if target}
    return ArmIK(
        urdf.read_text(encoding='utf-8'), arms, TIP_FRAMES,
        base_frame=config['base_frame'], max_iters=config['ik_max_iters'],
        damping=config['ik_damping'], tol_pos=config['ik_tol_pos'],
        tol_ori=config['ik_tol_ori'],
        max_step_pos=config['ik_max_step_pos'], max_step_ori=config['ik_max_step_ori'],
        rotation_weight=config['ik_rotation_weight'],
        joint_limits={name: (-math.inf, high)
                      for name, high in zip(arms, config['ik_limit_upper'])},
        null_gain=null_gain,
        null_target=null_target,
        null_gate=tuple(config['ik_null_gate']))


@pytest.fixture(scope='module')
def configured_ik():
    return _configured_ik()


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


def test_rotation_weight_zero_releases_orientation_and_updates_live(ik):
    seed = np.zeros(14)
    target = ik.fk(seed)['right'].copy()
    half = math.sin(math.pi / 8.0)
    target[3:7] = [0.0, 0.0, half, math.cos(math.pi / 8.0)]

    ik.set_rotation_weight(0.0)
    position_only, pos_err, ori_err, iters = ik.solve(seed, {'right': target})
    assert np.array_equal(position_only, seed)
    assert pos_err == pytest.approx(0.0)
    assert ori_err > 0.5
    assert iters == 0

    ik.set_rotation_weight(1.0)
    full_pose, _, full_ori_err, iters = ik.solve(seed, {'right': target})
    assert not np.allclose(full_pose, seed)
    assert full_ori_err < ori_err
    assert iters > 0


def test_rotation_weight_rejects_invalid_values(ik):
    for bad in (-0.01, 1.01, math.nan, math.inf):
        with pytest.raises(ValueError):
            ik.set_rotation_weight(bad)


def test_redundant_dof_does_not_drift_along_a_closed_path(configured_ik):
    """末端绕闭合路径回到原点后，冗余自由度必须也回到原处。

    手臂 7 自由度、任务 6 维，多出来的那一维完全由种子定。这里验的是求解本身：
    种子恒定、目标绕回原处时，位形必须也回到原处，不能自己往一个方向爬。
    （节点实际用的是热启动，那一支的累积漂移由 ik_rescue_err 和 arm_rate_limit 管，
    见 test_arm_pipeline_* 那几个用例。）
    """
    ik = configured_ik
    seed = STAND_POSTURE(ik)
    center = ik.fk(seed)['right']
    yaw = ik.joint_names.index('right_shoulder_yaw_joint')

    track = []
    for k in range(300):                      # 3 整圈
        angle = 2 * np.pi * k / 100
        target = dict(ik.fk(seed))
        target['right'] = center.copy()
        target['right'][1] += 0.03 * np.cos(angle) - 0.03
        target['right'][2] += 0.03 * np.sin(angle)
        q, _, _, _ = ik.solve(seed, target)   # 种子恒定，解不回灌
        track.append(q.copy())

    track = np.asarray(track)
    assert abs(track[-1, yaw] - track[100, yaw]) < 0.02, '绕整圈回来后 shoulder_yaw 漂了'
    assert np.abs(np.diff(track[100:], axis=0)).max() < 0.05, '单帧跳变过大'


def test_unreachable_target_settles_instead_of_chattering(ik):
    """够不着的目标保持不动时，解必须逐帧一致，不能持续抖。

    按节点的实际用法调用：种子恒定，解不回灌。目标恒定 + 种子恒定 => 输出必须恒定。
    """
    seed = np.zeros(14)
    goal = ik.fk(seed)['right'].copy()
    goal[0] += 0.35
    track = [ik.solve(seed, {'right': goal})[0] for _ in range(60)]
    assert np.abs(np.diff(np.asarray(track)[20:], axis=0)).max() < 1e-9


def test_far_unreachable_target_does_not_blow_up_the_step(ik):
    """够得**很远**的不可达目标：步长必须被限住，否则关节会顶穿限位再弹回来。

    DLS 的步长正比于误差（最坏增益 1/(2λ)=10 rad/m），所以"够不着"够狠时会失稳。
    上面那个用例只推了 0.35 m，落在阈值之内；这里推 0.60 m，实测不限幅时稳态每帧
    抖 5.76 rad，限幅后是 1e-14 量级。
    """
    here = ik.fk(np.zeros(14))
    goal = {side: pose.copy() for side, pose in here.items()}
    for pose in goal.values():
        pose[0] += 0.60
    seed = np.zeros(14)
    track = []
    for _ in range(120):
        q, _, _, _ = ik.solve(seed, goal)
        track.append(q.copy())
    assert np.abs(np.diff(np.asarray(track)[60:], axis=0)).max() < 1e-9
    assert np.all(np.isfinite(q))
    assert np.all(q >= ik.lower - 1e-9) and np.all(q <= ik.upper + 1e-9)


def test_warm_start_never_gets_trapped_past_the_elbow_singularity(configured_ik):
    """热启动 + 肘部限位必须能扛住"推远-拉回"，这是那条限位存在的全部理由。

    G1 的肘角不是"0 伸直、越大越弯"：肩到夹爪的距离在 elbow=1.571 处最大，那里才是完全
    伸直、也是奇异点，而 URDF 行程 [-1.047, 2.094] 把它夹在中间。**拿上一帧的解做种子**
    时，目标够不着会把肘推过 1.571 顶到 2.094（手臂看起来是直的），之后再也回不来——
    实测 8 轮里卡死 4 轮、残差 39 mm。把肘上限收到 1.4 后 0/8，代价只有 0.3 mm 可达半径。
    """
    ik = configured_ik
    seed = STAND_POSTURE(ik)
    home = ik.fk(seed)
    far = {side: pose.copy() for side, pose in home.items()}
    for pose in far.values():
        pose[0] += 0.50                       # 推到够不着的前方
    q = seed.copy()
    for _ in range(4):
        for _ in range(40):
            q, _, _, _ = ik.solve(q, far)     # 热启动：解回灌进下一帧
        for _ in range(60):
            q, pos_err, _, _ = ik.solve(q, home)
        assert pos_err < 5e-3, f'拉回后没收敛，残差 {pos_err * 1000:.1f} mm'


def test_elbow_limit_must_stay_below_the_extension_singularity(configured_ik):
    """肘的上限必须挡在伸直奇异点（1.571 rad）之前，否则热启动会把解推过去再也回不来。"""
    for index, name in enumerate(configured_ik.joint_names):
        if 'elbow' in name:
            assert configured_ik.upper[index] < 1.571, \
                f'{name} 上限 {configured_ik.upper[index]} 越过了伸直奇异点'


def _arm_pipeline(ik, q, targets, stand, rate, rescue):
    """复刻 policy_node._control 的上肢那一段：热启动 -> 逃生种子 -> 限速。"""
    solved, pos_err, _, _ = ik.solve(q, targets)
    fired = pos_err > rescue
    if fired:
        alt, alt_pos, _, _ = ik.solve(stand, targets)
        if alt_pos < pos_err - rescue:
            solved = alt
    return q + np.clip(solved - q, -rate, rate), fired


def test_arm_pipeline_always_comes_back_from_an_unreachable_target(configured_ik):
    """整条上肢管线必须能从任意方向的"推远"回到原位。收肘只堵住了肘那条陷阱。

    肩上还有一条同类的：热启动被推远后会翻进 shoulder_pitch≈+2.41 /
    shoulder_roll 顶死 ±2.252 的镜像解支，从那儿连原位都够不着（残差 70 mm，永不恢复），
    而且落进去之后正常跟随也一起崩。实测 60 轮里纯热启动卡死 10 轮，
    加上 ik_rescue_err 这一道逃生后 0 轮。下面这 8 个方向里纯热启动会卡死 3 个。
    """
    ik, config = configured_ik, _config()
    rate = config['arm_rate_limit'] / config['control_rate_hz']
    rescue, stand = config['ik_rescue_err'], STAND_POSTURE(ik)
    home = ik.fk(stand)
    rng = np.random.default_rng(1)
    for trial in range(8):
        offset = rng.uniform(0.25, 0.75, 3) * rng.choice([-1.0, 1.0], 3)
        far = {side: np.concatenate([pose[:3] + offset, pose[3:]])
               for side, pose in home.items()}
        q = stand.copy()
        for goal, frames in ((far, 80), (home, 200)):
            for _ in range(frames):
                q, _ = _arm_pipeline(ik, q, goal, stand, rate, rescue)
        back = ik.fk(q)
        residual = max(float(np.linalg.norm(back[side][:3] - home[side][:3]))
                       for side in home)
        assert residual < 5e-3, \
            f'第 {trial} 轮拉回后卡住，残差 {residual * 1000:.1f} mm，位形 {np.round(q, 3)}'


def test_arm_pipeline_rate_limit_caps_every_frame(configured_ik):
    """跨解支时求解值一帧能跳好几弧度，发布值必须被 arm_rate_limit 压住。"""
    ik, config = configured_ik, _config()
    rate = config['arm_rate_limit'] / config['control_rate_hz']
    rescue, stand = config['ik_rescue_err'], STAND_POSTURE(ik)
    home = ik.fk(stand)
    rng = np.random.default_rng(5)
    q, published = stand.copy(), []
    for _ in range(200):                      # 大幅随机跳目标，逼它反复换解支
        offset = rng.uniform(-0.2, 0.2, 3)
        goal = {side: np.concatenate([pose[:3] + offset, pose[3:]])
                for side, pose in home.items()}
        q, _ = _arm_pipeline(ik, q, goal, stand, rate, rescue)
        published.append(q.copy())
    step = np.abs(np.diff(np.asarray(published), axis=0)).max()
    assert step <= rate + 1e-9, f'单帧跳了 {step:.3f} rad，限速 {rate:.3f} 没生效'


def test_rescue_seed_never_fires_during_normal_tracking(configured_ik):
    """逃生那一道在正常跟随时必须一次都不触发，否则它就成了跟随精度的一部分。"""
    ik, config = configured_ik, _config()
    rate = config['arm_rate_limit'] / config['control_rate_hz']
    rescue, stand = config['ik_rescue_err'], STAND_POSTURE(ik)
    home = ik.fk(stand)
    q, fired = stand.copy(), 0
    for i in range(400):                      # 6 cm 画圆，末端每帧约 1 mm
        angle = 2 * math.pi * i / 200
        goal = {side: np.concatenate(
            [pose[:3] + [0.0, 0.06 * math.cos(angle), 0.06 * math.sin(angle)], pose[3:]])
            for side, pose in home.items()}
        q, hit = _arm_pipeline(ik, q, goal, stand, rate, rescue)
        fired += hit
    assert fired == 0, f'正常跟随里触发了 {fired} 次逃生求解'


def test_null_gain_rejects_bad_gains(ik):
    urdf = (Path(get_package_share_directory('unitree_g1_description'))
            / 'model' / 'final.urdf').read_text(encoding='utf-8')
    with pytest.raises(ValueError):
        ArmIK(urdf, ARM_JOINTS, TIP_FRAMES, null_gain={'no_such_joint': 0.2})
    with pytest.raises(ValueError):           # 负增益是把关节往外推，必须拦住
        ArmIK(urdf, ARM_JOINTS, TIP_FRAMES, null_gain={'right_elbow_joint': -0.2})
    with pytest.raises(ValueError):
        ArmIK(urdf, ARM_JOINTS, TIP_FRAMES, null_gain={'right_elbow_joint': math.nan})
    with pytest.raises(ValueError):           # 门控上下界颠倒
        ArmIK(urdf, ARM_JOINTS, TIP_FRAMES, null_gain={'right_elbow_joint': 0.2},
              null_gate=(1.2, 0.8))
    with pytest.raises(ValueError):
        ArmIK(urdf, ARM_JOINTS, TIP_FRAMES,
              null_gain={'right_shoulder_roll_joint': 0.05},
              null_target={'no_such_joint': 0.2})
    with pytest.raises(ValueError):
        ArmIK(urdf, ARM_JOINTS, TIP_FRAMES,
              null_gain={'right_shoulder_roll_joint': 0.05},
              null_target={'right_shoulder_roll_joint': math.nan})
    with pytest.raises(ValueError):           # 只有参考角、没有增益，不会产生有效偏好
        ArmIK(urdf, ARM_JOINTS, TIP_FRAMES,
              null_target={'right_shoulder_roll_joint': math.radians(-20.0)})


def test_null_target_is_ungated_and_keeps_the_tip_fixed(configured_ik):
    """显式参考是无门限软偏好：任务已收敛后，有零空间才继续靠近。"""
    ik, config = configured_ik, _config()
    targets = dict(zip(config['arm_joints'], config['ik_null_target']))
    left_target = targets['left_shoulder_roll_joint']
    right_target = targets['right_shoulder_roll_joint']
    assert left_target == pytest.approx(math.radians(20.0), abs=math.radians(1.0))
    assert right_target == pytest.approx(-left_target)

    seed = STAND_POSTURE(ik)
    goal = ik.fk(seed)
    solved, _, _, _ = ik.solve(seed, goal)
    for side, reference in (('left', left_target), ('right', right_target)):
        index = ik.joint_names.index(f'{side}_shoulder_roll_joint')
        before = abs(seed[index] - reference)
        after = abs(solved[index] - reference)
        assert before < config['ik_null_gate'][0], '测试起点必须落在门限内'
        assert after < before, f'{side} 第二关节没有朝 20° 软偏好移动'

        actual = ik.fk(solved)[side]
        assert np.linalg.norm(actual[:3] - goal[side][:3]) < 1e-3
        orientation_error = 2.0 * math.acos(min(
            1.0, abs(float(np.dot(actual[3:], goal[side][3:])))))
        assert orientation_error < config['ik_tol_ori']


def test_zero_reference_null_gate_stays_shut_during_normal_tracking():
    """原三根 q_ref=0 自转轴的门控在正常工况必须与完全无偏置逐位相同。

    没有门控时偏置会把稳态残差从 0.858 顶到 1.933 mm，越过 ik_tol_pos(1 mm)，
    于是求解器永远判不了收敛，稳态迭代从 0 变成跑满 10 次。
    """
    config = _config()
    zero_reference_gain = {
        name: gain for name, gain, target in zip(
            config['arm_joints'], config['ik_null_gain'], config['ik_null_target'])
        if gain and target == 0.0
    }
    # 对照组必须也用部署配置，只差零空间增益这一项，否则比的是别的参数。
    plain_ik = _configured_ik(null_gain={}, null_target={})
    gated_ik = _configured_ik(null_gain=zero_reference_gain, null_target={})
    stand = STAND_POSTURE(plain_ik)
    home = plain_ik.fk(stand)
    plain, biased = stand.copy(), stand.copy()
    plain_iters = biased_iters = 0
    for i in range(200):                      # 3 cm 画圆，肘角远低于门控下限
        angle = 2 * math.pi * i / 100
        goal = {side: np.concatenate(
            [pose[:3] + [0.0, 0.03 * math.cos(angle), 0.03 * math.sin(angle)], pose[3:]])
            for side, pose in home.items()}
        plain, _, _, plain_iters = plain_ik.solve(plain, goal)
        biased, _, _, biased_iters = gated_ik.solve(biased, goal)
    assert np.allclose(plain, biased), '正常跟随时门控没关严，解被偏置改了'
    assert biased_iters == plain_iters, '正常跟随时带偏置多花了迭代'


def test_null_gain_moves_the_posture_when_the_elbow_is_pinned(configured_ik, ik):
    """够不着、肘顶限位时门控打开，把三根自转轴往 0 拉。

    代价是稳态残差略增（实测 72.5 -> 73.5 mm，+1.4%）：DLS 的"零空间"是阻尼伪逆定义的
    软零空间，偏置有一小部分泄漏进任务空间。这一点泄漏正是效果的来源——把投影阻尼调小
    到 λ²/100，泄漏没了，位形改善也没了（同轴帧 9.1% -> 18.1%）。
    """
    stand = STAND_POSTURE(ik)
    goal = {side: np.concatenate([pose[:3] + [0.20, 0.0, 0.0], pose[3:]])
            for side, pose in ik.fk(stand).items()}
    plain, biased = stand.copy(), stand.copy()
    for _ in range(300):
        plain, _, _, _ = ik.solve(plain, goal)
        biased, _, _, _ = configured_ik.solve(biased, goal)
    triple = [i for i, name in enumerate(ik.joint_names)
              if any(k in name for k in ('shoulder_yaw', 'elbow', 'wrist_roll'))]
    assert np.abs(biased[triple]).max() < np.abs(plain[triple]).max(), \
        '肘顶限位时偏置没把三根自转轴往 0 拉'
    for side in goal:
        plain_err = float(np.linalg.norm(ik.fk(plain)[side][:3] - goal[side][:3]))
        biased_err = float(np.linalg.norm(ik.fk(biased)[side][:3] - goal[side][:3]))
        assert biased_err < plain_err * 1.05, \
            f'{side} 侧残差劣化超过 5%：{plain_err:.4f} -> {biased_err:.4f} m'


def test_null_gain_settles_instead_of_chattering(configured_ik):
    """偏置是 -k*(q-q_ref)，到参考值就自动归零，所以稳态必须是不动点。

    换成常幅值步（-k*sign(q)，即 L1 梯度）实测稳态每帧还抖 0.103 rad = 5 rad/s。
    """
    ik = configured_ik
    stand = STAND_POSTURE(ik)
    goal = {side: np.concatenate([pose[:3] + [0.20, 0.0, 0.0], pose[3:]])
            for side, pose in ik.fk(stand).items()}   # 够不着，确保门控是开的
    track, q = [], stand.copy()
    for _ in range(160):
        q, _, _, _ = ik.solve(q, goal)
        track.append(q.copy())
    assert np.abs(np.diff(np.asarray(track)[80:], axis=0)).max() < 1e-9, \
        '门控打开时解在稳态还在抖'


def test_same_seed_and_target_give_the_same_solution(configured_ik):
    """求解本身必须是纯函数：同一种子 + 同一目标 => 同一位形。"""
    ik = configured_ik
    seed = STAND_POSTURE(ik)
    target = ik.fk(np.full(14, 0.3))
    first, _, _, _ = ik.solve(seed, target)
    for _ in range(5):
        again, _, _, _ = ik.solve(seed, target)
        assert np.array_equal(first, again)
