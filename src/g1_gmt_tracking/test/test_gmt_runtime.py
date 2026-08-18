"""不接机器人、不起 ROS 的离线自测：契约解析、观测装配、推理闭环。

跑法（``:$PYTHONPATH`` 不能省，否则 ``import rclpy`` 会失败）::

    cd src/g1_gmt_tracking && PYTHONPATH=.:$PYTHONPATH python3 -m pytest test/test_gmt_runtime.py -q
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path
from types import SimpleNamespace
import threading

import numpy as np
import pytest

from g1_gmt_tracking.gmt_runtime import (
    CONTRACT_JSON,
    GmtPolicy,
    load_policy,
    spec_matches,
)
from g1_gmt_tracking.motion_library import MotionClip, resolve_indices
from g1_gmt_tracking.tracking_node import GmtTrackingNode, State
from g1_gmt_tracking.rotations import (
    quat_from_xyzw,
    quat_mul,
    quat_to_mat,
    torso_quat_from_pelvis,
    yaw_quat,
)

CONFIG = Path(__file__).resolve().parent.parent / 'config'
IDENTITY = np.array([1.0, 0.0, 0.0, 0.0])


@pytest.fixture(scope='module')
def loaded():
    session, spec = load_policy(str(CONFIG / 'policy.onnx'))
    idx = resolve_indices(spec.all_body_names, spec.anchor_body_name,
                          spec.root_body_name, spec.obs_joint_names,
                          spec.action_joint_names, spec.control_dt)
    clip = MotionClip(CONFIG / 'motions' / 'proc_stand.npz', **idx)
    policy = GmtPolicy(session, spec,
                       target_lower=np.full(spec.action_dim, -6.0),
                       target_upper=np.full(spec.action_dim, +6.0))
    return session, spec, clip, policy


def test_contract(loaded):
    _, spec, _, _ = loaded
    assert spec.obs_dim == 866
    assert spec.action_dim == 29
    assert len(spec.obs_joint_names) == 31
    assert spec.history_length == 5
    assert len(spec.lookahead_steps) == 10
    assert spec.control_dt == pytest.approx(0.02)
    # 夹爪轴进观测但不进动作，且不在名单末尾——按名字查槽位而不是切片。
    assert 'left_eccentric_joint' in spec.obs_joint_names
    assert 'left_eccentric_joint' not in spec.action_joint_names


def test_lookahead_is_yaw_and_translation_invariant(loaded):
    """整段动作绕 z 旋转 + 平移后，前瞻特征必须逐位不变。

    这是策略能换动作而不重训的前提；一旦有人把绝对位姿混进前瞻，这条会先炸。
    """
    _, spec, clip, _ = loaded
    base = clip.lookahead(10, spec.lookahead_steps)

    rotated = MotionClip(CONFIG / 'motions' / 'proc_stand.npz',
                         anchor_index=0, root_index=0,
                         policy_joint_ids=clip._policy_joint_ids,
                         expected_fps=1.0 / spec.control_dt)
    yaw = np.array([np.cos(0.6), 0.0, 0.0, np.sin(0.6)])
    rot = quat_to_mat(yaw)
    for arr in ('_root_pos', '_root_lin', '_root_ang'):
        setattr(rotated, arr, getattr(rotated, arr) @ rot.T)
    rotated._root_pos[:, :2] += np.array([3.0, -2.0])
    rotated._root_quat = np.stack([quat_mul(yaw, q) for q in rotated._root_quat])

    np.testing.assert_allclose(rotated.lookahead(10, spec.lookahead_steps),
                               base, atol=1e-9)


def test_anchor_ori_b_is_zero_rotation_when_aligned(loaded):
    """机器人姿态与参考锚姿态一致时，相对旋转应是单位阵的前两列。"""
    _, _, clip, _ = loaded
    clip.align_yaw(clip.anchor_quat(0))
    out = clip.anchor_ori_b(0, clip.anchor_quat(0))
    np.testing.assert_allclose(out, np.array([1, 0, 0, 1, 0, 0]), atol=1e-9)


def test_align_yaw_only_touches_yaw(loaded):
    """对齐只能改偏航：俯仰/横滚是动作内容，动了就是篡改参考。"""
    _, _, clip, _ = loaded
    robot = quat_mul(np.array([np.cos(0.4), 0, 0, np.sin(0.4)]), clip.anchor_quat(0))
    clip.align_yaw(robot)
    # 对齐量本身必须是纯偏航。
    np.testing.assert_allclose(clip._align, yaw_quat(clip._align), atol=1e-9)


def test_torso_from_pelvis_matches_pure_waist_rotation():
    """腰三轴全零时躯干姿态就等于盆骨姿态。"""
    pelvis = quat_from_xyzw((0.0, 0.0, 0.2588, 0.9659))
    np.testing.assert_allclose(
        torso_quat_from_pelvis(pelvis, 0.0, 0.0, 0.0), pelvis, atol=1e-12)


def test_closed_loop_stays_finite_and_bounded(loaded):
    """喂参考动作自身的位形跑 100 拍，输出必须有限且在保护边界内。"""
    _, spec, clip, policy = loaded
    policy.reset()
    clip.align_yaw(IDENTITY)
    n = len(spec.obs_joint_names)
    for _ in range(100):
        target = policy.step(
            clip=clip,
            joint_pos=clip.joint_pos[clip.clamp(policy.frame)][:n],
            joint_vel=np.zeros(n),
            ang_vel=np.zeros(3),
            anchor_quat=IDENTITY,
        )
        assert target.shape == (spec.action_dim,)
        assert np.all(np.isfinite(target))
        assert np.all(target >= -6.0) and np.all(target <= 6.0)
    assert policy.frame == 100


def test_spec_mismatch_is_rejected(loaded):
    """关节顺序对不上必须拒绝启动，而不是静默跑错。"""
    _, spec, _, _ = loaded
    wrong = list(spec.action_joint_names)
    wrong[0], wrong[6] = wrong[6], wrong[0]  # 左右腿互换
    with pytest.raises(ValueError, match='拒绝启动'):
        spec_matches(spec, spec.obs_joint_names, wrong)


@pytest.mark.parametrize('key, index, value', [
    ('action_joint_names', 0, 'right_hip_pitch_joint'),  # 左右腿互换
    ('action_scale', 3, 9.9),
])
def test_contract_that_disagrees_with_the_weights_is_rejected(tmp_path, key, index, value):
    """缺的 metadata 由 contract 补，但拿 A 的契约配 B 的权重必须当场拦下。"""
    shutil.copy(CONFIG / 'policy.onnx', tmp_path / 'policy.onnx')
    contract = json.loads((CONFIG / CONTRACT_JSON).read_text(encoding='utf-8'))
    contract[key][index] = value
    (tmp_path / CONTRACT_JSON).write_text(json.dumps(contract), encoding='utf-8')

    with pytest.raises(ValueError, match=key):
        load_policy(str(tmp_path / 'policy.onnx'))


def test_full_precision_numbers_come_from_the_contract(loaded):
    """ONNX metadata 只存到 3 位小数，直接乘进关节目标的量要用 contract 的全精度值。"""
    _, spec, _, _ = loaded
    contract = json.loads((CONFIG / CONTRACT_JSON).read_text(encoding='utf-8'))
    np.testing.assert_array_equal(spec.action_scale, contract['action_scale'])
    assert not np.array_equal(spec.action_scale, np.round(spec.action_scale, 3))


def test_motion_with_a_different_fps_is_rejected(loaded, tmp_path):
    """帧率必须等于控制频率：一个控制拍推进一帧，30 fps 会被静默放快 1.67 倍。"""
    _, spec, _, _ = loaded
    source = dict(np.load(CONFIG / 'motions' / 'proc_stand.npz'))
    source['fps'] = np.array([30.0])
    np.savez(tmp_path / 'wrong_fps.npz', **source)
    idx = resolve_indices(spec.all_body_names, spec.anchor_body_name,
                          spec.root_body_name, spec.obs_joint_names,
                          spec.action_joint_names, spec.control_dt)

    with pytest.raises(ValueError, match='fps'):
        MotionClip(tmp_path / 'wrong_fps.npz', **idx)


def test_yaw_is_locked_when_the_first_frame_plays_not_when_stand_begins(loaded):
    """STAND 会把腰三轴插到参考位形、躯干朝向随之变，所以偏航只能在放第 0 帧那拍锁。"""
    _, spec, clip, policy = loaded
    policy.reset()
    clip._align = IDENTITY.copy()
    n = len(spec.obs_joint_names)
    at_stand = quat_mul(np.array([np.cos(0.15), 0, 0, np.sin(0.15)]), clip.anchor_quat(0))
    at_running = quat_mul(np.array([np.cos(0.35), 0, 0, np.sin(0.35)]), clip.anchor_quat(0))
    node = SimpleNamespace(
        _lock=threading.Lock(), _state=State.STAND, _reason='',
        _stale=lambda: '', _now=lambda: 0.0, _stand_start=0.0, _stand_s=1.0,
        _measured=clip.joint_pos[0][:n].copy(), _measured_vel=np.zeros(n),
        _stand_from=clip.joint_pos[0][:n].copy(), _imu_omega=np.zeros(3),
        _torso_quat_locked=lambda: at_stand, _tilt_limit=0.8,
        _clip=clip, _policy=policy, _loop=False, _joints=list(spec.obs_joint_names),
        _obs_slots=np.arange(n), _action_slots=policy.action_slots,
        _gripper_slots=np.setdiff1d(np.arange(n), policy.action_slots),
        _gripper_targets=np.zeros(2), _message=SimpleNamespace(data=[]),
        _estop=lambda reason: pytest.fail(f'不该急停: {reason}'),
        get_logger=lambda: SimpleNamespace(info=lambda *_: None),
        _publisher=SimpleNamespace(publish=lambda _: None),
    )

    GmtTrackingNode._control(node)          # STAND 插值中
    node._now = lambda: 10.0
    GmtTrackingNode._control(node)          # 插完切 RUNNING，还没跑策略
    assert node._state is State.RUNNING and policy.frame == 0
    np.testing.assert_array_equal(clip._align, IDENTITY,
                                  err_msg='STAND 期间不该锁偏航')

    node._torso_quat_locked = lambda: at_running
    GmtTrackingNode._control(node)
    np.testing.assert_allclose(clip.anchor_ori_b(0, at_running),
                               np.array([1, 0, 0, 1, 0, 0]), atol=1e-9)


def test_observation_length_matches_contract(loaded):
    _, spec, clip, policy = loaded
    policy.reset()
    clip.align_yaw(IDENTITY)
    n = len(spec.obs_joint_names)
    obs = policy.observe(clip=clip, joint_pos=np.zeros(n), joint_vel=np.zeros(n),
                         ang_vel=np.zeros(3), anchor_quat=IDENTITY)
    assert obs.shape == (spec.obs_dim,)
    # 前 390 维是前瞻，必须和 clip 直接算出来的一致。
    np.testing.assert_allclose(obs[:390],
                               clip.lookahead(0, spec.lookahead_steps), atol=0)


def test_engage_recovers_from_estop_after_controller_reactivates():
    node = SimpleNamespace(
        _lock=threading.Lock(), _state=State.ESTOP, _reason='人工急停',
        _switch_controller=lambda *, activate: (activate, '已激活'))
    response = SimpleNamespace(success=False, message='')

    GmtTrackingNode._on_engage(node, None, response)

    assert response.success
    assert node._state is State.IDLE
    assert node._reason == ''


def test_engage_keeps_estop_when_controller_reactivation_fails():
    node = SimpleNamespace(
        _lock=threading.Lock(), _state=State.ESTOP, _reason='人工急停',
        _switch_controller=lambda *, activate: (False, '切换失败'))
    response = SimpleNamespace(success=False, message='')

    GmtTrackingNode._on_engage(node, None, response)

    assert not response.success
    assert node._state is State.ESTOP
    assert node._reason == '人工急停'
