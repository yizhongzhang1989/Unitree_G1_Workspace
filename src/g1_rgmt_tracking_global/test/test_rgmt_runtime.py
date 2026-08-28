"""离线单测：契约、观测装配、坐标变换、安全钳位。不接机器人，不依赖 ROS。

跑法::

    cd src/g1_rgmt_tracking_global && python3 -m pytest test/ -q

最关键的一项是 :func:`test_reference_window_matches_training`——它把部署端的参考窗口
和训练侧 ``reference_tokens`` 的公式逐位对拍。这个通不过就绝对不要上机：观测错位不会
报错，只会让策略输出看起来正常但完全无意义的动作。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from g1_rgmt_tracking_global.motion_library import MotionClip  # noqa: E402
from g1_rgmt_tracking_global.odometry import OdometryFuser  # noqa: E402
from g1_rgmt_tracking_global.rotations import (  # noqa: E402
    quat_apply,
    quat_conj,
    quat_from_axis,
    quat_mul,
    rotate_inverse,
    torso_pos_from_pelvis,
    yaw_quat,
)

CONFIG = Path(__file__).resolve().parents[1] / 'config'
KEY_BODIES = ('torso_link', 'left_wrist_yaw_link', 'right_wrist_yaw_link',
              'left_ankle_roll_link', 'right_ankle_roll_link')
OFFSETS = np.array([-15, -13, -11, -9, -8, -6, -5, -3, -2, -1, 0,
                    1, 2, 3, 5, 6, 8, 9, 11, 13, 15])


def _clip() -> MotionClip:
    paths = sorted((CONFIG / 'motions').glob('*.npz'))
    if not paths:
        pytest.skip('config/motions 下没有 NPZ')
    return MotionClip(paths[0], anchor_index=0, root_index=0,
                      key_indexes=range(len(KEY_BODIES)),
                      policy_joint_ids=np.arange(29))


def _aligned_clip():
    """把参考对齐到它自己的首帧锚位姿，此时 align 是恒等变换，便于和训练侧比。"""
    clip = _clip()
    data = np.load(sorted((CONFIG / 'motions').glob('*.npz'))[0])
    clip.align(data['anchor_pos'][0], data['anchor_quat'][0])
    return clip, data


def test_align_is_identity_when_aligned_to_itself():
    clip, data = _aligned_clip()
    pos, quat = clip.anchor_pose_world(0)
    # 高度不参与对齐，只比水平分量
    assert np.allclose(pos[:2], data['anchor_pos'][0][:2], atol=1e-9)
    assert np.allclose(np.abs(quat), np.abs(data['anchor_quat'][0]), atol=1e-6)


def test_reference_window_matches_training():
    """与训练侧 ``GeneralMotionCommand.reference_tokens`` 的公式逐位对拍。

    训练侧原式（commands.py）::

        tokens = cat([lin_vel, ang_vel, proj_gravity, joint_pos])     # 参考自身根系
        rel    = ref_pos_w - robot_anchor_pos_w
        inv    = quat_inv(yaw_quat(robot_anchor_quat_w))
        local  = quat_apply(inv, rel)
        vel    = quat_apply(inv, ref_vel_w)
        return cat([tokens, local, vel])
    """
    clip, data = _aligned_clip()
    rng = np.random.default_rng(0)
    frame = 200
    # 机器人不在参考位置上——差值正是这 15 维要表达的漂移量，取 0 就测不出错位
    robot_pos = data['anchor_pos'][frame] + rng.normal(scale=0.2, size=3)
    robot_quat = quat_mul(data['anchor_quat'][frame],
                          quat_from_axis('z', 0.3))

    got = clip.reference_window(frame, OFFSETS, robot_pos, robot_quat)
    assert got.shape == (21, 68)

    idx = np.clip(frame + OFFSETS, 0, len(data['joint_pos']) - 1)
    root_quat = data['root_quat'][idx].astype(np.float64)
    gravity = np.broadcast_to(np.array([0.0, 0.0, -1.0]), (len(idx), 3))
    want_head = np.concatenate([
        rotate_inverse(root_quat, data['root_lin_vel'][idx].astype(np.float64)),
        rotate_inverse(root_quat, data['root_ang_vel'][idx].astype(np.float64)),
        rotate_inverse(root_quat, gravity),
        data['joint_pos'][idx][:, :29].astype(np.float64),
    ], axis=-1)
    assert np.allclose(got[:, :38], want_head, atol=1e-6)

    inv = quat_conj(yaw_quat(robot_quat))
    rel = data['key_pos'][idx].astype(np.float64) - robot_pos
    want_pos = quat_apply(inv, rel.reshape(-1, 3)).reshape(len(idx), -1)
    want_vel = quat_apply(inv, data['key_lin_vel'][idx].astype(np.float64).reshape(-1, 3)
                          ).reshape(len(idx), -1)
    assert np.allclose(got[:, 38:53], want_pos, atol=1e-6)
    assert np.allclose(got[:, 53:68], want_vel, atol=1e-6)


def test_head_is_yaw_and_translation_invariant():
    """前 38 维只描述"该做什么动作"，机器人在哪、朝哪都不该影响它。"""
    clip, data = _aligned_clip()
    base = clip.reference_window(100, OFFSETS, data['anchor_pos'][100],
                                 data['anchor_quat'][100])
    moved = clip.reference_window(100, OFFSETS,
                                  data['anchor_pos'][100] + np.array([3.0, -2.0, 0.0]),
                                  quat_mul(data['anchor_quat'][100], quat_from_axis('z', 1.1)))
    assert np.allclose(base[:, :38], moved[:, :38], atol=1e-9)
    # 后 30 维必须变——它们正是误差通道，不变就说明位置信息没进去
    assert not np.allclose(base[:, 38:], moved[:, 38:], atol=1e-3)


def test_anchor_channel_is_zero_when_tracking_perfectly():
    """机器人正好站在参考锚点上时，key body 的头 3 维应当为 0（那就是漂移量）。"""
    clip, data = _aligned_clip()
    frame = 150
    got = clip.reference_window(frame, OFFSETS, data['anchor_pos'][frame],
                                data['anchor_quat'][frame])
    current = list(OFFSETS).index(0)
    assert np.allclose(got[current, 38:41], 0.0, atol=1e-6)


def test_window_clamps_at_both_ends():
    clip, data = _aligned_clip()
    head = clip.reference_window(0, OFFSETS, data['anchor_pos'][0], data['anchor_quat'][0])
    # 过去侧全部钳到第 0 帧，前几个 token 的动作段应当相同
    assert np.allclose(head[0, 9:38], head[1, 9:38], atol=1e-9)
    last = clip.num_frames - 1
    tail = clip.reference_window(last, OFFSETS, data['anchor_pos'][last],
                                 data['anchor_quat'][last])
    assert np.allclose(tail[-1, 9:38], tail[-2, 9:38], atol=1e-9)


def test_align_required_before_use():
    clip = _clip()
    with pytest.raises(RuntimeError, match='尚未对齐'):
        clip.reference_window(0, OFFSETS, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))


def test_torso_fk_matches_model_geometry():
    """零腰角时躯干原点相对盆骨就是 MJCF 里的固定偏移。"""
    pos = torso_pos_from_pelvis(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), 0.0)
    assert np.allclose(pos, [-0.003964, 0.0, 0.044], atol=1e-9)
    # 腰偏航 90 度后偏移随之转过去
    turned = torso_pos_from_pelvis(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), np.pi / 2)
    assert np.allclose(turned, [0.0, -0.003964, 0.044], atol=1e-9)


def test_odom_only_passes_through():
    fuser = OdometryFuser(mode='odom_only')
    fuser.push_odom(1.0, [1.0, 2.0, 0.8], [1.0, 0.0, 0.0, 0.0])
    assert np.allclose(fuser.torso_position(), [1.0, 2.0, 0.8])
    assert fuser.stale(1.05) is None
    assert fuser.stale(2.0) is not None


def test_fused_first_lidar_frame_snaps_to_lidar():
    """首帧雷达直接置位，不走低通——否则起步阶段修正量要好几秒才收敛。"""
    fuser = OdometryFuser(mode='fused')
    assert fuser.stale(0.0) is not None  # 没有里程计就该判超时
    fuser.push_odom(1.0, [0.0, 0.0, 0.8], [1.0, 0.0, 0.0, 0.0])
    fuser.push_lidar(1.0, [5.0, 3.0, 0.8], [1.0, 0.0, 0.0, 0.0])
    assert np.allclose(fuser.torso_position(), [5.0, 3.0, 0.8], atol=1e-9)


def test_fused_pairs_lidar_with_matching_odom_stamp():
    """雷达 stamp 滞后约 34 ms，必须拿同一时刻的 odom 配对，不能用当前值。"""
    fuser = OdometryFuser(mode='fused')
    for i in range(20):
        t = 1.0 + 0.002 * i
        fuser.push_odom(t, [t - 1.0, 0.0, 0.8], [1.0, 0.0, 0.0, 0.0])
    # 雷达说 t=1.010 时机器人在 x=0.010，与 odom 完全一致 -> 修正量应为 0
    fuser.push_lidar(1.010, [0.010, 0.0, 0.8], [1.0, 0.0, 0.0, 0.0])
    corr, _ = fuser.correction
    assert np.allclose(corr, 0.0, atol=1e-9)
    # 若错误地和最新 odom(x=0.038) 配对，修正量会是 -0.028
    assert np.allclose(fuser.torso_position()[0], 0.038, atol=1e-9)


def test_fused_rejects_unpairable_lidar():
    fuser = OdometryFuser(mode='fused')
    fuser.push_odom(1.0, [0.0, 0.0, 0.8], [1.0, 0.0, 0.0, 0.0])
    assert not fuser.push_lidar(50.0, [1.0, 0.0, 0.8], [1.0, 0.0, 0.0, 0.0])


def test_odom_stamp_regression_clears_buffer():
    fuser = OdometryFuser(mode='odom_only')
    fuser.push_odom(10.0, [1.0, 0.0, 0.8], [1.0, 0.0, 0.0, 0.0])
    fuser.push_odom(1.0, [0.0, 0.0, 0.8], [1.0, 0.0, 0.0, 0.0])
    assert np.allclose(fuser.torso_position(), [0.0, 0.0, 0.8])


def _policy(max_offset: float = 0.3):
    onnx = CONFIG / 'policy.onnx'
    if not onnx.exists():
        pytest.skip('config/policy.onnx 不存在')
    pytest.importorskip('onnxruntime')
    from g1_rgmt_tracking_global.rgmt_runtime import RgmtPolicy
    return RgmtPolicy(str(onnx), target_lower=-np.full(29, 5.0),
                      target_upper=np.full(29, 5.0), max_anchor_offset_m=max_offset)


def test_contract_matches_onnx_graph():
    spec = _policy().spec
    assert spec.history_len == 10
    assert spec.token_dim == 68
    assert len(spec.window_offsets) == 21
    assert spec.reference_key_bodies == KEY_BODIES
    assert spec.key_pos_offset == 38
    assert np.allclose(spec.window_offsets, OFFSETS)


def test_closed_loop_stays_finite():
    """闭环跑 100 拍，输出必须始终有限且落在限位内。"""
    policy = _policy()
    clip, data = _aligned_clip()
    n_obs = len(policy.spec.obs_joint_names)
    rng = np.random.default_rng(1)
    pos = data['anchor_pos'][0].copy()
    for _ in range(100):
        target = policy.step(
            joint_pos=policy.spec.default_joint_pos + rng.normal(scale=0.02, size=n_obs),
            joint_vel=rng.normal(scale=0.1, size=n_obs),
            ang_vel=rng.normal(scale=0.05, size=3),
            base_quat=np.array([1.0, 0.0, 0.0, 0.0]),
            clip=clip,
            robot_anchor_pos=pos,
            robot_anchor_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        assert target.shape == (29,)
        assert np.all(np.isfinite(target))
        pos = pos + np.array([0.002, 0.0, 0.0])
    assert policy.frame == 100


def test_anchor_offset_is_clamped():
    """里程计跑飞时漂移量必须被钳住——失效模式是正反馈，不钳会把机器人推倒。"""
    policy = _policy(max_offset=0.1)
    clip, data = _aligned_clip()
    n_obs = len(policy.spec.obs_joint_names)
    policy.step(
        joint_pos=policy.spec.default_joint_pos,
        joint_vel=np.zeros(n_obs),
        ang_vel=np.zeros(3),
        base_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        clip=clip,
        # 谎报机器人在 50 米外，模拟里程计发散
        robot_anchor_pos=data['anchor_pos'][0] + np.array([50.0, 0.0, 0.0]),
        robot_anchor_quat=np.array([1.0, 0.0, 0.0, 0.0]),
    )
    assert policy.anchor_clamped


def test_spec_matches_rejects_swapped_joints():
    """权重换了而配置没跟着改 -> 左右腿指令互换。宁可起不来也不要跑错。"""
    from g1_rgmt_tracking_global.rgmt_runtime import spec_matches
    spec = _policy().spec
    good = list(spec.action_joint_names)
    swapped = good.copy()
    swapped[0], swapped[6] = swapped[6], swapped[0]
    spec_matches(spec, list(spec.obs_joint_names), good, list(spec.reference_key_bodies))
    with pytest.raises(ValueError, match='动作关节顺序'):
        spec_matches(spec, list(spec.obs_joint_names), swapped,
                     list(spec.reference_key_bodies))
    with pytest.raises(ValueError, match='key body'):
        spec_matches(spec, list(spec.obs_joint_names), good, ['pelvis'])


def test_projected_gravity_uses_base_not_anchor():
    """投影重力挂 pelvis，key body 局部化挂 anchor，两个刚体不能混。

    训练侧 ``projected_gravity_b = quat_apply_inverse(root_link_quat_w, gravity_vec_w)``，
    ``root_link`` 是自由关节所在的 pelvis；且 ``gravity_vec_w`` 是归一化的 [0,0,-1]。
    取错刚体或用 -9.81 都不会报错，只会让策略完全失效。
    """
    policy = _policy()
    clip, data = _aligned_clip()
    n_obs = len(policy.spec.obs_joint_names)
    # 盆骨前倾 30 度，anchor 保持直立
    half = np.deg2rad(30.0) / 2.0
    tilted = np.array([np.cos(half), 0.0, np.sin(half), 0.0])
    policy.step(joint_pos=policy.spec.default_joint_pos, joint_vel=np.zeros(n_obs),
                ang_vel=np.zeros(3), base_quat=tilted, clip=clip,
                robot_anchor_pos=data['anchor_pos'][0],
                robot_anchor_quat=np.array([1.0, 0.0, 0.0, 0.0]))
    got = policy._hist['rg_projected_gravity'][-1]
    assert np.isclose(np.linalg.norm(got), 1.0, atol=1e-9), '重力向量必须归一化'
    # 绕 y 轴前倾 theta 后，局部系重力应为 [sin(theta), 0, -cos(theta)]
    want = np.array([np.sin(np.deg2rad(30.0)), 0.0, -np.cos(np.deg2rad(30.0))])
    assert np.allclose(got, want, atol=1e-6)
