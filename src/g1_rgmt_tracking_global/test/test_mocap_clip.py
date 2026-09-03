"""实时动捕参考窗口的单测。不需要头显，也不需要网络。

假流由 :mod:`g1_mocap` 的重定向真跑一遍造出来，所以这里量的是**装配**那一层：
播放头怎么延迟、速度怎么差分、对齐怎么建立、断流怎么发现。
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from g1_mocap.kinematics import G1Kinematics
from g1_mocap.retarget import LEGS, ARMS, RetargetCalibration, Retargeter
from g1_mocap.skeleton import SMPL_JOINTS, BodyFrame
from g1_mocap.stream import StreamStats, _RingBuffer

from g1_rgmt_tracking_global.mocap_clip import MocapClip, lead_frames_for

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
CONTROL_DT = 0.02
IDENTITY_QUAT = np.array([1.0, 0.0, 0.0, 0.0])
# 与 policy_contract.json 的 lookahead_steps 一致。
CONTRACT_OFFSETS = np.array([-15, -13, -11, -9, -8, -6, -5, -3, -2, -1, 0,
                             1, 2, 3, 5, 6, 8, 9, 11, 13, 15])
TOKEN_DIM = 68


def _rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _skeleton(kin: G1Kinematics, q29, pelvis_pos, pelvis_rot) -> np.ndarray:
    kin.key_body_pos(q29, ('pelvis',))

    def place(link, offset=None):
        local = kin.frame_pos(link)
        if offset is not None:
            local = local + kin.frame_rot(link) @ offset
        return pelvis_pos + pelvis_rot @ local

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
    for group, tip_len in ((LEGS, 0.12), (ARMS, 0.08)):
        for side, spec in group.items():
            prefix = 'LEFT' if side == 'left' else 'RIGHT'
            names = ('HIP', 'KNEE', 'ANKLE', 'FOOT') if group is LEGS \
                else ('SHOULDER', 'ELBOW', 'WRIST', 'HAND')
            points[f'{prefix}_{names[0]}'] = place(spec.kin_links[0])
            points[f'{prefix}_{names[1]}'] = place(spec.kin_links[1])
            points[f'{prefix}_{names[2]}'] = place(spec.kin_links[2])
            points[f'{prefix}_{names[3]}'] = place(spec.tip_link,
                                                   np.array([tip_len, 0.0, 0.0]))
    return np.array([points[name] for name in SMPL_JOINTS])


class FakeStream:
    """一条匀速平移、匀速偏航的假动捕流，其余关节保持不动。

    速度/角速度取的是解析上的常量，参考窗口那边算出来的差分必须和它对得上。
    """

    def __init__(self, kin, retargeter, *, rate=90.0, speed=0.5, acceleration=0.0,
                 yaw_rate=0.4, duration=6.0) -> None:
        self.speed, self.acceleration = speed, acceleration
        self.yaw_rate, self.rate = yaw_rate, rate
        self._buffer = _RingBuffer(int(duration * rate) + 8, 29, len(KEY_BODIES))
        self._stats = StreamStats(connected=True, status=1, frames=int(duration * rate))
        calibration = RetargetCalibration.identity(0.78)
        for i in range(int(duration * rate)):
            t = i / rate
            x = speed * t + 0.5 * acceleration * t * t
            positions = _skeleton(kin, DEFAULT_Q, np.array([x, 0.0, 0.78]),
                                  _rot_z(yaw_rate * t))
            frame = BodyFrame(t=t, seq=i, positions=positions, status=1, message=0)
            self._buffer.push(t, retargeter.solve(frame, calibration))

    def stats(self):
        return self._stats

    def span(self):
        return self._buffer.span()

    def sample(self, times):
        return self._buffer.sample(times)


@pytest.fixture(scope='module')
def fake_stream() -> FakeStream:
    kin = G1Kinematics(URDF, ACTION_JOINTS)
    retargeter = Retargeter(kin, key_bodies=KEY_BODIES, anchor_body='torso_link',
                            default_joint_pos=DEFAULT_Q)
    return FakeStream(kin, retargeter)


@pytest.fixture(scope='module')
def accelerating_stream() -> FakeStream:
    kin = G1Kinematics(URDF, ACTION_JOINTS)
    retargeter = Retargeter(kin, key_bodies=KEY_BODIES, anchor_body='torso_link',
                            default_joint_pos=DEFAULT_Q)
    return FakeStream(kin, retargeter, speed=0.0, acceleration=2.0, yaw_rate=0.0)


def make_clip(stream, offsets=CONTRACT_OFFSETS, **kwargs) -> MocapClip:
    return MocapClip(stream, control_dt=CONTROL_DT,
                     lead_frames=lead_frames_for(offsets),
                     stand_joint_pos=DEFAULT_Q, **kwargs)


##
# 窗口布局
##

def test_window_shape_matches_the_contract(fake_stream):
    clip = make_clip(fake_stream)
    clip.align(np.zeros(3), IDENTITY_QUAT)
    window = clip.reference_window(0, CONTRACT_OFFSETS, np.zeros(3), IDENTITY_QUAT)
    assert window.shape == (len(CONTRACT_OFFSETS), TOKEN_DIM)
    assert np.all(np.isfinite(window))


def test_window_segments_carry_the_right_quantities(fake_stream):
    """逐段对拍。段位错了不会报错，只会让策略读到一堆看着正常但毫无意义的数。

    布局：lin_vel3 ang_vel3 proj_gravity3 joint_pos29 key_pos15 key_vel15
    """
    clip = make_clip(fake_stream)
    clip.align(np.zeros(3), IDENTITY_QUAT)
    window = clip.reference_window(0, CONTRACT_OFFSETS, np.zeros(3), IDENTITY_QUAT)

    # 参考自己根系下的线速度：匀速前进，偏航不改变它的模长。
    assert np.allclose(np.linalg.norm(window[:, 0:3], axis=-1), fake_stream.speed, atol=3e-3)
    # 角速度只有绕 z 的分量。
    assert np.allclose(window[:, 3:5], 0.0, atol=3e-3)
    assert np.allclose(window[:, 5], fake_stream.yaw_rate, atol=3e-3)
    # 投影重力：根只有偏航，所以恒等于 (0, 0, -1)。
    assert np.allclose(window[:, 6:9], np.array([0.0, 0.0, -1.0]), atol=1e-6)
    # 关节角就是造流时用的那一组。
    assert np.allclose(window[:, 9:38], DEFAULT_Q, atol=2e-2)


def test_key_velocity_segment_tracks_the_root(fake_stream):
    """key body 速度那 15 维：整段动作只有平移和偏航，各刚体速度不该乱飘。"""
    clip = make_clip(fake_stream)
    clip.align(np.zeros(3), IDENTITY_QUAT)
    window = clip.reference_window(0, CONTRACT_OFFSETS, np.zeros(3), IDENTITY_QUAT)
    anchor_velocity = window[:, 53:56]
    assert np.allclose(np.linalg.norm(anchor_velocity, axis=-1), fake_stream.speed,
                       atol=5e-3)


def test_velocity_uses_the_training_forward_difference(accelerating_stream):
    """训练 NPZ 把第 t 帧速度定义为 (x[t+1] - x[t]) / dt。"""
    offsets = np.array([0])
    clip = make_clip(accelerating_stream, offsets)
    clip.align(np.zeros(3), IDENTITY_QUAT)
    window = clip.reference_window(0, offsets, np.zeros(3), IDENTITY_QUAT)
    t = clip._last_play
    expected = accelerating_stream.acceleration * (t + 0.5 * CONTROL_DT)
    assert window[0, 0] == pytest.approx(expected, abs=3e-3)
    assert window[0, 53] == pytest.approx(expected, abs=3e-3)


##
# 对齐
##

def test_alignment_uses_the_fk_ground_height(fake_stream):
    """垂直平移对齐双踝最低点，不要求参考和机器人具有相同 torso 高度。"""
    offsets = np.array([-15, 0, 15])
    clip = make_clip(fake_stream, offsets)
    robot_pos = np.array([1.5, -0.3, 0.80])
    robot_quat = np.array([math.cos(0.55), 0.0, 0.0, math.sin(0.55)])
    reference = fake_stream.sample(np.array([clip._latest_playable()]))
    assert reference is not None
    reference_ground_z = float(np.min(reference.key_pos[0, -2:, 2]))
    robot_ground_z = -0.02
    clip.align(robot_pos, robot_quat, robot_ground_z)
    window = clip.reference_window(0, offsets, robot_pos, robot_quat)
    expected_anchor_z = reference.anchor_pos[0, 2] + robot_ground_z - reference_ground_z
    assert window[1, 40] == pytest.approx(expected_anchor_z - robot_pos[2], abs=1e-6)
    assert np.linalg.norm(window[1, 38:40]) < 1e-6


def test_alignment_is_deferred_until_the_first_window(fake_stream):
    """``~/start`` 之后还有几秒 STAND，那期间人一直在动。

    在 ``align()`` 就把变换定死，等于把这几秒的人体运动全算成跟踪误差。
    """
    offsets = np.array([-15, 0, 15])
    clip = make_clip(fake_stream, offsets)
    clip.align(np.zeros(3), IDENTITY_QUAT)
    assert clip.aligned is True
    # 还没取过窗口，anchor_pose_world 只能回报要对齐到的那个位姿。
    position, _ = clip.anchor_pose_world(0)
    assert np.allclose(position, np.zeros(3))


def test_window_before_align_is_refused(fake_stream):
    clip = make_clip(fake_stream)
    with pytest.raises(RuntimeError, match='尚未对齐'):
        clip.reference_window(0, CONTRACT_OFFSETS, np.zeros(3), IDENTITY_QUAT)


##
# 播放时钟
##

def test_playhead_lags_by_the_full_lookahead(fake_stream):
    """播放头必须落后最新帧至少一个前瞻跨度。

    否则 ``+15`` 那个 token 会被钳位成"当前帧"，前瞻整段失效——而且不会报错，
    只是策略突然没有了未来信息。
    """
    lead = lead_frames_for(CONTRACT_OFFSETS)
    assert lead >= int(CONTRACT_OFFSETS.max())
    clip = make_clip(fake_stream)
    clip.align(np.zeros(3), IDENTITY_QUAT)
    clip.reference_window(0, CONTRACT_OFFSETS, np.zeros(3), IDENTITY_QUAT)
    lag = fake_stream.span()[1] - clip._last_play
    assert lag == pytest.approx(lead * CONTROL_DT, abs=1e-9)


def test_playhead_advances_one_control_step_per_frame(fake_stream):
    """播放头按控制环的整数拍走，参考速度才落在和训练同一个 50 Hz 网格上。"""
    clip = make_clip(fake_stream)
    clip.align(np.zeros(3), IDENTITY_QUAT)
    stamps = []
    for frame in range(5):
        clip.reference_window(frame, CONTRACT_OFFSETS, np.zeros(3), IDENTITY_QUAT)
        stamps.append(clip._last_play)
    steps = np.diff(stamps)
    # 慢修正每拍最多动 2%，所以步长应该紧贴一个控制周期。
    assert np.allclose(steps, CONTROL_DT, rtol=0.05)


def test_playhead_hard_resyncs_after_a_gap(fake_stream):
    """慢修正追不上的那种断流（缓冲被清空、头显重连）只能硬跳。"""
    clip = make_clip(fake_stream, hard_resync_s=0.5)
    clip.align(np.zeros(3), IDENTITY_QUAT)
    clip.reference_window(0, CONTRACT_OFFSETS, np.zeros(3), IDENTITY_QUAT)
    # 隔了 500 拍才回来，相位差 10 s，远超硬同步阈值。
    clip.reference_window(500, CONTRACT_OFFSETS, np.zeros(3), IDENTITY_QUAT)
    lead = lead_frames_for(CONTRACT_OFFSETS) * CONTROL_DT
    assert fake_stream.span()[1] - clip._last_play == pytest.approx(lead, abs=1e-9)


##
# 与 MotionClip 的接口一致性
##

def test_streaming_flag_separates_it_from_a_finite_clip(fake_stream):
    """tracking_node 靠它区分「放完回 IDLE」和「一直跟着人走」。"""
    clip = make_clip(fake_stream)
    assert clip.streaming is True
    assert clip.num_frames > 10 ** 6
    assert math.isinf(clip.duration_s)


def test_stand_joint_pos_is_fixed_before_squeeze(fake_stream):
    clip = make_clip(fake_stream)
    assert np.allclose(clip.stand_joint_pos(), DEFAULT_Q, atol=2e-2)


def test_live_joint_pos_preserves_the_original_engaged_start(fake_stream):
    clip = make_clip(fake_stream)
    expected = fake_stream.sample(np.array([clip._latest_playable()]))
    assert expected is not None
    assert np.allclose(clip.live_joint_pos(), expected.joint_pos[0])


def test_explicit_live_alignment_uses_the_squeeze_frame(fake_stream):
    offsets = np.array([0])
    clip = make_clip(fake_stream, offsets)
    reference_time = clip._latest_playable()
    reference = fake_stream.sample(np.array([reference_time]))
    assert reference is not None
    robot_pos = np.array([2.0, -1.0, 0.8])
    robot_ground_z = -0.03
    reference_ground_z = float(np.min(reference.key_pos[0, -2:, 2]))
    clip.align_from_reference(robot_pos, IDENTITY_QUAT, robot_ground_z,
                              reference_time, reference.anchor_pos[0],
                              reference.anchor_quat[0],
                              reference_ground_z)
    window = clip.reference_window(0, offsets, robot_pos, IDENTITY_QUAT)
    assert np.linalg.norm(window[0, 38:40]) < 1e-6
    expected_anchor_z = reference.anchor_pos[0, 2] + robot_ground_z - reference_ground_z
    assert window[0, 40] == pytest.approx(expected_anchor_z - robot_pos[2], abs=1e-6)


def test_stale_detects_a_dead_link(fake_stream):
    clip = make_clip(fake_stream, stale_timeout_s=0.3)
    assert clip.stale(fake_stream.span()[1] + 0.1) == ''
    assert '断流' in clip.stale(fake_stream.span()[1] + 1.0)


def test_stale_refuses_to_guess_the_clock_domain(fake_stream):
    """``now`` 必须由调用者给，不能有默认值。

    缓冲的时间轴跟着数据源走：订 ``/mocap/frame`` 时是消息的 ``header.stamp``
    （ROS 时钟）。要是默认个 ``time.monotonic()``，两个域能差几小时，永远判成断流。
    """
    clip = make_clip(fake_stream, stale_timeout_s=0.3)
    with pytest.raises(TypeError):
        clip.stale()


def test_clip_accepts_a_frame_buffer():
    """``MocapClip`` 只认 span/sample/stats 三个方法，不认具体数据源。

    跟踪层现在喂的是 ``FrameBuffer``（订 topic），这条把那个接口契约钉住——
    对不上的话下面这几行会抛 AttributeError 而不是给出理由。
    """
    from g1_mocap.consumer import FrameBuffer

    buffer = FrameBuffer(n_joints=len(ACTION_JOINTS), n_keys=len(KEY_BODIES))
    clip = MocapClip(buffer, control_dt=CONTROL_DT,
                     lead_frames=lead_frames_for(CONTRACT_OFFSETS),
                     stand_joint_pos=DEFAULT_Q)
    # 还没收到任何 status，也没有帧：必须给出明确理由，而不是当成正常。
    assert clip.stale(0.0) != ''
    assert clip.streaming is True
