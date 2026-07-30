"""policy_runtime 的离线校验。

这里只测和实机安全直接相关的三件事：观测装配的顺序与数值、动作到目标的映射、
以及"ONNX 和配置对不上就必须拒绝启动"。跑起来不需要真机，也不需要 ROS。
"""

import math

import numpy as np
import pytest

from g1_lower_body_policy.policy_runtime import (
    EXPECTED_OBS_TERMS,
    LowerBodyPolicy,
    PolicySpec,
    gait_phase,
    projected_gravity,
    spec_matches,
)

OBS_DIM = sum(dim for _, dim in EXPECTED_OBS_TERMS)
ACT_DIM = 15


class FakeSession:
    """回放固定动作的假 session，顺便记下最后一次收到的观测。"""

    def __init__(self, action=None):
        self.action = np.arange(ACT_DIM, dtype=np.float32) if action is None else action
        self.last_obs = None

    class _Port:
        name = 'obs'
        shape = [1, OBS_DIM]

    def get_inputs(self):
        return [self._Port()]

    def run(self, _outputs, feed):
        self.last_obs = np.array(feed['obs'][0])
        return [self.action.reshape(1, -1)]


def make_policy(session=None, control_dt=0.02):
    spec = PolicySpec(
        joint_names=tuple(f'j{i}' for i in range(ACT_DIM)),
        default_pos=np.full(ACT_DIM, 0.1),
        action_scale=np.full(ACT_DIM, 0.5),
        obs_dim=OBS_DIM,
        action_dim=ACT_DIM,
    )
    return LowerBodyPolicy(
        session or FakeSession(), spec, control_dt=control_dt,
        target_lower=np.full(ACT_DIM, -10.0), target_upper=np.full(ACT_DIM, 10.0))


def test_projected_gravity_upright():
    assert projected_gravity((0.0, 0.0, 0.0, 1.0)) == pytest.approx([0.0, 0.0, -1.0])


def test_projected_gravity_pitched_forward():
    # 绕 y 轴 +90°：机体 x 轴指天，重力在机体系里落到 +x。
    half = math.sqrt(0.5)
    assert projected_gravity((0.0, half, 0.0, half)) == pytest.approx(
        [1.0, 0.0, 0.0], abs=1e-9)


def test_projected_gravity_rejects_zero_quaternion():
    with pytest.raises(ValueError):
        projected_gravity((0.0, 0.0, 0.0, 0.0))


def test_gait_phase_zero_when_standing():
    assert gait_phase(0.3, [0.0, 0.0, 0.05, 0.74]) == pytest.approx([0.0, 0.0])


def test_gait_phase_wraps_on_period():
    walking = [0.5, 0.0, 0.0, 0.74]
    assert gait_phase(0.0, walking) == pytest.approx([0.0, 1.0])
    assert gait_phase(0.15, walking) == pytest.approx([1.0, 0.0])  # 四分之一周期
    assert gait_phase(0.6, walking) == pytest.approx(gait_phase(0.0, walking))


def test_observation_layout_matches_training_order():
    session = FakeSession()
    policy = make_policy(session)
    joint_pos = np.linspace(0.0, 1.4, ACT_DIM)
    joint_vel = np.linspace(-0.7, 0.7, ACT_DIM)
    policy.step(joint_pos=joint_pos, joint_vel=joint_vel,
                ang_vel=(0.1, 0.2, 0.3), quat_xyzw=(0.0, 0.0, 0.0, 1.0),
                command=(0.4, 0.0, 0.0, 0.72))

    obs = session.last_obs
    assert obs.shape == (OBS_DIM,)
    assert obs[0:3] == pytest.approx([0.1, 0.2, 0.3])
    assert obs[3:6] == pytest.approx([0.0, 0.0, -1.0])
    assert obs[6:9] == pytest.approx([0.4, 0.0, 0.0])
    assert obs[9] == pytest.approx(0.72)
    # 第一拍相位角为 0 -> [sin, cos] = [0, 1]。
    assert obs[10:12] == pytest.approx([0.0, 1.0])
    assert obs[12:27] == pytest.approx(joint_pos - 0.1)
    assert obs[27:42] == pytest.approx(joint_vel)
    assert obs[42:57] == pytest.approx(np.zeros(ACT_DIM))  # 第一拍还没有上一动作


def test_last_action_feeds_back_unclipped():
    """训练里 actions 观测是未裁剪的原始输出，实机必须一样。"""
    session = FakeSession(action=np.full(ACT_DIM, 100.0, dtype=np.float32))
    policy = make_policy(session)
    zeros = np.zeros(ACT_DIM)
    for _ in range(2):
        policy.step(joint_pos=zeros, joint_vel=zeros, ang_vel=(0, 0, 0),
                    quat_xyzw=(0, 0, 0, 1), command=(0, 0, 0, 0.74))
    assert session.last_obs[42:57] == pytest.approx(np.full(ACT_DIM, 100.0))


def test_target_is_default_plus_scaled_action():
    session = FakeSession(action=np.full(ACT_DIM, 2.0, dtype=np.float32))
    policy = make_policy(session)
    zeros = np.zeros(ACT_DIM)
    target = policy.step(joint_pos=zeros, joint_vel=zeros, ang_vel=(0, 0, 0),
                         quat_xyzw=(0, 0, 0, 1), command=(0, 0, 0, 0.74))
    assert target == pytest.approx(np.full(ACT_DIM, 0.1 + 0.5 * 2.0))


def test_target_may_leave_the_joint_range():
    """底层是 PD，靠到行程边上还想要力就必须把目标顶到行程之外。

    训练侧 mjlab 建 <position> 执行器时 ctrllimited=False，实测 1.0 m/s 前进时
    18.6% 的拍目标在硬行程之外。那个行程不能拿来当部署侧的闸。
    """
    # waist_roll 行程 ±0.520，ctrlrange ±2.274。目标落在两者之间时必须原样通过。
    spec = PolicySpec(
        joint_names=tuple(f'j{i}' for i in range(ACT_DIM)),
        default_pos=np.zeros(ACT_DIM), action_scale=np.ones(ACT_DIM),
        obs_dim=OBS_DIM, action_dim=ACT_DIM)
    policy = LowerBodyPolicy(
        FakeSession(action=np.full(ACT_DIM, -0.763, dtype=np.float32)), spec,
        control_dt=0.02, target_lower=np.full(ACT_DIM, -2.274),
        target_upper=np.full(ACT_DIM, 2.274))
    zeros = np.zeros(ACT_DIM)
    target = policy.step(joint_pos=zeros, joint_vel=zeros, ang_vel=(0, 0, 0),
                         quat_xyzw=(0, 0, 0, 1), command=(0, 0, 0, 0.74))
    assert target == pytest.approx(np.full(ACT_DIM, -0.763))


def test_target_is_clipped_at_ctrlrange():
    """超过 ctrlrange 后力矩已饱和，裁在那里只为了拦跑飞的输出。"""
    spec = PolicySpec(
        joint_names=tuple(f'j{i}' for i in range(ACT_DIM)),
        default_pos=np.zeros(ACT_DIM), action_scale=np.ones(ACT_DIM),
        obs_dim=OBS_DIM, action_dim=ACT_DIM)
    policy = LowerBodyPolicy(
        FakeSession(action=np.full(ACT_DIM, 500.0, dtype=np.float32)), spec,
        control_dt=0.02, target_lower=np.full(ACT_DIM, -2.274),
        target_upper=np.full(ACT_DIM, 2.274))
    zeros = np.zeros(ACT_DIM)
    target = policy.step(joint_pos=zeros, joint_vel=zeros, ang_vel=(0, 0, 0),
                         quat_xyzw=(0, 0, 0, 1), command=(0, 0, 0, 0.74))
    assert target == pytest.approx(np.full(ACT_DIM, 2.274))


def test_reset_clears_action_and_phase():
    session = FakeSession(action=np.full(ACT_DIM, 3.0, dtype=np.float32))
    policy = make_policy(session)
    walking = (0.5, 0.0, 0.0, 0.74)
    zeros = np.zeros(ACT_DIM)
    for _ in range(5):
        policy.step(joint_pos=zeros, joint_vel=zeros, ang_vel=(0, 0, 0),
                    quat_xyzw=(0, 0, 0, 1), command=walking)
    policy.reset()
    policy.step(joint_pos=zeros, joint_vel=zeros, ang_vel=(0, 0, 0),
                quat_xyzw=(0, 0, 0, 1), command=walking)
    assert session.last_obs[10:12] == pytest.approx([0.0, 1.0])
    assert session.last_obs[42:57] == pytest.approx(np.zeros(ACT_DIM))


def test_non_finite_output_raises():
    session = FakeSession(action=np.full(ACT_DIM, np.nan, dtype=np.float32))
    policy = make_policy(session)
    zeros = np.zeros(ACT_DIM)
    with pytest.raises(ValueError):
        policy.step(joint_pos=zeros, joint_vel=zeros, ang_vel=(0, 0, 0),
                    quat_xyzw=(0, 0, 0, 1), command=(0, 0, 0, 0.74))


def test_spec_mismatch_is_rejected():
    spec = PolicySpec(joint_names=('a', 'b'), default_pos=np.zeros(2),
                      action_scale=np.ones(2), obs_dim=OBS_DIM, action_dim=2)
    spec_matches(spec, ['a', 'b'])
    with pytest.raises(ValueError):
        spec_matches(spec, ['b', 'a'])  # 顺序反了 = 左右互换级别的事故
