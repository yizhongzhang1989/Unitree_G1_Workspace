"""policy_runtime 的离线校验。

这里只测和实机安全直接相关的四件事：观测装配的顺序与数值、动作到目标的映射、
"ONNX 和配置对不上就必须拒绝启动"，以及**两套契约都能跑**——速度跟踪那套带指令与
步态相位、只看下肢 15 轴，站立那套没有指令、看全身 29 轴。跑起来不需要真机也不需要 ROS。
"""

import math

import numpy as np
import pytest

from g1_motion_control.policy_runtime import (
    LocomotionPolicy,
    PolicySpec,
    gait_phase,
    projected_gravity,
    spec_matches,
)

ACT_DIM = 15  # 下肢：12 腿 + 3 腰
ARM_DIM = 14

# 速度跟踪契约：3+3+3+1+2+15+15+15
VEL_TERMS = ('base_ang_vel', 'projected_gravity', 'command_twist',
             'command_height', 'phase', 'joint_pos', 'joint_vel', 'actions')
VEL_OBS_DIM = 57
# 站立契约：3+3+29+29+15
STAND_TERMS = ('base_ang_vel', 'projected_gravity', 'joint_pos', 'joint_vel', 'actions')
STAND_JOINTS = ACT_DIM + ARM_DIM
STAND_OBS_DIM = 3 + 3 + 2 * STAND_JOINTS + ACT_DIM

DEFAULT_POS = 0.1
ACTION_SCALE = 0.5
HIDDEN_SHAPE = (1, 1, 32)


class FakeSession:
    """回放固定动作的假 GRU session，顺便记下最后一次收到的观测与隐状态。"""

    def __init__(self, obs_dim, action=None):
        self.obs_dim = obs_dim
        self.action = np.arange(ACT_DIM, dtype=np.float32) if action is None else action
        self.last_obs = None

    def get_inputs(self):
        port = type('Port', (), {'name': 'obs', 'shape': [1, self.obs_dim]})
        return [port()]

    def run(self, _outputs, feed):
        self.last_obs = np.array(feed['obs'][0])
        self.last_hidden_in = np.array(feed['h_in'])
        return [self.action.reshape(1, -1), self.last_hidden_in]


def make_spec(terms, obs_joints, obs_dim, *, default_pos=DEFAULT_POS,
              action_scale=ACTION_SCALE):
    return PolicySpec(
        obs_terms=terms,
        action_joint_names=tuple(f'j{i}' for i in range(ACT_DIM)),
        action_default_pos=np.full(ACT_DIM, default_pos),
        action_scale=np.full(ACT_DIM, action_scale),
        obs_joint_names=tuple(f'j{i}' for i in range(obs_joints)),
        obs_default_pos=np.full(obs_joints, default_pos),
        obs_dim=obs_dim,
        action_dim=ACT_DIM,
        hidden_name='h_in',
        hidden_shape=HIDDEN_SHAPE,
    )


def vel_spec(**kwargs):
    return make_spec(VEL_TERMS, ACT_DIM, VEL_OBS_DIM, **kwargs)


def stand_spec(**kwargs):
    return make_spec(STAND_TERMS, STAND_JOINTS, STAND_OBS_DIM, **kwargs)


def make_policy(spec, session=None, limit=10.0, control_dt=0.02):
    return LocomotionPolicy(
        session or FakeSession(spec.obs_dim), spec, control_dt=control_dt,
        target_lower=np.full(ACT_DIM, -limit), target_upper=np.full(ACT_DIM, limit))


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


def test_velocity_contract_observation_layout():
    """老策略：指令与相位必须出现在训练时的那几位上。"""
    spec = vel_spec()
    session = FakeSession(spec.obs_dim)
    policy = make_policy(spec, session)
    joint_pos = np.linspace(0.0, 1.4, ACT_DIM)
    joint_vel = np.linspace(-0.7, 0.7, ACT_DIM)
    policy.step(joint_pos=joint_pos, joint_vel=joint_vel,
                ang_vel=(0.1, 0.2, 0.3), quat_xyzw=(0.0, 0.0, 0.0, 1.0),
                command=(0.4, 0.0, 0.0, 0.72))

    obs = session.last_obs
    assert obs.shape == (VEL_OBS_DIM,)
    assert obs[0:3] == pytest.approx([0.1, 0.2, 0.3])
    assert obs[3:6] == pytest.approx([0.0, 0.0, -1.0])
    assert obs[6:9] == pytest.approx([0.4, 0.0, 0.0])
    assert obs[9] == pytest.approx(0.72)
    assert obs[10:12] == pytest.approx([0.0, 1.0])  # 第一拍相位角 0
    assert obs[12:27] == pytest.approx(joint_pos - DEFAULT_POS)
    assert obs[27:42] == pytest.approx(joint_vel)
    assert obs[42:57] == pytest.approx(np.zeros(ACT_DIM))  # 还没有上一动作


def test_standing_contract_observation_layout():
    """站立策略：29 轴关节量，指令项一个都没有。"""
    spec = stand_spec()
    session = FakeSession(spec.obs_dim)
    policy = make_policy(spec, session)
    joint_pos = np.linspace(0.0, 1.4, STAND_JOINTS)
    joint_vel = np.linspace(-0.7, 0.7, STAND_JOINTS)
    policy.step(joint_pos=joint_pos, joint_vel=joint_vel,
                ang_vel=(0.1, 0.2, 0.3), quat_xyzw=(0.0, 0.0, 0.0, 1.0),
                command=(0.4, 0.0, 0.0, 0.72))

    obs = session.last_obs
    assert obs.shape == (STAND_OBS_DIM,)
    assert obs[0:3] == pytest.approx([0.1, 0.2, 0.3])
    assert obs[3:6] == pytest.approx([0.0, 0.0, -1.0])
    assert obs[6:6 + STAND_JOINTS] == pytest.approx(joint_pos - DEFAULT_POS)
    assert obs[6 + STAND_JOINTS:6 + 2 * STAND_JOINTS] == pytest.approx(joint_vel)
    assert obs[6 + 2 * STAND_JOINTS:] == pytest.approx(np.zeros(ACT_DIM))


def test_standing_policy_ignores_command_entirely():
    """指令照收，但对输出必须一点影响都没有——否则「不响应指令」就是空话。"""
    spec = stand_spec()
    zeros = np.zeros(STAND_JOINTS)
    outputs = []
    for command in ((0.0, 0.0, 0.0, 0.74), (1.0, -0.8, 1.5, 0.60)):
        policy = make_policy(spec, FakeSession(spec.obs_dim))
        outputs.append(policy.step(joint_pos=zeros, joint_vel=zeros,
                                   ang_vel=(0.1, 0.2, 0.3), quat_xyzw=(0, 0, 0, 1),
                                   command=command))
    assert outputs[0] == pytest.approx(outputs[1])


def test_uses_command_flag():
    assert vel_spec().uses_command
    assert not stand_spec().uses_command


def test_wrong_joint_count_is_rejected():
    """给站立策略只喂 15 轴必须报错：手臂由 VR IK 自己动，它摆到哪儿决定质心在哪儿。"""
    spec = stand_spec()
    policy = make_policy(spec)
    with pytest.raises(ValueError):
        policy.step(joint_pos=np.zeros(ACT_DIM), joint_vel=np.zeros(ACT_DIM),
                    ang_vel=(0, 0, 0), quat_xyzw=(0, 0, 0, 1),
                    command=(0, 0, 0, 0.74))


@pytest.mark.parametrize('spec_fn', [vel_spec, stand_spec])
def test_last_action_feeds_back_unclipped(spec_fn):
    """训练里 actions 观测是未裁剪的原始输出，实机必须一样。"""
    spec = spec_fn()
    session = FakeSession(spec.obs_dim, np.full(ACT_DIM, 100.0, dtype=np.float32))
    policy = make_policy(spec, session)
    zeros = np.zeros(len(spec.obs_joint_names))
    for _ in range(2):
        policy.step(joint_pos=zeros, joint_vel=zeros, ang_vel=(0, 0, 0),
                    quat_xyzw=(0, 0, 0, 1), command=(0, 0, 0, 0.74))
    assert session.last_obs[-ACT_DIM:] == pytest.approx(np.full(ACT_DIM, 100.0))


@pytest.mark.parametrize('spec_fn', [vel_spec, stand_spec])
def test_target_is_default_plus_scaled_action(spec_fn):
    spec = spec_fn()
    session = FakeSession(spec.obs_dim, np.full(ACT_DIM, 2.0, dtype=np.float32))
    policy = make_policy(spec, session)
    zeros = np.zeros(len(spec.obs_joint_names))
    target = policy.step(joint_pos=zeros, joint_vel=zeros, ang_vel=(0, 0, 0),
                         quat_xyzw=(0, 0, 0, 1), command=(0, 0, 0, 0.74))
    assert target == pytest.approx(np.full(ACT_DIM, DEFAULT_POS + ACTION_SCALE * 2.0))


def test_target_may_leave_the_joint_range():
    """底层是 PD，靠到行程边上还想要力就必须把目标顶到行程之外。

    训练侧 mjlab 建 <position> 执行器时 ctrllimited=False，实测 1.0 m/s 前进时
    18.6% 的拍目标在硬行程之外。那个行程不能拿来当部署侧的闸。
    """
    # waist_roll 行程 ±0.520，ctrlrange ±2.274。目标落在两者之间时必须原样通过。
    spec = stand_spec(default_pos=0.0, action_scale=1.0)
    policy = make_policy(
        spec, FakeSession(spec.obs_dim, np.full(ACT_DIM, -0.763, dtype=np.float32)),
        limit=2.274)
    zeros = np.zeros(STAND_JOINTS)
    target = policy.step(joint_pos=zeros, joint_vel=zeros, ang_vel=(0, 0, 0),
                         quat_xyzw=(0, 0, 0, 1), command=(0, 0, 0, 0.74))
    assert target == pytest.approx(np.full(ACT_DIM, -0.763))


def test_target_is_clipped_at_ctrlrange():
    """超过 ctrlrange 后力矩已饱和，裁在那里只为了拦跑飞的输出。"""
    spec = stand_spec(default_pos=0.0, action_scale=1.0)
    policy = make_policy(
        spec, FakeSession(spec.obs_dim, np.full(ACT_DIM, 500.0, dtype=np.float32)),
        limit=2.274)
    zeros = np.zeros(STAND_JOINTS)
    target = policy.step(joint_pos=zeros, joint_vel=zeros, ang_vel=(0, 0, 0),
                         quat_xyzw=(0, 0, 0, 1), command=(0, 0, 0, 0.74))
    assert target == pytest.approx(np.full(ACT_DIM, 2.274))


def test_reset_clears_action_and_phase():
    spec = vel_spec()
    session = FakeSession(spec.obs_dim, np.full(ACT_DIM, 3.0, dtype=np.float32))
    policy = make_policy(spec, session)
    walking = (0.5, 0.0, 0.0, 0.74)
    zeros = np.zeros(ACT_DIM)
    for _ in range(5):
        policy.step(joint_pos=zeros, joint_vel=zeros, ang_vel=(0, 0, 0),
                    quat_xyzw=(0, 0, 0, 1), command=walking)
    policy.reset()
    policy.step(joint_pos=zeros, joint_vel=zeros, ang_vel=(0, 0, 0),
                quat_xyzw=(0, 0, 0, 1), command=walking)
    assert session.last_obs[10:12] == pytest.approx([0.0, 1.0])


def test_non_finite_output_raises():
    spec = stand_spec()
    session = FakeSession(spec.obs_dim, np.full(ACT_DIM, np.nan, dtype=np.float32))
    policy = make_policy(spec, session)
    zeros = np.zeros(STAND_JOINTS)
    with pytest.raises(ValueError):
        policy.step(joint_pos=zeros, joint_vel=zeros, ang_vel=(0, 0, 0),
                    quat_xyzw=(0, 0, 0, 1), command=(0, 0, 0, 0.74))


def test_spec_mismatch_is_rejected():
    spec = stand_spec()
    actions = list(spec.action_joint_names)
    observed = list(spec.obs_joint_names)
    spec_matches(spec, actions, observed)
    with pytest.raises(ValueError):
        spec_matches(spec, actions[::-1], observed)  # 顺序反了 = 左右互换级别的事故
    with pytest.raises(ValueError):
        spec_matches(spec, actions, observed[:ACT_DIM])  # 漏掉手臂 = 观测整段错位


def test_velocity_spec_passes_with_identical_lists():
    """老契约里观测关节就是动作关节，节点会把同一份列表传两次。"""
    spec = vel_spec()
    names = list(spec.action_joint_names)
    spec_matches(spec, names, names)
