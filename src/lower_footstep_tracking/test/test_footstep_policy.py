import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import onnxruntime
import pytest

from lower_footstep_tracking.footstep_policy import FootstepPolicy, FootstepScheduler


ROOT = Path(__file__).resolve().parents[3]
ASSETS = Path(__file__).resolve().parents[1] / 'config'
MODEL = ASSETS / 'policy.onnx'
CONTRACT = json.loads(onnxruntime.InferenceSession(
    str(MODEL), providers=['CPUExecutionProvider']).get_modelmeta().custom_metadata_map['footstep_contract'])


def scheduler():
    result = FootstepScheduler(CONTRACT)
    result.reset(np.array(((0., .1, 0.), (0., -.1, 0.))))
    return result


def test_touchdown_consumes_fixed_side_and_reanchors_without_moving_targets():
    result = scheduler()
    result.set_plan([0, 1, 0], [[.2, .2, 0.], [.4, 0., 0.], [.6, .2, 0.]],
                    1., result.anchor_id)
    initial = result.slots.copy()
    feet = np.array(((.22, .12, .1), (0., -.1, 0.)))
    result.phase = math.pi / 2 - .05
    result.advance(.02, feet)
    np.testing.assert_array_equal(result.slots[0], initial[2])
    np.testing.assert_array_equal(result.slots[1], initial[1])
    np.testing.assert_array_equal(result.anchor, feet[0])
    assert result.anchor_side == 0
    anchor_id = result.anchor_id
    result.advance(.02, feet)
    assert result.anchor_id == anchor_id
    out = np.empty(6)
    result.encode(out)
    for index, goal in enumerate(result.slots[2:]):
        cosine, sine = math.cos(result.anchor[2]), math.sin(result.anchor[2])
        actual = out[index * 3:index * 3 + 3]
        restored = (result.anchor[0] + cosine * actual[0] - sine * actual[1],
                    result.anchor[1] + sine * actual[0] + cosine * actual[1])
        np.testing.assert_allclose(restored, goal[:2], atol=1e-12)


def test_real_footstep_model_observation_and_recurrent_standing():
    encoder_offset = .1
    joints = CONTRACT['joint_names'] + ['left_eccentric_joint', 'right_eccentric_joint']
    policy = FootstepPolicy(MODEL, joints=joints, control_dt=.02,
                            sole_offsets=[0., 0., -.035] * 2, encoder_offsets=[encoder_offset] * 29)
    description = (ROOT / 'src/unitree_g1_description/model/final.urdf').read_text()
    policy.bind_description(description)
    state = SimpleNamespace(position=np.r_[np.array(CONTRACT['default_joint_pos']) - encoder_offset, 0., 0.],
                            velocity=np.zeros(31), orientation=(0., 0., 0., 1.),
                            angular_velocity=(0., 0., 0.))
    policy.reset(state, 0.)
    phase = policy.scheduler.phase
    first = policy.step(state, .02)
    hidden = policy._feed['h_in'].copy()
    second = policy.step(state, .04)
    assert first.shape == second.shape == (15,)
    assert np.all(np.isfinite(second))
    assert np.any(hidden)
    assert np.any(policy._feed['h_in'] != hidden)
    np.testing.assert_allclose(policy._obs[0, :29], state.position[:29] + encoder_offset)
    assert policy.scheduler.phase == phase and policy.scheduler.frequency == 0.
    assert policy.spec.obs_dim == 79 and policy._obs.shape == (1, 79)
    assert policy.theta == 0. and policy._obs[0, 78] == pytest.approx(0.)
    standing = np.empty(6)
    policy.scheduler.encode(standing)
    np.testing.assert_allclose(policy._obs[0, 72:78], standing)
    hidden_before_plan = policy._feed['h_in'].copy()
    goals = np.array(((.3, .2, .1), (.4, 0., -.2)))
    policy.set_footsteps(policy.scheduler.anchor_id, 1., [0, 1], goals.reshape(-1), .04, theta=.35)
    np.testing.assert_array_equal(policy._feed['h_in'], hidden_before_plan)
    expected_obs = policy._obs.copy()
    expected_obs[0, 70] = (phase + math.tau * .02) % math.tau
    expected_obs[0, 71] = 1.
    expected_obs[0, 72:78] = goals.reshape(-1)
    expected_obs[0, 78] = .35
    actions, expected_hidden = policy._session.run(
        None, {'obs': expected_obs, 'h_in': hidden_before_plan})
    assert isinstance(actions, np.ndarray)
    assert isinstance(expected_hidden, np.ndarray)
    target = policy.step(state, .06)
    np.testing.assert_allclose(policy._obs, expected_obs, atol=1e-7)
    np.testing.assert_allclose(target, np.asarray(CONTRACT['actions']['default_joint_pos']) - encoder_offset
                               + np.asarray(CONTRACT['actions']['scale']) * actions[0])
    np.testing.assert_array_equal(policy._feed['h_in'], expected_hidden)


def test_stop_consumes_current_landing_before_freezing_holds():
    result = scheduler()
    result.set_plan([0], [[.3, .2, 0.]], 1., result.anchor_id)
    result.phase = 1.3
    feet = result.slots[:2].copy()
    feet[0, 0] = .3
    result.stop()
    result.advance(.02, feet)
    assert result.frequency == 1.
    for _ in range(3):
        result.advance(.02, feet)
    assert result.frequency == 0.
    assert result.slots[0, 0] == pytest.approx(.3)
    encoded = np.empty(6)
    result.encode(encoded)
    np.testing.assert_allclose(encoded, (result.slots[:2] - result.anchor).reshape(-1))


def test_invalid_frequency_and_stale_anchor_do_not_mutate_plan():
    result = scheduler()
    before = result.slots.copy()
    with pytest.raises(ValueError, match='phase sampling limit'):
        result.set_plan([0], [[.3, .2, 0.]], 2.5, result.anchor_id)
    with pytest.raises(ValueError, match='anchor_id'):
        result.set_plan([0], [[.3, .2, 0.]], 1., result.anchor_id - 1)
    assert result.frequency == 0. and not result._queues[0]
    np.testing.assert_array_equal(result.slots, before)


def test_world_fk_transform_and_managed_standing_preserves_gru():
    from lower_footstep_tracking.footstep_planner import WorldFootstepQueue

    policy = FootstepPolicy(MODEL, joints=CONTRACT['joint_names'], control_dt=.02,
                            sole_offsets=[0., 0., -.035] * 2, encoder_offsets=[0.] * 29)
    policy.bind_description((ROOT / 'src/unitree_g1_description/model/final.urdf').read_text())
    state = SimpleNamespace(position=np.array(CONTRACT['default_joint_pos']),
                            velocity=np.zeros(29), orientation=(0., 0., 0., 1.),
                            angular_velocity=(0., 0., 0.))
    policy.reset(state, 0.)
    local = policy.observe(state)
    correction = np.array((2., -1., .7))
    rotation = np.array(((0., -1., 0.), (1., 0., 0.), (0., 0., 1.)))
    policy.world_transform = lambda snapshot, torso: (rotation, correction)
    actual = policy.observe(state)
    for side in (0, 1):
        placement = policy._data.oMf[policy._foot_frames[side]]
        expected = correction + rotation @ (placement.translation + placement.rotation @ policy._sole_offsets[side])
        np.testing.assert_allclose(actual[side, :2], expected[:2])
    assert not np.allclose(actual, local)
    policy.scheduler.reset(actual)
    policy.controller = WorldFootstepQueue(policy.scheduler)
    policy.set_theta(-math.pi / 4)
    policy.step(state, .02)
    assert policy._obs[0, 78] == pytest.approx(-math.pi / 4)
    hidden = policy._feed['h_in'].copy()
    policy.step(state, .04)
    assert np.any(policy._feed['h_in'] != hidden)
    with pytest.raises(ValueError, match='theta'):
        policy.set_theta(math.pi / 2 + .01)
    with pytest.raises(ValueError, match='World queue'):
        policy.set_footsteps(policy.scheduler.anchor_id, 1., [], [], .04)
