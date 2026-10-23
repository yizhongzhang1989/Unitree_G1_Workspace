"""Footstep GRU and frozen-anchor scheduling, without ROS or hardware writes."""

from collections import deque
from collections.abc import Callable
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import onnxruntime
import pinocchio as pin

from g1_motion_session.policy_support import PolicySpec, projected_gravity
from .footstep_planner import WorldFootstepQueue


def _heading(rotation):
    return math.atan2(rotation[1, 0], rotation[0, 0])


def _rotation(yaw):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.array(((cosine, -sine, 0.), (sine, cosine, 0.), (0., 0., 1.)))


class FootstepScheduler:
    def __init__(self, contract, control_dt=0.02):
        phase = contract['phase']
        self.touchdown = tuple(phase[side + '_touchdown_rad'] for side in ('left', 'right'))
        self.windows = phase['contact_schedule']['double_support_windows_rad']
        self.liftoff = phase['contact_schedule']['liftoff_rad']
        self._maximum_advance = min(high - center for center, (_, high) in zip(self.touchdown, self.windows))
        self._control_dt = control_dt
        self.phase = 0.
        self.frequency = 0.
        self.anchor_id = 0
        self.anchor = np.zeros(3)
        self.anchor_side = 1
        self.slots = np.zeros((4, 3))
        self._queues = (deque(), deque())
        self._stopping = False
        self.initialized = False
        self._preview_initialized = [False, False]

    def reset(self, feet, first_swing=0):
        self.anchor_side = 1 - first_swing
        self.anchor[:] = feet[self.anchor_side]
        self.phase = self.touchdown[self.anchor_side]
        self.frequency = 0.
        self.anchor_id += 1
        self.slots[:2] = feet
        self.slots[2:] = feet
        self._queues[0].clear()
        self._queues[1].clear()
        self._stopping = False
        self.initialized = True
        self._preview_initialized[:] = [False, False]

    def set_plan(self, sides, poses, frequency, anchor_id):
        if not self.initialized or anchor_id != self.anchor_id:
            raise ValueError('footstep plan requires the current initialized anchor_id')
        if not math.isfinite(frequency) or frequency < 0.:
            raise ValueError('footstep frequency must be finite and nonnegative')
        if math.tau * frequency * self._control_dt >= self._maximum_advance:
            raise ValueError('footstep frequency exceeds the phase sampling limit')
        goals = np.asarray(poses, dtype=np.float64).reshape(-1, 3)
        if len(sides) != len(goals) or not np.all(np.isfinite(goals)):
            raise ValueError('footstep plan must contain one finite pose per side')
        if any(side not in (0, 1) for side in sides):
            raise ValueError('footstep side must be left=0 or right=1')
        cosine, sine = math.cos(self.anchor[2]), math.sin(self.anchor[2])
        for side, pose in zip(sides, goals):
            goal = np.array((self.anchor[0] + cosine * pose[0] - sine * pose[1],
                             self.anchor[1] + sine * pose[0] + cosine * pose[1],
                             math.remainder(self.anchor[2] + pose[2], math.tau)))
            self._queues[side].append(goal)
        for side in (0, 1):
            if self._queues[side] and not self._preview_initialized[side]:
                self.slots[side + 2] = self._queues[side].popleft()
                self._preview_initialized[side] = True
        self._stopping = frequency == 0. and self.frequency > 0.
        if not self._stopping:
            self.frequency = frequency

    def stop(self):
        self._stopping = self.frequency > 0.

    def advance(self, dt, feet):
        if self.frequency == 0.:
            return
        advance = math.tau * self.frequency * dt
        old_phase = self.phase
        for side, liftoff in enumerate(self.liftoff):
            distance = (liftoff - old_phase) % math.tau
            if 0. < distance <= advance:
                self._preview_initialized[side] = True
        for side, touchdown in enumerate(self.touchdown):
            distance = (touchdown - old_phase) % math.tau
            if 0. < distance <= advance:
                self.slots[side] = self.slots[side + 2]
                self._preview_initialized[side] = bool(self._queues[side])
                self.slots[side + 2] = (self._queues[side].popleft()
                                        if self._preview_initialized[side] else self.slots[side])
                self.anchor[:] = feet[side]
                self.anchor_side = side
                self.anchor_id += 1
        self.phase = (old_phase + advance) % math.tau
        landed_window = any(
            center <= self.phase < high for center, (_, high) in zip(self.touchdown, self.windows))
        if self._stopping and landed_window:
            self.frequency = 0.
            self._stopping = False

    def encode(self, out):
        cosine, sine = math.cos(self.anchor[2]), math.sin(self.anchor[2])
        for index in range(2):
            slot = self.slots[index + 2 if self.frequency else index]
            delta_x, delta_y = slot[:2] - self.anchor[:2]
            out[index * 3] = cosine * delta_x + sine * delta_y
            out[index * 3 + 1] = -sine * delta_x + cosine * delta_y
            out[index * 3 + 2] = math.remainder(slot[2] - self.anchor[2], math.tau)


class FootstepPolicy:
    name = 'footstep'
    command = ()

    def __init__(self, path, *, joints, control_dt, sole_offsets,
                 encoder_offsets, first_swing=0):
        options = onnxruntime.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self._session = onnxruntime.InferenceSession(
            str(path), options, providers=['CPUExecutionProvider'])
        self._model_sha256 = hashlib.sha256(Path(path).read_bytes()).hexdigest()
        self.contract = json.loads(self._session.get_modelmeta().custom_metadata_map['footstep_contract'])
        contract = self.contract
        if control_dt != contract['control_dt_s']:
            raise ValueError('footstep control period must match the model contract')
        if [(port.name, port.shape) for port in self._session.get_inputs()] != [
                ('obs', [1, 79]), ('h_in', [1, 1, 64])]:
            raise ValueError('footstep ONNX input contract mismatch')
        if contract['footsteps']['slots'] != ['L_next', 'R_next']:
            raise ValueError('footstep model requires fixed left/right next landing slots')
        actions = contract['actions']
        self._offsets = np.asarray(encoder_offsets, dtype=np.float64)
        self._sole_offsets = np.asarray(sole_offsets, dtype=np.float64).reshape(2, 3)
        if self._offsets.shape != (29,) or not np.all(np.isfinite(self._sole_offsets)):
            raise ValueError('footstep requires 29 encoder offsets and two calibrated sole offsets')
        self.spec = PolicySpec(
            joint_names=tuple(actions['joint_names']),
            default_pos=np.asarray(actions['default_joint_pos']) - self._offsets[:15],
            action_scale=np.asarray(actions['scale']), obs_dim=79, action_dim=15,
            hidden_name='h_in', hidden_shape=(1, 1, 64))
        self._joint_slots = [joints.index(name) for name in contract['joint_names']]
        self._dt = control_dt
        self._first_swing = first_swing
        self.scheduler = FootstepScheduler(contract, control_dt)
        self.theta = 0.
        self._obs = np.empty((1, 79), dtype=np.float32)
        self._feed = {'obs': self._obs, 'h_in': np.zeros((1, 1, 64), dtype=np.float32)}
        self._model = None
        self._origin_rotation = np.eye(3)
        self._foot_positions = np.zeros((2, 3))
        self._last_command = 0.
        self.command_timeout = 0.5
        self.controller: WorldFootstepQueue | None = None
        self.world_transform: Callable[..., tuple[np.ndarray, np.ndarray]] | None = None
        self.feet = np.zeros((2, 3))
        self.robot_pose = np.zeros(3)
        self.stop_trace = deque(maxlen=round(3. / control_dt))
        self._trace_tail = 0

    @property
    def ready(self):
        return self._model is not None

    def status(self):
        scheduler = self.scheduler
        return {'footstep': {
            'phase': scheduler.phase, 'frequency': scheduler.frequency,
            'theta': self.theta,
            'anchor_id': scheduler.anchor_id, 'anchor_side': scheduler.anchor_side,
            'anchor': scheduler.anchor.tolist(), 'slots': scheduler.slots.tolist(),
            'feet': self.feet.tolist(),
            'robot_pose': self.robot_pose.tolist()}}

    def bind_description(self, description):
        full = pin.buildModelFromXML(description)
        missing = [name for name in self.spec.joint_names if not full.existJointName(name)]
        frames = ('torso_link', 'left_ankle_roll_link', 'right_ankle_roll_link')
        missing.extend(frame for frame in frames if not full.existFrame(frame))
        if missing:
            raise ValueError(f'footstep model is missing required joints or frames: {missing}')
        keep = {full.getJointId(name) for name in self.spec.joint_names}
        model = pin.buildReducedModel(
            full, [index for index in range(1, full.njoints) if index not in keep], pin.neutral(full))
        if model.nq != 15 or model.nv != 15:
            raise ValueError('footstep kinematics requires 15 scalar joints')
        model_slots = [self.contract['joint_names'].index(name) for name in model.names[1:]]
        self._data = model.createData()
        self._model_slots = model_slots
        self._torso_frame = model.getFrameId(frames[0])
        self._foot_frames = (model.getFrameId(frames[1]), model.getFrameId(frames[2]))
        self._model = model

    def _update_placements(self, state):
        position = state.position[self._joint_slots] + self._offsets
        velocity = state.velocity[self._joint_slots]
        pin.forwardKinematics(self._model, self._data,
                              position[self._model_slots], velocity[self._model_slots])
        pin.updateFramePlacements(self._model, self._data)
        return position, velocity

    def observe(self, state):
        position, velocity = self._update_placements(state)
        quaternion = np.asarray(state.orientation, dtype=np.float64)
        quaternion /= np.linalg.norm(quaternion)
        rotation = self._origin_rotation @ pin.Quaternion(quaternion).matrix()
        translation = None
        if self.world_transform is not None:
            rotation, translation = self.world_transform(state, self._data.oMf[self._torso_frame])
        feet = np.empty((2, 3))
        for side, frame in enumerate(self._foot_frames):
            pose = self._data.oMf[frame]
            self._foot_positions[side] = rotation @ (pose.translation + pose.rotation @ self._sole_offsets[side])
            feet[side, :2] = self._foot_positions[side, :2]
            feet[side, 2] = _heading(rotation @ pose.rotation)
        anchor_side = self.scheduler.anchor_side
        if translation is None:
            translation = np.array((*self.scheduler.anchor[:2], 0.)) - self._foot_positions[anchor_side]
        feet[:, :2] += translation[:2]
        gravity = projected_gravity(state.orientation)
        torso = self._data.oMf[self._torso_frame]
        torso_angular = (torso.rotation.T @ state.angular_velocity
                         + pin.getFrameVelocity(self._model, self._data, self._torso_frame, pin.LOCAL).angular)
        obs = self._obs[0]
        obs[:29] = position
        obs[29:58] = velocity
        obs[58:61] = state.angular_velocity
        obs[61:64] = gravity
        obs[64:67] = torso_angular
        obs[67:70] = torso.rotation.T @ gravity
        self.feet[:] = feet
        torso_world = translation + rotation @ torso.translation
        self.robot_pose[:] = (*torso_world[:2], _heading(rotation @ torso.rotation))
        return feet

    def torso_pose(self, state):
        self._update_placements(state)
        return self._data.oMf[self._torso_frame].copy()

    def reset(self, state, now):
        self._origin_rotation[:] = np.eye(3)
        self.scheduler.anchor[:] = 0.
        self.scheduler.anchor_side = 1 - self._first_swing
        feet = self.observe(state)
        self._origin_rotation[:] = _rotation(-feet[self.scheduler.anchor_side, 2])
        feet = self.observe(state)
        self.scheduler.reset(feet, self._first_swing)
        self.theta = 0.
        self._feed['h_in'].fill(0.)
        self._trace_tail = 0
        self._last_command = now

    def set_theta(self, theta):
        theta = float(theta)
        if not math.isfinite(theta) or not -math.pi / 2 - 1e-9 <= theta <= math.pi / 2 + 1e-9:
            raise ValueError('footstep theta must be within [-pi/2, pi/2] rad')
        self.theta = float(np.clip(theta, -math.pi / 2, math.pi / 2))

    def set_footsteps(self, anchor_id, frequency, sides, poses, now, theta=None):
        if self.controller is not None:
            raise ValueError('World queue owns footsteps; legacy plans are disabled in web mode')
        self.scheduler.set_plan(sides, poses, frequency, anchor_id)
        if theta is not None:
            self.set_theta(theta)
        self._last_command = now

    def step(self, state, now):
        feet = self.observe(state)
        if self.controller is not None:
            self.controller.tick(self._dt, feet)
        else:
            if now - self._last_command > self.command_timeout:
                self.scheduler.stop()
            self.scheduler.advance(self._dt, feet)
        obs = self._obs[0]
        obs[70] = self.scheduler.phase
        obs[71] = self.scheduler.frequency
        self.scheduler.encode(obs[72:78])
        obs[78] = self.theta
        if not np.all(np.isfinite(obs)):
            raise ValueError('footstep observation contains non-finite values')
        h_in = self._feed['h_in']
        actions, hidden = self._session.run(None, self._feed)
        if not isinstance(actions, np.ndarray) or not isinstance(hidden, np.ndarray):
            raise ValueError('footstep inference must return dense arrays')
        if not np.all(np.isfinite(actions)) or not np.all(np.isfinite(hidden)):
            raise ValueError('footstep inference contains non-finite values')
        self._feed['h_in'] = hidden
        target = self.spec.default_pos + self.spec.action_scale * actions[0]
        if self.scheduler.frequency > 0. or self._trace_tail:
            self.stop_trace.append((now, self._obs.copy(), h_in.copy(), target.copy(),
                                    feet.copy(), self._foot_positions.copy()))
            self._trace_tail = (round(1. / self._dt) if self.scheduler.frequency > 0.
                                else self._trace_tail - 1)
        return target

    def save_stop_trace(self, path):
        if not self.stop_trace:
            raise ValueError('No walking or stopping samples have been captured')
        times, observations, hidden, targets, feet, foot_relative_pelvis = zip(*self.stop_trace)
        np.savez_compressed(
            path, time_s=times, obs=np.stack(observations), h_in=np.stack(hidden),
            target=np.stack(targets), feet=np.stack(feet),
            foot_relative_pelvis=np.stack(foot_relative_pelvis),
            model_sha256=self._model_sha256, contract_version=self.contract['version'],
            joint_names=self.contract['joint_names'], action_joint_names=self.spec.joint_names,
            action_default_pos=self.spec.default_pos, action_scale=self.spec.action_scale,
            encoder_offsets=self._offsets, sole_offsets=self._sole_offsets, control_dt_s=self._dt)
