"""Lower-body execution owns task commands and policy state, not hardware."""

import numpy as np

from g1_motion_session.policy_support import spec_matches
from lower_velocity_tracking.policy_runtime import LocomotionPolicy, load_policy


class VelocityPolicy:
    name = 'velocity'
    ready = True

    def status(self) -> dict[str, bool | str | list[float]]:
        return {}

    def __init__(self, runtime, spec, *, slots, command_lower, command_upper,
                 command_rate, initial_height, command_timeout) -> None:
        self.runtime = runtime
        self.spec = spec
        self.slots = slots
        self._lower = command_lower
        self._upper = command_upper
        self._rate = command_rate
        self._initial_height = initial_height
        self._timeout = command_timeout
        self._request = np.array([0.0, 0.0, 0.0, initial_height])
        self.command = self._request.copy()
        self._stamp = 0.0
        self._delta = np.empty(4)

    def set_command(self, values, now) -> None:
        self._request = np.clip(values, self._lower, self._upper)
        self._stamp = now

    def reset(self, state, now) -> None:
        self.runtime.reset()
        self._request = np.array([0.0, 0.0, 0.0, self._initial_height])
        self.command = self._request.copy()
        self._stamp = now

    def step(self, state, now) -> np.ndarray:
        request = self._request.copy()
        if now - self._stamp > self._timeout:
            request[:3] = 0.0
        np.subtract(request, self.command, out=self._delta)
        np.maximum(self._delta, -self._rate, out=self._delta)
        np.minimum(self._delta, self._rate, out=self._delta)
        np.add(self.command, self._delta, out=self.command)
        np.maximum(self.command, self._lower, out=self.command)
        np.minimum(self.command, self._upper, out=self.command)
        return self.runtime.step(
            joint_pos=state.position[self.slots], joint_vel=state.velocity[self.slots],
            ang_vel=state.angular_velocity, quat_xyzw=state.orientation,
            command=self.command)


def create_velocity(p, path, policy_joints, slots, dt) -> VelocityPolicy:
    timeout = p('command_timeout_s', 0.5).get_parameter_value().double_value
    lower = np.asarray(p('target_lower_limits', [0.0] * len(policy_joints)).get_parameter_value().double_array_value)
    upper = np.asarray(p('target_upper_limits', [0.0] * len(policy_joints)).get_parameter_value().double_array_value)
    limits = p('command_limits', [-0.5, 0.8, -0.5, 0.5, -1.5, 1.5, 0.50, 0.78]).get_parameter_value().double_array_value
    accel = p('linear_accel_limit', 1.5).get_parameter_value().double_value
    angular_accel = p('angular_accel_limit', 3.0).get_parameter_value().double_value
    height_rate = p('height_rate_limit', 0.15).get_parameter_value().double_value
    height = p('initial_height', 0.74).get_parameter_value().double_value
    session, spec = load_policy(str(path))
    runtime = LocomotionPolicy(session, spec, control_dt=dt,
                               target_lower=lower, target_upper=upper)
    result = VelocityPolicy(
        runtime, spec, slots=slots, command_lower=np.asarray(limits[0::2]),
        command_upper=np.asarray(limits[1::2]),
        command_rate=dt * np.array([accel, accel, angular_accel, height_rate]),
        initial_height=height, command_timeout=timeout)
    spec_matches(result.spec, policy_joints)
    return result
