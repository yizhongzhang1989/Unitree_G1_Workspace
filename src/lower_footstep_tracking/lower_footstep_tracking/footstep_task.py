"""Footstep input, standing and recurrent execution for the lower joint mask."""

import math
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from lower_footstep_tracking.msg import FootstepPlan

from g1_motion_session.policy_support import spec_matches
from g1_motion_session.task_base import State, TaskBase
from .footstep_policy import FootstepPolicy


class FootstepTask(TaskBase):
    def __init__(self, *, ros_input=True):
        super().__init__('lower_footstep_tracking', slots=range(15))
        p = self.declare_parameter
        self.source = p('lower_source', 'planner').get_parameter_value().string_value
        names = list(p('policy_joints', self.joints31[:15]).get_parameter_value().string_array_value)
        self.policy_slots = [self.joints31.index(name) for name in names]
        value = p('policy_path', 'package://lower_footstep_tracking/config/policy.onnx')\
            .get_parameter_value().string_value
        if value.startswith('package://'):
            package, _, relative = value[len('package://'):].partition('/')
            path = Path(get_package_share_directory(package)) / relative
        else:
            path = Path(value).expanduser()
        sole_offsets = p(
            'foot_sole_offsets', Parameter.Type.DOUBLE_ARRAY).get_parameter_value().double_array_value
        if len(sole_offsets) != 6:
            raise ValueError('footstep requires calibrated foot_sole_offsets (left xyz, right xyz)')
        encoder_offsets = p('foot_encoder_offsets', [0.0] * 29).get_parameter_value().double_array_value
        first_swing = p('foot_first_swing', 0).get_parameter_value().integer_value
        self.runtime = FootstepPolicy(
            path, joints=self.joints31, control_dt=self.dt, sole_offsets=sole_offsets,
            encoder_offsets=encoder_offsets, first_swing=first_swing)
        self.runtime.command_timeout = p('command_timeout_s', 0.5).get_parameter_value().double_value
        spec_matches(self.runtime.spec, names)
        self.waist_output_cutoff_hz = p('waist_output_cutoff_hz', 10.0).get_parameter_value().double_value
        if not math.isfinite(self.waist_output_cutoff_hz) or not 0. <= self.waist_output_cutoff_hz < .5 / self.dt:
            raise ValueError('waist_output_cutoff_hz must be zero or below the control Nyquist frequency')
        self._waist_slots = [self.joints31.index(name) for name in
                             ('waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint')]
        tangent = math.tan(math.pi * self.waist_output_cutoff_hz * self.dt)
        self._waist_b = tangent / (1. + tangent)
        self._waist_a = (tangent - 1.) / (1. + tangent)
        self._waist_previous_input = np.zeros(3)
        self._waist_previous_output = np.zeros(3)
        self._stand_goal[self.policy_slots] = self.runtime.spec.default_pos
        self._stopped = True
        if ros_input:
            self.create_subscription(
                FootstepPlan, f'/motion_control/lower/input/{self.source}/footsteps', self.on_plan,
                QoSProfile(depth=4, reliability=ReliabilityPolicy.BEST_EFFORT),
                callback_group=self.control_group)
        if not self.has_parameter('robot_description_topic'):
            p('robot_description_topic', '/robot_description')
        description = self.get_parameter('robot_description_topic').get_parameter_value().string_value
        self.create_subscription(
            String, description, self.on_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(self.dt, self.control, callback_group=self.control_group)

    @property
    def ready(self):
        runtime = getattr(self, 'runtime', None)
        return runtime is not None and runtime.ready

    def on_description(self, message):
        with self.execution_lock:
            if not self.ready:
                try:
                    self.runtime.bind_description(message.data)
                except Exception as error:
                    self.get_logger().error(f'脚步模型绑定失败: {error}')

    def on_plan(self, message):
        self.set_plan(message.anchor_id, message.frequency, message.sides, message.poses, message.theta)

    def set_plan(self, anchor_id, frequency, sides, poses, theta=0.):
        with self.execution_lock:
            try:
                self.runtime.set_footsteps(
                    anchor_id, frequency, sides, poses, self.now(), theta=theta)
            except ValueError as error:
                self.get_logger().warning(f'脚步计划拒绝: {error}', throttle_duration_sec=1.)

    def start(self, snapshot):
        with self.execution_lock:
            self.runtime.reset(snapshot, self.now())
            self._reset_waist_filter(snapshot)
            self._stopped = False

    def _reset_waist_filter(self, snapshot):
        self._waist_previous_input[:] = snapshot.position[self._waist_slots]
        self._waist_previous_output[:] = self._waist_previous_input

    def stop(self):
        with self.execution_lock:
            self._stopped = True

    def control(self):
        with self.execution_lock:
            with self.lock:
                state, generation = self.state, self.generation
                if self._stopped or state is not State.RUNNING:
                    return
                snapshot, now = self.snapshot(), self.now()
                reason = self.stale_reason()
            if reason:
                self.estop(reason)
                return
            try:
                target = self.runtime.step(snapshot, now)
                self.output.positions[self.policy_slots] = target
                if self.waist_output_cutoff_hz > 0.:
                    waist = self.output.positions[self._waist_slots].copy()
                    filtered = (self._waist_b * (waist + self._waist_previous_input)
                                - self._waist_a * self._waist_previous_output)
                    self._waist_previous_input[:] = waist
                    self._waist_previous_output[:] = filtered
                    self.output.positions[self._waist_slots] = filtered
            except Exception as error:
                self.estop(f'下肢任务失败: {error}')
                return
            with self.lock:
                if generation == self.generation and self.state is State.RUNNING:
                    self.output.publish()

    def status(self):
        return dict(lower_ready=self.ready, lower_executor=self.runtime.name,
                    lower_source=self.source,
                    command=[round(float(value), 3) for value in self.runtime.command],
                    **self.runtime.status())


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor()
    try:
        node = FootstepTask()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
