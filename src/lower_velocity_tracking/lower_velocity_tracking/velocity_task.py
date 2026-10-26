"""Velocity input, standing and recurrent execution for the lower joint mask."""

from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray

from g1_motion_session.task_base import State, TaskBase
from lower_velocity_tracking.velocity_runtime import create_velocity


class VelocityTask(TaskBase):
    def __init__(self, *, ros_input=True) -> None:
        super().__init__('lower_velocity_tracking', slots=range(15))
        self.source = self.declare_parameter('lower_source', 'keyboard').get_parameter_value().string_value
        names = list(self.declare_parameter('policy_joints', self.joints31[:15]).get_parameter_value().string_array_value)
        self.policy_slots = [self.joints31.index(name) for name in names]
        value = self.declare_parameter(
            'policy_path', 'package://lower_velocity_tracking/config/policy.onnx').get_parameter_value().string_value
        if value.startswith('package://'):
            package, _, relative = value[len('package://'):].partition('/')
            path = Path(get_package_share_directory(package)) / relative
        else:
            path = Path(value).expanduser()
        self.runtime = create_velocity(
            lambda name, default: self.get_parameter(name) if self.has_parameter(name)
            else self.declare_parameter(name, default), path, names, self.policy_slots, self.dt)
        self._stand_goal[self.policy_slots] = self.runtime.spec.default_pos
        self._stopped = True
        if ros_input:
            self.create_subscription(
                Float64MultiArray, f'/motion_control/lower/input/{self.source}', self.on_command,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                callback_group=self.control_group)
        self.create_timer(self.dt, self.control, callback_group=self.control_group)

    @property
    def ready(self) -> bool:
        runtime = getattr(self, 'runtime', None)
        return runtime is not None and runtime.ready

    def on_command(self, message: Float64MultiArray) -> None:
        self.set_command(message.data)

    def set_command(self, command) -> None:
        values = np.asarray(command, dtype=np.float64)
        if values.shape == (4,) and np.all(np.isfinite(values)):
            with self.execution_lock:
                self.runtime.set_command(values, self.now())

    def start(self, snapshot) -> None:
        with self.execution_lock:
            self.runtime.reset(snapshot, self.now())
            self._stopped = False

    def stop(self) -> None:
        with self.execution_lock:
            self._stopped = True

    def control(self) -> None:
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
            except Exception as error:
                self.estop(f'下肢任务失败: {error}')
                return
            with self.lock:
                if generation == self.generation and self.state is State.RUNNING:
                    self.output.publish()

    def status(self) -> dict[str, bool | str | list[float]]:
        return dict(lower_ready=self.ready, lower_executor=self.runtime.name,
                    lower_source=self.source,
                    command=[round(float(value), 3) for value in self.runtime.command],
                    **self.runtime.status())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor()
    try:
        node = VelocityTask()
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
