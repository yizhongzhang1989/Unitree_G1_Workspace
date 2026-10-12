"""Follow measured arm joints and retain the requested gripper positions."""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray

from g1_motion_session.task_base import State, TaskBase


class FloatTask(TaskBase):
    def __init__(self) -> None:
        super().__init__('upper_float', slots=range(15, 31), imu_required=False)
        self.declare('arm_joints', self.joints31[15:29])
        self.arm_joints = list(self.get_parameter('arm_joints').get_parameter_value().string_array_value)
        self.arm_slots = [self.joints31.index(name) for name in self.arm_joints]
        if len(self.arm_slots) != 14 or set(self.arm_slots) != set(range(15, 29)):
            raise ValueError('arm_joints must cover the 14 arm slots')
        self.declare('gripper_joints', self.joints31[29:])
        grippers = list(self.get_parameter('gripper_joints').get_parameter_value().string_array_value)
        self.gripper_slots = [self.joints31.index(name) for name in grippers]
        if len(grippers) != 2 or set(self.gripper_slots) != {29, 30}:
            raise ValueError('gripper_joints must cover the two gripper slots')
        self.declare('gripper_limits', [0.0, 2.76377472169236])
        self._grip_lo, self._grip_hi = self.get_parameter('gripper_limits').get_parameter_value().double_array_value
        self.declare('gripper_source', 'keyboard')
        self.gripper_source = self.get_parameter('gripper_source').get_parameter_value().string_value
        self.declare('gripper_command_topic', f'/motion_control/upper/input/{self.gripper_source}')
        topic = self.get_parameter('gripper_command_topic').get_parameter_value().string_value
        self.create_subscription(Float64MultiArray, topic, self._on_gripper,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                                 callback_group=self.control_group)
        self.create_timer(self.dt, self._control, callback_group=self.control_group)

    def prepared(self, snapshot) -> None:
        self.state = State.RUNNING

    def _on_gripper(self, message: Float64MultiArray) -> None:
        grip = np.asarray(message.data, dtype=float)
        if grip.shape != (2,) or not np.isfinite(grip).all():
            return
        with self.execution_lock, self.lock:
            if self.state is State.RUNNING and not self._preparing:
                self.output.positions[self.gripper_slots] = np.minimum(
                    np.maximum(grip, self._grip_lo), self._grip_hi)

    def _control(self) -> None:
        with self.execution_lock, self.lock:
            if self.state is not State.RUNNING or self._preparing:
                return
            target = self.output.positions.copy()
            for slot in self.arm_slots:
                side = self.joints31[slot].split('_', 1)[0]
                if side not in self._homes:
                    target[slot] = self._q[slot]
            self.publish_target(target)

    def status(self) -> dict[str, bool | str | list[float]]:
        return dict(upper_ready=True, upper_executor='float', arm_mode='passthrough',
                    upper_source='none', gripper_source=self.gripper_source,
                    grip=self.output.positions[self.gripper_slots].tolist(),
                    joint_positions=self.output.positions[self.arm_slots].tolist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FloatTask()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
