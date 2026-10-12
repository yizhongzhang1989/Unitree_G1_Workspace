"""Exchange measured arm positions with configurable joint direction mapping."""

import time
from typing import cast

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from unitree_hg.msg import LowState, MotorState

from g1_motion_session.task_base import State, TaskBase


ARM_JOINTS = (
    'shoulder_pitch_joint', 'shoulder_roll_joint', 'shoulder_yaw_joint',
    'elbow_joint', 'wrist_roll_joint', 'wrist_pitch_joint', 'wrist_yaw_joint',
)


class ForceFeedbackTask(TaskBase):
    def __init__(self) -> None:
        super().__init__('upper_force_feedback', slots=range(15, 29), imu_required=False)
        self.left_slots = [self.joints31.index(f'left_{joint}') for joint in ARM_JOINTS]
        self.right_slots = [self.joints31.index(f'right_{joint}') for joint in ARM_JOINTS]
        if set(self.left_slots + self.right_slots) != set(self.slots):
            raise ValueError('joints must place the 14 arm joints in slots 15..28')
        self.arm_signs = np.asarray(self.declare('arm_signs', [1.0, -1.0, -1.0, 1.0, -1.0, 1.0, 1.0]))
        if self.arm_signs.shape != (7,) or not np.isin(self.arm_signs, [-1.0, 1.0]).all():
            raise ValueError('arm_signs must contain seven values of +1 or -1')
        self.create_timer(self.dt, self._check_feedback, callback_group=self.control_group)

    def prepared(self, snapshot) -> None:
        self.state = State.RUNNING

    def _subscribe_feedback(self, qos, callback_group) -> None:
        self.create_subscription(LowState, '/lowstate', self._on_lowstate,
                                 qos, callback_group=callback_group)

    def _on_lowstate(self, message: LowState) -> None:
        with self.execution_lock, self.lock:
            motors = cast(list[MotorState], message.motor_state)
            self._q[:29] = [motor.q for motor in motors[:29]]
            self._dq[:29] = [motor.dq for motor in motors[:29]]
            self._joint_stamp = time.monotonic()
            self._control()

    def stale_reason(self) -> str:
        if time.monotonic() - self._joint_stamp > self._state_timeout:
            return '/lowstate timeout'
        return ''

    def _check_feedback(self) -> None:
        with self.execution_lock, self.lock:
            if self.state is State.RUNNING and not self._preparing:
                reason = self.stale_reason()
                if reason:
                    self.estop(reason)

    def _control(self) -> None:
        with self.execution_lock, self.lock:
            if self.state is not State.RUNNING or self._preparing:
                return
            reason = self.stale_reason()
            if reason:
                self.estop(reason)
                return
            if not np.isfinite(self._q[self.left_slots + self.right_slots]).all():
                self.estop('non-finite arm feedback')
                return
            target = self.output.positions.copy()
            target[self.left_slots] = self.arm_signs * self._q[self.right_slots]
            target[self.right_slots] = self.arm_signs * self._q[self.left_slots]
            self.publish_target(target)

    def status(self) -> dict[str, bool | str | list[float]]:
        return dict(upper_ready=True, upper_executor='force_feedback', arm_mode='passthrough',
                    upper_source='lowstate', gripper_source='none',
                    joint_positions=self.output.positions[15:29].tolist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ForceFeedbackTask()
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
