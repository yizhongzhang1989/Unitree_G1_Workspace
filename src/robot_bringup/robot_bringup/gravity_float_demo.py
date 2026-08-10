#!/usr/bin/env python3
"""Let the arms float weightlessly so they can be pushed around by hand.

The arm entries of the target keep following ``/joint_states``. The position
controller then writes ``q_cmd = q_meas + G(q_meas) / kp``, so the motor applies

    tau = kp * (q_cmd - q_meas) - kd * dq = G(q_meas) - kd * dq

The position term cancels exactly and only gravity plus damping is left: the
arm carries its own weight at any pose and the operator only feels the damping.

Every other joint is snapshotted once and held. Tracking those too would leave
them with ``tau = -kd * dq`` -- no gravity term, because the compensation
controller only covers the arms -- and the legs would collapse under the robot.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class GravityFloatDemo(Node):
    def __init__(self) -> None:
        super().__init__("gravity_float_demo")
        self._joints = list(self.declare_parameter(
            "joints", [""]).get_parameter_value().string_array_value)
        if not self._joints or any(not name for name in self._joints):
            raise ValueError("joints must list every controller joint in order")
        floating = list(self.declare_parameter(
            "floating_joints", [""]).get_parameter_value().string_array_value)
        unknown = [name for name in floating if name not in self._joints]
        if not floating or unknown:
            raise ValueError("floating_joints must be a subset of joints: %s" % unknown)
        # Pair the slot with the name once so the hot path is a plain loop.
        self._floating = [(self._joints.index(name), name) for name in floating]

        target_topic = self.declare_parameter(
            "target_topic", "/forward_position_controller/commands"
        ).get_parameter_value().string_value
        rate = self.declare_parameter(
            "publish_rate_hz", 100.0).get_parameter_value().double_value
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        stream_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._target = None
        self._message = Float64MultiArray()
        self._publisher = self.create_publisher(
            Float64MultiArray, target_topic, stream_qos)
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, stream_qos)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            "Waiting for a complete /joint_states before floating %d of %d joints"
            % (len(floating), len(self._joints)))

    def _on_joint_states(self, message: JointState) -> None:
        measured = dict(zip(message.name, message.position))
        if self._target is None:
            # The snapshot freezes every held joint, so it needs the full vector.
            if any(name not in measured for name in self._joints):
                return
            self._target = [measured[name] for name in self._joints]
            self.get_logger().info("Arms are floating; push them by hand")
            return
        # Steady state: only the floating entries move, the rest stay frozen at
        # the snapshot. joint_state_broadcaster owns the topic alone and always
        # publishes the same joint set, so the names stay resolvable.
        for slot, name in self._floating:
            self._target[slot] = measured[name]

    def _publish(self) -> None:
        if self._target is None:
            return
        # The message is reused: rclpy serialises inside publish() and the
        # executor is single threaded, so nothing can observe a torn buffer.
        self._message.data = self._target
        self._publisher.publish(self._message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GravityFloatDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
