#!/usr/bin/env python3
"""Take the robot out of low-level (debug) mode without going through ros2_control.

Stopping a publisher does not release a joint: the firmware has no watchdog, so
whatever ``kp``/``kd``/``q`` arrived last keeps being held. An arm left raised
draws current until it overheats. Something has to actively wind the gains down.

This tool talks to ``/lowcmd`` and the motion switcher directly, so it also works
when the control stack was killed, crashed, or never ran. ``G1TopicSystem`` does
the same thing from ``stop()``; this is the escape hatch for when it could not.

The arm always ends at its hanging pose -- holding it up costs torque, so every
release ends there. The ramp only decides whether it sags or drops.
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from unitree_api.msg import Request
from unitree_hg.msg import LowCmd, LowState

from arm_gravity_compensation.lowcmd import MotorSetpoint, populate_arm_command

G1_JOINT_COUNT = 29
SELECT_MODE = 1002
MOTION_TOPIC = "/api/motion_switcher/request"
RATE_HZ = 100.0
# Only sets how far the command sits from the measurement; it cancels out of the
# torque, see fade_out(). Any positive value produces the same descent.
RELEASE_STIFFNESS = 40.0


class ExitDebugMode(Node):
    def __init__(self, ramp_s: float, damping: float, restore: str) -> None:
        super().__init__("exit_debug_mode")
        self._ramp_s = ramp_s
        self._damping = damping
        self._restore = restore
        self._state = None

        best_effort = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(LowState, "/lowstate", self._on_lowstate, best_effort)
        self._lowcmd = self.create_publisher(LowCmd, "/lowcmd", reliable)
        self._motion = self.create_publisher(Request, MOTION_TOPIC, reliable)

    def _on_lowstate(self, message: LowState) -> None:
        if len(message.motor_state) >= G1_JOINT_COUNT:
            self._state = message

    def wait_for_state(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while self._state is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._state is not None

    def fade_out(self) -> None:
        """Wind the holding torque down to zero over ``ramp_s``.

        Commanding the measured pose would zero the torque on the very first
        frame and drop the arm, which is exactly what has to be avoided. So the
        command is offset by the torque the motor is producing right now::

            q_cmd  = q_meas + tau_est / kp     (fixed for the whole ramp)
            kp_cmd = kp * scale                (scale walks 1 -> 0)
            => torque = kp_cmd * (q_cmd - q_meas) = tau_est * scale

        ``kp`` cancels, so its value only has to be positive. The torque starts
        exactly where the motor already was -- no step at handover -- and fades
        to nothing. ``kd`` stays at full value the whole way down and is dropped
        only on the last frame, where the pose is at rest and removing it moves
        nothing.
        """
        state = self._state
        motors = state.motor_state
        commands = [motors[i].q + motors[i].tau_est / RELEASE_STIFFNESS
                    for i in range(G1_JOINT_COUNT)]
        steps = max(1, int(self._ramp_s * RATE_HZ))

        self.get_logger().info(
            "Fading %d joints out over %.1f s" % (G1_JOINT_COUNT, self._ramp_s))
        for step in range(1, steps + 2):
            last = step > steps
            scale = 0.0 if last else 1.0 - step / steps
            message = LowCmd()
            populate_arm_command(message, state.mode_machine, {
                index: MotorSetpoint(
                    tau=0.0, q=commands[index],
                    kp=RELEASE_STIFFNESS * scale,
                    kd=0.0 if last else self._damping)
                for index in range(G1_JOINT_COUNT)
            })
            self._lowcmd.publish(message)
            time.sleep(1.0 / RATE_HZ)

    def restore_motion(self) -> None:
        if not self._restore:
            return
        request = Request()
        request.header.identity.id = time.monotonic_ns()
        request.header.identity.api_id = SELECT_MODE
        request.parameter = '{"name":"%s"}' % self._restore
        self._motion.publish(request)
        self.get_logger().info("Requested motion mode '%s'" % self._restore)
        # Fire and forget: reading the reply needs the full switcher client and
        # this tool must also work when that service is unreachable.
        time.sleep(0.5)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ramp-s", type=float, default=2.0,
                        help="seconds to fade the holding torque out (default 2, "
                             "same as the hardware plugin's release_ramp_s)")
    parser.add_argument("--damping", type=float, default=1.5,
                        help="kd held for the whole descent (default 1.5)")
    parser.add_argument("--motion-mode", default="ai",
                        help="motion mode to hand back to; 'ai' is the zero "
                             "torque mode and the hardware plugin's fallback")
    parser.add_argument("--keep-debug-mode", action="store_true",
                        help="fade the joints out but stay in low-level mode")
    args, ros_args = parser.parse_known_args(argv)
    if args.ramp_s < 0.0 or args.damping < 0.0:
        parser.error("ramp and damping must be non-negative")
    if not args.motion_mode.replace("_", "").isalnum():
        parser.error("motion mode must be alphanumeric")

    rclpy.init(args=ros_args)
    node = ExitDebugMode(
        args.ramp_s, args.damping,
        "" if args.keep_debug_mode else args.motion_mode)
    try:
        if not node.wait_for_state(5.0):
            node.get_logger().error(
                "No /lowstate: the robot is not reachable, nothing was changed")
            return 1
        node.fade_out()
        node.restore_motion()
        node.get_logger().info("Low-level output released")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
