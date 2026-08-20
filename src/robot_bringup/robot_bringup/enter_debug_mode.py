#!/usr/bin/env python3
"""Put the robot into low-level (debug) mode without going through ros2_control.

A motion mode owns ``/lowcmd`` and has to be released before anything else can
drive it. Idle ``ai`` publishes 1 kHz frames of its own, all zero -- enabled but
producing nothing. They pull the joints nowhere, so an external publisher does
appear to work; really the two streams interleave and the command gets diluted.
Releasing takes ``/lowcmd`` to 0 Hz, and only then is the bus yours.
``G1TopicSystem`` does the same from ``on_activate``.

Nothing is commanded here, so the robot is left limp with no owner at all.
Whatever publishes ``/lowcmd`` next has to ramp its own gains up from zero, and
it is also the one that has to check the bus is quiet -- a check that only means
anything after the release.

Releasing from a mode that is holding the robot up drops it, so this refuses to
run unless the current mode is in ``--allow-from``. Hand the robot back with
``ros2 run robot_bringup exit_debug_mode``.
"""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from unitree_api.msg import Request, Response
from unitree_hg.msg import LowState

CHECK_MODE = 1001
RELEASE_MODE = 1003
MOTION_REQUEST_TOPIC = "/api/motion_switcher/request"
MOTION_RESPONSE_TOPIC = "/api/motion_switcher/response"
RETRY_PERIOD_S = 0.2


class EnterDebugMode(Node):
    def __init__(self, call_timeout: float) -> None:
        super().__init__("enter_debug_mode")
        self._call_timeout = call_timeout
        self._state = None
        self._pending_id = None
        self._response = None
        self._established = False

        best_effort = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(LowState, "/lowstate", self._on_lowstate, best_effort)
        self.create_subscription(
            Response, MOTION_RESPONSE_TOPIC, self._on_response, reliable)
        self._motion = self.create_publisher(Request, MOTION_REQUEST_TOPIC, reliable)

    def _on_lowstate(self, message: LowState) -> None:
        self._state = message

    def _on_response(self, message: Response) -> None:
        # 这是话题不是服务，应答是广播的：不按 identity 配对就会收下插件或
        # 遥控器发起的那次调用的回复。
        if int(message.header.identity.id) == self._pending_id:
            self._response = message

    def wait_for_state(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while self._state is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._state is not None

    @property
    def mode_machine(self) -> int:
        assert self._state is not None, "wait_for_state() must return True first"
        return int(self._state.mode_machine)

    def _call(self, api_id: int) -> Response:
        """Publish the request until its own reply comes back.

        The first request out of a cold process never gets answered: DDS has
        matched us to the switcher's request subscription, but not the switcher
        to our response subscription, so the reply is dropped. Waiting longer
        before the first publish does not help -- only sending again does.

        Resending stops at the first reply, because only the opening query is
        safe to repeat: ``ReleaseMode`` is a state change and answers 7002
        (switcher is busy) if a second one lands on top of it.
        """
        request = Request()
        request.header.identity.api_id = api_id
        request.parameter = ""
        self._response = None
        deadline = time.monotonic() + self._call_timeout
        next_send = 0.0
        while self._response is None and time.monotonic() < deadline:
            if time.monotonic() >= next_send:
                self._pending_id = request.header.identity.id = time.monotonic_ns()
                self._motion.publish(request)
                next_send = (deadline if self._established
                             else time.monotonic() + RETRY_PERIOD_S)
            rclpy.spin_once(self, timeout_sec=0.02)
        if self._response is None:
            raise RuntimeError(
                "motion switcher did not answer api_id %d within %.1f s"
                % (api_id, self._call_timeout))
        self._established = True
        code = int(self._response.header.status.code)
        if code != 0:
            raise RuntimeError(
                "motion switcher returned status %d for api_id %d" % (code, api_id))
        return self._response

    def check_mode(self) -> str:
        response = self._call(CHECK_MODE)
        try:
            return str(json.loads(response.data or "{}").get("name", ""))
        except json.JSONDecodeError as error:
            raise RuntimeError("invalid CheckMode response: %s" % error)

    def release_mode(self, timeout_s: float) -> bool:
        """Release and confirm: a zero status does not mean the mode cleared."""
        self._call(RELEASE_MODE)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if not self.check_mode():
                return True
            time.sleep(0.2)
        return False


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--allow-from", default="ai",
                        help="comma separated modes it is safe to release from; "
                             "'' always counts as safe (default: ai)")
    parser.add_argument("--force", action="store_true",
                        help="release whatever mode is active -- the robot drops "
                             "if that mode was holding it up")
    parser.add_argument("--call-timeout", type=float, default=3.0,
                        help="seconds to wait for one switcher reply (default 3)")
    parser.add_argument("--release-timeout", type=float, default=10.0,
                        help="seconds to wait for the mode to clear (default 10)")
    args, ros_args = parser.parse_known_args(argv)
    if args.call_timeout <= 0.0 or args.release_timeout <= 0.0:
        parser.error("timeouts must be positive")

    allowed = {name.strip() for name in args.allow_from.split(",")}
    allowed.add("")

    rclpy.init(args=ros_args)
    node = EnterDebugMode(args.call_timeout)
    logger = node.get_logger()
    try:
        if not node.wait_for_state(5.0):
            logger.error("No /lowstate: the robot is not reachable, "
                         "nothing was changed")
            return 1

        try:
            current = node.check_mode()
            if current and current not in allowed and not args.force:
                logger.error(
                    "Motion mode '%s' is active and is not in --allow-from; it "
                    "may be holding the robot up. Switch to 'ai' from the "
                    "remote (L2+B damping, then hang the robot), or pass "
                    "--force." % current)
                return 1
            logger.info("Current motion mode: '%s'" % (current or "<none>"))
            if not node.release_mode(args.release_timeout):
                logger.error("Motion mode did not clear within %.1f s"
                             % args.release_timeout)
                return 1
        except RuntimeError as error:
            logger.error(str(error))
            return 1

        mode_machine = node.mode_machine
        logger.info("Low-level mode active, mode_machine=%d. Publish /lowcmd "
                    "with that value and ramp your gains up from zero; "
                    "'ros2 run robot_bringup exit_debug_mode' hands it back."
                    % mode_machine)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
