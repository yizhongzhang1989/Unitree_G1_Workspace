#!/usr/bin/env python3
"""Expose dashboard-compatible position control and distribute MIT commands."""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Dict, Optional, Sequence, Tuple, cast

import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from gloria_ros.msg import MitCommand
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from unitree_hg.msg import LowCmd, LowState

from unitree_g1_description.joint_state_mapping import G1_JOINT_NAMES
from unitree_g1_description.mit_command import (
    LowCommand,
    MotorCommand,
    load_g1_mit_gains,
    load_position_limits,
    low_cmd_crc,
)
from unitree_g1_description.motion_switcher import MotionSwitcherClient


GRIPPER_JOINT_NAMES: Tuple[str, str] = (
    "left_eccentric_joint",
    "right_eccentric_joint",
)
CONTROLLED_JOINT_NAMES: Tuple[str, ...] = (
    *G1_JOINT_NAMES,
    *GRIPPER_JOINT_NAMES,
)
_CONTROLLER_TYPE = "forward_command_controller/ForwardCommandController"
_GRIPPER_SIDES = ("left", "right")


class MitPositionController(Node):
    """A small controller-manager facade for the whole-body test dashboard."""

    def __init__(self) -> None:
        super().__init__("g1_mit_position_controller")
        package_share = Path(get_package_share_directory(
            "unitree_g1_description"))

        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter("controller_name", "whole_body_controller")
        self.declare_parameter("command_topic", "")
        self.declare_parameter("lowstate_topic", "/lowstate")
        self.declare_parameter("lowcmd_topic", "/lowcmd")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("left_gripper_command_topic", "/grip_arm0/mit_command")
        self.declare_parameter("right_gripper_command_topic", "/grip_arm1/mit_command")
        self.declare_parameter("left_gripper_node", "/grip_arm0")
        self.declare_parameter("right_gripper_node", "/grip_arm1")
        self.declare_parameter("gain_file", str(package_share / "config" / "default_29dof_param.yaml"))
        self.declare_parameter("joint_limits_file", str(package_share / "model" / "final.urdf"))
        self.declare_parameter("g1_command_rate_hz", 500.0)
        self.declare_parameter("gripper_command_rate_hz", 100.0)
        self.declare_parameter("command_timeout_s", 0.25)
        self.declare_parameter("state_timeout_s", 0.25)
        self.declare_parameter("max_initial_position_error", 0.2)
        self.declare_parameter("max_command_step", 0.1)
        self.declare_parameter("require_pr_mode", True)
        self.declare_parameter("gripper_kp", 10.0)
        self.declare_parameter("gripper_kd", 5.0)
        self.declare_parameter("gripper_service_timeout_s", 3.0)
        self.declare_parameter("manage_motion_mode", True)
        self.declare_parameter("restore_motion_mode", True)
        self.declare_parameter("fallback_motion_mode", "ai")
        self.declare_parameter("motion_switch_timeout_s", 1.0)
        self.declare_parameter("motion_select_timeout_s", 10.0)
        self.declare_parameter("motion_release_attempts", 3)
        self.declare_parameter("motion_release_retry_s", 0.2)
        self.declare_parameter("lowcmd_quiet_period_s", 0.1)
        self.declare_parameter("lowcmd_quiet_timeout_s", 2.0)

        gp = self.get_parameter
        controller_manager = gp(
            "controller_manager").get_parameter_value().string_value.rstrip("/")
        self._controller_name = gp("controller_name").get_parameter_value().string_value
        command_topic = gp("command_topic").get_parameter_value().string_value
        if not command_topic:
            command_topic = f"/{self._controller_name}/commands"
        self._command_timeout = self._positive_parameter("command_timeout_s")
        self._state_timeout = self._positive_parameter("state_timeout_s")
        self._max_initial_error = self._positive_parameter("max_initial_position_error")
        self._max_command_step = self._positive_parameter("max_command_step")
        self._require_pr_mode = gp("require_pr_mode").get_parameter_value().bool_value
        self._gripper_kp = gp("gripper_kp").get_parameter_value().double_value
        self._gripper_kd = gp("gripper_kd").get_parameter_value().double_value
        self._gripper_service_timeout = self._positive_parameter(
            "gripper_service_timeout_s")
        self._manage_motion_mode = gp("manage_motion_mode").get_parameter_value().bool_value
        self._restore_motion_mode = gp("restore_motion_mode").get_parameter_value().bool_value
        self._fallback_motion_mode = gp("fallback_motion_mode").get_parameter_value().string_value
        self._motion_switch_timeout = self._positive_parameter("motion_switch_timeout_s")
        self._motion_select_timeout = self._positive_parameter("motion_select_timeout_s")
        self._motion_release_attempts = gp("motion_release_attempts").get_parameter_value().integer_value
        self._motion_release_retry = gp("motion_release_retry_s").get_parameter_value().double_value
        self._lowcmd_quiet_period = self._positive_parameter("lowcmd_quiet_period_s")
        self._lowcmd_quiet_timeout = self._positive_parameter("lowcmd_quiet_timeout_s")

        if not controller_manager or not self._controller_name:
            raise ValueError(
                "controller_manager and controller_name must not be empty")
        gripper_nodes = {
            side: gp(f"{side}_gripper_node").get_parameter_value()
            .string_value.rstrip("/")
            for side in _GRIPPER_SIDES
        }
        if any(not name.startswith("/") or name == "" for name in gripper_nodes.values()):
            raise ValueError("left_gripper_node and right_gripper_node must be absolute names")
        if not 0.0 <= self._gripper_kp <= 500.0:
            raise ValueError("gripper_kp must be within the SDK range [0, 500]")
        if not 0.0 <= self._gripper_kd <= 5.0:
            raise ValueError("gripper_kd must be within the SDK range [0, 5]")
        if self._motion_release_attempts <= 0:
            raise ValueError("motion_release_attempts must be greater than zero")
        if (not math.isfinite(self._motion_release_retry)
                or self._motion_release_retry < 0.0):
            raise ValueError(
                "motion_release_retry_s must be finite and non-negative")
        if self._restore_motion_mode and not self._fallback_motion_mode:
            raise ValueError(
                "fallback_motion_mode must not be empty when restoration is enabled")

        gain_file = Path(
            gp("gain_file").get_parameter_value().string_value)
        limits_file = Path(
            gp("joint_limits_file").get_parameter_value().string_value)
        self._stiffness, self._damping = load_g1_mit_gains(gain_file)
        self._position_limits = load_position_limits(
            limits_file, CONTROLLED_JOINT_NAMES)

        self._lock = threading.Lock()
        self._switch_lock = threading.Lock()
        self._command_publish_lock = threading.Lock()
        self._lowcmd_condition = threading.Condition()
        self._callback_group = ReentrantCallbackGroup()
        self._active = False
        self._activated_at = 0.0
        self._target_positions: Optional[Tuple[float, ...]] = None
        self._last_command_at = 0.0
        self._joint_positions: Dict[str, float] = {}
        self._joint_state_at = 0.0
        self._lowstate_at = 0.0
        self._mode_pr = 0
        self._mode_machine = 0
        self._last_warning: Dict[str, float] = {}
        self._last_observed_lowcmd_at = 0.0
        self._previous_motion_mode = ""
        self._g1_publisher_failed = False
        self._g1_message_key: Optional[
            Tuple[Tuple[float, ...], int]
        ] = None
        self._g1_message: Optional[LowCmd] = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        lowcmd_topic = gp(
            "lowcmd_topic").get_parameter_value().string_value
        self._lowcmd_publisher = self.create_publisher(
            LowCmd, lowcmd_topic, reliable_qos)
        self._gripper_publishers = (
            self.create_publisher(
                MitCommand,
                gp("left_gripper_command_topic")
                .get_parameter_value().string_value,
                reliable_qos,
            ),
            self.create_publisher(
                MitCommand,
                gp("right_gripper_command_topic")
                .get_parameter_value().string_value,
                reliable_qos,
            ),
        )
        self._command_subscription = self.create_subscription(
            Float64MultiArray, command_topic, self._on_command, reliable_qos,
            callback_group=self._callback_group)
        self._lowstate_subscription = self.create_subscription(
            LowState,
            gp("lowstate_topic").get_parameter_value().string_value,
            self._on_lowstate,
            sensor_qos,
            callback_group=self._callback_group,
        )
        self._joint_state_subscription = self.create_subscription(
            JointState,
            gp("joint_states_topic").get_parameter_value().string_value,
            self._on_joint_state,
            reliable_qos,
            callback_group=self._callback_group,
        )
        self._lowcmd_observer = self.create_subscription(
            LowCmd,
            lowcmd_topic,
            self._on_lowcmd,
            sensor_qos,
            callback_group=self._callback_group,
        )
        self._list_service = self.create_service(
            ListControllers,
            f"{controller_manager}/list_controllers",
            self._list_controllers,
            callback_group=self._callback_group,
        )
        self._switch_service = self.create_service(
            SwitchController,
            f"{controller_manager}/switch_controller",
            self._switch_controller,
            callback_group=self._callback_group,
        )
        self._gripper_service_clients = {
            side: {
                action: (
                    f"{node_name}/{action}",
                    self.create_client(
                        Trigger,
                        f"{node_name}/{action}",
                        callback_group=self._callback_group,
                    ),
                )
                for action in ("enable", "disable")
            }
            for side, node_name in gripper_nodes.items()
        }
        self._motion_switcher = MotionSwitcherClient(
            self, callback_group=self._callback_group)

        g1_rate = self._positive_parameter("g1_command_rate_hz")
        gripper_rate = self._positive_parameter("gripper_command_rate_hz")
        self._g1_period = 1.0 / g1_rate
        self._g1_stop = threading.Event()
        self._g1_thread = threading.Thread(
            target=self._g1_publish_loop,
            name="g1-lowcmd-publisher",
            daemon=True,
        )
        self._g1_thread.start()
        self._gripper_timer = self.create_timer(
            1.0 / gripper_rate, self._publish_grippers,
            callback_group=self._callback_group)

        self.get_logger().info(
            f"controller /{self._controller_name}/commands -> /lowcmd and "
            f"two Gloria MIT topics; G1={g1_rate:g} Hz, "
            f"Gloria={gripper_rate:g} Hz, Gloria kp={self._gripper_kp:g}, "
            f"kd={self._gripper_kd:g}")

    def _positive_parameter(self, name: str) -> float:
        value = self.get_parameter(
            name).get_parameter_value().double_value
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and greater than zero")
        return value

    def _call_gripper_service(
            self, side: str, action: str) -> Tuple[bool, str]:
        endpoint, client = self._gripper_service_clients[side][action]
        deadline = time.monotonic() + self._gripper_service_timeout
        try:
            available = client.wait_for_service(
                timeout_sec=self._gripper_service_timeout)
        except Exception as exc:  # noqa: BLE001
            return False, f"service check failed: {endpoint}: {exc}"
        if not available:
            return False, f"service unavailable: {endpoint}"
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return False, f"service timed out: {endpoint}"
        try:
            future = client.call_async(Trigger.Request())
        except Exception as exc:  # noqa: BLE001
            return False, f"service call failed: {endpoint}: {exc}"
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        remaining = deadline - time.monotonic()
        if remaining <= 0.0 or not completed.wait(remaining):
            future.cancel()
            return False, f"service timed out: {endpoint}"
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            return False, f"service call failed: {endpoint}: {exc}"
        if response is None:
            return False, f"service returned no response: {endpoint}"
        if not response.success:
            detail = response.message or "request rejected"
            return False, f"{endpoint} failed: {detail}"
        return True, response.message

    def _call_gripper_services(self, action: str) -> Tuple[bool, str]:
        results: Dict[str, Tuple[bool, str]] = {}
        results_lock = threading.Lock()

        def call(side: str) -> None:
            try:
                result = self._call_gripper_service(side, action)
            except Exception as exc:  # noqa: BLE001
                result = (
                    False, f"unexpected {side} gripper service error: {exc}")
            with results_lock:
                results[side] = result

        threads = [
            threading.Thread(
                target=call,
                args=(side,),
                name=f"gripper-{side}-{action}",
            )
            for side in _GRIPPER_SIDES
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        errors = [
            results[side][1]
            for side in _GRIPPER_SIDES
            if not results[side][0]
        ]
        return not errors, "; ".join(errors)

    def _enable_grippers(self) -> Tuple[bool, str]:
        return self._call_gripper_services("enable")

    def _disable_grippers(self) -> Tuple[bool, str]:
        return self._call_gripper_services("disable")

    def _disable_grippers_and_restore_mode(
            self, restore_mode: bool) -> Tuple[bool, str]:
        errors = []
        ok, error = self._disable_grippers()
        if not ok:
            errors.append(error)
        if restore_mode:
            ok, error = self._restore_previous_mode()
            if not ok:
                errors.append(error)
        return not errors, "; ".join(errors)

    def _on_lowstate(self, message: LowState) -> None:
        now = time.monotonic()
        with self._lock:
            self._lowstate_at = now
            self._mode_pr = int(message.mode_pr)
            self._mode_machine = int(message.mode_machine)

    def _on_joint_state(self, message: JointState) -> None:
        positions: Dict[str, float] = {}
        for index, name in enumerate(message.name):
            if index >= len(message.position):
                break
            value = float(message.position[index])
            if math.isfinite(value):
                positions[name] = value
        if not positions:
            return
        with self._lock:
            self._joint_positions.update(positions)
            self._joint_state_at = time.monotonic()

    def _on_lowcmd(self, _message: LowCmd) -> None:
        with self._lowcmd_condition:
            self._last_observed_lowcmd_at = time.monotonic()
            self._lowcmd_condition.notify_all()

    def _on_command(self, message: Float64MultiArray) -> None:
        now = time.monotonic()
        values = tuple(float(value) for value in message.data)
        with self._lock:
            if not self._active:
                return
            error = self._command_error_locked(values, now)
            if error:
                self._warn_locked(
                    "invalid_command", f"discarding dashboard target: {error}")
                return
            self._target_positions = values
            self._last_command_at = now

    def _command_error_locked(
            self, values: Sequence[float], now: float) -> str:
        if len(values) != len(CONTROLLED_JOINT_NAMES):
            return (
                f"command has {len(values)} positions; "
                f"expected {len(CONTROLLED_JOINT_NAMES)}")
        if not all(math.isfinite(value) for value in values):
            return "command contains a non-finite position"
        state_error = self._state_error_locked(now)
        if state_error:
            return state_error

        for name, value in zip(CONTROLLED_JOINT_NAMES, values):
            lower, upper = self._position_limits[name]
            if value < lower or value > upper:
                return (
                    f"{name} target {value:.4f} is outside "
                    f"[{lower:.4f}, {upper:.4f}]")

        if self._target_positions is None:
            largest_error = max(
                abs(value - self._joint_positions[name])
                for name, value in zip(CONTROLLED_JOINT_NAMES, values)
            )
            if largest_error > self._max_initial_error:
                return (
                    f"first target differs from feedback by {largest_error:.4f} "
                    f"rad (limit {self._max_initial_error:.4f})")
        else:
            largest_step = max(
                abs(value - previous)
                for value, previous in zip(values, self._target_positions)
            )
            if largest_step > self._max_command_step:
                return (
                    f"target step is {largest_step:.4f} rad "
                    f"(limit {self._max_command_step:.4f})")
        return ""

    def _state_error_locked(self, now: float) -> str:
        if self._g1_publisher_failed:
            return "G1 command publisher has stopped"
        if now - self._lowstate_at > self._state_timeout:
            return "LowState is unavailable or stale"
        if now - self._joint_state_at > self._state_timeout:
            return "JointState is unavailable or stale"
        if self._require_pr_mode and self._mode_pr != 0:
            return f"mode_pr={self._mode_pr}; PR mode 0 is required"
        missing = [
            name for name in CONTROLLED_JOINT_NAMES
            if name not in self._joint_positions
        ]
        if missing:
            return "JointState is missing: " + ", ".join(missing[:3])
        return ""

    def _ready_target(self) -> Optional[Tuple[Tuple[float, ...], int]]:
        now = time.monotonic()
        fault = ""
        with self._lock:
            if not self._active:
                return None
            state_error = self._state_error_locked(now)
            if state_error:
                self._deactivate_locked(state_error)
                fault = state_error
            else:
                deadline_from = (
                    self._last_command_at
                    if self._target_positions is not None
                    else self._activated_at
                )
                if now - deadline_from > self._command_timeout:
                    self._target_positions = tuple(
                        self._joint_positions[name]
                        for name in CONTROLLED_JOINT_NAMES)
                    self._last_command_at = now
                    self._warn_locked(
                        "command_timeout",
                        "dashboard command timed out; holding latest feedback pose",
                    )
                if self._target_positions is not None:
                    return self._target_positions, self._mode_machine
        if fault:
            self._wait_for_command_publishers_idle()
            with self._switch_lock:
                with self._lock:
                    if self._active:
                        return None
                ok, error = self._disable_grippers_and_restore_mode(True)
            if not ok:
                self.get_logger().error(
                    f"failed to stop external control after {fault}: {error}")
        return None

    def _wait_for_lowcmd_quiet(self) -> bool:
        deadline = time.monotonic() + self._lowcmd_quiet_timeout
        with self._lowcmd_condition:
            while True:
                now = time.monotonic()
                quiet_for = now - self._last_observed_lowcmd_at
                if quiet_for >= self._lowcmd_quiet_period:
                    return True
                remaining = deadline - now
                if remaining <= 0.0:
                    return False
                self._lowcmd_condition.wait(min(
                    remaining, self._lowcmd_quiet_period - quiet_for))

    def _prepare_low_level_control(self) -> Tuple[bool, str]:
        if not self._manage_motion_mode:
            return True, ""
        ok, mode, error = self._motion_switcher.check_mode(
            self._motion_switch_timeout)
        if not ok:
            return False, error
        self._previous_motion_mode = mode

        for attempt in range(self._motion_release_attempts):
            if not mode:
                break
            ok, error = self._motion_switcher.release_mode(
                self._motion_switch_timeout)
            if not ok:
                self._restore_previous_mode()
                return False, error
            if self._motion_release_retry > 0.0:
                time.sleep(self._motion_release_retry)
            ok, mode, error = self._motion_switcher.check_mode(
                self._motion_switch_timeout)
            if not ok:
                self._restore_previous_mode()
                return False, error
            self.get_logger().info(
                f"motion mode release attempt {attempt + 1}: "
                f"active={mode or '<none>'}")
        if mode:
            self._restore_previous_mode()
            return (
                False,
                f"motion mode {mode!r} remains active after "
                f"{self._motion_release_attempts} release attempts",
            )
        if not self._wait_for_lowcmd_quiet():
            self._restore_previous_mode()
            return False, "existing /lowcmd stream did not become quiet"
        return True, ""

    def _restore_previous_mode(self) -> Tuple[bool, str]:
        if not self._manage_motion_mode or not self._restore_motion_mode:
            return True, ""
        mode = self._previous_motion_mode or self._fallback_motion_mode
        deadline = time.monotonic() + self._motion_select_timeout
        last_error = ""
        selected = ""
        attempts = 0
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            ok, selected, error = self._motion_switcher.check_mode(
                min(self._motion_switch_timeout, remaining))
            if ok and selected == mode:
                return True, ""
            if ok and selected:
                return (
                    False,
                    f"motion mode {selected!r} became active while restoring "
                    f"{mode!r}",
                )
            if error:
                last_error = error

            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            attempts += 1
            ok, error = self._motion_switcher.select_mode(
                mode, min(self._motion_switch_timeout, remaining))
            if not ok:
                last_error = error
            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                time.sleep(min(0.2, remaining))

        detail = last_error or f"active mode is {selected or '<none>'}"
        return (
            False,
            f"motion mode {mode!r} was not restored after {attempts} attempts "
            f"within {self._motion_select_timeout:g}s: {detail}",
        )

    def _publish_g1(self) -> None:
        message = self._g1_message
        if message is None:
            return
        with self._command_publish_lock:
            if message is not self._g1_message:
                return
            self._lowcmd_publisher.publish(message)

    def _update_g1_message(
            self, target: Tuple[float, ...], mode_machine: int) -> None:
        positions = target[:len(G1_JOINT_NAMES)]
        key = positions, mode_machine
        if key == self._g1_message_key:
            return
        message = LowCmd()
        message.mode_pr = 0
        message.mode_machine = mode_machine
        motor_commands = cast(Sequence[MotorCommand], message.motor_cmd)
        for index, position in enumerate(positions):
            command = motor_commands[index]
            command.mode = 1
            command.q = position
            command.dq = 0.0
            command.tau = 0.0
            command.kp = self._stiffness[index]
            command.kd = self._damping[index]
        message.crc = low_cmd_crc(cast(LowCommand, message))
        with self._lock:
            if (not self._active
                    or self._target_positions != target
                    or self._mode_machine != mode_machine):
                return
            self._g1_message = message
            self._g1_message_key = key

    def _g1_publish_loop(self) -> None:
        deadline = time.monotonic()
        try:
            while not self._g1_stop.is_set():
                deadline += self._g1_period
                self._publish_g1()
                remaining = deadline - time.monotonic()
                if remaining > 0.0:
                    self._g1_stop.wait(remaining)
                else:
                    deadline = time.monotonic()
        except Exception as exc:
            self.get_logger().error(f"G1 command publisher stopped: {exc}")
            with self._lock:
                self._g1_publisher_failed = True
                self._deactivate_locked("G1 command publisher failed")
            with self._switch_lock:
                ok, error = self._disable_grippers_and_restore_mode(True)
            if not ok:
                self.get_logger().error(
                    f"failed to stop external control after publisher failure: {error}")

    def _wait_for_command_publishers_idle(self) -> None:
        with self._command_publish_lock:
            pass

    def stop_g1_publisher(self) -> None:
        self._g1_stop.set()
        if self._g1_thread is not threading.current_thread():
            self._g1_thread.join()

    def _publish_grippers(self) -> None:
        ready = self._ready_target()
        if ready is None:
            return
        target, mode_machine = ready
        self._update_g1_message(target, mode_machine)
        messages = []
        for publisher, position in zip(
                self._gripper_publishers, target[-len(GRIPPER_JOINT_NAMES):]):
            message = MitCommand()
            message.q = position
            message.dq = 0.0
            message.kp = self._gripper_kp
            message.kd = self._gripper_kd
            message.tau = 0.0
            messages.append((publisher, message))
        with self._command_publish_lock:
            with self._lock:
                if not self._active:
                    return
            for publisher, message in messages:
                publisher.publish(message)

    def _list_controllers(
            self, _request: ListControllers.Request,
            response: ListControllers.Response) -> ListControllers.Response:
        controller = ControllerState()
        controller.name = self._controller_name
        controller.type = _CONTROLLER_TYPE
        with self._lock:
            controller.state = "active" if self._active else "inactive"
        interfaces = [f"{name}/position" for name in CONTROLLED_JOINT_NAMES]
        if hasattr(controller, "claimed_interfaces"):
            controller.claimed_interfaces = interfaces
        if hasattr(controller, "required_command_interfaces"):
            setattr(controller, "required_command_interfaces", interfaces)
        response.controller = [controller]
        return response

    def _switch_controller(
            self, request: SwitchController.Request,
            response: SwitchController.Response) -> SwitchController.Response:
        starts = list(getattr(
            request, "activate_controllers",
            getattr(request, "start_controllers", [])))
        stops = list(getattr(
            request, "deactivate_controllers",
            getattr(request, "stop_controllers", [])))
        requested = starts + stops
        unknown = [name for name in requested if name != self._controller_name]
        strict = int(request.strictness) == int(
            getattr(SwitchController.Request, "STRICT", 2))
        if unknown and strict:
            response.ok = False
            return response
        if self._controller_name in starts and self._controller_name in stops:
            self.get_logger().error(
                f"cannot start and stop {self._controller_name} "
                "in the same request")
            response.ok = False
            return response

        with self._switch_lock:
            if self._controller_name in stops:
                with self._lock:
                    active = self._active
                    self._deactivate_locked(
                        "dashboard disengaged controller")
                if active:
                    self._wait_for_command_publishers_idle()
                ok, error = self._disable_grippers_and_restore_mode(active)
                if not ok:
                    self.get_logger().error(
                        f"cannot complete controller disengage; low-level "
                        f"output remains stopped: {error}")
                    response.ok = False
                    return response
            if self._controller_name in starts:
                with self._lock:
                    if self._active:
                        response.ok = True
                        return response
                    state_error = self._state_error_locked(time.monotonic())
                if state_error:
                    ok, error = self._disable_grippers()
                    if not ok:
                        self.get_logger().error(
                            f"failed to disable grippers after activation "
                            f"precheck failed: {error}")
                    self._warn_locked("activation", state_error)
                    response.ok = False
                    return response

                ok, error = self._prepare_low_level_control()
                if not ok:
                    disable_ok, disable_error = self._disable_grippers()
                    if not disable_ok:
                        error = f"{error}; gripper disable failed: {disable_error}"
                    self.get_logger().error(
                        f"cannot activate {self._controller_name}: {error}")
                    response.ok = False
                    return response
                ok, error = self._enable_grippers()
                if not ok:
                    rollback_ok, rollback_error = (
                        self._disable_grippers_and_restore_mode(True))
                    if not rollback_ok:
                        error = f"{error}; rollback failed: {rollback_error}"
                    self.get_logger().error(
                        f"cannot activate {self._controller_name}: {error}")
                    response.ok = False
                    return response
                with self._lock:
                    state_error = self._state_error_locked(time.monotonic())
                    if not state_error:
                        now = time.monotonic()
                        self._target_positions = tuple(
                            self._joint_positions[name]
                            for name in CONTROLLED_JOINT_NAMES)
                        self._last_command_at = now
                        self._activated_at = now
                        self._active = True
                if state_error:
                    rollback_ok, rollback_error = (
                        self._disable_grippers_and_restore_mode(True))
                    if not rollback_ok:
                        self.get_logger().error(
                            f"activation rollback failed: {rollback_error}")
                    self._warn_locked("activation", state_error)
                    response.ok = False
                    return response
                self.get_logger().info(
                    f"activated {self._controller_name}; holding the latest "
                    "feedback pose before dashboard targets arrive")
        response.ok = True
        return response

    def _deactivate_locked(self, reason: str) -> None:
        was_active = self._active
        self._active = False
        self._target_positions = None
        self._g1_message = None
        self._g1_message_key = None
        self._last_command_at = 0.0
        if was_active:
            self.get_logger().warning(
                f"deactivated {self._controller_name}: {reason}")

    def shutdown_control(self) -> None:
        """Restore the previous motion service before this process exits."""
        with self._switch_lock:
            with self._lock:
                if not self._active:
                    return
                self._deactivate_locked("controller node is shutting down")
            self._wait_for_command_publishers_idle()
            ok, error = self._disable_grippers_and_restore_mode(True)
            if not ok:
                self.get_logger().error(
                    f"failed to stop external control during shutdown: {error}")

    def _warn_locked(self, key: str, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warning.get(key, 0.0) >= 1.0:
            self.get_logger().warning(message)
            self._last_warning[key] = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = MitPositionController()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    finally:
        if node is not None and executor is not None and rclpy.ok():
            shutdown_thread = threading.Thread(target=node.shutdown_control)
            shutdown_thread.start()
            while shutdown_thread.is_alive() and rclpy.ok():
                executor.spin_once(timeout_sec=0.05)
            shutdown_thread.join()
        if node is not None:
            node.stop_g1_publisher()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()