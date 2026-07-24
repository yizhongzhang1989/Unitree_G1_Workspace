#!/usr/bin/env python3
"""Web workflow for passive pose capture and torque-only arm calibration."""

import json
import mimetypes
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time
from typing import Dict, Optional, Sequence
from urllib.parse import parse_qs, urlparse

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy)
from unitree_api.msg import Request, Response
from unitree_hg.msg import LowCmd, LowState

from .calibration import StaticSample, fit_selected_joints
from .capture import PassivePoseCapture
from .constants import (ALL_ARM_JOINTS, ALL_ARM_MOTOR_INDICES,
                        ARM_JOINTS, ARM_MOTOR_INDICES, SIDES)
from .gravity_model import TorsoArmGravityModel
from .imu import ImuSampleWindow
from .lowcmd import populate_torque_only
from .parameter_store import ParameterStore
from .torque_control import PoseStabilityWindow, TorquePoseController


_STATIC_DIR = Path(__file__).resolve().parent / "static"
_CHECK_MODE = 1001
_SELECT_MODE = 1002
_RELEASE_MODE = 1003
_CONFIRMATION = "START_TORQUE_CALIBRATION"


class CalibrationStopped(RuntimeError):
    pass


class ArmGravityWorkflow(Node):
    def __init__(self) -> None:
        super().__init__("arm_gravity_compensation")
        description_share = Path(get_package_share_directory(
            "unitree_g1_description"))
        default_urdf = description_share / "model" / "final.urdf"
        default_data = Path.home() / ".ros" / "arm_gravity_compensation"

        self._urdf_path = str(Path(self.declare_parameter(
            "urdf_path", str(default_urdf)).value).expanduser().resolve())
        self._parameter_path = str(Path(self.declare_parameter(
            "parameter_file", str(default_data / "parameters.json")).value
                                        ).expanduser().resolve())
        self._output_urdf = str(Path(self.declare_parameter(
            "calibrated_urdf", str(default_data / "calibrated.urdf")).value
                                    ).expanduser().resolve())
        self._lowstate_topic = str(self.declare_parameter(
            "lowstate_topic", "/lowstate").value)
        self._lowcmd_topic = str(self.declare_parameter(
            "lowcmd_topic", "/lowcmd").value)
        self._host = str(self.declare_parameter("host", "0.0.0.0").value)
        self._port = int(self.declare_parameter("port", 8310).value)
        self._control_rate = float(self.declare_parameter(
            "control_rate_hz", 200.0).value)
        self._state_timeout = float(self.declare_parameter(
            "state_timeout_s", 0.25).value)
        self._target_timeout = float(self.declare_parameter(
            "target_timeout_s", 20.0).value)
        self._settle_duration = float(self.declare_parameter(
            "settle_duration_s", 0.6).value)
        self._stability_position_range = float(self.declare_parameter(
            "stability_position_range", 0.02).value)
        self._sample_duration = float(self.declare_parameter(
            "sample_duration_s", 1.0).value)
        self._imu_duration = float(self.declare_parameter(
            "imu_duration_s", 1.0).value)
        self._imu_samples = int(self.declare_parameter(
            "imu_minimum_samples", 100).value)
        self._imu_timeout = float(self.declare_parameter(
            "imu_timeout_s", 10.0).value)
        self._acceleration_sign = float(self.declare_parameter(
            "accelerometer_to_gravity_sign", -1.0).value)
        self._lowcmd_quiet_period = float(self.declare_parameter(
            "lowcmd_quiet_period_s", 0.2).value)
        self._lowcmd_quiet_timeout = float(self.declare_parameter(
            "lowcmd_quiet_timeout_s", 3.0).value)
        self._motion_timeout = float(self.declare_parameter(
            "motion_switch_timeout_s", 1.5).value)
        self._restore_motion = bool(self.declare_parameter(
            "restore_motion_mode", True).value)
        self._allow_torque_output = bool(self.declare_parameter(
            "allow_torque_output", False).value)
        self._fallback_motion = str(self.declare_parameter(
            "fallback_motion_mode", "ai").value)

        self._controller_kwargs = {
            "stiffness": self.declare_parameter(
                "software_stiffness", [10.0, 10.0, 8.0, 8.0,
                                       4.0, 4.0, 3.0]).value,
            "damping": self.declare_parameter(
                "software_damping", [2.0, 2.0, 1.5, 1.5,
                                     1.0, 1.0, 0.5]).value,
            "torque_slew_rate": self.declare_parameter(
                "torque_slew_rate", [30.0] * 7).value,
            "maximum_speed": float(self.declare_parameter(
                "maximum_reference_speed", 0.35).value),
            "position_tolerance": float(self.declare_parameter(
                "position_tolerance", 0.04).value),
            "velocity_tolerance": float(self.declare_parameter(
                "velocity_tolerance", 0.05).value),
            "minimum_duration": float(self.declare_parameter(
                "minimum_move_duration", 2.0).value),
        }

        self._lock = threading.RLock()
        self._file_lock = threading.RLock()
        self._state_condition = threading.Condition(self._lock)
        self._motion_call_lock = threading.Lock()
        self._motion_pending_id: Optional[int] = None
        self._motion_response: Optional[Response] = None
        self._motion_event = threading.Event()
        self._stop_event = threading.Event()
        self._worker: Optional[threading.Thread] = None
        self._httpd: Optional[ThreadingHTTPServer] = None
        self._lowcmd_publisher = None
        self._imu_window: Optional[ImuSampleWindow] = None
        self._capture: Optional[PassivePoseCapture] = None
        self._capture_automatic = False

        self._phase = "idle"
        self._message = "Waiting for LowState"
        self._progress = {"side": None, "target": 0, "total": 0,
                          "stage": "idle", "iteration": 0}
        self._position = np.zeros(14, dtype=float)
        self._velocity = np.zeros(14, dtype=float)
        self._estimated_torque = np.zeros(14, dtype=float)
        self._acceleration = np.zeros(3, dtype=float)
        self._gyroscope = np.zeros(3, dtype=float)
        self._mode_pr = 0
        self._mode_machine = 0
        self._state_stamp = 0.0
        self._last_observed_lowcmd = 0.0
        self._last_command = {
            side: np.zeros(7, dtype=float) for side in SIDES
        }
        self._last_gravity = np.array([0.0, 0.0, -9.81])

        self._store = ParameterStore(self._parameter_path)
        with self._file_lock:
            document = self._store.initialize(self._urdf_path)
        self._model = TorsoArmGravityModel.from_urdf_file(self._urdf_path)
        self._load_model_parameters(document)

        sensor_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        reliable_qos = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._lowstate_subscription = self.create_subscription(
            LowState, self._lowstate_topic, self._on_lowstate, sensor_qos)
        self._lowcmd_subscription = self.create_subscription(
            LowCmd, self._lowcmd_topic, self._on_lowcmd, sensor_qos)
        self._motion_request = self.create_publisher(
            Request, "/api/motion_switcher/request", reliable_qos)
        self._motion_subscription = self.create_subscription(
            Response, "/api/motion_switcher/response",
            self._on_motion_response, reliable_qos)
        self._reliable_qos = reliable_qos
        self._start_http()
        self.get_logger().info(
            "Arm gravity workflow: http://%s:%d (parameters: %s)"
            % (self._host, self._port, self._parameter_path))

    def _load_model_parameters(self, document: dict) -> None:
        for side in SIDES:
            expected = tuple(document["model_scope"]["parameter_links"][side])
            if expected != self._model.parameter_links[side]:
                raise ValueError(
                    "%s parameter file link order does not match the URDF" % side)
            scales, biases = self._store.link_estimate(side)
            self._model.set_arm_parameters(side, scales, biases)

    def _on_lowstate(self, message: LowState) -> None:
        now = time.monotonic()
        position = np.array([
            message.motor_state[index].q for index in ALL_ARM_MOTOR_INDICES
        ], dtype=float)
        velocity = np.array([
            message.motor_state[index].dq for index in ALL_ARM_MOTOR_INDICES
        ], dtype=float)
        estimated = np.array([
            message.motor_state[index].tau_est
            for index in ALL_ARM_MOTOR_INDICES
        ], dtype=float)
        acceleration = np.asarray(message.imu_state.accelerometer, dtype=float)
        gyroscope = np.asarray(message.imu_state.gyroscope, dtype=float)
        if not all(np.all(np.isfinite(value)) for value in
                   (position, velocity, estimated, acceleration, gyroscope)):
            return

        captured = None
        with self._lock:
            first_state = self._state_stamp <= 0.0
            self._position = position
            self._velocity = velocity
            self._estimated_torque = estimated
            self._acceleration = acceleration
            self._gyroscope = gyroscope
            self._mode_pr = int(message.mode_pr)
            self._mode_machine = int(message.mode_machine)
            self._state_stamp = now
            if first_state and self._phase == "idle":
                self._message = "LowState connected; select joints and capture poses"
            if self._imu_window is not None:
                self._imu_window.add(now, acceleration, gyroscope)
            if self._phase == "passive_capture" and self._capture_automatic:
                captured = self._capture.update(now, position, velocity)
            self._state_condition.notify_all()
        if captured is not None:
            try:
                self._append_target(captured, "automatic_settle")
            except Exception as error:  # noqa: BLE001
                self._set_message("Automatic capture failed: %s" % error)

    def _on_lowcmd(self, _message: LowCmd) -> None:
        with self._lock:
            self._last_observed_lowcmd = time.monotonic()
            self._state_condition.notify_all()

    def _on_motion_response(self, message: Response) -> None:
        with self._lock:
            if (self._motion_pending_id is None or
                    int(message.header.identity.id) != self._motion_pending_id):
                return
            self._motion_response = message
            self._motion_event.set()

    def _append_target(self, position: Sequence[float], source: str) -> dict:
        values = {
            name: float(value)
            for name, value in zip(ALL_ARM_JOINTS, position)
        }
        with self._file_lock:
            target = self._store.append_target(values, source=source)
        self._set_message("Captured pose %d" % target["id"])
        return target

    def start_capture(self, selected_joints: Sequence[str], automatic: bool) -> dict:
        selected = list(dict.fromkeys(str(name) for name in selected_joints))
        invalid = [name for name in selected if name not in ALL_ARM_JOINTS]
        if not selected or invalid:
            raise ValueError("select one or more valid arm joints")
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                raise RuntimeError("automatic calibration is running")
            self._require_fresh_state_locked()
            selected_sides = {
                side for side in SIDES
                if any(name in ARM_JOINTS[side] for name in selected)
            }
            indices = [
                index for index, name in enumerate(ALL_ARM_JOINTS)
                if any(name in ARM_JOINTS[side] for side in selected_sides)
            ]
            capture = PassivePoseCapture(indices)
            capture.reset(self._position)
            self._capture = capture
            self._capture_automatic = bool(automatic)
            self._phase = "passive_capture"
            self._message = "Passive capture active; LowCmd output is disabled"
        with self._file_lock:
            self._store.set_selected_joints(selected)
        return {"ok": True, "message": self._message}

    def capture_current(self) -> dict:
        with self._lock:
            if self._phase != "passive_capture":
                raise RuntimeError("passive capture is not active")
            self._require_fresh_state_locked()
            position = self._position.copy()
        target = self._append_target(position, "manual")
        return {"ok": True, "target": target}

    def stop_capture(self) -> dict:
        with self._lock:
            if self._phase == "passive_capture":
                self._phase = "ready"
                self._message = "Capture stopped; review poses before calibration"
            self._capture = None
            self._capture_automatic = False
        return {"ok": True, "message": self._message}

    def remove_target(self, target_id: int) -> dict:
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                raise RuntimeError("cannot edit poses during calibration")
        with self._file_lock:
            removed = self._store.remove_target(int(target_id))
        return {"ok": removed, "message": ("Pose removed" if removed
                                             else "Pose not found")}

    def clear_targets(self) -> dict:
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                raise RuntimeError("cannot edit poses during calibration")
        with self._file_lock:
            document = self._store.load()
            document["calibration"]["targets"] = []
            self._store.save(document)
        return {"ok": True, "message": "All captured poses removed"}

    def start_calibration(self, confirmation: str) -> dict:
        if not self._allow_torque_output:
            raise RuntimeError(
                "torque output is disabled; relaunch with "
                "allow_torque_output:=true after supporting the robot")
        if confirmation != _CONFIRMATION:
            raise ValueError("torque calibration confirmation is missing")
        with self._file_lock:
            document = self._store.load()
        selected = document["calibration"]["selected_joints"]
        targets = document["calibration"]["targets"]
        if not selected:
            raise ValueError("select joints before calibration")
        if not targets:
            raise ValueError("capture at least one pose before calibration")
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                raise RuntimeError("automatic calibration is already running")
            self._require_fresh_state_locked()
            self._capture = None
            self._capture_automatic = False
            self._stop_event.clear()
            self._phase = "preflight"
            self._message = "Checking IMU and low-level control ownership"
            self._progress = {
                "side": None, "target": 0, "total": len(targets),
                "stage": "preflight", "iteration": 0,
            }
            self._worker = threading.Thread(
                target=self._run_calibration,
                name="arm-gravity-calibration", daemon=True)
            self._worker.start()
        return {"ok": True, "message": self._message}

    def stop_calibration(self) -> dict:
        self._stop_event.set()
        self._set_message("Stop requested; ramping torque output down")
        return {"ok": True, "message": self._message}

    def export_urdf(self) -> dict:
        with self._file_lock:
            output = self._store.export_calibrated_urdf(self._output_urdf)
        return {"ok": True, "path": output}

    def _run_calibration(self) -> None:
        previous_mode = ""
        motion_managed = False
        try:
            with self._file_lock:
                document = self._store.load()
            selected = tuple(document["calibration"]["selected_joints"])
            targets = tuple(document["calibration"]["targets"])
            sides = tuple(side for side in SIDES
                          if any(name in ARM_JOINTS[side] for name in selected))
            selected_set = set(selected)
            for baseline_side in SIDES:
                scales, biases = self._store.link_estimate(baseline_side)
                scales = np.asarray(scales, dtype=float)
                biases = np.asarray(biases, dtype=float)
                for index, link_name in enumerate(
                        self._model.parameter_links[baseline_side]):
                    if self._model.parameter_owner[link_name] in selected_set:
                        scales[index] = 1.0
                for index, joint_name in enumerate(ARM_JOINTS[baseline_side]):
                    if joint_name in selected_set:
                        biases[index] = 0.0
                self._model.set_arm_parameters(
                    baseline_side, scales, biases)
            first_gravity = self._collect_gravity(control_tick=None)
            previous_mode = self._check_motion_mode()
            motion_managed = True
            if previous_mode:
                self._set_progress(stage="release_motion")
                self._call_motion(_RELEASE_MODE, "")
            self._wait_for_external_lowcmd_quiet()
            self._lowcmd_publisher = self.create_publisher(
                LowCmd, self._lowcmd_topic, self._reliable_qos)

            hold_controllers = {}
            with self._lock:
                initial_positions = self._position.copy()
            for hold_side in SIDES:
                hold_position = self._side_values(
                    initial_positions, hold_side)
                hold_controller = TorquePoseController(**self._controller_kwargs)
                hold_controller.start(
                    time.monotonic(), hold_position, hold_position,
                    initial_torque=np.zeros(7))
                hold_controllers[hold_side] = hold_controller
            self._set_progress(stage="torque_ramp")
            ramp_deadline = time.monotonic() + 0.5
            while time.monotonic() < ramp_deadline:
                self._check_stop()
                self._control_tick(
                    "left", hold_controllers["left"], first_gravity,
                    hold_controllers)
                time.sleep(1.0 / self._control_rate)

            for side in sides:
                side_selected = tuple(name for name in selected
                                      if name in ARM_JOINTS[side])
                samples = []
                hold_gravity = first_gravity
                for target_number, target in enumerate(targets, start=1):
                    self._check_stop()
                    self._set_progress(
                        side=side, target=target_number, total=len(targets),
                        stage="move", iteration=len(samples) + 1)

                    target_array = np.array([
                        target["positions"][name] for name in ARM_JOINTS[side]
                    ], dtype=float)
                    with self._lock:
                        self._require_fresh_state_locked()
                        side_position = self._side_values(self._position, side)
                    controller = TorquePoseController(**self._controller_kwargs)
                    controller.start(
                        time.monotonic(), side_position, target_array,
                        initial_torque=self._last_command[side])
                    hold_controllers[side] = controller
                    self._move_until_stable(
                        side, controller, hold_gravity, hold_controllers)
                    self._set_progress(stage="static_average")
                    sample = self._sample_static_pose(
                        int(target["id"]), side, target_array,
                        controller, hold_gravity, hold_controllers)
                    samples.append(sample)
                    hold_gravity = sample.gravity.copy()
                    self._set_message(
                        "%s pose %d/%d recorded; parameters unchanged"
                        % (side, target_number, len(targets)))

                self._set_progress(
                    side=side, target=len(targets), total=len(targets),
                    stage="fit", iteration=1)
                fit = fit_selected_joints(
                    self._model, side, side_selected, samples)
                iteration = {
                    "target_ids": [sample.target_id for sample in samples],
                    "sample_count": len(samples),
                    "selected_joints": list(side_selected),
                    "rank": fit.rank,
                    "nullity": fit.nullity,
                    "singular_values": [float(value)
                                        for value in fit.singular_values],
                    "rmse_before": fit.rmse_before,
                    "rmse_after": fit.rmse_after,
                    "inlier_fraction": fit.em.inlier_fraction,
                    "noise_std": fit.em.noise_std,
                    "samples": [{
                        "target_id": sample.target_id,
                        "gravity": [float(value)
                                    for value in sample.gravity],
                        "measured_position": [float(value)
                                              for value in sample.q],
                        "position_error": [float(value)
                                           for value in sample.position_error],
                        "applied_torque": [float(value)
                                           for value in sample.applied_torque],
                        "estimated_torque": [float(value)
                                             for value in sample.estimated_torque],
                        "velocity_std": [float(value)
                                         for value in sample.velocity_std],
                    } for sample in samples],
                }
                with self._file_lock:
                    self._store.apply_link_estimate(
                        side, fit.parameter_links, fit.mass_scales,
                        fit.torque_bias, fit.scale_observability,
                        fit.bias_observability, iteration)
                    self._store.export_calibrated_urdf(self._output_urdf)
                self._model.set_arm_parameters(
                    side, fit.mass_scales, fit.torque_bias)
                self._set_message(
                    "%s batch complete: %d poses, RMSE %.4f -> %.4f, "
                    "rank %d, nullity %d"
                    % (side, len(samples), fit.rmse_before,
                       fit.rmse_after, fit.rank, fit.nullity))

            self._set_phase(
                "complete", "Calibration complete; parameters and URDF written")
        except CalibrationStopped:
            self._set_phase("ready", "Calibration stopped")
        except Exception as error:  # noqa: BLE001
            self.get_logger().error("Calibration failed: %s" % error)
            self._set_phase("error", "Calibration failed: %s" % error)
        finally:
            self._close_lowcmd_output()
            if motion_managed:
                try:
                    target_mode = previous_mode or self._fallback_motion
                    if self._restore_motion and target_mode:
                        self._call_motion(
                            _SELECT_MODE, json.dumps({"name": target_mode}))
                except Exception as error:  # noqa: BLE001
                    self._set_message(
                        "%s; motion restore failed: %s"
                        % (self._message, error))
            with self._lock:
                self._worker = None
                self._progress["stage"] = "idle"

    def _collect_gravity(self, control_tick) -> np.ndarray:
        deadline = time.monotonic() + self._imu_timeout
        last_error = "no complete IMU window"
        while time.monotonic() < deadline:
            self._check_stop()
            window = ImuSampleWindow()
            with self._lock:
                self._imu_window = window
            while not window.ready(self._imu_duration, self._imu_samples):
                self._check_stop()
                if time.monotonic() >= deadline:
                    break
                if control_tick is not None:
                    control_tick()
                with self._state_condition:
                    self._state_condition.wait(timeout=1.0 / self._control_rate)
            with self._lock:
                if self._imu_window is window:
                    self._imu_window = None
            if not window.ready(self._imu_duration, self._imu_samples):
                break
            try:
                estimate = window.estimate(
                    self._model.imu_to_torso,
                    acceleration_sign=self._acceleration_sign)
                with self._lock:
                    self._last_gravity = estimate.gravity.copy()
                self._set_message(
                    "IMU stable: mean=[%.3f, %.3f, %.3f], n=%d"
                    % (*estimate.mean_acceleration, estimate.sample_count))
                return estimate.gravity
            except ValueError as error:
                last_error = str(error)
        with self._lock:
            self._imu_window = None
        raise RuntimeError("stable IMU window unavailable: %s" % last_error)

    def _move_until_stable(
        self,
        side: str,
        controller: TorquePoseController,
        gravity: np.ndarray,
        controllers: Dict[str, TorquePoseController],
    ) -> None:
        deadline = time.monotonic() + self._target_timeout
        stability = PoseStabilityWindow(
            duration=self._settle_duration,
            position_range_tolerance=self._stability_position_range,
        )
        last_step = None
        while time.monotonic() < deadline:
            self._check_stop()
            step = self._control_tick(side, controller, gravity, controllers)
            last_step = step
            if step.trajectory_complete:
                self._set_progress(stage="settle")
                with self._lock:
                    position = self._side_values(self._position, side)
                    velocity = self._side_values(self._velocity, side)
                if stability.update(time.monotonic(), position, velocity):
                    return
            time.sleep(1.0 / self._control_rate)
        error = (float(np.max(np.abs(last_step.target_error)))
                 if last_step is not None else float("nan"))
        raise RuntimeError(
            "measured pose did not stabilize before timeout "
            "(target error %.4f rad, velocity %.4f rad/s, "
            "position range %.4f rad)"
            % (error, stability.max_velocity, stability.max_position_range))

    def _sample_static_pose(
        self,
        target_id: int,
        side: str,
        target: np.ndarray,
        controller: TorquePoseController,
        gravity: np.ndarray,
        controllers: Dict[str, TorquePoseController],
    ) -> StaticSample:
        overall_deadline = time.monotonic() + self._imu_timeout
        last_error = "no complete static averaging window"
        while time.monotonic() < overall_deadline:
            positions = []
            velocities = []
            estimated = []
            commands = []
            imu_window = ImuSampleWindow()
            with self._lock:
                self._imu_window = imu_window
            try:
                while not imu_window.ready(
                        self._sample_duration, self._imu_samples):
                    self._check_stop()
                    if time.monotonic() >= overall_deadline:
                        break
                    step = self._control_tick(
                        side, controller, gravity, controllers)
                    with self._lock:
                        positions.append(self._position.copy())
                        velocities.append(self._side_values(
                            self._velocity, side))
                        estimated.append(self._side_values(
                            self._estimated_torque, side))
                    commands.append(step.torque.copy())
                    time.sleep(1.0 / self._control_rate)
            finally:
                with self._lock:
                    if self._imu_window is imu_window:
                        self._imu_window = None

            if (not positions or not imu_window.ready(
                    self._sample_duration, self._imu_samples)):
                last_error = "static averaging window timed out"
                continue
            position_array = np.asarray(positions)
            velocity_array = np.asarray(velocities)
            max_velocity = float(np.max(np.abs(velocity_array)))
            side_positions = position_array[:, self._side_slice(side)]
            position_range = float(np.max(np.ptp(side_positions, axis=0)))
            if position_range > self._stability_position_range:
                last_error = (
                    "pose moved while averaging (reported velocity %.4f rad/s, "
                    "position range %.4f rad)"
                    % (max_velocity, position_range))
                continue
            try:
                gravity_estimate = imu_window.estimate(
                    self._model.imu_to_torso,
                    acceleration_sign=self._acceleration_sign)
            except ValueError as error:
                last_error = str(error)
                continue

            q = np.mean(position_array, axis=0)
            side_position = self._side_values(q, side)
            with self._lock:
                self._last_gravity = gravity_estimate.gravity.copy()
            return StaticSample(
                target_id=target_id,
                q=q,
                gravity=gravity_estimate.gravity.copy(),
                applied_torque=np.mean(np.asarray(commands), axis=0),
                estimated_torque=np.mean(np.asarray(estimated), axis=0),
                position_error=target - side_position,
                velocity_std=np.std(velocity_array, axis=0),
            )
        raise RuntimeError(
            "stable pose/IMU averaging unavailable: %s" % last_error)

    def _control_tick(
        self,
        side: str,
        controller: TorquePoseController,
        gravity: np.ndarray,
        controllers: Optional[Dict[str, TorquePoseController]] = None,
    ):
        with self._lock:
            self._require_fresh_state_locked()
            q = self._position.copy()
            velocity_all = self._velocity.copy()
            position = self._side_values(self._position, side)
            velocity = self._side_values(velocity_all, side)
            mode_machine = self._mode_machine
        # 计算重力补偿
        gravity_torque = self._model.compensation(side, q, gravity) # type: ignore
        step = controller.step(time.monotonic(), position, velocity, gravity_torque) # type: ignore
        torques = {side: step.torque}
        for hold_side, hold_controller in (controllers or {}).items():
            if hold_side == side:
                continue
            hold_position = self._side_values(q, hold_side)
            hold_velocity = self._side_values(velocity_all, hold_side)
            hold_gravity_torque = self._model.compensation(
                hold_side, q, gravity)
            hold_step = hold_controller.step(
                time.monotonic(), hold_position, hold_velocity,
                hold_gravity_torque)
            torques[hold_side] = hold_step.torque
        self._publish_torque(torques, mode_machine)
        return step

    def _publish_torque(
        self, torques: Dict[str, Sequence[float]], mode_machine: int,
    ) -> None:
        publisher = self._lowcmd_publisher
        if publisher is None:
            raise RuntimeError("LowCmd output is not active")
        mapping = {}
        for side in SIDES:
            values = np.asarray(
                torques.get(side, self._last_command[side]), dtype=float)
            mapping.update({
                index: float(value)
                for index, value in zip(ARM_MOTOR_INDICES[side], values)
            })
        message = LowCmd()
        populate_torque_only(message, mode_machine, mapping)
        publisher.publish(message)
        with self._lock:
            for side, values in torques.items():
                self._last_command[side] = np.asarray(
                    values, dtype=float).copy()

    def _close_lowcmd_output(self) -> None:
        publisher = self._lowcmd_publisher
        if publisher is None:
            return
        with self._lock:
            commands = {side: values.copy()
                        for side, values in self._last_command.items()}
            mode_machine = self._mode_machine
        for ratio in np.linspace(0.9, 0.0, 10):
            mapping = {}
            for side in SIDES:
                mapping.update({
                    index: float(value * ratio)
                    for index, value in zip(
                        ARM_MOTOR_INDICES[side], commands[side])
                })
            message = LowCmd()
            populate_torque_only(message, mode_machine, mapping)
            publisher.publish(message)
            time.sleep(0.01)
        try:
            self.destroy_publisher(publisher)
        except Exception:  # noqa: BLE001
            pass
        self._lowcmd_publisher = None
        with self._lock:
            self._last_command = {
                side: np.zeros(7, dtype=float) for side in SIDES
            }

    def _call_motion(self, api_id: int, parameter: str) -> Response:
        with self._motion_call_lock:
            identity = time.monotonic_ns()
            request = Request()
            request.header.identity.id = identity
            request.header.identity.api_id = int(api_id)
            request.parameter = parameter
            with self._lock:
                self._motion_pending_id = identity
                self._motion_response = None
                self._motion_event.clear()
            self._motion_request.publish(request)
            if not self._motion_event.wait(self._motion_timeout):
                with self._lock:
                    self._motion_pending_id = None
                raise RuntimeError("motion switcher request timed out")
            with self._lock:
                response = self._motion_response
                self._motion_pending_id = None
                self._motion_response = None
            if response is None:
                raise RuntimeError("motion switcher returned no response")
            if int(response.header.status.code) != 0:
                raise RuntimeError(
                    "motion switcher status %d"
                    % int(response.header.status.code))
            return response

    def _check_motion_mode(self) -> str:
        response = self._call_motion(_CHECK_MODE, "")
        try:
            data = json.loads(response.data or "{}")
        except json.JSONDecodeError as error:
            raise RuntimeError("invalid CheckMode response: %s" % error)
        return str(data.get("name", ""))

    def _wait_for_external_lowcmd_quiet(self) -> None:
        deadline = time.monotonic() + self._lowcmd_quiet_timeout
        with self._state_condition:
            while time.monotonic() < deadline:
                quiet_for = time.monotonic() - self._last_observed_lowcmd
                if quiet_for >= self._lowcmd_quiet_period:
                    return
                self._state_condition.wait(
                    timeout=max(0.0, self._lowcmd_quiet_period - quiet_for))
        raise RuntimeError("external LowCmd remained active")

    def _require_fresh_state_locked(self) -> None:
        if self._state_stamp <= 0.0:
            raise RuntimeError("LowState has not been received")
        age = time.monotonic() - self._state_stamp
        if age > self._state_timeout:
            raise RuntimeError("LowState is stale (%.3f s)" % age)
        if self._mode_pr != 0:
            raise RuntimeError("LowState mode_pr must be 0")

    def _check_stop(self) -> None:
        if self._stop_event.is_set():
            raise CalibrationStopped("calibration stop requested")

    @staticmethod
    def _side_values(values: np.ndarray, side: str) -> np.ndarray:
        return values[ArmGravityWorkflow._side_slice(side)].copy()

    @staticmethod
    def _side_slice(side: str) -> slice:
        offset = 0 if side == "left" else 7
        return slice(offset, offset + 7)

    def _set_phase(self, phase: str, message: str) -> None:
        with self._lock:
            self._phase = phase
            self._message = message

    def _set_message(self, message: str) -> None:
        with self._lock:
            self._message = message

    def _set_progress(self, **values) -> None:
        with self._lock:
            self._progress.update(values)

    def snapshot(self) -> dict:
        with self._file_lock:
            document = self._store.load()
        with self._lock:
            state_age = (None if self._state_stamp <= 0.0 else
                         time.monotonic() - self._state_stamp)
            runtime = {
                "phase": self._phase,
                "message": self._message,
                "progress": dict(self._progress),
                "lowstate_age": state_age,
                "mode_pr": self._mode_pr,
                "mode_machine": self._mode_machine,
                "position": self._position.tolist(),
                "velocity": self._velocity.tolist(),
                "estimated_torque": self._estimated_torque.tolist(),
                "accelerometer": self._acceleration.tolist(),
                "gravity": self._last_gravity.tolist(),
                "lowcmd_active": self._lowcmd_publisher is not None,
                "capture_automatic": self._capture_automatic,
                "torque_output_allowed": self._allow_torque_output,
            }
        parameter_groups = {}
        for side in SIDES:
            parameter_groups[side] = []
            for joint_name in ARM_JOINTS[side]:
                links = []
                for link_name in self._model.parameter_groups(side)[joint_name]:
                    inertial = document["links"][link_name]["inertial"]
                    links.append({
                        "name": link_name,
                        "mass": inertial["calibrated"]["mass"],
                        "scale": inertial["scale"],
                        "identification": inertial["identification"],
                    })
                parameter_groups[side].append({
                    "joint": joint_name,
                    "links": links,
                })
        return {
            "runtime": runtime,
            "files": {
                "parameter": self._parameter_path,
                "source_urdf": self._urdf_path,
                "calibrated_urdf": self._output_urdf,
                "schema_version": document["schema_version"],
                "source_sha256": document["source_urdf"]["sha256"],
            },
            "joint_names": list(ALL_ARM_JOINTS),
            "selected_joints": document["calibration"]["selected_joints"],
            "targets": document["calibration"]["targets"],
            "iterations": document["calibration"]["iterations"],
            "parameter_groups": parameter_groups,
        }

    def _start_http(self) -> None:
        workflow = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_args):
                return

            def _send(self, code, body, content_type="application/json"):
                if isinstance(body, str):
                    body = body.encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    self.wfile.write(body)
                except Exception:  # noqa: BLE001
                    pass

            def _json(self) -> dict:
                length = int(self.headers.get("Content-Length", "0") or "0")
                payload = self.rfile.read(length) if length else b"{}"
                return json.loads(payload or b"{}")

            def do_GET(self):
                parsed = urlparse(self.path)
                if parsed.path in ("/", "/index.html"):
                    return self._serve("index.html")
                if parsed.path == "/api/state":
                    return self._send(200, json.dumps(workflow.snapshot()))
                if parsed.path == "/api/file":
                    query = parse_qs(parsed.query)
                    kind = query.get("kind", [""])[0]
                    path = (workflow._parameter_path if kind == "parameters"
                            else workflow._output_urdf if kind == "urdf"
                            else "")
                    if not path or not Path(path).is_file():
                        return self._send(404, "not found", "text/plain")
                    content_type = ("application/json" if kind == "parameters"
                                    else "application/xml")
                    return self._send(
                        200, Path(path).read_bytes(), content_type)
                if parsed.path.startswith("/static/"):
                    return self._serve(parsed.path[len("/static/"):])
                return self._send(404, '{"error":"not found"}')

            def do_POST(self):
                path = urlparse(self.path).path
                try:
                    body = self._json()
                    routes = {
                        "/api/capture/start": lambda: workflow.start_capture(
                            body.get("selected_joints", []),
                            bool(body.get("automatic", True))),
                        "/api/capture/point": workflow.capture_current,
                        "/api/capture/stop": workflow.stop_capture,
                        "/api/targets/remove": lambda: workflow.remove_target(
                            int(body.get("id", 0))),
                        "/api/targets/clear": workflow.clear_targets,
                        "/api/calibration/start": lambda: workflow.start_calibration(
                            str(body.get("confirmation", ""))),
                        "/api/calibration/stop": workflow.stop_calibration,
                        "/api/export": workflow.export_urdf,
                    }
                    route = routes.get(path)
                    if route is None:
                        return self._send(404, '{"error":"not found"}')
                    result = route()
                    return self._send(200, json.dumps(result))
                except Exception as error:  # noqa: BLE001
                    return self._send(400, json.dumps({
                        "ok": False, "message": str(error)}))

            def _serve(self, relative_path: str):
                path = (_STATIC_DIR / relative_path).resolve()
                if (not str(path).startswith(str(_STATIC_DIR.resolve())) or
                        not path.is_file()):
                    return self._send(404, "not found", "text/plain")
                content_type = mimetypes.guess_type(str(path))[0] \
                    or "application/octet-stream"
                if path.suffix == ".js":
                    content_type = "text/javascript"
                return self._send(200, path.read_bytes(), content_type)

        self._httpd = ThreadingHTTPServer((self._host, self._port), Handler)
        threading.Thread(
            target=self._httpd.serve_forever,
            name="arm-gravity-http", daemon=True).start()

    def destroy_node(self):
        self._stop_event.set()
        worker = self._worker
        if worker is not None and worker.is_alive():
            worker.join(timeout=3.0)
        self._close_lowcmd_output()
        if self._httpd is not None:
            try:
                self._httpd.shutdown()
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmGravityWorkflow()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()