#!/usr/bin/env python3
"""Web workflow for passive pose capture and torque-only arm calibration."""

import json
import mimetypes
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time
from typing import Any, Dict, Mapping, Optional, Sequence, cast
from urllib.parse import parse_qs, urlparse

import numpy as np
import rclpy
import yaml
from numpy.typing import ArrayLike
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import WrenchStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy)
from unitree_api.msg import Request, Response
from unitree_hg.msg import IMUState, LowCmd, LowState

from .calibration import StaticSample, fit_selected_joints
from .capture import PassivePoseCapture
from .constants import (ALL_ARM_JOINTS, ALL_ARM_MOTOR_INDICES,
                        ARM_JOINTS, ARM_MOTOR_INDICES, FT_SENSOR_LINKS, SIDES)
from .ft_model import (FtSample, KGF_TO_NEWTON, orientation_coverage,
                       solve_ft_calibration, suggest_measurement_origin)
from .gravity_model import TorsoArmGravityModel
from .imu import ImuSampleWindow, gravity_from_acceleration
from .lowcmd import MotorSetpoint, populate_arm_command
from .parameter_store import ParameterStore, atomic_write
from .torque_control import (PoseStabilityWindow, TorquePoseController,
                            TorqueStep)


_STATIC_DIR = Path(__file__).resolve().parent / "static"
_CHECK_MODE = 1001
_SELECT_MODE = 1002
_RELEASE_MODE = 1003
_CONFIRMATION = "START_TORQUE_CALIBRATION"


class CalibrationStopped(RuntimeError):
    pass


class WrenchWindow:
    """Running mean and spread of one sensor between two resets.

    The spread is what tells a settled pose from one where somebody is still
    holding the tool, which is the only way a hand-guided sample can be trusted.
    """

    def __init__(self) -> None:
        self.reset()
        self.stamp = 0.0

    def reset(self) -> None:
        self._sum = np.zeros(6, dtype=float)
        self._square = np.zeros(6, dtype=float)
        self._count = 0

    def add(self, values: np.ndarray, now: float) -> None:
        self._sum += values
        self._square += values * values
        self._count += 1
        self.stamp = now

    @property
    def count(self) -> int:
        return self._count

    def summary(self):
        if self._count <= 0:
            raise ValueError("no wrench samples in this window")
        mean = self._sum / self._count
        variance = np.maximum(self._square / self._count - mean * mean, 0.0)
        return mean, np.sqrt(variance), self._count


class ArmGravityWorkflow(Node):
    def __init__(self) -> None:
        super().__init__("arm_gravity_compensation")
        description_share = Path(get_package_share_directory(
            "unitree_g1_description"))
        default_urdf = description_share / "model" / "final.urdf"
        # Installed as symlinks by --symlink-install, and every writer resolves
        # the link, so exporting lands on the version controlled source files.
        default_data = Path(get_package_share_directory(
            "arm_gravity_compensation")) / "config"

        self._urdf_path = str(Path(self.declare_parameter(
            "urdf_path", str(default_urdf)
        ).get_parameter_value().string_value).expanduser().resolve())
        self._parameter_path = str(Path(self.declare_parameter(
            "parameter_file", str(default_data / "parameters.json")
        ).get_parameter_value().string_value).expanduser().resolve())
        self._output_urdf = str(Path(self.declare_parameter(
            "calibrated_urdf", str(default_data / "calibrated.urdf")
        ).get_parameter_value().string_value).expanduser().resolve())
        # 一次性接受当前 URDF：早期参数文件按整份文件做摘要，渲染改动也会让它
        # 对不上，而那种摘要没法反推出模型部分变没变，只能人来确认一次。
        self._rebind_urdf = self.declare_parameter(
            "rebind_urdf", False).get_parameter_value().bool_value
        # Lumped rigid-body chain consumed by the ros2_control gravity
        # compensation controller, written as a ready-to-load parameter file.
        self._gravity_table_path = str(Path(self.declare_parameter(
            "gravity_table", str(default_data / "gravity_table.yaml")
        ).get_parameter_value().string_value).expanduser().resolve())
        self._controller_name = self.declare_parameter(
            "gravity_controller_name", "arm_gravity_compensation"
        ).get_parameter_value().string_value
        self._lowstate_topic = self.declare_parameter(
            "lowstate_topic", "/lowstate").get_parameter_value().string_value
        # The torso IMU. G1 carries two: ``LowState.imu_state`` sits in the
        # pelvis and is separated from ``torso_link`` by the three waist
        # joints, while ``/secondary_imu`` sits at ``imu_in_torso``, which is
        # exactly the frame this gravity model is rooted in.
        self._imu_topic = self.declare_parameter(
            "imu_topic", "/secondary_imu").get_parameter_value().string_value
        self._lowcmd_topic = self.declare_parameter(
            "lowcmd_topic", "/lowcmd").get_parameter_value().string_value
        self._host = self.declare_parameter(
            "host", "0.0.0.0").get_parameter_value().string_value
        self._port = self.declare_parameter(
            "port", 8310).get_parameter_value().integer_value
        self._control_rate = self.declare_parameter(
            "control_rate_hz", 200.0).get_parameter_value().double_value
        self._state_timeout = self.declare_parameter(
            "state_timeout_s", 0.25).get_parameter_value().double_value
        self._target_timeout = self.declare_parameter(
            "target_timeout_s", 20.0).get_parameter_value().double_value
        self._settle_duration = self.declare_parameter(
            "settle_duration_s", 0.6).get_parameter_value().double_value
        self._stability_position_range = self.declare_parameter(
            "stability_position_range", 0.02
        ).get_parameter_value().double_value
        # 退到哪里再回来。必须大于死区 2*tau_s/kp，否则关节根本没挪到另一侧，
        # 两次采的是同一个静止点，平均下来等于没消。本臂实测死区最大 0.107 rad。
        # 置 0 则退回旧的单向采样。
        self._approach_offset = self.declare_parameter(
            "approach_offset_rad", 0.12).get_parameter_value().double_value
        self._imu_duration = self.declare_parameter(
            "imu_duration_s", 1.0).get_parameter_value().double_value
        self._imu_samples = self.declare_parameter(
            "imu_minimum_samples", 100).get_parameter_value().integer_value
        self._imu_timeout = self.declare_parameter(
            "imu_timeout_s", 10.0).get_parameter_value().double_value
        self._acceleration_sign = self.declare_parameter(
            "accelerometer_to_gravity_sign", -1.0
        ).get_parameter_value().double_value
        self._lowcmd_quiet_period = self.declare_parameter(
            "lowcmd_quiet_period_s", 0.2).get_parameter_value().double_value
        self._lowcmd_quiet_timeout = self.declare_parameter(
            "lowcmd_quiet_timeout_s", 3.0).get_parameter_value().double_value
        self._motion_timeout = self.declare_parameter(
            "motion_switch_timeout_s", 1.5).get_parameter_value().double_value
        self._restore_motion = self.declare_parameter(
            "restore_motion_mode", True).get_parameter_value().bool_value
        self._allow_torque_output = self.declare_parameter(
            "allow_torque_output", False).get_parameter_value().bool_value
        self._fallback_motion = self.declare_parameter(
            "fallback_motion_mode", "ai").get_parameter_value().string_value

        # 力传感器那一路只订阅，标定不需要任何力矩输出：把手臂摆到一个朝向、
        # 松手（或交给现有重力补偿悬停）就能采一个点。
        self._wrench_topics = {
            "left": self.declare_parameter(
                "left_wrench_topic", "/arm0/wrench_raw"
            ).get_parameter_value().string_value,
            "right": self.declare_parameter(
                "right_wrench_topic", "/arm1/wrench_raw"
            ).get_parameter_value().string_value,
        }
        # 与参数文件同目录：那条路径已经跟随 --symlink-install 的链回到源码树，
        # 而新文件在 share 里没有链可跟随，直接拼 share 会写进 install 树。
        self._ft_calibration_path = str(Path(self.declare_parameter(
            "ft_calibration_file",
            str(Path(self._parameter_path).parent / "ft_calibration.yaml")
        ).get_parameter_value().string_value).expanduser().resolve())
        self._ft_node_name = self.declare_parameter(
            "ft_node_name", "ft_wrench_compensator"
        ).get_parameter_value().string_value
        # 驱动默认发 kgf/kgf·m，模型全程用 SI。
        self._ft_unit_scale = (
            1.0 if self.declare_parameter(
                "ft_input_unit", "si").get_parameter_value().string_value
            .lower() == "si" else KGF_TO_NEWTON)
        # 1 kHz 的回调在 Python 里不便宜，而静态采样只需要几百个点就够平均。
        self._ft_decimation = max(1, self.declare_parameter(
            "ft_decimation", 10).get_parameter_value().integer_value)
        self._ft_force_spread = self.declare_parameter(
            "ft_force_spread_limit", 1.0).get_parameter_value().double_value
        self._ft_torque_spread = self.declare_parameter(
            "ft_torque_spread_limit", 0.1).get_parameter_value().double_value

        self._controller_kwargs = {
            "stiffness": self.declare_parameter(
                "motor_stiffness", [40.0, 40.0, 40.0, 40.0,
                                    40.0, 20.0, 20.0]
            ).get_parameter_value().double_array_value,
            "damping": self.declare_parameter(
                "motor_damping", [3.0, 3.0, 3.0, 3.0,
                                  3.0, 1.5, 1.5]
            ).get_parameter_value().double_array_value,
            "torque_slew_rate": self.declare_parameter(
                "torque_slew_rate", [30.0] * 7
            ).get_parameter_value().double_array_value,
            "maximum_speed": self.declare_parameter(
                "maximum_reference_speed", 0.35
            ).get_parameter_value().double_value,
            "minimum_duration": self.declare_parameter(
                "minimum_move_duration", 2.0
            ).get_parameter_value().double_value,
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
        self._next_tick = 0.0
        self._imu_window: Optional[ImuSampleWindow] = None
        self._capture: Optional[PassivePoseCapture] = None
        self._capture_automatic = False
        self._capture_side = "both"
        self._wrench_windows = {side: WrenchWindow() for side in SIDES}
        self._wrench_counter = {side: 0 for side in SIDES}

        self._phase = "idle"
        self._message = "Waiting for LowState"
        self._progress = {"side": None, "target": 0, "total": 0,
                          "stage": "idle", "iteration": 0}
        self._position = np.zeros(14, dtype=float)
        self._velocity = np.zeros(14, dtype=float)
        self._estimated_torque = np.zeros(14, dtype=float)
        self._acceleration = np.zeros(3, dtype=float)
        self._mode_pr = 0
        self._mode_machine = 0
        self._state_stamp = 0.0
        self._last_observed_lowcmd = 0.0
        self._last_setpoints: Dict[int, MotorSetpoint] = {}

        self._store = ParameterStore(self._parameter_path)
        with self._file_lock:
            document = self._store.initialize(
                self._urdf_path, rebind=self._rebind_urdf)
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
        self._imu_subscription = self.create_subscription(
            IMUState, self._imu_topic, self._on_torso_imu, sensor_qos)
        self._lowcmd_subscription = self.create_subscription(
            LowCmd, self._lowcmd_topic, self._on_lowcmd, sensor_qos)
        self._wrench_subscriptions = [
            self.create_subscription(
                WrenchStamped, topic,
                (lambda message, side=side: self._on_wrench(side, message)),
                sensor_qos)
            for side, topic in self._wrench_topics.items() if topic
        ]
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
        motor_state = cast(Any, message.motor_state)
        position = np.array([
            motor_state[index].q for index in ALL_ARM_MOTOR_INDICES
        ], dtype=float)
        velocity = np.array([
            motor_state[index].dq for index in ALL_ARM_MOTOR_INDICES
        ], dtype=float)
        estimated = np.array([
            motor_state[index].tau_est
            for index in ALL_ARM_MOTOR_INDICES
        ], dtype=float)
        if not all(np.all(np.isfinite(value)) for value in
                   (position, velocity, estimated)):
            return

        captured = None
        with self._lock:
            first_state = self._state_stamp <= 0.0
            self._position = position
            self._velocity = velocity
            self._estimated_torque = estimated
            self._mode_pr = int(message.mode_pr)
            self._mode_machine = int(message.mode_machine)
            self._state_stamp = now
            if first_state and self._phase == "idle":
                self._message = "LowState connected; select joints and capture poses"
            capture = self._capture
            if (self._phase == "passive_capture" and self._capture_automatic and capture is not None):
                captured = capture.update(now, position, velocity)
            self._state_condition.notify_all()
        if captured is not None:
            try:
                self._append_target(captured, "automatic_settle")
            except Exception as error:  # noqa: BLE001
                self._set_message("Automatic capture failed: %s" % error)

    def _on_torso_imu(self, message: IMUState) -> None:
        now = time.monotonic()
        acceleration = np.asarray(message.accelerometer, dtype=float)
        gyroscope = np.asarray(message.gyroscope, dtype=float)
        if not (np.all(np.isfinite(acceleration)) and np.all(np.isfinite(gyroscope))):
            return
        with self._lock:
            self._acceleration = acceleration
            if self._imu_window is not None:
                self._imu_window.add(now, acceleration, gyroscope)
            self._state_condition.notify_all()

    def _on_lowcmd(self, _message: LowCmd) -> None:
        with self._lock:
            self._last_observed_lowcmd = time.monotonic()
            self._state_condition.notify_all()

    def _on_wrench(self, side: str, message: WrenchStamped) -> None:
        counter = self._wrench_counter[side] + 1
        self._wrench_counter[side] = counter
        if counter % self._ft_decimation:
            return
        force = message.wrench.force
        torque = message.wrench.torque
        values = np.array([force.x, force.y, force.z,
                           torque.x, torque.y, torque.z],
                          dtype=float) * self._ft_unit_scale
        if not np.all(np.isfinite(values)):
            return
        with self._lock:
            self._wrench_windows[side].add(values, time.monotonic())

    def _on_motion_response(self, message: Response) -> None:
        with self._lock:
            if (self._motion_pending_id is None or
                    int(message.header.identity.id) != self._motion_pending_id):
                return
            self._motion_response = message
            self._motion_event.set()

    def _append_target(self, position: ArrayLike, source: str) -> dict:
        position_array = np.asarray(position, dtype=float)
        values = {
            name: float(value)
            for name, value in zip(ALL_ARM_JOINTS, position_array)
        }
        with self._lock:
            side = self._capture_side
        with self._file_lock:
            target = self._store.append_target(
                values, source=source, side=side)
        self._set_message("Captured pose %d on the %s arm" % (target["id"], side))
        return target

    @staticmethod
    def _validated_selection(selected_joints: Sequence[str]) -> list:
        selected = list(dict.fromkeys(str(name) for name in selected_joints))
        invalid = [name for name in selected if name not in ALL_ARM_JOINTS]
        if not selected or invalid:
            raise ValueError("select one or more valid arm joints")
        return selected

    @staticmethod
    def _selected_sides(selected_joints: Sequence[str]) -> list:
        return [side for side in SIDES
                if any(name in ARM_JOINTS[side] for name in selected_joints)]

    def start_capture(self, selected_joints: Sequence[str], automatic: bool) -> dict:
        selected = self._validated_selection(selected_joints)
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                raise RuntimeError("automatic calibration is running")
            self._require_fresh_state_locked()
            selected_sides = self._selected_sides(selected)
            indices = [
                index for index, name in enumerate(ALL_ARM_JOINTS)
                if any(name in ARM_JOINTS[side] for side in selected_sides)
            ]
            capture = PassivePoseCapture(indices)
            capture.reset(self._position)
            self._capture = capture
            self._capture_automatic = bool(automatic)
            self._capture_side = (selected_sides[0]
                                  if len(selected_sides) == 1 else "both")
            self._phase = "passive_capture"
            self._message = (
                "Passive capture active on the %s arm; LowCmd output is disabled"
                % self._capture_side)
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

    # ------------------------------------------------------------------ #
    # 力传感器：一次性线性标定。全程只读，不发任何 LowCmd，所以把手臂交给现有的
    # 重力补偿悬停、或者干脆用手扶住前臂都行，只要工具那一端没人碰。
    # ------------------------------------------------------------------ #

    def capture_ft_sample(self) -> dict:
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                raise RuntimeError("automatic calibration is running")
            self._require_fresh_state_locked()
            live = self._live_sensors_locked()
            if not live:
                raise RuntimeError("no force sensor is publishing")
        gravity, positions = self._average_static_window()
        recorded = self._record_ft_samples(
            np.mean(positions, axis=0), gravity, live, "manual")
        self._set_message(
            "Recorded a force sample on %s" % ", ".join(recorded))
        return {"ok": True, "sides": recorded}

    def remove_ft_sample(self, sample_id: int) -> dict:
        with self._file_lock:
            removed = self._store.remove_ft_sample(int(sample_id))
        return {"ok": removed,
                "message": "Sample removed" if removed else "Sample not found"}

    def clear_ft_samples(self, side: str = "") -> dict:
        with self._file_lock:
            removed = self._store.clear_ft_samples(side)
        return {"ok": True, "message": "Removed %d force samples" % removed}

    def fit_ft_sensor(self, sides: Sequence[str],
                      estimate_orientation: bool = True,
                      origins: Optional[Mapping[str, Sequence[float]]] = None
                      ) -> dict:
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                raise RuntimeError("automatic calibration is running")
        chosen = [side for side in (list(sides) or list(SIDES)) if side in SIDES]
        if not chosen:
            raise ValueError("select at least one side")
        results = {}
        for side in chosen:
            origin = (origins or {}).get(side, self._stored_origin(side))
            solution = solve_ft_calibration(
                self._ft_samples(side),
                estimate_orientation=bool(estimate_orientation),
                origin=origin)
            diagnostics = dict(
                solution.diagnostics, **self._tool_reference(side, solution))
            with self._file_lock:
                results[side] = self._store.set_ft_calibration(
                    side, solution.calibration.to_dict(), diagnostics)
        self._set_message(
            "Force sensor solved for %s" % ", ".join(chosen))
        return {"ok": True, "results": results}

    def _stored_origin(self, side: str) -> np.ndarray:
        record = self._store.load()["ft_sensor"].get(side)
        if not record:
            return np.zeros(3)
        return np.asarray(
            record["calibration"].get("measurement_origin", np.zeros(3)),
            dtype=float)

    def _tool_reference(self, side: str, solution) -> dict:
        """What the URDF thinks the tool is, and where that puts the torque point.

        The sensor reports torque about its own reference point, which sits on
        the tool flange rather than at the link origin. Gravity cannot separate
        the two, so the model has to supply the missing half.
        """
        if side not in self._model.sensor_placement:
            return {}
        rotation, translation = self._model.sensor_placement[side]
        mass, moment = self._model.distal_inertia(side)
        if mass <= 0.0:
            return {}
        modelled = rotation.T @ (moment / mass - translation)
        return {
            "modelled_tool_mass": float(mass),
            "modelled_tool_com": [float(value) for value in modelled],
            "suggested_origin": [
                float(value) for value in suggest_measurement_origin(
                    solution.calibration, modelled)],
        }

    def _ft_samples(self, side: str) -> list:
        """Turn stored poses into the sensor-frame gravity the solver wants."""
        samples = []
        for item in self._store.ft_samples(side):
            q = np.array([item["positions"][name] for name in ALL_ARM_JOINTS],
                         dtype=float)
            rotation = self._model.sensor_orientation(side, q)
            samples.append(FtSample(
                gravity=rotation.T @ np.asarray(item["gravity"], dtype=float),
                wrench=item["wrench"]))
        if not samples:
            raise ValueError("no force samples captured on the %s arm" % side)
        return samples

    def _live_sensors_locked(self) -> list:
        return [side for side in SIDES
                if 0.0 < self._wrench_windows[side].stamp and
                time.monotonic() - self._wrench_windows[side].stamp <
                self._state_timeout]

    def _average_static_window(self):
        """Average one settled pose: torso gravity plus the arm positions."""
        window = ImuSampleWindow()
        positions = []
        with self._lock:
            self._imu_window = window
            for side in SIDES:
                self._wrench_windows[side].reset()
        deadline = time.monotonic() + self._imu_timeout
        try:
            while not window.ready(self._imu_duration, self._imu_samples):
                if time.monotonic() >= deadline:
                    raise RuntimeError("IMU averaging window timed out")
                with self._state_condition:
                    self._state_condition.wait(timeout=0.05)
                with self._lock:
                    positions.append(self._position.copy())
        finally:
            with self._lock:
                if self._imu_window is window:
                    self._imu_window = None
        array = np.asarray(positions)
        spread = float(np.max(np.ptp(array, axis=0))) if array.size else np.inf
        if spread > self._stability_position_range:
            raise RuntimeError(
                "the arm moved while averaging (%.4f rad)" % spread)
        return window.estimate(
            self._model.imu_to_torso,
            acceleration_sign=self._acceleration_sign).gravity, array

    def _record_ft_samples(self, position: np.ndarray, gravity: np.ndarray,
                           sides: Sequence[str], source: str) -> list:
        values = {name: float(value)
                  for name, value in zip(ALL_ARM_JOINTS, position)}
        recorded = []
        for side in sides:
            with self._lock:
                mean, spread, count = self._wrench_windows[side].summary()
            if (np.max(spread[:3]) > self._ft_force_spread or
                    np.max(spread[3:]) > self._ft_torque_spread):
                raise RuntimeError(
                    "%s reading is unsettled (force %.2f N, torque %.3f N·m); "
                    "let go of the tool"
                    % (side, np.max(spread[:3]), np.max(spread[3:])))
            if count < 5:
                raise RuntimeError("%s produced only %d readings" % (side, count))
            with self._file_lock:
                self._store.append_ft_sample(
                    side, values, gravity, mean, spread, source=source)
            recorded.append(side)
        return recorded

    def start_calibration(self, confirmation: str,
                          selected_joints: Sequence[str]) -> dict:
        if not self._allow_torque_output:
            raise RuntimeError(
                "torque output is disabled; relaunch with "
                "allow_torque_output:=true after supporting the robot")
        if confirmation != _CONFIRMATION:
            raise ValueError("torque calibration confirmation is missing")
        # The request carries the selection so that a run can never inherit the
        # joints of an earlier capture session and move the wrong arm.
        selected = self._validated_selection(selected_joints)
        sides = self._selected_sides(selected)
        with self._file_lock:
            self._store.set_selected_joints(selected)
            document = self._store.load()
        targets = document["calibration"]["targets"]
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
            self._message = (
                "Calibrating the %s arm; checking IMU and low-level control "
                "ownership" % " and ".join(sides))
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

    def export_urdf(self, adopt_tool: bool = False) -> dict:
        """Write the calibrated URDF, the gravity table and the sensor file.

        ``adopt_tool`` replaces the modelled gripper of the wrist group with
        what the sensor weighed. It is off by default because the identified
        wrist group already absorbs the real tool: leaving it off keeps every
        joint torque exactly as it is today and only records where the payload
        hangs.
        """
        with self._file_lock:
            document = self._store.load()
            calibrated = {side: record for side in SIDES
                          for record in [document["ft_sensor"].get(side)]
                          if record}
            tool = None
            if adopt_tool:
                if not calibrated:
                    raise ValueError("no force sensor calibration to adopt")
                tool = {
                    side: {"mass": record["calibration"]["tool_mass"],
                           "com": record["calibration"]["tool_com"]}
                    for side, record in calibrated.items()
                }
            output = self._store.export_calibrated_urdf(self._output_urdf)
            table = atomic_write(
                self._gravity_table_path,
                yaml.safe_dump(
                    {self._controller_name: {
                        "ros__parameters": self._model.gravity_table(tool=tool)}},
                    default_flow_style=None, sort_keys=False).encode("utf-8"))
            sensors = ""
            if calibrated:
                sensors = atomic_write(
                    self._ft_calibration_path,
                    yaml.safe_dump({self._ft_node_name: {"ros__parameters": {
                        side: dict(record["calibration"],
                                   frame=FT_SENSOR_LINKS[side],
                                   calibrated_at=record["calibrated_at"])
                        for side, record in calibrated.items()}}},
                        default_flow_style=None, sort_keys=False).encode("utf-8"))
        return {"ok": True, "path": output, "gravity_table": table,
                "ft_calibration": sensors, "adopted_tool": bool(tool)}

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
            # Each run continues from the stored estimate so that repeated
            # runs are true outer iterations of the same fit.
            for baseline_side in SIDES:
                scales, biases = self._store.link_estimate(baseline_side)
                self._model.set_arm_parameters(baseline_side, scales, biases)
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
                self._sleep_until_next_tick()

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

                    target_array = self._store.target_positions(target, side)
                    sample = self._sample_from_both_sides(
                        int(target["id"]), side, target_array,
                        hold_gravity, hold_controllers)
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
                    "condition_number": (
                        fit.condition_number
                        if np.isfinite(fit.condition_number) else None),
                    "group_scales": [float(value)
                                     for value in fit.group_scales],
                    "joint_noise": [float(value)
                                    for value in fit.joint_noise],
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
                        "friction": [float(value)
                                     for value in sample.friction],
                    } for sample in samples],
                }
                with self._file_lock:
                    self._store.apply_link_estimate(
                        side, fit.parameter_links, fit.mass_scales, # type: ignore
                        fit.torque_bias, fit.scale_observability, # type: ignore
                        fit.bias_observability, iteration) # type: ignore
                    self._store.export_calibrated_urdf(self._output_urdf)
                self._model.set_arm_parameters(
                    side, fit.mass_scales, fit.torque_bias)
                self._report_friction(side, samples)
                self._set_message(
                    "%s batch complete: %d poses, RMSE %.4f -> %.4f, "
                    "rank %d, nullity %d, cond %.1f"
                    % (side, len(samples), fit.rmse_before,
                       fit.rmse_after, fit.rank, fit.nullity,
                       fit.condition_number))

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
            self._sleep_until_next_tick()
        error = (float(np.max(np.abs(last_step.target_error)))
                 if last_step is not None else float("nan"))
        raise RuntimeError(
            "measured pose did not stabilize before timeout "
            "(target error %.4f rad, velocity %.4f rad/s, "
            "position range %.4f rad)"
            % (error, stability.max_velocity, stability.max_position_range))

    def _settle_at(
        self,
        side: str,
        goal: np.ndarray,
        gravity: np.ndarray,
        controllers: Dict[str, TorquePoseController],
    ) -> TorquePoseController:
        """Drive one arm to ``goal`` and wait for it to stop there."""
        with self._lock:
            self._require_fresh_state_locked()
            side_position = self._side_values(self._position, side)
        controller = TorquePoseController(**self._controller_kwargs)
        controller.start(
            time.monotonic(), side_position, goal,
            initial_torque=self._last_feedforward(side))
        controllers[side] = controller
        self._move_until_stable(side, controller, gravity, controllers)
        return controller

    def _sample_from_both_sides(
        self,
        target_id: int,
        side: str,
        target: np.ndarray,
        gravity: np.ndarray,
        controllers: Dict[str, TorquePoseController],
    ) -> StaticSample:
        """Record the pose twice, reached from below and then from above.

        A joint at rest satisfies ``tau_applied + tau_g + tau_f = 0`` with the
        friction free to take any value in ``[-tau_s, +tau_s]``, so one static
        sample carries up to ``tau_s`` of error - measured on this arm at
        0.05 to 0.77 N.m, which is five to fifty percent of the gravity load
        being identified. Approaching from either side pins the friction to a
        known sign, and averaging the pair cancels it exactly. What survives is
        the asymmetry between the two directions, a second-order term.
        """
        if self._approach_offset <= 0.0:
            controller = self._settle_at(side, target, gravity, controllers)
            self._set_progress(stage="static_average")
            return self._sample_static_pose(
                target_id, side, target, controller, gravity, controllers)

        offset = np.full(7, self._approach_offset)
        pair = []
        for index, sign in enumerate((-1.0, 1.0)):
            self._set_progress(
                stage="approach_below" if sign < 0.0 else "approach_above")
            # 先退开再回来，最后一段的运动方向才是确定的；直接走到目标点的话
            # 方向取决于手臂原先在哪，两次可能同向，摩擦就消不掉。
            self._settle_at(side, target + sign * offset, gravity, controllers)
            controller = self._settle_at(side, target, gravity, controllers)
            self._set_progress(stage="static_average", iteration=index + 1)
            pair.append(self._sample_static_pose(
                target_id, side, target, controller, gravity, controllers))

        below, above = pair
        # 和给重力，差给摩擦：同一对样本的两个无关分量。差不进拟合，只落盘。
        return StaticSample(
            target_id=target_id,
            q=0.5 * (below.q + above.q),
            gravity=0.5 * (below.gravity + above.gravity),
            applied_torque=0.5 * (below.applied_torque + above.applied_torque),
            estimated_torque=0.5 * (
                below.estimated_torque + above.estimated_torque),
            position_error=0.5 * (below.position_error + above.position_error),
            # 两次各自的抖动都要能被离群点检测看见，所以取大的那个。
            velocity_std=np.maximum(below.velocity_std, above.velocity_std),
            friction=0.5 * (below.applied_torque - above.applied_torque),
        )

    def _report_friction(self, side: str, samples) -> None:
        """Log the half difference the gravity fit throws away.

        每个位姿的两次逼近相减就是那一点的摩擦力矩，与拟合无关，白拿的。跨位姿取
        中位数而不是平均：摩擦随姿态变，个别位姿没停稳会给出离谱值。
        """
        friction = np.asarray([sample.friction for sample in samples])
        if friction.size == 0 or not np.any(friction):
            return
        median = np.median(np.abs(friction), axis=0)
        spread = np.percentile(np.abs(friction), 90, axis=0) - median
        names = [name.replace(side + "_", "").replace("_joint", "")
                 for name in ARM_JOINTS[side]]
        self.get_logger().info(
            "%s 摩擦力矩（%d 个位姿的双向半差，中位 +p90 增量，N·m）: %s"
            % (side, len(samples), "  ".join(
                "%s %.3f+%.3f" % (name, value, extra)
                for name, value, extra in zip(names, median, spread))))

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
                for each_side in SIDES:
                    self._wrench_windows[each_side].reset()
            try:
                while not imu_window.ready(
                        self._imu_duration, self._imu_samples):
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
                    commands.append(step.applied.copy())
                    self._sleep_until_next_tick()
            finally:
                with self._lock:
                    if self._imu_window is imu_window:
                        self._imu_window = None

            if (not positions or not imu_window.ready(
                    self._imu_duration, self._imu_samples)):
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
            # 力矩标定停稳的那一刻正好是力传感器最好的样本：工具那端没人碰。
            with self._lock:
                live = self._live_sensors_locked()
            if live:
                try:
                    self._record_ft_samples(
                        q, gravity_estimate.gravity, live, "automatic_settle")
                except Exception as error:  # noqa: BLE001
                    self.get_logger().warning(
                        "force sample skipped: %s" % error)
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
            mode_machine = self._mode_machine
        active = dict(controllers or {})
        active[side] = controller
        now = time.monotonic()
        steps = {
            each_side: each_controller.step(
                now,
                self._side_values(q, each_side),
                self._side_values(velocity_all, each_side),
                self._model.compensation(each_side, q, gravity))
            for each_side, each_controller in active.items()
        }
        self._publish_setpoints(steps, active, mode_machine)
        return steps[side]

    def _publish_setpoints(
        self,
        steps: Dict[str, TorqueStep],
        controllers: Dict[str, TorquePoseController],
        mode_machine: int,
    ) -> None:
        publisher = self._lowcmd_publisher
        if publisher is None:
            raise RuntimeError("LowCmd output is not active")
        setpoints = dict(self._last_setpoints)
        for side, step in steps.items():
            controller = controllers[side]
            setpoints.update({
                index: MotorSetpoint(
                    tau=float(tau), q=float(position),
                    kp=float(stiffness), kd=float(damping))
                for index, tau, position, stiffness, damping in zip(
                    ARM_MOTOR_INDICES[side], step.feedforward, step.reference,
                    controller.stiffness, controller.damping)
            })
        message = LowCmd()
        populate_arm_command(message, mode_machine, setpoints)
        publisher.publish(message)
        with self._lock:
            self._last_setpoints = setpoints

    def _last_feedforward(self, side: str) -> np.ndarray:
        """The gravity feed-forward still on the wire, so a new controller can
        ramp on from it instead of stepping the torque."""
        with self._lock:
            setpoints = self._last_setpoints
            return np.array([
                setpoints[index].tau if index in setpoints else 0.0
                for index in ARM_MOTOR_INDICES[side]
            ], dtype=float)

    def _sleep_until_next_tick(self) -> None:
        period = 1.0 / self._control_rate
        now = time.monotonic()
        if self._next_tick < now:
            self._next_tick = now
        self._next_tick += period
        remaining = self._next_tick - time.monotonic()
        if remaining > 0.0:
            time.sleep(remaining)

    def _close_lowcmd_output(self) -> None:
        publisher = self._lowcmd_publisher
        if publisher is None:
            return
        with self._lock:
            setpoints = dict(self._last_setpoints)
            mode_machine = self._mode_machine
        for ratio in np.linspace(0.9, 0.0, 10):
            message = LowCmd()
            populate_arm_command(message, mode_machine, {
                index: MotorSetpoint(
                    tau=setpoint.tau * ratio, q=setpoint.q,
                    kp=setpoint.kp * ratio, kd=setpoint.kd * ratio)
                for index, setpoint in setpoints.items()
            })
            publisher.publish(message)
            time.sleep(0.01)
        try:
            self.destroy_publisher(publisher)
        except Exception:  # noqa: BLE001
            pass
        self._lowcmd_publisher = None
        with self._lock:
            self._last_setpoints = {}

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

    def _live_gravity_locked(self) -> np.ndarray:
        """Torso gravity from the newest accelerometer sample, for display."""
        try:
            return gravity_from_acceleration(
                self._model.imu_to_torso, self._acceleration,
                acceleration_sign=self._acceleration_sign)
        except ValueError:
            return np.zeros(3, dtype=float)

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
                "gravity": self._live_gravity_locked().tolist(),
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
                "gravity_table": self._gravity_table_path,
                "ft_calibration": self._ft_calibration_path,
                "schema_version": document["schema_version"],
                "source_sha256": document["source_urdf"]["sha256"],
            },
            "joint_names": list(ALL_ARM_JOINTS),
            "selected_joints": document["calibration"]["selected_joints"],
            "targets": document["calibration"]["targets"],
            "iterations": document["calibration"]["iterations"],
            "parameter_groups": parameter_groups,
            "ft_sensor": self._ft_snapshot(document),
        }

    def _ft_snapshot(self, document: dict) -> dict:
        with self._lock:
            live = {side: (None if self._wrench_windows[side].stamp <= 0.0 else
                           time.monotonic() - self._wrench_windows[side].stamp)
                    for side in SIDES}
            latest = {side: (self._wrench_windows[side].summary()[0].tolist()
                             if self._wrench_windows[side].count else None)
                      for side in SIDES}
        stored = document["ft_sensor"]
        sides = {}
        for side in SIDES:
            samples = [item for item in stored["samples"]
                       if item["side"] == side]
            mounted = side in self._model.sensor_placement
            coverage = orientation_coverage([
                self._model.sensor_orientation(
                    side,
                    [item["positions"][name] for name in ALL_ARM_JOINTS]).T @
                np.asarray(item["gravity"], dtype=float)
                for item in samples] if mounted else [])
            sides[side] = {
                "topic": self._wrench_topics[side],
                "age": live[side],
                "wrench": latest[side],
                "frame": FT_SENSOR_LINKS[side],
                "samples": [{"id": item["id"], "captured_at": item["captured_at"],
                             "source": item["source"], "wrench": item["wrench"],
                             "wrench_std": item["wrench_std"]}
                            for item in samples],
                "coverage": coverage,
                "result": stored[side],
            }
        return sides

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
                            str(body.get("confirmation", "")),
                            body.get("selected_joints", [])),
                        "/api/calibration/stop": workflow.stop_calibration,
                        "/api/export": lambda: workflow.export_urdf(
                            bool(body.get("adopt_tool", False))),
                        "/api/ft/capture": workflow.capture_ft_sample,
                        "/api/ft/remove": lambda: workflow.remove_ft_sample(
                            int(body.get("id", 0))),
                        "/api/ft/clear": lambda: workflow.clear_ft_samples(
                            str(body.get("side", ""))),
                        "/api/ft/solve": lambda: workflow.fit_ft_sensor(
                            body.get("sides", []),
                            bool(body.get("estimate_orientation", True)),
                            body.get("origins")),
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