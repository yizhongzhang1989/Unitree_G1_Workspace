"""Calibrate both force sensors using mirrored poses and FPC interpolation."""

import argparse
from collections import deque
import json
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import Vector3Stamped, WrenchStamped
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from .constants import ALL_ARM_JOINTS, ARM_JOINTS, FT_SENSOR_LINKS, SIDES, mirror_arm_values
from .ft_model import FtSample, KGF_TO_NEWTON, MINIMUM_ORIENTATION_SAMPLES, net_wrench, solve_ft_calibration
from .parameter_store import atomic_write, load_parameter_document, utc_now
from .table import load_gravity_table, sensor_orientation_from_table


FPC = "forward_position_controller"
MANAGER = "/controller_manager"
COMPENSATOR = "/ft_wrench_compensator"
PAYLOAD = "/payload_estimator"


def resolve_path(value):
    if value.startswith("package://"):
        package, relative = value[10:].split("/", 1)
        value = str(Path(get_package_share_directory(package)) / relative)
    return Path(value).expanduser().resolve()


def check_coverage(gravities):
    values = np.asarray(gravities, dtype=float)
    if values.ndim != 2 or values.shape[1] != 3 or len(values) < MINIMUM_ORIENTATION_SAMPLES:
        raise ValueError("at least four orientations are required")
    norms = np.linalg.norm(values, axis=1)
    if not np.all(np.isfinite(values)) or np.any(norms < 1.0):
        raise ValueError("invalid gravity samples")
    singular = np.linalg.svd(np.column_stack((values / norms[:, None], np.ones(len(values)))), compute_uv=False)
    if singular[-1] / singular[0] < 0.03:
        raise ValueError("insufficient orientation coverage")


def load_poses(config, limits):
    if config["left_joints"] != list(ARM_JOINTS["left"]):
        raise ValueError("incorrect left joint order")
    for name, minimum in (("transition_s", 0.1), ("minimum_wait_s", 0.5)):
        if not np.isfinite(config[name]) or config[name] < minimum:
            raise ValueError("%s must be >= %s seconds" % (name, minimum))
    left = np.asarray(config["left_positions"], dtype=float)
    if left.ndim != 2 or left.shape[1] != 7 or len(left) < MINIMUM_ORIENTATION_SAMPLES or not np.all(np.isfinite(left)):
        raise ValueError("left_positions must contain at least four finite seven-angle arrays")
    poses = np.hstack((left, [mirror_arm_values(pose) for pose in left]))
    for index, joint in enumerate(ALL_ARM_JOINTS):
        bound = limits[joint]["limit"]
        if np.any(poses[:, index] < bound["lower"]) or np.any(poses[:, index] > bound["upper"]):
            raise ValueError("pose exceeds joint limit: " + joint)
    return poses


def interpolate(start, finish, phase):
    phase = min(1.0, max(0.0, phase))
    blend = phase ** 3 * (10.0 + phase * (-15.0 + 6.0 * phase))
    return start + blend * (finish - start) if phase < 1.0 else finish.copy()


def stable(rows, now, duration):
    return (len(rows) >= 10 and rows[-1][0] - rows[0][0] >= duration * 0.9
            and now - rows[-1][0] < 0.1 and np.max(np.diff([row[0] for row in rows])) < 0.15
            and np.max(np.ptp([row[1] for row in rows], axis=0)) <= 0.01
            and np.max(np.abs([row[2] for row in rows])) <= 0.04)


class AutoFtCalibration(Node):
    def __init__(self, config_path, parameter_path=None):
        super().__init__("auto_ft_calibration")
        self.config = yaml.safe_load(resolve_path(config_path).read_text())
        self.parameter_path = resolve_path(parameter_path or "package://arm_gravity_compensation/config/parameters.json")
        self.positions, self.gravity = {}, {}
        self.stamps = {}
        self.joints = deque(maxlen=2000)
        self.samples = {side: deque(maxlen=2000) for side in SIDES}
        self.records = []
        self.target = np.empty(0)
        self.moving = False
        self.last_publish = 0.0
        parameters = self.parameters(COMPENSATOR, [
            "ft_calibration", "gravity_table", "input_unit", "left_input_topic", "right_input_topic",
            "left_gravity_topic", "right_gravity_topic"])
        self.calibration_path = resolve_path(parameters["ft_calibration"])
        self.table = load_gravity_table(str(resolve_path(parameters["gravity_table"])))
        self.scale = KGF_TO_NEWTON if parameters["input_unit"] == "kgf" else 1.0
        self.create_subscription(JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        for side in SIDES:
            self.create_subscription(Vector3Stamped, parameters[side + "_gravity_topic"],
                                     lambda message, side=side: self.on_gravity(side, message), qos_profile_sensor_data)
            self.create_subscription(WrenchStamped, parameters[side + "_input_topic"],
                                     lambda message, side=side: self.on_wrench(side, message), qos_profile_sensor_data)

    def wait(self, condition, timeout):
        deadline = time.monotonic() + timeout
        while not condition():
            if not rclpy.ok() or time.monotonic() >= deadline:
                raise TimeoutError("timed out waiting for service or sensor data")
            rclpy.spin_once(self, timeout_sec=0.01)

    def call(self, kind, name, request, timeout=5.0) -> Any:
        client = self.create_client(kind, name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError("service unavailable: " + name)
            future = client.call_async(request)
            self.wait(future.done, timeout)
            result = future.result()
            if result is None:
                raise RuntimeError("empty service response: " + name)
            return result
        finally:
            self.destroy_client(client)

    def parameters(self, node, names) -> dict[str, Any]:
        response = self.call(GetParameters, node + "/get_parameters", GetParameters.Request(names=names))
        return dict(zip(names, map(parameter_value_to_python, response.values)))

    def on_joints(self, message):
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        if not all(name in positions and name in velocities for name in ALL_ARM_JOINTS):
            return
        if not np.all(np.isfinite(list(positions.values()) + list(velocities.values()))):
            return
        self.positions = positions
        now = self.stamps["joints"] = time.monotonic()
        self.joints.append((now, [positions[name] for name in ALL_ARM_JOINTS],
                            [velocities[name] for name in ALL_ARM_JOINTS]))

    def on_gravity(self, side, message):
        values = np.array([message.vector.x, message.vector.y, message.vector.z])
        if np.all(np.isfinite(values)) and 8 < np.linalg.norm(values) < 11:
            self.gravity[side] = values
            self.stamps[side + "_gravity"] = time.monotonic()

    def on_wrench(self, side, message):
        force, torque = message.wrench.force, message.wrench.torque
        values = self.scale * np.array([force.x, force.y, force.z, torque.x, torque.y, torque.z])
        if not np.all(np.isfinite(values)):
            return
        now = self.stamps[side] = time.monotonic()
        if now - min(self.stamps.get("joints", 0), self.stamps.get(side + "_gravity", 0)) > 0.1:
            return
        if self.samples[side] and now - self.samples[side][-1][0] < 0.01:
            return
        self.samples[side].append((now, [self.positions[name] for name in ALL_ARM_JOINTS],
                                   self.gravity[side].copy(), values))

    def stale_streams(self):
        now = time.monotonic()
        return {name: now - self.stamps.get(name, 0)
                for name in ("joints", "left", "right", "left_gravity", "right_gravity")
                if now - self.stamps.get(name, 0) >= 0.5}

    def fresh(self):
        return not self.stale_streams()

    def tick(self):
        deadline = time.monotonic() + 0.01
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.001)
        stale = self.stale_streams()
        if stale:
            raise RuntimeError("stale data (left/right = raw wrench): " + ", ".join(
                "%s %.3fs" % (name, age) for name, age in stale.items()))
        if time.monotonic() - self.last_publish > 0.2:
            raise RuntimeError("FPC command stream stalled")
        self.output.publish(Float64MultiArray(data=self.target.tolist()))
        self.last_publish = time.monotonic()

    def controller_state(self):
        response = self.call(ListControllers, MANAGER + "/list_controllers", ListControllers.Request())
        if any(item.name != FPC and item.state == "active" and item.claimed_interfaces for item in response.controller):
            raise RuntimeError("another controller owns the command interfaces")
        state = next((item.state for item in response.controller if item.name == FPC), None)
        if state not in ("active", "inactive"):
            raise RuntimeError("FPC must be configured")
        return state

    def switch(self, state):
        if self.controller_state() == state:
            return
        request = SwitchController.Request(strictness=SwitchController.Request.STRICT)
        request.activate_controllers = [FPC] if state == "active" else []
        request.deactivate_controllers = [FPC] if state == "inactive" else []
        request.timeout.sec = 20
        if not self.call(SwitchController, MANAGER + "/switch_controller", request, 25.0).ok or self.controller_state() != state:
            raise RuntimeError("FPC state switch failed")

    def move(self, finish):
        self.moving = True
        start, began = self.target.copy(), time.monotonic()
        while True:
            phase = (time.monotonic() - began) / self.config["transition_s"]
            self.target = interpolate(start, finish, phase)
            self.tick()
            if phase >= 1:
                break
        self.moving = False

    def settle(self):
        self.joints.clear()
        started = time.monotonic()
        while time.monotonic() - started < self.config["minimum_wait_s"] + 15:
            self.tick()
            now = time.monotonic()
            rows = [row for row in self.joints if row[0] >= now - 0.5]
            if now - started >= self.config["minimum_wait_s"] and stable(rows, now, 0.5):
                return
        raise RuntimeError("arms did not settle")

    def capture(self):
        self.settle()
        for buffer in self.samples.values():
            buffer.clear()
        started = time.monotonic()
        while time.monotonic() - started < 15:
            self.tick()
            ended = time.monotonic()
            if ended - started < 1.0:
                continue
            joints = [row for row in self.joints if row[0] >= ended - 1.0]
            if not stable(joints, ended, 1.0):
                continue
            pair, records = {}, []
            captured_at = utc_now()
            for side in SIDES:
                rows = [row for row in self.samples[side] if ended - 1.0 <= row[0] <= ended]
                if (len(rows) < 20 or rows[-1][0] - rows[0][0] < 0.9 or ended - rows[-1][0] > 0.1
                        or np.max(np.diff([row[0] for row in rows])) > 0.1):
                    break
                angles, gravity, wrench = (np.array([row[index] for row in rows]) for index in (1, 2, 3))
                spread = wrench.std(axis=0)
                if np.max(np.ptp(gravity, axis=0)) > 0.15 or max(spread[:3]) > 1.0 or max(spread[3:]) > 0.1:
                    break
                position, direction, reading = angles.mean(axis=0), gravity.mean(axis=0), wrench.mean(axis=0)
                offset = 0 if side == "left" else 7
                torso_gravity = sensor_orientation_from_table(self.table, side, position[offset:offset + 7]) @ direction
                records.append(dict(id=len(self.records) + len(records) + 1, side=side, source="automatic_ft", captured_at=captured_at,
                                    positions=dict(zip(ALL_ARM_JOINTS, position.tolist())), gravity=torso_gravity.tolist(),
                                    wrench=reading.tolist(), wrench_std=spread.tolist()))
                pair[side] = FtSample(direction, reading)
            if len(pair) == 2:
                self.records.extend(records)
                return pair
        raise RuntimeError("no stable simultaneous force sample")

    def run(self):
        document = load_parameter_document(str(self.parameter_path))
        poses = load_poses(self.config, document["joints"])
        payload = self.parameters(PAYLOAD, ["ft_calibration", "estimation_enabled"])
        if payload["estimation_enabled"] is not False:
            raise RuntimeError("payload estimation must be disabled")
        if resolve_path(payload["ft_calibration"]) != self.calibration_path:
            raise RuntimeError("force and payload nodes use different calibration files")
        self.wait(self.fresh, 10.0)
        state = self.controller_state()
        topic = "/" + FPC + "/commands"
        if self.count_publishers(topic):
            raise RuntimeError("stop other FPC command publishers before calibration")
        names = self.parameters("/" + FPC, ["joints"])["joints"]
        if not names or not set(ALL_ARM_JOINTS).issubset(names) or not all(name in self.positions for name in names):
            raise RuntimeError("incomplete FPC joint feedback")
        initial = np.array([self.positions[name] for name in names])
        slots = np.array([names.index(name) for name in ALL_ARM_JOINTS])
        for index, side in enumerate(SIDES):
            arm_slice = slice(index * 7, index * 7 + 7)
            gravity = sensor_orientation_from_table(self.table, side, initial[slots[arm_slice]]) @ self.gravity[side]
            check_coverage([sensor_orientation_from_table(self.table, side, pose[arm_slice]).T @ gravity for pose in poses])
        self.output = self.create_publisher(Float64MultiArray, topic, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.wait(lambda: self.output.get_subscription_count() > 0, 3.0)
        self.switch("active")
        self.target, self.last_publish = initial.copy(), time.monotonic()
        samples = {side: [] for side in SIDES}
        try:
            for index, pose in enumerate(poses):
                self.get_logger().info("Both arms pose %d/%d" % (index + 1, len(poses)))
                target = initial.copy()
                target[slots] = pose
                self.move(target)
                for side, sample in self.capture().items():
                    samples[side].append(sample)
        finally:
            failed = sys.exc_info()[0] is not None
            if not self.moving and self.fresh():
                try:
                    self.move(initial)
                    self.settle()
                    self.switch(state)
                except (Exception, KeyboardInterrupt) as error:
                    self.get_logger().error("Could not restore initial position/FPC state: " + str(error))
                    if not failed:
                        raise
            else:
                self.get_logger().error("Feedback/interpolation interrupted; FPC state not restored")
                if not failed:
                    raise RuntimeError("initial state not restored")
        self.save(document, samples)

    def save(self, document, samples):
        previous = yaml.safe_load(self.calibration_path.read_text())["ft_wrench_compensator"]["ros__parameters"]
        results = {}
        for side in SIDES:
            check_coverage([sample.gravity for sample in samples[side]])
            solution = solve_ft_calibration(samples[side], origin=previous[side]["measurement_origin"])
            residual = np.array([net_wrench(sample.wrench, solution.calibration, sample.gravity) for sample in samples[side]])
            rms = np.sqrt(np.mean(residual ** 2, axis=0))
            report = "%s: mass=%.4f kg, force RMS=%s N, torque RMS=%s Nm" % (
                side, solution.calibration.mass, np.round(rms[:3], 4).tolist(), np.round(rms[3:], 4).tolist())
            print(report, flush=True)
            results[side] = dict(solution.calibration.to_dict(), frame=FT_SENSOR_LINKS[side])
            document["ft_sensor"][side] = dict(calibrated_at=utc_now(), calibration=solution.calibration.to_dict(),
                                               diagnostics=solution.diagnostics)
        document["ft_sensor"]["samples"] = self.records
        document["ft_sensor"].pop("automatic_run", None)
        document["updated_at"] = utc_now()
        calibration = yaml.safe_dump({"ft_wrench_compensator": {"ros__parameters": results}}, sort_keys=False).encode()
        parameters = (json.dumps(document, ensure_ascii=False, indent=2) + "\n").encode()
        atomic_write(str(self.calibration_path), calibration)
        atomic_write(str(self.parameter_path), parameters)
        try:
            for node in (COMPENSATOR, PAYLOAD):
                response = self.call(Trigger, node + "/reload_calibration", Trigger.Request())
                if not response.success:
                    raise RuntimeError(node + ": " + response.message)
        except (Exception, KeyboardInterrupt) as error:
            raise RuntimeError("files saved; runtime reload incomplete: %s" % error) from error
        print("Calibration saved and reloaded; original position and FPC state restored.")


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="package://arm_gravity_compensation/config/auto_ft_calibration.yaml")
    options = parser.parse_args(args)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = AutoFtCalibration(options.config)
        node.run()
        return 0
    except (Exception, KeyboardInterrupt) as error:
        print("Stopped:", str(error) or "interrupted", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
