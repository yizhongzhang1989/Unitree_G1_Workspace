#!/usr/bin/env python3
"""Foxy adapter for the G1 controller test dashboard."""

from dataclasses import dataclass
import importlib
import math
import threading
from typing import Dict, List, Optional, Sequence, Tuple
from xml.etree import ElementTree

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


RobotTestDashboard = importlib.import_module(
    "robot_test_dashboard.dashboard_node").RobotTestDashboard


JOINT_KINDS = ("forward_position", "joint_trajectory")
SWITCH_TIMEOUT_S = 30.0


Matrix = List[List[float]]


@dataclass(frozen=True)
class MimicJointSpec:
    name: str
    source: str
    parent: str
    child: str
    joint_type: str
    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float]
    axis: Tuple[float, float, float]
    multiplier: float
    offset: float
    lower: float
    upper: float


def _vector(element, attribute: str,
            default: str) -> Tuple[float, float, float]:
    values = tuple(
        float(value) for value in element.get(attribute, default).split())
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError(f"{attribute} must contain three finite values")
    return values


def _parse_mimic_joints(urdf_xml: str) -> Tuple[MimicJointSpec, ...]:
    root = ElementTree.fromstring(urdf_xml)
    result = []
    for joint in root.findall("joint"):
        mimic = joint.find("mimic")
        if mimic is None:
            continue
        parent = joint.find("parent")
        child = joint.find("child")
        limit = joint.find("limit")
        if parent is None or child is None or limit is None:
            raise ValueError(
                f"mimic joint {joint.get('name')!r} is incomplete")
        origin = joint.find("origin")
        axis = joint.find("axis")
        values = (
            float(mimic.get("multiplier", "1")),
            float(mimic.get("offset", "0")),
            float(limit.get("lower", "-inf")),
            float(limit.get("upper", "inf")),
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError(
                f"mimic joint {joint.get('name')!r} has invalid values")
        result.append(MimicJointSpec(
            name=joint.get("name", ""),
            source=mimic.get("joint", ""),
            parent=parent.get("link", ""),
            child=child.get("link", ""),
            joint_type=joint.get("type", ""),
            xyz=(
                _vector(origin, "xyz", "0 0 0")
                if origin is not None else (0.0, 0.0, 0.0)),
            rpy=(
                _vector(origin, "rpy", "0 0 0")
                if origin is not None else (0.0, 0.0, 0.0)),
            axis=(
                _vector(axis, "xyz", "1 0 0")
                if axis is not None else (1.0, 0.0, 0.0)),
            multiplier=values[0],
            offset=values[1],
            lower=values[2],
            upper=values[3],
        ))
    return tuple(result)


def _multiply(left: Matrix, right: Matrix) -> Matrix:
    return [[
        sum(left[row][index] * right[index][column] for index in range(4))
        for column in range(4)
    ] for row in range(4)]


def _origin_matrix(spec: MimicJointSpec) -> Matrix:
    roll, pitch, yaw = spec.rpy
    cosine_roll, sine_roll = math.cos(roll), math.sin(roll)
    cosine_pitch, sine_pitch = math.cos(pitch), math.sin(pitch)
    cosine_yaw, sine_yaw = math.cos(yaw), math.sin(yaw)
    x, y, z = spec.xyz
    return [
        [cosine_yaw * cosine_pitch,
         cosine_yaw * sine_pitch * sine_roll - sine_yaw * cosine_roll,
         cosine_yaw * sine_pitch * cosine_roll + sine_yaw * sine_roll, x],
        [sine_yaw * cosine_pitch,
         sine_yaw * sine_pitch * sine_roll + cosine_yaw * cosine_roll,
         sine_yaw * sine_pitch * cosine_roll - cosine_yaw * sine_roll, y],
        [-sine_pitch, cosine_pitch * sine_roll,
         cosine_pitch * cosine_roll, z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _motion_matrix(spec: MimicJointSpec, value: float) -> Matrix:
    axis_norm = math.sqrt(sum(component * component for component in spec.axis))
    if axis_norm <= 0.0:
        raise ValueError(f"mimic joint {spec.name!r} has a zero axis")
    x, y, z = (component / axis_norm for component in spec.axis)
    if spec.joint_type == "prismatic":
        return [
            [1.0, 0.0, 0.0, x * value],
            [0.0, 1.0, 0.0, y * value],
            [0.0, 0.0, 1.0, z * value],
            [0.0, 0.0, 0.0, 1.0],
        ]
    if spec.joint_type not in ("revolute", "continuous"):
        raise ValueError(f"unsupported mimic joint type {spec.joint_type!r}")
    cosine, sine = math.cos(value), math.sin(value)
    one_minus_cosine = 1.0 - cosine
    return [
        [cosine + x * x * one_minus_cosine,
         x * y * one_minus_cosine - z * sine,
         x * z * one_minus_cosine + y * sine, 0.0],
        [y * x * one_minus_cosine + z * sine,
         cosine + y * y * one_minus_cosine,
         y * z * one_minus_cosine - x * sine, 0.0],
        [z * x * one_minus_cosine - y * sine,
         z * y * one_minus_cosine + x * sine,
         cosine + z * z * one_minus_cosine, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _mimic_values(
        specs: Sequence[MimicJointSpec], positions: Dict[str, float]
        ) -> Dict[str, float]:
    values = dict(positions)
    pending = list(specs)
    while pending:
        unresolved = []
        for spec in pending:
            source = values.get(spec.source)
            if source is None or not math.isfinite(source):
                unresolved.append(spec)
                continue
            raw = spec.multiplier * source + spec.offset
            values[spec.name] = min(max(raw, spec.lower), spec.upper)
        if len(unresolved) == len(pending):
            break
        pending = unresolved
    return values


def _correct_mimic_transforms(
        link_tf: Dict[str, Matrix], specs: Sequence[MimicJointSpec],
        positions: Dict[str, float]) -> Dict[str, Matrix]:
    values = _mimic_values(specs, positions)
    resolved = [spec for spec in specs if spec.name in values]
    mimic_children = {spec.child for spec in resolved}
    corrected = {
        link: transform for link, transform in link_tf.items()
        if link not in mimic_children
    }
    pending = resolved
    while pending:
        unresolved = []
        for spec in pending:
            parent = corrected.get(spec.parent)
            value = values.get(spec.name)
            if parent is None or value is None:
                unresolved.append(spec)
                continue
            local = _multiply(
                _origin_matrix(spec), _motion_matrix(spec, value))
            corrected[spec.child] = _multiply(parent, local)
        if len(unresolved) == len(pending):
            break
        pending = unresolved
    return corrected


def _is_internal_name(name: str) -> bool:
    return name.startswith("internal_") or "_internal_" in name


def _collapse_joint_tree(
        joint_tree: Sequence[dict], hidden_links: set) -> List[dict]:
    parent_by_child = {
        joint["child"]: joint["parent"] for joint in joint_tree}
    result = []
    for joint in joint_tree:
        child = joint["child"]
        if child in hidden_links:
            continue
        parent = joint["parent"]
        visited = set()
        while parent in hidden_links and parent not in visited:
            visited.add(parent)
            parent = parent_by_child.get(parent, "")
        if parent:
            result.append({**joint, "parent": parent})
    return result


def _hidden_mimic_names(
        specs: Sequence[MimicJointSpec]) -> Tuple[set, set]:
    return (
        {spec.child for spec in specs if _is_internal_name(spec.child)},
        {spec.name for spec in specs if _is_internal_name(spec.name)},
    )


def _filter_mimic_snapshot(
        state: dict, hidden_links: set, hidden_joints: set) -> dict:
    if "links" in state:
        state["links"] = [
            link for link in state["links"] if link not in hidden_links]
    if "link_tf" in state:
        state["link_tf"] = {
            link: transform for link, transform in state["link_tf"].items()
            if link not in hidden_links
        }
    if "skeleton" in state:
        state["skeleton"] = {
            link: [matrix[0][3], matrix[1][3], matrix[2][3]]
            for link, matrix in state.get("link_tf", {}).items()
        }
    if "visuals" in state:
        state["visuals"] = [
            visual for visual in state["visuals"]
            if visual["link"] not in hidden_links
        ]
        if "has_meshes" in state:
            state["has_meshes"] = bool(state["visuals"])
    if "joint_tree" in state:
        state["joint_tree"] = _collapse_joint_tree(
            state["joint_tree"], hidden_links)
    if "movable_joints" in state:
        state["movable_joints"] = [
            joint for joint in state["movable_joints"]
            if joint["name"] not in hidden_joints
        ]
    if "joints" in state:
        state["joints"] = [
            joint for joint in state["joints"]
            if joint not in hidden_joints
        ]
    for key in ("joint_values", "joint_limits"):
        if key in state:
            state[key] = {
                joint: value for joint, value in state[key].items()
                if joint not in hidden_joints
            }
    if "joint_state_names" in state:
        state["joint_state_names"] = [
            joint for joint in state["joint_state_names"]
            if joint not in hidden_joints
        ]

    status = state.get("status")
    if isinstance(status, dict):
        state["status"] = dict(status)
        for key in ("available_links",):
            values = state["status"].get(key)
            if isinstance(values, list):
                state["status"][key] = [
                    value for value in values if value not in hidden_links]
        for key in (
                "available_joints", "fixed_joints", "joints",
                "group_joints", "command_joints"):
            values = state["status"].get(key)
            if isinstance(values, list):
                state["status"][key] = [
                    value for value in values if value not in hidden_joints]
        for key in ("joint_speed_limits", "joint_accel_limits"):
            values = state["status"].get(key)
            if isinstance(values, dict):
                state["status"][key] = {
                    joint: value for joint, value in values.items()
                    if joint not in hidden_joints
                }
    return state


def _wait_result(future, timeout_s: float):
    done = threading.Event()
    future.add_done_callback(lambda _future: done.set())
    if not done.wait(timeout=timeout_s):
        return None
    try:
        return future.result()
    except Exception:  # noqa: BLE001
        return None


def _parameter_service(controller_manager: str, controller: str) -> str:
    manager = "/" + controller_manager.strip("/")
    namespace = manager.rpartition("/")[0]
    path = "/" + controller.strip("/")
    if namespace and not path.startswith(namespace + "/"):
        path = namespace + path
    return path + "/get_parameters"


class G1RobotTestDashboard(RobotTestDashboard):
    def __init__(self) -> None:
        self._joint_meta_lock = threading.Lock()
        self._joint_clients = {}
        self._joint_pending = set()
        self._joint_cache: Dict[str, List[str]] = {}
        self._mimic_specs: Tuple[MimicJointSpec, ...] = ()
        self._hidden_mimic_links = set()
        self._hidden_mimic_joints = set()
        self._joint_parameter_group = ReentrantCallbackGroup()
        super().__init__()

    def _on_urdf(self, message) -> None:
        super()._on_urdf(message)
        try:
            specs = _parse_mimic_joints(message.data)
        except (ValueError, ElementTree.ParseError) as error:
            self.get_logger().warning(f"cannot parse mimic joints: {error}")
            specs = ()
        hidden_links, hidden_joints = _hidden_mimic_names(specs)
        with self._lock:
            self._mimic_specs = specs
            self._hidden_mimic_links = hidden_links
            self._hidden_mimic_joints = hidden_joints

    def _link_tf(self) -> Dict[str, Matrix]:
        transforms = super()._link_tf()
        with self._lock:
            specs = self._mimic_specs
            positions = dict(self._joint_pos)
        return _correct_mimic_transforms(transforms, specs, positions)

    def snapshot(self) -> dict:
        state = super().snapshot()
        with self._lock:
            hidden_links = set(self._hidden_mimic_links)
            hidden_joints = set(self._hidden_mimic_joints)
        return _filter_mimic_snapshot(
            state, hidden_links, hidden_joints)

    def _on_controllers(self, future) -> None:
        super()._on_controllers(future)
        pending = []
        with self._joint_meta_lock:
            cached = dict(self._joint_cache)
        with self._lock:
            for controller in self._controllers:
                if controller["cmd_ifaces"]:
                    continue
                interfaces = cached.get(controller["name"])
                if interfaces:
                    self._apply_interfaces(controller, interfaces)
                elif controller["kind"] in JOINT_KINDS:
                    pending.append(controller["name"])
        for name in pending:
            self._request_joints(name)

    @staticmethod
    def _apply_interfaces(controller: dict, interfaces: List[str]) -> None:
        controller["cmd_ifaces"] = list(interfaces)
        controller["joints"] = [
            interface.rpartition("/")[0] for interface in interfaces]

    def _request_joints(self, name: str) -> None:
        with self._joint_meta_lock:
            if name in self._joint_cache or name in self._joint_pending:
                return
            client = self._joint_clients.get(name)
            if client is None:
                client = self.create_client(
                    GetParameters,
                    _parameter_service(self._cm_ns, name),
                    callback_group=self._joint_parameter_group,
                )
                self._joint_clients[name] = client
            if not client.service_is_ready():
                return
            self._joint_pending.add(name)
        request = GetParameters.Request()
        request.names = ["joints"]
        future = client.call_async(request)
        future.add_done_callback(
            lambda done, controller=name: self._store_joints(controller, done))

    def _store_joints(self, name: str, future) -> None:
        try:
            response = future.result()
        except Exception:  # noqa: BLE001
            response = None
        values = list(getattr(response, "values", []) or []) if response else []
        interfaces = [
            f"{joint}/position" for joint in
            (values[0].string_array_value if values else []) if joint]
        with self._joint_meta_lock:
            self._joint_pending.discard(name)
            if interfaces:
                self._joint_cache[name] = interfaces
        if interfaces:
            with self._lock:
                for controller in self._controllers:
                    if controller["name"] == name and not controller["cmd_ifaces"]:
                        self._apply_interfaces(controller, interfaces)
                        break

    def _switch(self, activate: List[str], deactivate: List[str],
                timeout: float = SWITCH_TIMEOUT_S) -> bool:
        if self._cli_switch is None or not (activate or deactivate):
            return not (activate or deactivate)
        if not self._cli_switch.wait_for_service(timeout_sec=timeout):
            return False
        request = SwitchController.Request()
        request.start_controllers = list(activate)
        request.stop_controllers = list(deactivate)
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.start_asap = True
        response = self._wait(
            self._cli_switch.call_async(request), timeout)
        if response is None or not response.ok:
            return False
        if self._cli_list is None or not self._cli_list.wait_for_service(
                timeout_sec=2.0):
            return False
        listed = _wait_result(
            self._cli_list.call_async(ListControllers.Request()), 3.0)
        states = {
            controller.name: controller.state
            for controller in (getattr(listed, "controller", []) or [])
        }
        return all(states.get(name) == "active" for name in activate) and all(
            states.get(name) == "inactive"
            for name in deactivate if name not in activate)


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=list(args) if args is not None else None)
    node = G1RobotTestDashboard()
    executor = MultiThreadedExecutor()
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
