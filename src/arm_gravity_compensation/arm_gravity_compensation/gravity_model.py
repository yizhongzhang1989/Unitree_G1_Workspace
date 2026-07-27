"""Pinocchio gravity model containing only both arms relative to the torso."""

import copy
import hashlib
import xml.etree.ElementTree as ET
from typing import Dict, Mapping, Sequence, Tuple

import numpy as np
from numpy.typing import ArrayLike
import pinocchio as pin

from .constants import ALL_ARM_JOINTS, ARM_JOINTS, SIDES


def _rpy_matrix(rpy: ArrayLike) -> np.ndarray:
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


_UNIT_AXES = {"JointModelRX": (1.0, 0.0, 0.0),
              "JointModelRY": (0.0, 1.0, 0.0),
              "JointModelRZ": (0.0, 0.0, 1.0)}


def _joint_axis(joint) -> np.ndarray:
    """Return the rotation axis of a revolute joint in its own frame."""
    aligned = _UNIT_AXES.get(joint.shortname())
    if aligned is not None:
        return np.array(aligned, dtype=float)
    axis = getattr(joint, "axis", None)
    if axis is None:
        raise ValueError("unsupported joint type %s" % joint.shortname())
    return np.asarray(axis, dtype=float)


def _axis_rotation(axis: ArrayLike, angle: float) -> np.ndarray:
    """Rodrigues rotation about a unit axis."""
    unit = np.asarray(axis, dtype=float)
    cross = np.array([[0.0, -unit[2], unit[1]],
                      [unit[2], 0.0, -unit[0]],
                      [-unit[1], unit[0], 0.0]])
    return (np.eye(3) + np.sin(angle) * cross +
            (1.0 - np.cos(angle)) * (cross @ cross))


def imu_to_torso_rotation(urdf_xml: str) -> np.ndarray:
    root = ET.fromstring(urdf_xml)
    for joint in root.findall("joint"):
        if joint.get("name") != "imu_in_torso_joint":
            continue
        parent = joint.find("parent")
        child = joint.find("child")
        if (parent is None or parent.get("link") != "torso_link" or
                child is None or child.get("link") != "imu_in_torso"):
            raise ValueError("imu_in_torso_joint is not fixed to torso_link")
        origin = joint.find("origin")
        rpy = (origin.get("rpy", "0 0 0") if origin is not None
               else "0 0 0")
        return _rpy_matrix([float(value) for value in rpy.split()])
    raise ValueError("URDF does not contain imu_in_torso_joint")


def _model_link(source: ET.Element) -> ET.Element:
    link = ET.Element("link", {"name": source.get("name")})
    inertial = source.find("inertial")
    if inertial is not None:
        link.append(copy.deepcopy(inertial))
    return link


def extract_torso_arm_urdf(
    urdf_xml: str,
) -> Tuple[str, Dict[str, Tuple[str, ...]], Dict[str, str]]:
    """Extract both complete shoulder subtrees with a fixed torso root.

    Each inertial link is assigned to its nearest upstream Unitree arm joint.
    This keeps fixed sensors and locked gripper parts as individual parameters
    while associating them with the wrist-yaw selection in the dashboard.
    """
    source = ET.fromstring(urdf_xml)
    links = {link.get("name"): link for link in source.findall("link")}
    joints = {joint.get("name"): joint for joint in source.findall("joint")}
    children: Dict[str, list] = {}
    for joint in source.findall("joint"):
        parent = joint.find("parent")
        if parent is not None:
            children.setdefault(parent.get("link"), []).append(joint)
    missing = [name for name in ALL_ARM_JOINTS if name not in joints]
    if missing or "torso_link" not in links:
        raise ValueError("URDF is missing arm elements: %s" % missing)

    reduced = ET.Element("robot", {"name": "g1_complete_torso_arms"})
    ET.SubElement(reduced, "link", {"name": "torso_link"})
    parameter_links = {side: [] for side in SIDES}
    parameter_owner: Dict[str, str] = {}
    added_links = {"torso_link"}
    added_joints = set()

    def append_subtree(joint: ET.Element, owner: str = "") -> None:
        joint_name = joint.get("name")
        if joint_name in added_joints:
            return
        child = joint.find("child")
        if child is None:
            raise ValueError("joint %s has no child" % joint_name)
        child_name = child.get("link")
        if child_name not in links:
            raise ValueError("joint %s child link is missing" % joint_name)
        if joint_name in ALL_ARM_JOINTS:
            owner = joint_name
        if not owner:
            raise ValueError("link %s has no controlled arm owner" % child_name)
        if child_name not in added_links:
            reduced.append(_model_link(links[child_name]))
            added_links.add(child_name)
            inertial = links[child_name].find("inertial")
            mass = inertial.find("mass") if inertial is not None else None
            if mass is not None and float(mass.get("value", "0")) > 0.0:
                side = "left" if owner.startswith("left_") else "right"
                parameter_links[side].append(child_name)
                parameter_owner[child_name] = owner
        reduced.append(copy.deepcopy(joint))
        added_joints.add(joint_name)
        for descendant in children.get(child_name, []):
            append_subtree(descendant, owner)

    for side in SIDES:
        append_subtree(joints[side + "_shoulder_pitch_joint"])
    return (
        ET.tostring(reduced, encoding="unicode"),
        {side: tuple(parameter_links[side]) for side in SIDES},
        parameter_owner,
    )


def _scale_inertial(inertial: ET.Element, scale: float) -> None:
    mass = inertial.find("mass")
    inertia = inertial.find("inertia")
    if mass is None or inertia is None:
        raise ValueError("inertial is missing mass or inertia")
    mass.set("value", "%.17g" % (float(mass.get("value")) * scale))
    for name in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
        inertia.set(name, "%.17g" % (float(inertia.get(name)) * scale))


def _full_reference_configuration(model, root: ET.Element) -> np.ndarray:
    configuration = pin.neutral(model)
    mimic_joints = []
    for joint in root.findall("joint"):
        mimic = joint.find("mimic")
        if mimic is not None:
            mimic_joints.append((joint, mimic))
    for _ in range(max(1, len(mimic_joints))):
        changed = False
        for joint, mimic in mimic_joints:
            joint_id = int(model.getJointId(joint.get("name")))
            source_id = int(model.getJointId(mimic.get("joint")))
            if joint_id == 0 or source_id == 0:
                raise ValueError("mimic joint is missing from Pinocchio model")
            index = int(model.idx_qs[joint_id])
            value = (
                float(mimic.get("multiplier", "1")) *
                configuration[int(model.idx_qs[source_id])] +
                float(mimic.get("offset", "0"))
            )
            if configuration[index] != value:
                configuration[index] = value
                changed = True
        if not changed:
            break
    return configuration


def _build_reduced_model(urdf_xml: str):
    full_model = pin.buildModelFromXML(urdf_xml)
    reference = _full_reference_configuration(
        full_model, ET.fromstring(urdf_xml))
    arm_ids = {int(full_model.getJointId(name)) for name in ALL_ARM_JOINTS}
    if 0 in arm_ids:
        raise ValueError("Pinocchio model is missing an arm joint")
    locked = [joint_id for joint_id in range(1, full_model.njoints)
              if joint_id not in arm_ids]
    model = pin.buildReducedModel(full_model, locked, reference)
    if model.nq != 14 or model.nv != 14:
        raise ValueError(
            "expected reduced 14-DoF arms, got nq=%d nv=%d"
            % (model.nq, model.nv))
    return model


class TorsoArmGravityModel:
    """A 14-DoF model retaining one scale per original arm-subtree link."""

    def __init__(self, urdf_xml: str) -> None:
        self.imu_to_torso = imu_to_torso_rotation(urdf_xml)
        (reduced_xml, self.parameter_links,
         self.parameter_owner) = extract_torso_arm_urdf(urdf_xml)
        self._template = ET.fromstring(reduced_xml)
        self._scales = {
            side: np.ones(len(self.parameter_links[side]), dtype=float)
            for side in SIDES
        }
        self._biases = {side: np.zeros(7, dtype=float) for side in SIDES}
        template_masses = {
            link.get("name"): float(link.find("inertial").find("mass").get("value"))
            for link in self._template.findall("link")
            if link.find("inertial") is not None
        }
        self.parameter_masses = {
            side: np.array([template_masses[name]
                            for name in self.parameter_links[side]], dtype=float)
            for side in SIDES
        }
        template_efforts = {
            joint.get("name"): float(joint.find("limit").get("effort"))
            for joint in self._template.findall("joint")
            if joint.find("limit") is not None
        }
        self.joint_efforts = {
            side: np.array([template_efforts[name] for name in ARM_JOINTS[side]],
                           dtype=float)
            for side in SIDES
        }
        if any(np.any(values <= 0.0) for values in self.joint_efforts.values()):
            raise ValueError("arm joints must declare a positive effort limit")
        self.model = _build_reduced_model(reduced_xml)
        self.data = self.model.createData()
        self.joint_names = ALL_ARM_JOINTS
        self._refresh_indices()
        self._basis_models: Dict[str, tuple] = {}

    @classmethod
    def from_urdf_file(cls, path: str) -> "TorsoArmGravityModel":
        with open(path, "r", encoding="utf-8") as stream:
            return cls(stream.read())

    def q_indices(self, side: str) -> np.ndarray:
        self._check_side(side)
        return np.array([self._q_indices[name] for name in ARM_JOINTS[side]],
                        dtype=int)

    def configuration(self, arm_positions: Mapping[str, ArrayLike]) -> np.ndarray:
        q = np.zeros(self.model.nq, dtype=float)
        for side, values in arm_positions.items():
            self._check_side(side)
            array = np.asarray(values, dtype=float)
            if array.shape != (7,) or not np.all(np.isfinite(array)):
                raise ValueError("%s arm positions must be seven finite values" % side)
            q[self.q_indices(side)] = array
        return q

    def set_arm_parameters(
        self,
        side: str,
        mass_scales: ArrayLike,
        torque_bias: ArrayLike,
    ) -> None:
        self._check_side(side)
        scales = np.asarray(mass_scales, dtype=float)
        biases = np.asarray(torque_bias, dtype=float)
        expected = len(self.parameter_links[side])
        if (scales.shape != (expected,) or biases.shape != (7,) or
                not np.all(np.isfinite(scales)) or
                not np.all(np.isfinite(biases))):
            raise ValueError(
                "mass scales must contain %d values and torque bias seven" % expected)
        if np.any(scales <= 0.0):
            raise ValueError("mass scales must be positive")
        self._scales[side] = scales.copy()
        self._biases[side] = biases.copy()
        self.model = _build_reduced_model(self._scaled_urdf())
        self.data = self.model.createData()
        self._refresh_indices()

    def arm_parameters(self, side: str) -> Tuple[np.ndarray, np.ndarray]:
        self._check_side(side)
        return self._scales[side].copy(), self._biases[side].copy()

    def group_aggregation(self, side: str) -> np.ndarray:
        """Return the link-by-joint indicator of rigidly welded link groups.

        Links sharing an owner joint are welded to the same moving body, so
        gravity only reveals their aggregate mass and first moment.
        """
        self._check_side(side)
        joint_names = ARM_JOINTS[side]
        matrix = np.zeros(
            (len(self.parameter_links[side]), len(joint_names)), dtype=float)
        for row, link_name in enumerate(self.parameter_links[side]):
            matrix[row, joint_names.index(self.parameter_owner[link_name])] = 1.0
        return matrix

    def group_scales(self, side: str) -> np.ndarray:
        """Collapse the stored per-link scales into one scale per group."""
        self._check_side(side)
        aggregation = self.group_aggregation(side)
        masses = self.parameter_masses[side]
        total = aggregation.T @ masses
        weighted = aggregation.T @ (masses * self._scales[side])
        return np.where(total > 0.0, weighted / np.where(total > 0.0, total, 1.0),
                        1.0)

    def compensation(self, side: str, q: ArrayLike,
                     gravity: ArrayLike) -> np.ndarray:
        self._check_side(side)
        configuration = self._validate_configuration(q)
        self._set_gravity(self.model, gravity)
        torque = pin.computeGeneralizedGravity(
            self.model, self.data, configuration)
        rows = np.array([self._v_indices[name] for name in ARM_JOINTS[side]])
        return np.asarray(torque, dtype=float)[rows] + self._biases[side]

    def design_matrix(self, side: str, q: ArrayLike,
                      gravity: ArrayLike) -> np.ndarray:
        """Return one scale column per original link plus seven bias columns."""
        self._check_side(side)
        configuration = self._validate_configuration(q)
        rows = np.array([self._v_indices[name] for name in ARM_JOINTS[side]])
        matrix = np.zeros((7, len(self.parameter_links[side]) + 7), dtype=float)
        for column, link_name in enumerate(self.parameter_links[side]):
            basis_model, basis_data = self._basis_model(link_name)
            self._set_gravity(basis_model, gravity)
            torque = pin.computeGeneralizedGravity(
                basis_model, basis_data, configuration)
            matrix[:, column] = np.asarray(torque, dtype=float)[rows]
        matrix[:, len(self.parameter_links[side]):] = np.eye(7)
        return matrix

    def parameter_groups(self, side: str) -> Dict[str, Tuple[str, ...]]:
        self._check_side(side)
        return {
            joint_name: tuple(
                link_name for link_name in self.parameter_links[side]
                if self.parameter_owner[link_name] == joint_name)
            for joint_name in ARM_JOINTS[side]
        }

    def gravity_table(self) -> Dict[str, object]:
        """Export the lumped serial chain a runtime controller needs.

        Every link welded to a moving body is already merged into that body by
        the reduced model, so seven rigid bodies per arm reproduce the joint
        torques exactly. The values are flat per-side arrays so the export is
        directly loadable as a ROS 2 parameter file; a consumer only needs
        forward kinematics plus one cross product per body.
        """
        table = {"imu_to_torso": [float(value)
                                  for value in self.imu_to_torso.ravel()]}
        for side in SIDES:
            names, axes, origins, rotations = [], [], [], []
            masses, centres = [], []
            for index, name in enumerate(ARM_JOINTS[side]):
                joint_id = int(self.model.getJointId(name))
                expected_parent = 0 if index == 0 else joint_id - 1
                if int(self.model.parents[joint_id]) != expected_parent:
                    raise ValueError(
                        "%s is not a serial chain rooted at torso_link" % side)
                placement = self.model.jointPlacements[joint_id]
                inertia = self.model.inertias[joint_id]
                names.append(name)
                axes.extend(_joint_axis(self.model.joints[joint_id]).tolist())
                origins.extend(placement.translation.tolist())
                rotations.extend(
                    float(value)
                    for value in np.asarray(placement.rotation).ravel())
                masses.append(float(inertia.mass))
                centres.extend(np.asarray(inertia.lever, dtype=float).tolist())
            table[side] = {
                "joints": names,
                "axis": axes,
                "origin_xyz": origins,
                "origin_rotation": rotations,
                "mass": masses,
                "com": centres,
            }
        return table

    @staticmethod
    def gravity_from_table(table: Dict[str, object], side: str,
                           q: ArrayLike,
                           gravity: ArrayLike) -> np.ndarray:
        """Reference implementation of what the runtime controller evaluates."""
        chain = table[side]
        angles = np.asarray(q, dtype=float)
        gravity_vector = np.asarray(gravity, dtype=float)
        masses = np.asarray(chain["mass"], dtype=float)
        count = masses.size

        rotation = np.eye(3)
        translation = np.zeros(3)
        origins = np.zeros((count, 3))
        axes = np.zeros((count, 3))
        moments = np.zeros((count, 3))
        for index in range(count):
            block = slice(3 * index, 3 * index + 3)
            translation = translation + rotation @ np.asarray(
                chain["origin_xyz"][block], dtype=float)
            rotation = rotation @ np.asarray(
                chain["origin_rotation"][9 * index:9 * index + 9],
                dtype=float).reshape(3, 3)
            axis = np.asarray(chain["axis"][block], dtype=float)
            rotation = rotation @ _axis_rotation(axis, float(angles[index]))
            origins[index] = translation
            axes[index] = rotation @ axis
            moments[index] = masses[index] * (
                translation + rotation @ np.asarray(
                    chain["com"][block], dtype=float))

        torque = np.zeros(count, dtype=float)
        for index in range(count):
            downstream_mass = float(np.sum(masses[index:]))
            downstream_moment = np.sum(moments[index:], axis=0)
            torque[index] -= axes[index] @ np.cross(
                downstream_moment - downstream_mass * origins[index],
                gravity_vector)
        return torque


    def _scaled_urdf(self, only_link: str = "") -> str:
        root = copy.deepcopy(self._template)
        scale_by_link = {
            link_name: float(scale)
            for side in SIDES
            for link_name, scale in zip(
                self.parameter_links[side], self._scales[side])
        }
        for link in root.findall("link"):
            inertial = link.find("inertial")
            if inertial is None:
                continue
            link_name = link.get("name")
            if only_link and link_name != only_link:
                link.remove(inertial)
                continue
            _scale_inertial(
                inertial, 1.0 if only_link else scale_by_link[link_name])
        return ET.tostring(root, encoding="unicode")

    def _basis_model(self, link_name: str):
        cached = self._basis_models.get(link_name)
        if cached is None:
            model = _build_reduced_model(self._scaled_urdf(only_link=link_name))
            cached = (model, model.createData())
            self._basis_models[link_name] = cached
        return cached

    def _refresh_indices(self) -> None:
        """Cache the Pinocchio indices and assert they match the motor order."""
        self._q_indices = {
            name: int(self.model.idx_qs[self.model.getJointId(name)])
            for name in ALL_ARM_JOINTS
        }
        self._v_indices = {
            name: int(self.model.idx_vs[self.model.getJointId(name)])
            for name in ALL_ARM_JOINTS
        }
        expected = list(range(len(ALL_ARM_JOINTS)))
        if ([self._q_indices[name] for name in ALL_ARM_JOINTS] != expected or
                [self._v_indices[name] for name in ALL_ARM_JOINTS] != expected):
            raise ValueError(
                "Pinocchio joint order does not match the LowState motor order")

    @staticmethod
    def _set_gravity(model, gravity: ArrayLike) -> None:
        vector = np.asarray(gravity, dtype=float)
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            raise ValueError("gravity must contain three finite values")
        if np.linalg.norm(vector) < 1e-6:
            raise ValueError("gravity vector must be non-zero")
        model.gravity.linear = vector

    def _validate_configuration(self, q: ArrayLike) -> np.ndarray:
        array = np.asarray(q, dtype=float)
        if array.shape != (self.model.nq,) or not np.all(np.isfinite(array)):
            raise ValueError("q must contain 14 finite arm positions")
        return array

    @staticmethod
    def _check_side(side: str) -> None:
        if side not in SIDES:
            raise ValueError("side must be 'left' or 'right'")