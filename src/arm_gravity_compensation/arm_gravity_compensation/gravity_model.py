"""Pinocchio gravity model containing only both arms relative to the torso."""

import copy
import hashlib
import xml.etree.ElementTree as ET
from typing import Dict, Mapping, Optional, Sequence, Tuple

import numpy as np
from numpy.typing import ArrayLike
import pinocchio as pin

from .constants import ALL_ARM_JOINTS, ARM_JOINTS, FT_SENSOR_LINKS, SIDES



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


def _origin(joint: ET.Element) -> Tuple[np.ndarray, np.ndarray]:
    origin = joint.find("origin")
    xyz = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
    rpy = origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
    return (_rpy_matrix([float(value) for value in rpy.split()]),
            np.array([float(value) for value in xyz.split()], dtype=float))


def welded_placement(root: ET.Element, ancestor: str,
                     link: str) -> Tuple[np.ndarray, np.ndarray]:
    """Return the constant pose of ``link`` in ``ancestor``'s frame.

    Only fixed joints may appear on the way up: a movable one would make the
    pose depend on a configuration, and every caller here needs a constant.
    """
    parent_joint = {joint.find("child").get("link"): joint
                    for joint in root.findall("joint")}
    rotation = np.eye(3)
    translation = np.zeros(3)
    current = link
    while current != ancestor:
        joint = parent_joint.get(current)
        if joint is None:
            raise ValueError("%s is not below %s" % (link, ancestor))
        if joint.get("type") != "fixed":
            raise ValueError("%s is not welded to %s" % (link, ancestor))
        joint_rotation, joint_translation = _origin(joint)
        translation = joint_translation + joint_rotation @ translation
        rotation = joint_rotation @ rotation
        current = joint.find("parent").get("link")
    return rotation, translation


def _descendants(root: ET.Element, link: str) -> Tuple[str, ...]:
    children: Dict[str, list] = {}
    for joint in root.findall("joint"):
        children.setdefault(joint.find("parent").get("link"), []).append(
            joint.find("child").get("link"))
    found: list = []
    pending = list(children.get(link, ()))
    while pending:
        current = pending.pop()
        found.append(current)
        pending.extend(children.get(current, ()))
    return tuple(found)


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
        # 力传感器固连在腕 yaw 上。它到腕 yaw 关节系的常值位姿既是负载的挂载点，
        # 也是运行时把重力方向转进测量系的那一步；它远端的 link 就是它称量的工具。
        # 没装传感器的 URDF（裸 G1）照样能用，只是没有负载那一路。
        present = {link.get("name") for link in self._template.findall("link")}
        self._wrist_links = {
            side: next(joint.find("child").get("link")
                       for joint in self._template.findall("joint")
                       if joint.get("name") == ARM_JOINTS[side][-1])
            for side in SIDES
        }
        self.sensor_placement = {
            side: welded_placement(
                self._template, self._wrist_links[side], FT_SENSOR_LINKS[side])
            for side in SIDES if FT_SENSOR_LINKS[side] in present
        }
        self.distal_links = {
            side: _descendants(self._template, FT_SENSOR_LINKS[side])
            for side in self.sensor_placement
        }
        self.model = _build_reduced_model(reduced_xml)
        self.data = self.model.createData()
        # 传感器姿态只用运动学，与质量缩放无关，所以这一对 (model, data) 建一次就
        # 不再动：标定线程随时会替换 self.model / self.data，而网页线程在同一时刻
        # 查询姿态，共用一个 Data 就是并发写同一块缓存。
        self._sensor_kinematics = (self.model, self.model.createData())
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

    def sensor_orientation(self, side: str, q: ArrayLike) -> np.ndarray:
        """Rotation of the force sensor frame in the torso frame.

        Gravity read in the torso frame becomes gravity in the sensor frame by
        transposing this, which is all the sensor calibration needs from the
        kinematics.
        """
        self._check_side(side)
        if side not in self.sensor_placement:
            raise ValueError("URDF has no %s" % FT_SENSOR_LINKS[side])
        model, data = self._sensor_kinematics
        pin.forwardKinematics(model, data, self._validate_configuration(q))
        joint_id = int(model.getJointId(ARM_JOINTS[side][-1]))
        return (np.asarray(data.oMi[joint_id].rotation) @
                self.sensor_placement[side][0])

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

    def distal_inertia(self, side: str) -> Tuple[float, np.ndarray]:
        """Mass and first moment of everything hanging off the force sensor.

        Expressed in the wrist-yaw joint frame at the current scales. The
        gripper closes through mimic joints rather than fixed ones, so the
        merge is left to Pinocchio at the same locked configuration the rest of
        the model uses.
        """
        self._check_side(side)
        if side not in self.distal_links:
            raise ValueError("URDF has no %s" % FT_SENSOR_LINKS[side])
        model = _build_reduced_model(
            self._scaled_urdf(keep=frozenset(self.distal_links[side])))
        inertia = model.inertias[int(model.getJointId(ARM_JOINTS[side][-1]))]
        mass = float(inertia.mass)
        if mass <= 0.0:
            return 0.0, np.zeros(3)
        return mass, mass * np.asarray(inertia.lever, dtype=float)

    def gravity_table(
        self,
        tool: Optional[Mapping[str, Mapping[str, ArrayLike]]] = None,
    ) -> Dict[str, object]:
        """Export the lumped serial chain a runtime controller needs.

        Every link welded to a moving body is already merged into that body by
        the reduced model, so seven rigid bodies per arm reproduce the joint
        torques exactly. The values are flat per-side arrays so the export is
        directly loadable as a ROS 2 parameter file; a consumer only needs
        forward kinematics plus one cross product per body.

        ``payload_origin_*`` is where the force sensor sits on the last body.
        A runtime payload is reported in that frame, so this is what lets the
        controller hang it on the chain, and what lets the sensor node rotate
        gravity into its own frame.

        Passing ``tool`` replaces the part of the last body that hangs off the
        sensor with what the sensor actually weighs. Leaving it out keeps the
        identified numbers untouched: the split is then bookkeeping only and
        every joint torque stays bit-for-bit what it was.
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
            rotation, translation = self.sensor_placement.get(
                side, (np.eye(3), np.zeros(3)))
            if tool is not None and side in tool:
                masses[-1], centres[-3:] = self._measured_tool_body(
                    side, masses[-1], np.asarray(centres[-3:], dtype=float),
                    tool[side])
            table[side] = {
                "joints": names,
                "axis": axes,
                "origin_xyz": origins,
                "origin_rotation": rotations,
                "mass": masses,
                "com": centres,
                "payload_origin_xyz": translation.tolist(),
                "payload_origin_rotation": [float(value)
                                            for value in rotation.ravel()],
            }
        return table

    def _measured_tool_body(self, side: str, mass: float, centre: np.ndarray,
                            tool: Mapping[str, ArrayLike]) -> Tuple[float, list]:
        """Swap the modelled tool of the last body for the weighed one."""
        rotation, translation = self.sensor_placement[side]
        measured_mass = float(tool["mass"])
        measured_centre = np.asarray(tool["com"], dtype=float).reshape(3)
        modelled_mass, modelled_moment = self.distal_inertia(side)
        total_mass = mass - modelled_mass + measured_mass
        if total_mass <= 0.0:
            raise ValueError(
                "%s tool weighs %.3f kg but the identified wrist group only "
                "holds %.3f kg" % (side, measured_mass, mass))
        moment = (mass * centre - modelled_moment + measured_mass *
                  (translation + rotation @ measured_centre))
        return total_mass, (moment / total_mass).tolist()

    def _scaled_urdf(self, keep: Optional[frozenset] = None,
                     scaled: bool = True) -> str:
        """Reduced URDF at the current scales; ``keep`` blanks every other link."""
        root = copy.deepcopy(self._template)
        scale_by_link = self._scale_by_link()
        for link in root.findall("link"):
            inertial = link.find("inertial")
            if inertial is None:
                continue
            link_name = link.get("name")
            if keep is not None and link_name not in keep:
                link.remove(inertial)
                continue
            _scale_inertial(
                inertial, scale_by_link.get(link_name, 1.0) if scaled else 1.0)
        return ET.tostring(root, encoding="unicode")

    def _scale_by_link(self) -> Dict[str, float]:
        return {
            link_name: float(scale)
            for side in SIDES
            for link_name, scale in zip(
                self.parameter_links[side], self._scales[side])
        }

    def _basis_model(self, link_name: str):
        cached = self._basis_models.get(link_name)
        if cached is None:
            model = _build_reduced_model(
                self._scaled_urdf(keep=frozenset([link_name]), scaled=False))
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