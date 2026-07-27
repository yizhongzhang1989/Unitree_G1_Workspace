"""Persistent inertial parameters extracted from a source URDF."""

import copy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import tempfile
from typing import Dict, Mapping, Optional, Sequence
import xml.etree.ElementTree as ET

import numpy as np
from numpy.typing import ArrayLike

from .constants import (ALL_ARM_JOINTS, ARM_JOINTS, SIDES, mirror_arm_values,
                        opposite_side)


SCHEMA_VERSION = 2
# A pose is hand-guided on one arm, or on both at once when the operator
# selected joints from each side.
CAPTURE_SIDES = SIDES + ("both",)
# Posterior variance reduction above which a link counts as measured rather
# than carried over from the URDF prior.
IDENTIFIED_OBSERVABILITY = 0.9


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _vector(element: Optional[ET.Element], attribute: str,
            default: str) -> list:
    text = element.get(attribute, default) if element is not None else default
    values = [float(value) for value in text.split()]
    if len(values) != 3:
        raise ValueError("%s must contain three values" % attribute)
    return values

# 类型转换用
def _number(element: ET.Element, attribute: str) -> float:
    text = element.get(attribute)
    if text is None:
        raise ValueError(
            "<%s> is missing the %s attribute" % (element.tag, attribute))
    return float(text)


def _inertial_parameters(link: ET.Element) -> Optional[Dict[str, object]]:
    inertial = link.find("inertial")
    if inertial is None:
        return None
    origin = inertial.find("origin")
    mass = inertial.find("mass")
    inertia = inertial.find("inertia")
    if mass is None or inertia is None:
        raise ValueError("link %s has an incomplete inertial" % link.get("name"))
    return {
        "origin_xyz": _vector(origin, "xyz", "0 0 0"),
        "origin_rpy": _vector(origin, "rpy", "0 0 0"),
        "mass": _number(mass, "value"),
        "inertia": {
            name: _number(inertia, name)
            for name in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
        },
    }


def create_parameter_document(urdf_xml: str, source_path: str) -> dict:
    root = ET.fromstring(urdf_xml)
    links = root.findall("link")
    joints = root.findall("joint")
    parent_joint = {}
    joint_parameters = {}
    for joint in joints:
        name = joint.get("name")
        parent = joint.find("parent")
        child = joint.find("child")
        if not name or parent is None or child is None:
            continue
        child_link = child.get("link")
        parent_joint[child_link] = name
        limit = joint.find("limit")
        joint_parameters[name] = {
            "type": joint.get("type", ""),
            "parent_link": parent.get("link"),
            "child_link": child_link,
            "limit": None if limit is None else {
                key: float(text)
                for key in ("lower", "upper", "effort", "velocity")
                if (text := limit.get(key)) is not None
            },
        }

    link_parameters = {}
    for link in links:
        name = link.get("name")
        if not name:
            continue
        original = _inertial_parameters(link)
        link_parameters[name] = {
            "parent_joint": parent_joint.get(name),
            "inertial": None if original is None else {
                "urdf": original,
                "calibrated": copy.deepcopy(original),
                "scale": 1.0,
                "identification": {
                    "source": "urdf_initial",
                    "observability": 0.0,
                },
            },
        }

    missing_joints = [name for name in ALL_ARM_JOINTS
                      if name not in joint_parameters]
    if missing_joints:
        raise ValueError("URDF is missing arm joints: %s" % missing_joints)
    if "imu_in_torso" not in link_parameters:
        raise ValueError("URDF is missing imu_in_torso")

    children = {}
    for joint_name, joint in joint_parameters.items():
        children.setdefault(joint["parent_link"], []).append(joint_name)
    parameter_links = {side: [] for side in SIDES}
    parameter_owner = {}

    def visit(joint_name: str, owner: str = "") -> None:
        joint = joint_parameters[joint_name]
        if joint_name in ALL_ARM_JOINTS:
            owner = joint_name
        if not owner:
            raise ValueError("arm subtree joint %s has no owner" % joint_name)
        child_link = joint["child_link"]
        inertial = link_parameters[child_link]["inertial"]
        if inertial is not None and inertial["urdf"]["mass"] > 0.0:
            side = "left" if owner.startswith("left_") else "right"
            parameter_links[side].append(child_link)
            parameter_owner[child_link] = owner
        for child_joint in children.get(child_link, []):
            visit(child_joint, owner)

    for side in SIDES:
        visit(side + "_shoulder_pitch_joint")

    return {
        "schema_version": SCHEMA_VERSION,
        "created_at": utc_now(),
        "updated_at": utc_now(),
        "source_urdf": {
            "path": str(Path(source_path).expanduser().resolve()),
            "sha256": hashlib.sha256(urdf_xml.encode("utf-8")).hexdigest(),
            "robot_name": root.get("name", ""),
        },
        "model_scope": {
            "reference_link": "torso_link",
            "imu_link": "imu_in_torso",
            "controlled_joints": list(ALL_ARM_JOINTS),
            "parameter_links": parameter_links,
            "parameter_owner": parameter_owner,
        },
        "links": link_parameters,
        "joints": joint_parameters,
        "calibration": {
            "selected_joints": [],
            "joint_torque_bias": {
                name: 0.0 for name in ALL_ARM_JOINTS
            },
            "targets": [],
            "iterations": [],
            "active_run": None,
        },
    }


def load_parameter_document(path: str) -> dict:
    with open(Path(path).expanduser(), "r", encoding="utf-8") as stream:
        document = json.load(stream)
    if document.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(
            "unsupported parameter schema version %r"
            % document.get("schema_version"))
    for key in ("source_urdf", "links", "joints", "calibration"):
        if key not in document:
            raise ValueError("parameter file is missing %s" % key)
    return document


def atomic_write(path: str, payload: bytes) -> str:
    """Replace ``path`` with ``payload`` only after it is fully on disk."""
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_path = tempfile.mkstemp(
        prefix=destination.name + ".", suffix=".tmp", dir=str(destination.parent))
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, destination)
    except Exception:
        try:
            os.unlink(temporary_path)
        except FileNotFoundError:
            pass
        raise
    return str(destination)


def atomic_write_parameter_document(path: str, document: Mapping) -> None:
    serializable = copy.deepcopy(dict(document))
    serializable["updated_at"] = utc_now()
    atomic_write(path, (json.dumps(
        serializable, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))


class ParameterStore:
    """Own one parameter file and persist every workflow transition."""

    def __init__(self, path: str) -> None:
        self.path = str(Path(path).expanduser().resolve())

    @property
    def exists(self) -> bool:
        return Path(self.path).is_file()

    def initialize(self, urdf_path: str, *, force: bool = False) -> dict:
        if self.exists and not force:
            document = self.load()
            source = Path(urdf_path).expanduser().resolve()
            with open(source, "r", encoding="utf-8") as stream:
                digest = hashlib.sha256(stream.read().encode("utf-8")).hexdigest()
            stored_source = document["source_urdf"]
            if (Path(stored_source["path"]).resolve() != source or
                    stored_source["sha256"] != digest):
                raise ValueError(
                    "existing parameter file belongs to a different URDF; "
                    "back it up and reinitialize explicitly")
            return self._backfill_target_sides(document)
        source = Path(urdf_path).expanduser().resolve()
        with open(source, "r", encoding="utf-8") as stream:
            document = create_parameter_document(stream.read(), str(source))
        self.save(document)
        return self.load()

    def _backfill_target_sides(self, document: dict) -> dict:
        """Tag targets captured before the capture side was recorded."""
        targets = [target for target in document["calibration"]["targets"]
                   if "side" not in target]
        if not targets:
            return document
        selected = document["calibration"]["selected_joints"]
        sides = {side for side in SIDES
                 if any(name in ARM_JOINTS[side] for name in selected)}
        inferred = sides.pop() if len(sides) == 1 else "both"
        for target in targets:
            target["side"] = inferred
        self.save(document)
        return self.load()

    def load(self) -> dict:
        return load_parameter_document(self.path)

    def save(self, document: Mapping) -> None:
        atomic_write_parameter_document(self.path, document)

    def set_selected_joints(self, joint_names: Sequence[str]) -> dict:
        selected = list(dict.fromkeys(joint_names))
        invalid = [name for name in selected if name not in ALL_ARM_JOINTS]
        if invalid:
            raise ValueError("unsupported arm joints: %s" % invalid)
        document = self.load()
        document["calibration"]["selected_joints"] = selected
        self.save(document)
        return self.load()

    def append_target(self, positions: Mapping[str, float],
                      *, source: str, side: str) -> dict:
        # A missing joint becomes NaN so it fails the same check as a bad value
        # instead of escaping as a KeyError from the comprehension.
        values = {name: float(positions.get(name, np.nan))
                  for name in ALL_ARM_JOINTS}
        if not np.all(np.isfinite(list(values.values()))):
            raise ValueError("target must contain 14 finite arm positions")
        if side not in CAPTURE_SIDES:
            raise ValueError("capture side must be one of %s" % (CAPTURE_SIDES,))
        document = self.load()
        targets = document["calibration"]["targets"]
        target = {
            "id": 1 + max((int(item["id"]) for item in targets), default=0),
            "captured_at": utc_now(),
            "source": source,
            "side": side,
            "positions": values,
        }
        targets.append(target)
        self.save(document)
        return target

    def target_positions(self, target: Mapping, side: str) -> np.ndarray:
        """Return the seven joint targets ``side`` should track for ``target``.

        A pose hand-guided on one arm is reused on the other by mirroring it,
        which is exact because the two arms are mirror images in the URDF.
        """
        if side not in SIDES:
            raise ValueError("side must be 'left' or 'right'")
        captured = target.get("side", "both")
        source = side if captured not in SIDES else captured
        values = [target["positions"][name] for name in ARM_JOINTS[source]]
        if source != side:
            values = mirror_arm_values(values)
        return np.asarray(values, dtype=float)

    def remove_target(self, target_id: int) -> bool:
        document = self.load()
        targets = document["calibration"]["targets"]
        remaining = [item for item in targets if int(item["id"]) != int(target_id)]
        if len(remaining) == len(targets):
            return False
        document["calibration"]["targets"] = remaining
        self.save(document)
        return True

    def apply_link_estimate(
        self,
        side: str,
        parameter_links: Sequence[str],
        mass_scales: ArrayLike,
        torque_bias: ArrayLike,
        scale_observability: ArrayLike,
        bias_observability: ArrayLike,
        iteration: Mapping,
    ) -> dict:
        if side not in SIDES:
            raise ValueError("side must be 'left' or 'right'")
        links = tuple(parameter_links)
        arrays = [np.asarray(value, dtype=float) for value in (
            mass_scales, torque_bias, scale_observability, bias_observability)]
        scales, biases = arrays[0], arrays[1]
        shapes = ((len(links),), (7,), (len(links),), (7,))
        if (any(array.shape != shape for array, shape in zip(arrays, shapes)) or
                not all(np.all(np.isfinite(array)) for array in arrays) or
                np.any(scales <= 0.0)):
            raise ValueError("link estimates have invalid dimensions or values")

        document = self.load()
        expected_links = tuple(document["model_scope"]["parameter_links"][side])
        if links != expected_links:
            raise ValueError("parameter links do not match the parameter file")
        for link_name, scale, observability in zip(links, scales, arrays[2]):
            inertial = document["links"][link_name]["inertial"]
            if inertial is None:
                raise ValueError("parameter link %s has no inertial" % link_name)
            original = inertial["urdf"]
            calibrated = copy.deepcopy(original)
            calibrated["mass"] = float(original["mass"] * scale)
            calibrated["inertia"] = {
                name: float(value * scale)
                for name, value in original["inertia"].items()
            }
            inertial["calibrated"] = calibrated
            inertial["scale"] = float(scale)
            inertial["identification"] = {
                "source": ("data_identified"
                           if observability >= IDENTIFIED_OBSERVABILITY
                           else "prior_distributed"),
                "observability": float(np.clip(observability, 0.0, 1.0)),
            }
        for joint_name, bias in zip(ARM_JOINTS[side], biases):
            document["calibration"]["joint_torque_bias"][joint_name] = float(bias)

        record = copy.deepcopy(dict(iteration))
        record.setdefault("timestamp", utc_now())
        record["side"] = side
        record["parameter_links"] = list(links)
        record["mass_scales"] = [float(value) for value in scales]
        record["torque_bias"] = [float(value) for value in biases]
        record["scale_observability"] = [float(value) for value in arrays[2]]
        record["bias_observability"] = [float(value) for value in arrays[3]]
        document["calibration"]["iterations"].append(record)
        self.save(document)
        return self.load()

    def link_estimate(self, side: str) -> tuple:
        if side not in SIDES:
            raise ValueError("side must be 'left' or 'right'")
        document = self.load()
        links = tuple(document["model_scope"]["parameter_links"][side])
        scales = [document["links"][link_name]["inertial"]["scale"]
                  for link_name in links]
        biases = [document["calibration"]["joint_torque_bias"][joint_name]
                  for joint_name in ARM_JOINTS[side]]
        return np.asarray(scales, dtype=float), np.asarray(biases, dtype=float)

    def mirror_link_estimate(self, source_side: str) -> dict:
        """Seed the opposite arm with the mirror of ``source_side``.

        Mass scales are dimensionless ratios of mirrored links so they carry
        over unchanged; joint torques mirror with ``MIRROR_SIGNS`` just like
        the gravity torque does. The result is only a starting estimate, the
        opposite arm still has to be measured.
        """
        scales, biases = self.link_estimate(source_side)
        document = self.load()
        links = document["model_scope"]["parameter_links"]
        observed = [document["links"][name]["inertial"]["identification"]
                    ["observability"] for name in links[source_side]]
        return self.apply_link_estimate(
            opposite_side(source_side), tuple(links[opposite_side(source_side)]),
            scales, mirror_arm_values(biases), observed, np.zeros(7),
            {"source": "mirrored_from_%s" % source_side, "sample_count": 0,
             "rank": 0, "nullity": 0, "rmse_before": 0.0, "rmse_after": 0.0})

    def export_calibrated_urdf(self, output_path: str) -> str:
        """Write calibrated inertials into a copy of the original URDF tree."""
        document = self.load()
        source_path = Path(document["source_urdf"]["path"])
        root = ET.parse(str(source_path)).getroot()
        for link in root.findall("link"):
            name = link.get("name")
            stored = document["links"].get(name)
            if stored is None or stored["inertial"] is None:
                continue
            values = stored["inertial"]["calibrated"]
            inertial = link.find("inertial")
            mass = inertial.find("mass") if inertial is not None else None
            inertia = inertial.find("inertia") if inertial is not None else None
            if mass is None or inertia is None:
                raise ValueError("source link %s has incomplete inertial" % name)
            mass.set("value", "%.17g" % values["mass"])
            for key, value in values["inertia"].items():
                inertia.set(key, "%.17g" % value)

        return atomic_write(output_path, ET.tostring(
            root, encoding="utf-8", xml_declaration=True))