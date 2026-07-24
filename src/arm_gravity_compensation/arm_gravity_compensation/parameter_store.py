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

from .constants import ALL_ARM_JOINTS, ARM_JOINTS, SIDES


SCHEMA_VERSION = 2


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _vector(element: Optional[ET.Element], attribute: str,
            default: str) -> list:
    text = element.get(attribute, default) if element is not None else default
    values = [float(value) for value in text.split()]
    if len(values) != 3:
        raise ValueError("%s must contain three values" % attribute)
    return values


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
        "mass": float(mass.get("value")),
        "inertia": {
            name: float(inertia.get(name))
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
                key: float(limit.get(key))
                for key in ("lower", "upper", "effort", "velocity")
                if limit.get(key) is not None
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


def atomic_write_parameter_document(path: str, document: Mapping) -> None:
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    serializable = copy.deepcopy(dict(document))
    serializable["updated_at"] = utc_now()
    descriptor, temporary_path = tempfile.mkstemp(
        prefix=destination.name + ".", suffix=".tmp",
        dir=str(destination.parent), text=True)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(serializable, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, destination)
    except Exception:
        try:
            os.unlink(temporary_path)
        except FileNotFoundError:
            pass
        raise


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
            return document
        source = Path(urdf_path).expanduser().resolve()
        with open(source, "r", encoding="utf-8") as stream:
            document = create_parameter_document(stream.read(), str(source))
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
                      *, source: str) -> dict:
        missing = [name for name in ALL_ARM_JOINTS if name not in positions]
        values = {name: float(positions[name]) for name in ALL_ARM_JOINTS}
        if missing or not np.all(np.isfinite(list(values.values()))):
            raise ValueError("target must contain 14 finite arm positions")
        document = self.load()
        targets = document["calibration"]["targets"]
        target = {
            "id": 1 + max((int(item["id"]) for item in targets), default=0),
            "captured_at": utc_now(),
            "source": source,
            "positions": values,
        }
        targets.append(target)
        self.save(document)
        return target

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
        mass_scales: Sequence[float],
        torque_bias: Sequence[float],
        scale_observability: Sequence[float],
        bias_observability: Sequence[float],
        iteration: Mapping,
    ) -> dict:
        if side not in SIDES:
            raise ValueError("side must be 'left' or 'right'")
        scales = np.asarray(mass_scales, dtype=float)
        biases = np.asarray(torque_bias, dtype=float)
        scale_observability_array = np.asarray(scale_observability, dtype=float)
        bias_observability_array = np.asarray(bias_observability, dtype=float)
        links = tuple(parameter_links)
        if (scales.shape != (len(links),) or biases.shape != (7,) or
                scale_observability_array.shape != scales.shape or
                bias_observability_array.shape != biases.shape or
                not np.all(np.isfinite(scales)) or
                not np.all(np.isfinite(biases)) or
                not np.all(np.isfinite(scale_observability_array)) or
                not np.all(np.isfinite(bias_observability_array)) or
                np.any(scales <= 0.0)):
            raise ValueError("link estimates have invalid dimensions or values")

        document = self.load()
        expected_links = tuple(document["model_scope"]["parameter_links"][side])
        if links != expected_links:
            raise ValueError("parameter links do not match the parameter file")
        for link_name, scale, observability in zip(
                links, scales, scale_observability_array):
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
                "source": ("data_identified" if observability >= 1.0 - 1e-6
                           else "prior_distributed"),
                "observability": float(np.clip(observability, 0.0, 1.0)),
            }
        for joint_name, bias, observability in zip(
                ARM_JOINTS[side], biases, bias_observability_array):
            document["calibration"]["joint_torque_bias"][joint_name] = float(bias)

        record = copy.deepcopy(dict(iteration))
        record.setdefault("timestamp", utc_now())
        record["side"] = side
        record["parameter_links"] = list(links)
        record["mass_scales"] = [float(value) for value in scales]
        record["torque_bias"] = [float(value) for value in biases]
        record["scale_observability"] = [
            float(value) for value in scale_observability_array]
        record["bias_observability"] = [
            float(value) for value in bias_observability_array]
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

        destination = Path(output_path).expanduser().resolve()
        destination.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_path = tempfile.mkstemp(
            prefix=destination.name + ".", suffix=".tmp",
            dir=str(destination.parent), text=True)
        try:
            with os.fdopen(descriptor, "wb") as stream:
                ET.ElementTree(root).write(
                    stream, encoding="utf-8", xml_declaration=True)
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