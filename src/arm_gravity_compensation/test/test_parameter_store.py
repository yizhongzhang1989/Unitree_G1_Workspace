import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from arm_gravity_compensation.constants import ARM_JOINTS
from arm_gravity_compensation.parameter_store import ParameterStore


URDF = (Path(__file__).parents[2] / "unitree_g1_description" / "model" /
        "g1_description" / "g1_29dof_mode_15.urdf")


def test_extracts_every_link_and_keeps_original_inertial(tmp_path):
    store = ParameterStore(str(tmp_path / "parameters.json"))
    document = store.initialize(str(URDF))

    assert len(document["links"]) > 30
    assert "imu_in_torso" in document["links"]
    shoulder = document["links"]["left_shoulder_pitch_link"]["inertial"]
    assert shoulder["urdf"]["mass"] == 0.718
    assert shoulder["calibrated"] == shoulder["urdf"]
    assert shoulder["scale"] == 1.0


def test_iteration_updates_calibrated_values_atomically(tmp_path):
    path = tmp_path / "parameters.json"
    store = ParameterStore(str(path))
    original = store.initialize(str(URDF))
    original_mass = original["links"]["left_shoulder_pitch_link"][
        "inertial"]["urdf"]["mass"]
    scales = np.linspace(0.8, 1.1, 7)
    biases = np.linspace(-0.2, 0.2, 7)
    links = original["model_scope"]["parameter_links"]["left"]

    updated = store.apply_link_estimate(
        "left", links, scales, biases, np.ones(7), np.ones(7),
        {"target_id": 3, "rmse": 0.04})

    shoulder = updated["links"]["left_shoulder_pitch_link"]["inertial"]
    assert shoulder["urdf"]["mass"] == original_mass
    assert shoulder["calibrated"]["mass"] == original_mass * scales[0]
    assert len(updated["calibration"]["iterations"]) == 1
    parsed = json.loads(path.read_text())
    assert parsed["calibration"]["joint_torque_bias"][
        ARM_JOINTS["left"][0]] == biases[0]


def test_existing_file_is_not_overwritten_without_force(tmp_path):
    store = ParameterStore(str(tmp_path / "parameters.json"))
    document = store.initialize(str(URDF))
    links = document["model_scope"]["parameter_links"]["right"]
    store.apply_link_estimate(
        "right", links, np.full(7, 1.2), np.zeros(7),
        np.ones(7), np.ones(7), {"target_id": 1})

    reopened = store.initialize(str(URDF))

    assert reopened["links"]["right_shoulder_pitch_link"][
        "inertial"]["scale"] == 1.2


def test_existing_file_rejects_changed_source_urdf(tmp_path):
    source = tmp_path / "model.urdf"
    source.write_text(URDF.read_text())
    store = ParameterStore(str(tmp_path / "parameters.json"))
    store.initialize(str(source))
    source.write_text(source.read_text().replace(
        '<mass value="0.718"/>', '<mass value="0.719"/>', 1))

    try:
        store.initialize(str(source))
    except ValueError as error:
        assert "different URDF" in str(error)
    else:
        raise AssertionError("changed source URDF reused stale parameters")


def test_final_urdf_groups_payload_and_exports_same_tree(tmp_path):
    final_urdf = (Path(__file__).parents[2] / "unitree_g1_description" /
                  "model" / "final.urdf")
    store = ParameterStore(str(tmp_path / "parameters.json"))
    document = store.initialize(str(final_urdf))
    links = document["model_scope"]["parameter_links"]["left"]
    owners = document["model_scope"]["parameter_owner"]
    wrist_links = [name for name in links
                   if owners[name] == "left_wrist_yaw_joint"]
    scales = np.ones(len(links))
    scales[links.index("left_kwr57b_link")] = 1.1
    observability = np.ones(len(links))
    observability[links.index("left_kwr57b_link")] = 0.5

    updated = store.apply_link_estimate(
        "left", links, scales, np.zeros(7), observability,
        np.ones(7), {"rank": 4, "nullity": 3})
    output = Path(store.export_calibrated_urdf(
        str(tmp_path / "calibrated.urdf")))
    source_root = ET.parse(str(final_urdf)).getroot()
    output_root = ET.parse(str(output)).getroot()
    source_links = {link.get("name"): link
                    for link in source_root.findall("link")}
    output_links = {link.get("name"): link
                    for link in output_root.findall("link")}

    assert len(wrist_links) == 8
    assert len(output_root.findall("link")) == len(source_root.findall("link"))
    assert len(output_root.findall("joint")) == len(source_root.findall("joint"))
    assert abs(float(output_links["left_kwr57b_link"].find(
        "inertial/mass").get("value")) - 0.286) < 1e-12
    source_inertial = source_links["left_kwr57b_link"].find("inertial")
    output_inertial = output_links["left_kwr57b_link"].find("inertial")
    assert output_inertial.find("origin").get("xyz") == "0 0 0.0265"
    for name in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
        source_value = float(source_inertial.find("inertia").get(name))
        output_value = float(output_inertial.find("inertia").get(name))
        assert abs(output_value - source_value * 1.1) < 1e-15
    identification = updated["links"]["left_kwr57b_link"]["inertial"][
        "identification"]
    assert identification["source"] == "prior_distributed"
    assert identification["observability"] == 0.5