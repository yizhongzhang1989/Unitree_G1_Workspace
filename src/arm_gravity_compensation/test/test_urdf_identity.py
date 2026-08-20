"""The URDF identity check: what should invalidate a calibration and what should not."""

import hashlib
import json
from pathlib import Path

import pytest

from arm_gravity_compensation.constants import ARM_JOINTS, SIDES
from arm_gravity_compensation.parameter_store import (
    ParameterStore, create_parameter_document, urdf_model_digest)


def build_urdf(*, colour="0.5 0.5 0.5 1", mass="1.25", origin="0 0.1 0.2",
               axis="0 1 0", lower="-3.089", inertia="0.01", extra=""):
    """A full two-arm chain, since the store insists on all fourteen joints."""
    body = [
        '<?xml version="1.0"?>', '<robot name="test">',
        '  <link name="torso_link">',
        '    <inertial><origin xyz="0 0 0.1" rpy="0 0 0"/>',
        '      <mass value="5.0"/>',
        '      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>',
        "    </inertial>",
        '    <visual><geometry><box size="1 1 1"/></geometry>',
        '      <material name="grey"><color rgba="%s"/></material></visual>' % colour,
        "  </link>",
        '  <link name="imu_in_torso"/>',
        '  <joint name="imu_in_torso_joint" type="fixed">',
        '    <parent link="torso_link"/>',
        '    <child link="imu_in_torso"/>',
        '    <origin xyz="0 0 0.05" rpy="0 0 0"/>',
        "  </joint>",
    ]
    for side in SIDES:
        parent = "torso_link"
        for index, joint in enumerate(ARM_JOINTS[side]):
            child = joint.replace("_joint", "_link")
            body += [
                '  <link name="%s">' % child,
                '    <inertial><origin xyz="0 0 -0.05" rpy="0 0 0"/>',
                '      <mass value="%s"/>' % mass,
                '      <inertia ixx="%s" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>'
                % inertia,
                "    </inertial>",
                "  </link>",
                '  <joint name="%s" type="revolute">' % joint,
                '    <parent link="%s"/>' % parent,
                '    <child link="%s"/>' % child,
                '    <origin xyz="%s" rpy="0 0 0"/>' % (
                    origin if index == 0 else "0 0 -0.1"),
                '    <axis xyz="%s"/>' % (axis if index == 0 else "1 0 0"),
                '    <limit effort="25" lower="%s" upper="2.670" velocity="37"/>'
                % (lower if index == 0 else "-2.0"),
                "  </joint>",
            ]
            parent = child
    return "\n".join(body + [extra, "</robot>", ""])


BASE = build_urdf()


def test_recolouring_does_not_change_the_model():
    recoloured = build_urdf(colour="1 0 0 1")
    assert recoloured != BASE

    assert urdf_model_digest(recoloured) == urdf_model_digest(BASE)


def test_adding_visuals_does_not_change_the_model():
    decorated = build_urdf(
        extra='  <link name="decor"><visual><geometry>'
              '<sphere radius="0.1"/></geometry></visual></link>')

    assert urdf_model_digest(decorated) == urdf_model_digest(BASE)


def test_number_formatting_does_not_change_the_model():
    assert urdf_model_digest(build_urdf(origin="0.0 0.10 0.200")) == \
        urdf_model_digest(BASE)


@pytest.mark.parametrize("kwargs", [
    {"mass": "1.40"},          # 换了个更重的连杆
    {"origin": "0 0.1 0.25"},  # 关节挪位置
    {"axis": "0 0 1"},         # 转轴换向
    {"lower": "-2.500"},       # 行程改了
    {"inertia": "0.02"},       # 惯量改了
])
def test_anything_that_moves_mass_or_geometry_does_change_it(kwargs):
    changed = build_urdf(**kwargs)
    assert changed != BASE

    assert urdf_model_digest(changed) != urdf_model_digest(BASE)


def prepare(tmp_path, document=None, text=BASE):
    source = tmp_path / "robot.urdf"
    source.write_text(text, encoding="utf-8")
    path = tmp_path / "parameters.json"
    path.write_text(
        json.dumps(document or create_parameter_document(text, str(source)),
                   indent=2),
        encoding="utf-8")
    return ParameterStore(str(path)), source


def test_a_recoloured_urdf_opens_without_a_rebind(tmp_path):
    store, source = prepare(tmp_path)
    source.write_text(build_urdf(colour="0 1 0 1"), encoding="utf-8")

    opened = store.initialize(str(source))

    assert opened["source_urdf"]["sha256"] == urdf_model_digest(BASE)


def test_a_changed_mass_still_refuses(tmp_path):
    store, source = prepare(tmp_path)
    source.write_text(build_urdf(mass="9.0"), encoding="utf-8")

    with pytest.raises(ValueError, match="different URDF"):
        store.initialize(str(source))


def test_a_legacy_whole_file_digest_upgrades_itself(tmp_path):
    document = create_parameter_document(BASE, "unused")
    # 老格式：整份文件的摘要，没有 digest_kind。
    document["source_urdf"].pop("digest_kind")
    document["source_urdf"]["sha256"] = hashlib.sha256(
        BASE.encode("utf-8")).hexdigest()
    store, source = prepare(tmp_path, document)

    opened = store.initialize(str(source))

    assert opened["source_urdf"]["digest_kind"] == "model"
    assert opened["source_urdf"]["sha256"] == urdf_model_digest(BASE)


def test_rebind_adopts_the_urdf_and_keeps_the_captured_poses(tmp_path):
    document = create_parameter_document(BASE, "unused")
    document["calibration"]["targets"] = [{"id": 7, "side": "left"}]
    document["model_scope"]["reference_link"] = "hand_tuned"
    store, source = prepare(tmp_path, document)
    heavier = build_urdf(mass="9.0")
    source.write_text(heavier, encoding="utf-8")

    opened = store.initialize(str(source), rebind=True)

    assert opened["source_urdf"]["sha256"] == urdf_model_digest(heavier)
    # 重绑只换身份：手工配置和采样点都不该跟着丢。
    assert opened["calibration"]["targets"] == [{"id": 7, "side": "left"}]
    assert opened["model_scope"]["reference_link"] == "hand_tuned"


def test_moving_the_file_is_not_a_new_model(tmp_path):
    store, _ = prepare(tmp_path)
    moved = tmp_path / "elsewhere.urdf"
    moved.write_text(BASE, encoding="utf-8")

    opened = store.initialize(str(moved))

    assert Path(opened["source_urdf"]["path"]) == moved
