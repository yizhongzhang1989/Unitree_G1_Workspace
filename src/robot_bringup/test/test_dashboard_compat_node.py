from pathlib import Path
from types import SimpleNamespace
from typing import cast

import pytest
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController

from robot_bringup.dashboard_compat_node import (
    G1RobotTestDashboard,
    _collapse_joint_tree,
    _correct_mimic_transforms,
    _filter_mimic_snapshot,
    _hidden_mimic_names,
    _is_internal_name,
    _mimic_values,
    _parse_mimic_joints,
)


GLORIA_URDF = (
    Path(__file__).parents[2]
    / "unitree_g1_description"
    / "model"
    / "Gloria-M"
    / "Gloria-M.urdf"
)
FINAL_URDF = GLORIA_URDF.parents[1] / "final.urdf"
IDENTITY = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]


class _Future:
    def __init__(self, response):
        self.response = response

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self.response


class _Client:
    def __init__(self, response):
        self.response = response
        self.requests = []

    def wait_for_service(self, timeout_sec):
        del timeout_sec
        return True

    def call_async(self, request):
        self.requests.append(request)
        return _Future(self.response)


def _state(name, state, controller_type="", interfaces=()):
    controller = ControllerState()
    controller.name = name
    controller.state = state
    controller.type = controller_type
    # Humble reports this for active AND inactive controllers, which is how the
    # bench knows a controller's joints before engaging it.
    controller.required_command_interfaces = list(interfaces)
    return controller


def test_clamps_piecewise_mimics_at_two_thirds_gripper_travel():
    specs = _parse_mimic_joints(GLORIA_URDF.read_text(encoding="utf-8"))
    theta = 2.0 * 2.76377472169236 / 3.0

    values = _mimic_values(specs, {"eccentric_joint": theta})
    left_slider = sum(
        values[spec.name] for spec in specs
        if spec.name.startswith("internal_left_slider_"))
    left_rod = sum(
        values[spec.name] for spec in specs
        if spec.name.startswith("internal_left_connecting_rod_"))

    assert len(specs) == 32
    assert left_slider == pytest.approx(-0.019143744411)
    assert left_rod == pytest.approx(-0.290796516076)


def test_rebuilds_physical_links_but_hides_internal_spline_frames():
    specs = _parse_mimic_joints(GLORIA_URDF.read_text(encoding="utf-8"))
    theta = 2.0 * 2.76377472169236 / 3.0

    transforms = _correct_mimic_transforms(
        {"gripper_base": IDENTITY}, specs, {"eccentric_joint": theta})
    visible = [name for name in transforms if not _is_internal_name(name)]
    joint_tree = [
        {"parent": spec.parent, "child": spec.child,
         "type": spec.joint_type}
        for spec in specs
    ]
    hidden = {spec.child for spec in specs if _is_internal_name(spec.child)}
    collapsed = _collapse_joint_tree(joint_tree, hidden)

    assert "left_slider" in visible
    assert "left_connecting_rod" in visible
    assert "right_slider" in visible
    assert "right_connecting_rod" in visible
    assert not any(_is_internal_name(name) for name in visible)
    assert {joint["child"] for joint in collapsed} == {
        "left_slider", "left_connecting_rod",
        "right_slider", "right_connecting_rod",
    }
    assert next(
        joint for joint in collapsed
        if joint["child"] == "left_connecting_rod")["parent"] == "left_slider"


def test_keeps_original_mimic_tf_until_source_state_arrives():
    specs = _parse_mimic_joints(GLORIA_URDF.read_text(encoding="utf-8"))
    original = {"gripper_base": IDENTITY, "left_slider": IDENTITY}

    assert _correct_mimic_transforms(original, specs, {}) == original


def test_parses_both_prefixed_grippers_from_assembled_model():
    specs = _parse_mimic_joints(FINAL_URDF.read_text(encoding="utf-8"))
    theta = 2.0 * 2.76377472169236 / 3.0
    values = _mimic_values(specs, {
        "left_eccentric_joint": theta,
        "right_eccentric_joint": theta,
    })

    assert sum(spec.name.startswith("left_internal_") for spec in specs) == 32
    assert sum(spec.name.startswith("right_internal_") for spec in specs) == 32
    assert all(spec.name in values for spec in specs)


def test_snapshot_hides_internal_model_data_after_correcting_fk():
    specs = _parse_mimic_joints(FINAL_URDF.read_text(encoding="utf-8"))
    hidden_links, hidden_joints = _hidden_mimic_names(specs)
    theta = 2.0 * 2.76377472169236 / 3.0
    link_tf = _correct_mimic_transforms(
        {"left_gripper_base": IDENTITY, "right_gripper_base": IDENTITY},
        specs, {"left_eccentric_joint": theta, "right_eccentric_joint": theta})
    state = {
        "link_tf": link_tf,
        "links": list(link_tf),
        "visuals": [{"link": link} for link in link_tf],
        "has_meshes": True,
        "joint_tree": [
            {"name": spec.name, "parent": spec.parent, "child": spec.child,
             "type": spec.joint_type}
            for spec in specs
        ],
        "movable_joints": [{"name": spec.name} for spec in specs],
        "joint_values": {spec.name: 0.0 for spec in specs},
        "joint_state_names": [spec.name for spec in specs],
    }

    filtered = _filter_mimic_snapshot(state, hidden_links, hidden_joints)

    assert "left_left_connecting_rod" in filtered["link_tf"]
    assert "right_right_connecting_rod" in filtered["link_tf"]
    assert not hidden_links.intersection(filtered["links"])
    assert not hidden_links.intersection(filtered["link_tf"])
    assert not hidden_links.intersection(
        visual["link"] for visual in filtered["visuals"])
    assert filtered["has_meshes"] is True
    assert not hidden_links.intersection(
        joint["child"] for joint in filtered["joint_tree"])
    assert not hidden_joints.intersection(
        joint["name"] for joint in filtered["movable_joints"])
    assert not hidden_joints.intersection(filtered["joint_values"])
    assert not hidden_joints.intersection(filtered["joint_state_names"])


def test_switch_uses_humble_fields_and_confirms_final_state():
    switch_response = SwitchController.Response()
    switch_response.ok = True
    listed = ListControllers.Response()
    listed.controller = [
        _state("forward_position_controller", "active"),
    ]
    switch_client = _Client(switch_response)
    dashboard = SimpleNamespace(
        _cli_switch=switch_client,
        _cli_list=_Client(listed),
        _wait=lambda future, timeout: (timeout, future.result())[1],
    )

    assert G1RobotTestDashboard._switch(
        cast(G1RobotTestDashboard, dashboard),
        ["forward_position_controller"],
        [],
    ) is True
    request = switch_client.requests[0]
    assert request.activate_controllers == ["forward_position_controller"]
    assert request.deactivate_controllers == []
    assert request.activate_asap is True


def test_switch_rejects_best_effort_partial_success():
    switch_response = SwitchController.Response()
    switch_response.ok = True
    listed = ListControllers.Response()
    listed.controller = [
        _state("forward_position_controller", "inactive"),
    ]
    dashboard = SimpleNamespace(
        _cli_switch=_Client(switch_response),
        _cli_list=_Client(listed),
        _wait=lambda future, timeout: (timeout, future.result())[1],
    )

    assert G1RobotTestDashboard._switch(
        cast(G1RobotTestDashboard, dashboard),
        ["forward_position_controller"],
        [],
    ) is False
