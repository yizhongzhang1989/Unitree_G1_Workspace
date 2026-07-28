import threading
from pathlib import Path
from types import SimpleNamespace
from typing import Any, cast

import pytest
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController

from robot_bringup.dashboard_compat_node import (
    G1RobotTestDashboard,
    RobotTestDashboard,
    _collapse_joint_tree,
    _correct_mimic_transforms,
    _is_internal_name,
    _mimic_values,
    _parameter_service,
    _parse_mimic_joints,
)


GRAVITY_TYPE = "unitree_g1_controllers/ArmGravityCompensation"
FPC_TYPE = "unitree_g1_forward_command_controller/ForwardCommandController"


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


def _gravity_dashboard(gravity_state="inactive", fpc_state="inactive"):
    dashboard = object.__new__(G1RobotTestDashboard)
    dashboard._lock = threading.Lock()
    dashboard._cm_ns = "/controller_manager"
    dashboard._cmd_pubs = {}
    dashboard._controllers = [
        {"name": "arm_gravity_compensation", "state": gravity_state,
         "type": GRAVITY_TYPE, "kind": "forward_position",
         "cmd_ifaces": [], "joints": ["joint_a"]},
        {"name": "forward_position_controller", "state": fpc_state,
         "type": FPC_TYPE, "kind": "forward_position",
         "cmd_ifaces": ["joint_a/position"], "joints": ["joint_a"]},
    ]
    return dashboard


def test_parameter_service_follows_manager_namespace():
    assert _parameter_service(
        "/robot/controller_manager", "forward_position_controller") == \
        "/robot/forward_position_controller/get_parameters"


def test_gravity_filter_is_offered_as_a_forward_position_controller():
    listed = ListControllers.Response()
    listed.controller = [
        _state("arm_gravity_compensation", "inactive", GRAVITY_TYPE),
        _state("forward_position_controller", "active", FPC_TYPE, ["joint_a/position"]),
    ]
    dashboard = object.__new__(G1RobotTestDashboard)
    dashboard._lock = threading.Lock()
    dashboard._joint_meta_lock = threading.Lock()
    dashboard._controllers = []
    dashboard._cm_ok = False
    # The gravity filter claims nothing, so ``required_command_interfaces`` is
    # empty and its joint list can only come from its own ``joints`` parameter.
    dashboard._joint_cache = {"arm_gravity_compensation": ["joint_a"]}

    dashboard._on_controllers(_Future(listed))

    gravity, fpc = dashboard._controllers
    assert gravity["kind"] == "forward_position"
    assert gravity["joints"] == ["joint_a"]
    assert gravity["cmd_ifaces"] == []
    assert fpc["cmd_ifaces"] == ["joint_a/position"]


def test_gravity_setpoints_go_to_the_filter_target_topic():
    dashboard = _gravity_dashboard()
    created = []
    cast(Any, dashboard).create_publisher = (
        lambda msg_type, topic, depth: created.append(topic) or "publisher")

    assert dashboard._cmd_pub("arm_gravity_compensation") == "publisher"
    assert created == ["/arm_gravity_compensation/target"]


def test_engaging_the_gravity_filter_brings_up_the_controller_it_feeds(
        monkeypatch):
    dashboard = _gravity_dashboard()
    activated = []
    monkeypatch.setattr(
        G1RobotTestDashboard, "_command_topic",
        lambda self, name: "/forward_position_controller/commands")
    monkeypatch.setattr(
        G1RobotTestDashboard, "_activate_exclusive",
        lambda self, info: bool(activated.append(info["name"])) or True)
    monkeypatch.setattr(
        RobotTestDashboard, "engage", lambda self, name: {"ok": True})

    assert dashboard.engage("arm_gravity_compensation") == {"ok": True}
    assert activated == ["forward_position_controller"]


def test_engaging_the_gravity_filter_needs_its_command_topic(monkeypatch):
    dashboard = _gravity_dashboard()
    monkeypatch.setattr(
        G1RobotTestDashboard, "_command_topic", lambda self, name: "")

    assert dashboard.engage("arm_gravity_compensation")["ok"] is False


def test_driving_the_fed_controller_releases_the_gravity_filter(monkeypatch):
    dashboard = _gravity_dashboard(gravity_state="active", fpc_state="active")
    switched = []
    monkeypatch.setattr(
        G1RobotTestDashboard, "_command_topic",
        lambda self, name: "/forward_position_controller/commands")
    monkeypatch.setattr(
        G1RobotTestDashboard, "_switch",
        lambda self, activate, deactivate, timeout=0.0:
            bool(switched.append((activate, deactivate))) or True)
    monkeypatch.setattr(
        RobotTestDashboard, "engage", lambda self, name: {"ok": True})

    assert dashboard.engage("forward_position_controller") == {"ok": True}
    assert switched == [([], ["arm_gravity_compensation"])]


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


def test_switch_uses_humble_fields_and_confirms_final_state():
    switch_response = SwitchController.Response()
    switch_response.ok = True
    listed = ListControllers.Response()
    listed.controller = [
        _state("forward_position_controller", "active"),
        _state("joint_trajectory_controller", "inactive"),
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
        ["joint_trajectory_controller"],
    ) is True
    request = switch_client.requests[0]
    assert request.activate_controllers == ["forward_position_controller"]
    assert request.deactivate_controllers == ["joint_trajectory_controller"]
    assert request.activate_asap is True


def test_switch_rejects_best_effort_partial_success():
    switch_response = SwitchController.Response()
    switch_response.ok = True
    listed = ListControllers.Response()
    listed.controller = [
        _state("forward_position_controller", "inactive"),
        _state("joint_trajectory_controller", "inactive"),
    ]
    dashboard = SimpleNamespace(
        _cli_switch=_Client(switch_response),
        _cli_list=_Client(listed),
        _wait=lambda future, timeout: (timeout, future.result())[1],
    )

    assert G1RobotTestDashboard._switch(
        cast(G1RobotTestDashboard, dashboard),
        ["forward_position_controller"],
        ["joint_trajectory_controller"],
    ) is False
