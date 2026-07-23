import importlib.util
from pathlib import Path
from typing import Any, Dict, List, cast

import pytest
import yaml
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.utilities import (
    normalize_to_list_of_substitutions,
    perform_substitutions,
)
from launch_ros.actions import Node


LAUNCH_DIR = Path(__file__).parents[1] / "launch"
WORKSPACE_SRC = Path(__file__).parents[2]


def _load_launch(name, launch_dir=LAUNCH_DIR):
    path = launch_dir / name
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load launch module: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _all_data_context(scope, topology="dual"):
    context = LaunchContext()
    context.launch_configurations.update({
        "scope": scope,
        "topology": topology,
        "enable_grippers_on_start": "true",
        "controller_manager": "/controller_manager",
        "lowstate_topic": "/lowstate",
        "arm_stiffness_scale": "2.5",
        "joint_states_topic": "/joint_states",
        "robot_description_topic": "/robot_description",
        "require_pr_mode": "true",
        "use_sim_time": "false",
    })
    return context


def _source_path(include):
    context = LaunchContext()
    include.launch_description_source.get_launch_description(context)
    return include.launch_description_source.location


def _perform(context, value):
    return perform_substitutions(
        context, normalize_to_list_of_substitutions(value))


def _node_package(node: Node) -> str:
    return cast(str, getattr(node, "_Node__package"))


def _node_executable(node: Node) -> str:
    return cast(str, getattr(node, "_Node__node_executable"))


def _node_parameters(node: Node) -> List[Dict[str, Any]]:
    return cast(List[Dict[str, Any]], getattr(node, "_Node__parameters"))


parametrize = cast(Any, pytest.mark.parametrize)


def test_all_data_scope_selects_expected_producers():
    module = _load_launch("all_data.launch.py")

    end_effectors = module._data_launches(
        _all_data_context("end_effectors", "single"))
    assert len(end_effectors) == 1
    assert all(isinstance(action, IncludeLaunchDescription)
               for action in end_effectors)
    assert "end_effectors_single_bus.launch.py" in _source_path(
        end_effectors[0])
    assert dict(end_effectors[0].launch_arguments)[
        "enable_grippers_on_start"].perform(
            _all_data_context("end_effectors", "single")) == "true"

    whole_body = module._data_launches(
        _all_data_context("whole_body", "dual"))
    assert len(whole_body) == 2
    assert all(isinstance(action, IncludeLaunchDescription)
               for action in whole_body)
    paths = [_source_path(action) for action in whole_body]
    assert any("end_effectors_dual_bus.launch.py" in path for path in paths)
    assert any("control.launch.py" in path for path in paths)
    assert all("dashboard" not in path.lower() for path in paths)
    control = next(
        action for action in whole_body
        if "control.launch.py" in _source_path(action))
    assert dict(control.launch_arguments)["arm_stiffness_scale"].perform(
        _all_data_context("whole_body", "dual")) == "2.5"


@parametrize("scope", ["bad", "", "all"])
def test_all_data_rejects_unknown_scope(scope):
    module = _load_launch("all_data.launch.py")
    try:
        module._data_launches(_all_data_context(scope))
    except ValueError as exc:
        assert "scope must be" in str(exc)
    else:
        pytest.fail("unknown scope was accepted")


@parametrize("topology", ["bad", "", "two"])
def test_all_data_rejects_unknown_topology(topology):
    module = _load_launch("all_data.launch.py")
    try:
        module._data_launches(_all_data_context("end_effectors", topology))
    except ValueError as exc:
        assert "topology must be" in str(exc)
    else:
        pytest.fail("unknown topology was accepted")


def test_end_effectors_dashboard_expands_to_one_web_node():
    module = _load_launch("end_effectors_dashboard.launch.py")
    context = LaunchContext()
    context.launch_configurations.update({
        "topology": "dual",
        "web_host": "0.0.0.0",
        "web_port": "8770",
        "request_timeout_s": "3.0",
        "state_stale_s": "1.0",
        "left_camera_url": "http://127.0.0.1:8010",
        "right_camera_url": "http://127.0.0.1:8011",
        "camera_timeout_s": "1.0",
        "camera_poll_period_s": "2.0",
    })
    actions = module._dashboard_node(context)
    assert len(actions) == 1
    assert isinstance(actions[0], Node)
    assert _node_package(actions[0]) == "robot_bringup"
    assert _node_executable(actions[0]) == "end_effectors_dashboard"


def test_whole_body_dashboard_only_creates_compat_web_node():
    module = _load_launch("whole_body_dashboard.launch.py")
    description = module.generate_launch_description()
    includes = [
        entity for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]
    assert includes == []
    nodes = [
        entity for entity in description.entities
        if isinstance(entity, Node)
    ]
    assert len(nodes) == 1
    assert _node_package(nodes[0]) == "robot_bringup"
    assert _node_executable(nodes[0]) == "whole_body_dashboard"


def test_ikt_pose_commander_uses_named_position_controllers():
    module = _load_launch("ikt_pose_commander.launch.py")
    description = module.generate_launch_description()
    nodes = [
        entity for entity in description.entities
        if isinstance(entity, Node)
    ]
    assert len(nodes) == 2
    by_executable = {
        _node_executable(node): node
        for node in nodes
    }
    commander = by_executable["ikt_pose_commander"]
    assert _node_package(commander) == "robot_bringup"
    assert _node_executable(commander) == "ikt_pose_commander"
    context = LaunchContext()
    context.launch_configurations["max_iters"] = module._DEFAULTS["max_iters"]
    parameters = {
        _perform(context, name): value
        for name, value in _node_parameters(commander)[0].items()
    }
    assert yaml.safe_load(_perform(context, parameters["command_mode"])) == "fpc"
    assert yaml.safe_load(_perform(context, parameters["fpc_controller"])) == \
        "forward_position_controller"
    assert yaml.safe_load(_perform(context, parameters["jtc_controller"])) == \
        "joint_trajectory_controller"
    assert yaml.safe_load(_perform(context, parameters["max_iters"])) == 20
    dashboard = by_executable["ikt_pose_commander_dashboard"]
    assert _node_package(dashboard) == "robot_bringup"


@parametrize(
    "launch_dir,name,package,executable",
    [
        (WORKSPACE_SRC / "kwr57_ros" / "launch",
         "web_demo.launch.py", "kwr57_ros", "web_wrench"),
        (WORKSPACE_SRC / "gloria_ros" / "launch",
         "web_gripper.launch.py", "gloria_ros", "web_gripper"),
    ],
)
def test_single_device_web_launches_only_create_web_node(
        launch_dir, name, package, executable):
    module = _load_launch(name, launch_dir)
    description = module.generate_launch_description()
    nodes = [
        entity for entity in description.entities
        if isinstance(entity, Node)
    ]
    includes = [
        entity for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]
    assert includes == []
    assert len(nodes) == 1
    assert _node_package(nodes[0]) == package
    assert _node_executable(nodes[0]) == executable


@parametrize(
    "launch_dir,name,package,executable",
    [
        (WORKSPACE_SRC / "kwr57_ros" / "launch",
         "ft_sensor.launch.py", "kwr57_ros", "ft_sensor_node"),
        (WORKSPACE_SRC / "gloria_ros" / "launch",
         "gripper.launch.py", "gloria_ros", "gripper_node"),
    ],
)
def test_device_launches_only_create_device_node(
        launch_dir, name, package, executable):
    module = _load_launch(name, launch_dir)
    description = module.generate_launch_description()
    nodes = [
        entity for entity in description.entities
        if isinstance(entity, Node)
    ]
    includes = [
        entity for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]
    assert includes == []
    assert len(nodes) == 1
    assert _node_package(nodes[0]) == package
    assert _node_executable(nodes[0]) == executable


def test_gripper_debug_owns_bridge_and_reuses_device_launch():
    module = _load_launch(
        "gripper_debug.launch.py", WORKSPACE_SRC / "gloria_ros" / "launch")
    context = LaunchContext()
    context.launch_configurations.update({
        "node_name": "grip_left",
        "command_id": "0x01",
        "feedback_id": "0x101",
        "safe_position_min": "0.0",
        "safe_position_max": "2.77",
    })
    actions = module._launch_nodes(context)
    assert len(actions) == 2
    assert isinstance(actions[0], Node)
    assert _node_package(actions[0]) == "can_bridge_ros"
    assert isinstance(actions[1], IncludeLaunchDescription)
    assert "gripper.launch.py" in _source_path(actions[1])
    launch_arguments = dict(actions[1].launch_arguments)
    assert launch_arguments == {
        "rx_topic": "/can0/grip_left/rx",
        "tx_topic": "/can0/tx",
        "command_id": "1",
        "feedback_id": "257",
        "safe_position_min": launch_arguments["safe_position_min"],
        "safe_position_max": launch_arguments["safe_position_max"],
        "enable_on_start": "false",
        "diagnostic_period_s": "1.0",
        "joint_name": "grip_left",
        "node_name": "grip_left",
    }
    assert _perform(context, launch_arguments["safe_position_min"]) == "0.0"
    assert _perform(context, launch_arguments["safe_position_max"]) == "2.77"


@parametrize("use_frame_handler,action_count", [
    ("true", 1),
    ("false", 2),
])
def test_ft_sensor_debug_owns_bridge_and_reuses_fallback_launch(
        use_frame_handler, action_count):
    module = _load_launch(
        "ft_sensor_debug.launch.py", WORKSPACE_SRC / "kwr57_ros" / "launch")
    context = LaunchContext()
    context.launch_configurations.update({
        "bridge_config": "single_bus.yaml",
        "use_frame_handler": use_frame_handler,
        "channel_id": "0",
        "bus_name": "can0",
        "cmd_id": "16",
        "data_base_id": "21",
        "topic": "/kwr57_ft_sensor/wrench_raw",
        "frame_id": "kwr57_ft_sensor_link",
        "period_ms": "1",
        "sample_rate_hz": "1000",
        "publish_rate": "0.0",
        "use_si": "false",
        "tare_on_start": "false",
    })
    actions = module._launch_nodes(context)
    assert len(actions) == action_count
    assert isinstance(actions[0], Node)
    assert _node_package(actions[0]) == "can_bridge_ros"
    if use_frame_handler == "false":
        assert isinstance(actions[1], IncludeLaunchDescription)
        assert "ft_sensor.launch.py" in _source_path(actions[1])
        assert dict(actions[1].launch_arguments) == {
            "rx_topic": "/can0/kwr57/rx",
            "tx_topic": "/can0/tx",
            "cmd_id": "16",
            "data_base_id": "21",
            "topic": "/kwr57_ft_sensor/wrench_raw",
            "frame_id": "kwr57_ft_sensor_link",
            "period_ms": "1",
            "sample_rate_hz": "1000",
            "publish_rate": "0.0",
            "use_si": "false",
            "autostart": "true",
            "tare_on_start": "false",
        }