from pathlib import Path
import importlib.util
from xml.etree import ElementTree

import yaml
from launch import LaunchContext
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).parents[1]


def _load_control_launch():
    path = PACKAGE_ROOT / "launch" / "control.launch.py"
    spec = importlib.util.spec_from_file_location("unitree_control_launch", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load launch module: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_default_controller_claims_g1_body_and_both_grippers():
    forward_config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "forward_position_controller.yaml").read_text(
            encoding="utf-8"))
    trajectory_config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "joint_trajectory_controller.yaml").read_text(
            encoding="utf-8"))
    gain_config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "default_29dof_param.yaml").read_text(
            encoding="utf-8"))

    joints = forward_config[
        "/forward_position_controller"]["ros__parameters"]["joints"]
    trajectory_parameters = trajectory_config[
        "/joint_trajectory_controller"]["ros__parameters"]
    assert joints[:29] == gain_config["joint_names"]
    assert joints[29:] == [
        "left_eccentric_joint",
        "right_eccentric_joint",
    ]
    assert len(joints) == 31
    assert trajectory_parameters["joints"] == joints
    assert trajectory_parameters["command_interfaces"] == ["position"]
    assert trajectory_parameters["state_interfaces"] == ["position", "velocity"]
    assert trajectory_parameters["state_publish_rate"] == 0.0
    assert trajectory_parameters["allow_partial_joints_goal"] is True
    constraints = trajectory_parameters["constraints"]
    assert constraints["goal_time"] == 2.0
    assert constraints["stopped_velocity_tolerance"] == 0.05
    assert set(constraints) == {"goal_time", "stopped_velocity_tolerance", *joints}
    assert all(constraints[joint]["goal"] == 0.05 for joint in joints)


def test_controller_manager_registers_mutually_exclusive_fpc_and_jtc():
    manager_config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "controllers.yaml").read_text(
            encoding="utf-8"))["controller_manager"]["ros__parameters"]

    assert manager_config["forward_position_controller"]["type"] == \
        "unitree_g1_forward_command_controller/ForwardCommandController"
    assert manager_config["joint_trajectory_controller"]["type"] == \
        "joint_trajectory_controller/JointTrajectoryController"


def test_arm_stiffness_uses_a_conservative_startup_scale():
    module = _load_control_launch()

    assert module._HARDWARE_ARGUMENTS["arm_stiffness_scale"] == "2.5"

    context = LaunchContext()
    context.launch_configurations.update(module._HARDWARE_ARGUMENTS)
    description = module._robot_description(context, PACKAGE_ROOT, "dual")
    hardware = ElementTree.fromstring(description).find(
        "./ros2_control/hardware")
    parameters = {
        parameter.get("name"): parameter.text
        for parameter in hardware.findall("param")
    }
    assert parameters["arm_stiffness_scale"] == "2.5"


def test_control_launch_loads_both_motion_controllers_stopped():
    module = _load_control_launch()
    context = LaunchContext()
    context.launch_configurations.update({
        "topology": "dual",
        "controller_manager": "/controller_manager",
        "joint_states_topic": "/joint_states",
        "robot_description_topic": "/robot_description",
        "use_sim_time": "false",
        **module._HARDWARE_ARGUMENTS,
    })
    nodes = module._control_nodes(context)
    spawners = {
        str(node._Node__arguments[0]): node._Node__arguments
        for node in nodes
        if isinstance(node, Node) and
        node._Node__node_executable == "spawner.py"
    }

    assert "--stopped" in spawners["forward_position_controller"]
    assert "--stopped" in spawners["joint_trajectory_controller"]
