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

    forward_parameters = forward_config[
        "/forward_position_controller"]["ros__parameters"]
    joints = forward_parameters["joints"]
    trajectory_parameters = trajectory_config[
        "/joint_trajectory_controller"]["ros__parameters"]
    assert joints[:29] == gain_config["joint_names"]
    assert joints[29:] == [
        "left_eccentric_joint",
        "right_eccentric_joint",
    ]
    assert len(joints) == 31
    # The gravity feed-forward is a parameter of this controller now, not a
    # second controller relaying into it.
    assert set(forward_parameters) == {
        "joints",
        "gravity_table",
        "gravity_filter_cutoff_hz",
        "offset_ramp_s",
        "compensation_scale",
    }
    assert forward_parameters["gravity_table"] == \
        "package://arm_gravity_compensation/config/gravity_table.yaml"
    assert forward_parameters["compensation_scale"] == 1.0
    assert trajectory_parameters["joints"] == joints
    assert trajectory_parameters["command_interfaces"] == ["position"]
    assert trajectory_parameters["state_interfaces"] == ["position", "velocity"]
    # Humble 的 JTC 没有“不发布状态”这个概念（校验下限 0.1 Hz），也没有理由
    # 偏离默认值，所以干脆不覆盖。
    assert "state_publish_rate" not in trajectory_parameters
    assert trajectory_parameters["allow_nonzero_velocity_at_trajectory_end"] is False
    assert trajectory_parameters["allow_partial_joints_goal"] is True
    # 轨迹点也是绝对位置，所以它拿到与 FPC 完全相同的一套重力前馈参数。
    assert trajectory_parameters["gravity_table"] == \
        forward_parameters["gravity_table"]
    assert trajectory_parameters["compensation_scale"] == 1.0
    assert trajectory_parameters["open_loop_control"] is False
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
        "unitree_g1_joint_trajectory_controller/JointTrajectoryController"
    # 重力补偿已并入两个运动控制器，不再是独立 controller。
    assert "arm_gravity_compensation" not in manager_config


def test_plugin_names_stay_recognisable_to_the_test_dashboard():
    # dashboard_node.classify_controller() 靠这两个后缀/子串归类；改名会让页面
    # 把它们当成 "other"，Engage 按钮和关节面板都会消失。
    plugins = ElementTree.parse(PACKAGE_ROOT / "controller_plugins.xml").getroot()
    names = {element.get("name").lower() for element in plugins.findall("class")}

    assert any(name.endswith("jointtrajectorycontroller") for name in names)
    assert any("forward_command_controller" in name for name in names)


def test_arm_stiffness_default_is_owned_by_hardware_plugin():
    module = _load_control_launch()

    assert module._HARDWARE_ARGUMENTS["arm_stiffness_scale"] == ""
    context = LaunchContext()
    context.launch_configurations.update(module._HARDWARE_ARGUMENTS)
    description = module._robot_description(context, PACKAGE_ROOT, "dual")
    hardware = ElementTree.fromstring(description).find(
        "./ros2_control/hardware")
    parameters = {
        parameter.get("name"): parameter.text
        for parameter in hardware.findall("param")
    }
    assert "arm_stiffness_scale" not in parameters
    header = (PACKAGE_ROOT / "include" / "unitree_g1_ros2_control" /
              "g1_topic_system.hpp").read_text(encoding="utf-8")
    assert "double arm_stiffness_scale_{1.0};" in header


def test_arm_stiffness_explicit_override_reaches_hardware_plugin():
    module = _load_control_launch()
    context = LaunchContext()
    context.launch_configurations.update(module._HARDWARE_ARGUMENTS)
    context.launch_configurations["arm_stiffness_scale"] = "2.5"

    description = module._robot_description(context, PACKAGE_ROOT, "dual")
    hardware = ElementTree.fromstring(description).find(
        "./ros2_control/hardware")
    parameters = {
        parameter.get("name"): parameter.text
        for parameter in hardware.findall("param")
    }
    assert parameters["arm_stiffness_scale"] == "2.5"


def test_control_launch_loads_both_motion_controllers_inactive():
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
        node._Node__node_executable == "spawner"
    }

    assert "--inactive" in spawners["forward_position_controller"]
    assert "--inactive" in spawners["joint_trajectory_controller"]
    assert "arm_gravity_compensation" not in spawners
