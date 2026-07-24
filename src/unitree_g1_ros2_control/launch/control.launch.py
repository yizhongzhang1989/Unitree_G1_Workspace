"""Start the real Foxy ros2_control stack for the assembled Unitree G1."""

from pathlib import Path
from typing import cast
from urllib.parse import urlparse
from xml.dom.minidom import Document
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


_TOPOLOGIES = {
    "dual": {
        "left_gripper_state_topic": "/grip_arm0/joint_states",
        "right_gripper_state_topic": "/grip_arm1/joint_states",
        "left_gripper_command_topic": "/grip_arm0/mit_command",
        "right_gripper_command_topic": "/grip_arm1/mit_command",
        "left_gripper_node": "/grip_arm0",
        "right_gripper_node": "/grip_arm1",
        "left_wrench_topic": "/arm0/wrench_raw",
        "right_wrench_topic": "/arm1/wrench_raw",
    },
    "single": {
        "left_gripper_state_topic": "/grip_left/joint_states",
        "right_gripper_state_topic": "/grip_right/joint_states",
        "left_gripper_command_topic": "/grip_left/mit_command",
        "right_gripper_command_topic": "/grip_right/mit_command",
        "left_gripper_node": "/grip_left",
        "right_gripper_node": "/grip_right",
        "left_wrench_topic": "/ft_left/wrench_raw",
        "right_wrench_topic": "/ft_right/wrench_raw",
    },
}

_HARDWARE_ARGUMENTS = {
    "lowstate_topic": "/lowstate",
    "lowcmd_topic": "/lowcmd",
    "gripper_command_rate_hz": "100.0",
    # 修改这个让上肢相应更及时（刚性）
    "arm_stiffness_scale": "",
    "state_timeout_s": "0.25",
    "gripper_state_timeout_s": "0.75",
    "require_pr_mode": "true",
    "gripper_kp": "10.0",
    "gripper_kd": "5.0",
    "gripper_service_timeout_s": "3.0",
    "manage_motion_mode": "true",
    "restore_motion_mode": "true",
    "fallback_motion_mode": "ai",
    "motion_switch_timeout_s": "1.0",
    "motion_select_timeout_s": "10.0",
    "motion_release_attempts": "3",
    "motion_release_retry_s": "0.2",
    "lowcmd_quiet_period_s": "0.1",
    "lowcmd_quiet_timeout_s": "2.0",
}


def _robot_description(context, package_share: Path, topology: str) -> str:
    mappings = dict(_TOPOLOGIES[topology])
    mappings.update({
        name: LaunchConfiguration(name).perform(context)
        for name in _HARDWARE_ARGUMENTS
    })
    mappings["gain_file"] = str(
        package_share / "config" / "default_29dof_param.yaml")
    document = cast(Document, xacro.process_file(
        str(package_share / "urdf" / "g1_with_ros2_control.urdf.xacro"),
        mappings=mappings,
    ))
    root = ElementTree.fromstring(document.toxml())
    model_prefix = "package://unitree_g1_description/model/"
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename")
        if filename and not urlparse(filename).scheme and not Path(filename).is_absolute():
            mesh.set("filename", model_prefix + filename)
    return ElementTree.tostring(root, encoding="unicode")


def _control_nodes(context):
    topology = LaunchConfiguration("topology").perform(context).lower()
    if topology not in _TOPOLOGIES:
        raise ValueError("topology must be 'single' or 'dual'")

    package_share = Path(get_package_share_directory("unitree_g1_ros2_control"))
    description = _robot_description(context, package_share, topology)
    description_parameter = ParameterValue(description, value_type=str)
    controller_manager = LaunchConfiguration("controller_manager").perform(context)
    manager_parts = [part for part in controller_manager.split("/") if part]
    if not manager_parts or manager_parts[-1] != "controller_manager":
        raise ValueError("controller_manager basename must be 'controller_manager'")
    manager_namespace = "/" + "/".join(manager_parts[:-1]) if len(manager_parts) > 1 else "/"
    controllers = str(package_share / "config" / "controllers.yaml")
    forward_position_parameters = str(
        package_share / "config" / "forward_position_controller.yaml")
    joint_trajectory_parameters = str(
        package_share / "config" / "joint_trajectory_controller.yaml")
    imu_parameters = str(
        package_share / "config" / "pelvis_imu_broadcaster.yaml")
    joint_state_parameters = str(
        package_share / "config" / "joint_state_broadcaster.yaml")

    return [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=manager_namespace,
            output="screen",
            parameters=[controllers, {
                "robot_description": description_parameter,
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool),
            }],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": description_parameter,
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool),
            }],
            remappings=[
                ("robot_description", LaunchConfiguration("robot_description_topic")),
                ("joint_states", LaunchConfiguration("joint_states_topic")),
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=[
                "joint_state_broadcaster",
                "--param-file", joint_state_parameters,
                "--controller-manager", controller_manager,
                "--controller-manager-timeout", "30",
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=[
                "pelvis_imu_broadcaster",
                "--param-file", imu_parameters,
                "--controller-manager", controller_manager,
                "--controller-manager-timeout", "30",
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=[
                "forward_position_controller",
                "--param-file", forward_position_parameters,
                "--stopped",
                "--controller-manager", controller_manager,
                "--controller-manager-timeout", "30",
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=[
                "joint_trajectory_controller",
                "--param-file", joint_trajectory_parameters,
                "--stopped",
                "--controller-manager", controller_manager,
                "--controller-manager-timeout", "30",
            ],
            output="screen",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("topology", default_value="dual"),
        DeclareLaunchArgument("controller_manager", default_value="/controller_manager"),
        DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
        DeclareLaunchArgument("robot_description_topic", default_value="/robot_description"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        *[
            DeclareLaunchArgument(name, default_value=value)
            for name, value in _HARDWARE_ARGUMENTS.items()
        ],
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=_control_nodes)])
