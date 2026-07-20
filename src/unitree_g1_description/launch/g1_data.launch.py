"""Publish all G1 model data available from this workspace.

This launch converts Unitree /lowstate to standard /joint_states, publishes the
assembled URDF on /robot_description, and starts robot_state_publisher for TF.
It does not provide ros2_control or command any motor.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("unitree_g1_description")
    lowstate_topic = LaunchConfiguration("lowstate_topic")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    left_gripper_topic = LaunchConfiguration(
        "left_gripper_joint_states_topic")
    right_gripper_topic = LaunchConfiguration(
        "right_gripper_joint_states_topic")
    robot_description_topic = LaunchConfiguration("robot_description_topic")
    require_pr_mode = LaunchConfiguration("require_pr_mode")
    joint_state_publish_rate_hz = LaunchConfiguration(
        "joint_state_publish_rate_hz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_share, "launch", "description.launch.py")),
        launch_arguments={
            "joint_states_topic": joint_states_topic,
            "robot_description_topic": robot_description_topic,
            "use_sim_time": use_sim_time,
        }.items(),
    )
    converter = Node(
        package="unitree_g1_description",
        executable="lowstate_to_joint_states",
        name="lowstate_to_joint_states",
        output="screen",
        parameters=[{
            "lowstate_topic": ParameterValue(lowstate_topic, value_type=str),
            "joint_states_topic": ParameterValue(
                joint_states_topic, value_type=str),
            "left_gripper_joint_states_topic": ParameterValue(
                left_gripper_topic, value_type=str),
            "right_gripper_joint_states_topic": ParameterValue(
                right_gripper_topic, value_type=str),
            "require_pr_mode": ParameterValue(
                require_pr_mode, value_type=bool),
            "joint_state_publish_rate_hz": ParameterValue(
                joint_state_publish_rate_hz, value_type=float),
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("lowstate_topic", default_value="/lowstate"),
        DeclareLaunchArgument(
            "joint_states_topic", default_value="/joint_states"),
        DeclareLaunchArgument(
            "left_gripper_joint_states_topic",
            default_value="/grip_arm0/joint_states"),
        DeclareLaunchArgument(
            "right_gripper_joint_states_topic",
            default_value="/grip_arm1/joint_states"),
        DeclareLaunchArgument(
            "robot_description_topic", default_value="/robot_description"),
        DeclareLaunchArgument("require_pr_mode", default_value="true"),
        DeclareLaunchArgument(
            "joint_state_publish_rate_hz", default_value="100.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        description,
        converter,
    ])