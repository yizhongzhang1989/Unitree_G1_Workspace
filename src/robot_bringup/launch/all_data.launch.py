"""Launch data producers without any web dashboard.

scope:=end_effectors starts the complete end-effector data path only.
scope:=whole_body additionally converts G1 LowState, publishes the assembled
robot description, and starts robot_state_publisher for TF.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


_TOPOLOGY_LAUNCHES = {
    "single": "end_effectors_single_bus.launch.py",
    "dual": "end_effectors_dual_bus.launch.py",
}
_SCOPES = ("end_effectors", "whole_body")
_GRIPPER_TOPICS = {
    "single": ("/grip_left/joint_states", "/grip_right/joint_states"),
    "dual": ("/grip_arm0/joint_states", "/grip_arm1/joint_states"),
}


def _data_launches(context):
    scope = LaunchConfiguration("scope").perform(context).lower()
    if scope not in _SCOPES:
        raise ValueError("scope must be 'end_effectors' or 'whole_body'")

    topology = LaunchConfiguration("topology").perform(context).lower()
    topology_launch = _TOPOLOGY_LAUNCHES.get(topology)
    if topology_launch is None:
        raise ValueError("topology must be 'single' or 'dual'")

    bringup_share = get_package_share_directory("robot_bringup")
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_share, "launch", topology_launch)),
        launch_arguments={
            "enable_grippers_on_start": LaunchConfiguration(
                "enable_grippers_on_start"),
        }.items(),
    )]

    if scope == "whole_body":
        left_gripper_topic, right_gripper_topic = _GRIPPER_TOPICS[topology]
        description_share = get_package_share_directory(
            "unitree_g1_description")
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                description_share, "launch", "g1_data.launch.py")),
            launch_arguments={
                "lowstate_topic": LaunchConfiguration("lowstate_topic"),
                "joint_states_topic": LaunchConfiguration(
                    "joint_states_topic"),
                "left_gripper_joint_states_topic": left_gripper_topic,
                "right_gripper_joint_states_topic": right_gripper_topic,
                "robot_description_topic": LaunchConfiguration(
                    "robot_description_topic"),
                "require_pr_mode": LaunchConfiguration("require_pr_mode"),
                "joint_state_publish_rate_hz": LaunchConfiguration(
                    "joint_state_publish_rate_hz"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ))

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("scope", default_value="whole_body"),
        DeclareLaunchArgument("topology", default_value="dual"),
        DeclareLaunchArgument(
            "enable_grippers_on_start", default_value="true"),
        DeclareLaunchArgument("lowstate_topic", default_value="/lowstate"),
        DeclareLaunchArgument(
            "joint_states_topic", default_value="/joint_states"),
        DeclareLaunchArgument(
            "robot_description_topic", default_value="/robot_description"),
        DeclareLaunchArgument("require_pr_mode", default_value="true"),
        DeclareLaunchArgument(
            "joint_state_publish_rate_hz", default_value="100.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_data_launches),
    ])