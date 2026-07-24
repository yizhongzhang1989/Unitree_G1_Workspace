"""Launch robot hardware producers without any web dashboard.

scope:=end_effectors starts the complete end-effector data path only.
scope:=whole_body additionally starts the real ros2_control manager, state
broadcasters, assembled robot description, and an inactive position controller.
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
        control_share = get_package_share_directory(
            "unitree_g1_ros2_control")
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                control_share, "launch", "control.launch.py")),
            launch_arguments={
                "topology": topology,
                "controller_manager": LaunchConfiguration("controller_manager"),
                "lowstate_topic": LaunchConfiguration("lowstate_topic"),
                "arm_stiffness_scale": LaunchConfiguration("arm_stiffness_scale"),
                "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                "robot_description_topic": LaunchConfiguration("robot_description_topic"),
                "require_pr_mode": LaunchConfiguration("require_pr_mode"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ))

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("scope", default_value="whole_body"),
        DeclareLaunchArgument("topology", default_value="dual"),
        DeclareLaunchArgument("enable_grippers_on_start", default_value="true"),
        DeclareLaunchArgument("controller_manager", default_value="/controller_manager"),
        DeclareLaunchArgument("lowstate_topic", default_value="/lowstate"),
        DeclareLaunchArgument("arm_stiffness_scale", default_value="2"),
        DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
        DeclareLaunchArgument("robot_description_topic", default_value="/robot_description"),
        DeclareLaunchArgument("require_pr_mode", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_data_launches),
    ])