"""Launch the G1 description publisher and whole-body control dashboard.

The robot control stack must provide /joint_states and /controller_manager.
This wrapper publishes /robot_description and starts robot_state_publisher, but
does not start or take over G1 low-level control.

  ros2 launch robot_bringup whole_body_dashboard.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


_DASHBOARD_DEFAULTS = {
    "dashboard_port": "8200",
    "controller_manager": "/controller_manager",
    "robot_description_topic": "/robot_description",
    "joint_states_topic": "/joint_states",
    "base_frame": "pelvis",
    "tip_frame": "right_gripper_base",
    "max_joint_speed": "0.5",
    "send_rate": "100.0",
}

_PUBLISHER_DEFAULTS = {
    "publish_robot_description": "true",
    "use_sim_time": "false",
}


def generate_launch_description() -> LaunchDescription:
    dashboard_share = get_package_share_directory("robot_test_dashboard")
    description_share = get_package_share_directory("unitree_g1_description")
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in {
            **_DASHBOARD_DEFAULTS,
            **_PUBLISHER_DEFAULTS,
        }.items()
    ]
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            description_share, "launch", "description.launch.py")),
        condition=IfCondition(LaunchConfiguration(
            "publish_robot_description")),
        launch_arguments={
            "joint_states_topic": LaunchConfiguration("joint_states_topic"),
            "robot_description_topic": LaunchConfiguration(
                "robot_description_topic"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )
    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            dashboard_share, "launch", "dashboard.launch.py")),
        launch_arguments={
            name: LaunchConfiguration(name)
            for name in _DASHBOARD_DEFAULTS
        }.items(),
    )
    return LaunchDescription([*arguments, description, dashboard])