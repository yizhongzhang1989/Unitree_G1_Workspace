"""Launch the dashboard for an already running whole-body control stack.

The robot control stack must provide /robot_description, /joint_states, TF,
and /controller_manager. This wrapper does not start or take over G1 low-level
control.

  ros2 launch robot_bringup whole_body_dashboard.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


_DEFAULTS = {
    "dashboard_port": "8200",
    "controller_manager": "/controller_manager",
    "robot_description_topic": "/robot_description",
    "joint_states_topic": "/joint_states",
    "base_frame": "base_link",
    "tip_frame": "",
    "max_joint_speed": "0.5",
    "send_rate": "100.0",
}


def generate_launch_description() -> LaunchDescription:
    dashboard_share = get_package_share_directory("robot_test_dashboard")
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in _DEFAULTS.items()
    ]
    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            dashboard_share, "launch", "dashboard.launch.py")),
        launch_arguments={
            name: LaunchConfiguration(name)
            for name in _DEFAULTS
        }.items(),
    )
    return LaunchDescription([*arguments, dashboard])