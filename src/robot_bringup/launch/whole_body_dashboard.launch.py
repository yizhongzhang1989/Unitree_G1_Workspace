"""Launch the web dashboard for an existing ros2_control manager.

Start all_data.launch.py with scope:=whole_body first. The position controller
is loaded but remains inactive until it is engaged from this dashboard.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_DEFAULTS = {
    "dashboard_port": "8200",
    "controller_manager": "/controller_manager",
    "robot_description_topic": "/robot_description",
    "joint_states_topic": "/joint_states",
    "base_frame": "pelvis",
    "tip_frame": "right_gripper_base",
    "max_joint_speed": "0.5",
    "send_rate": "100.0",
}

def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in _DEFAULTS.items()
    ]
    dashboard = Node(
        package="robot_bringup",
        executable="whole_body_dashboard",
        name="robot_test_dashboard",
        output="screen",
        parameters=[{
            name: LaunchConfiguration(name)
            for name in _DEFAULTS
        }],
    )
    return LaunchDescription([*arguments, dashboard])