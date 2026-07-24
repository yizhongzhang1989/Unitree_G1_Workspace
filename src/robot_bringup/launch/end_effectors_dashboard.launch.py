"""Launch only the end-effector web dashboard.

Start end_effectors_single_bus.launch.py or end_effectors_dual_bus.launch.py
first. This launch never opens the CAN adapter or starts device/camera nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_bringup.end_effectors.topology import dashboard_topology_parameters


def _dashboard_node(context):
    topology = LaunchConfiguration("topology").perform(context).lower()

    return [Node(
        package="robot_bringup",
        executable="end_effectors_dashboard",
        name="end_effectors_dashboard",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "host": LaunchConfiguration("web_host").perform(context),
            "port": int(LaunchConfiguration("web_port").perform(context)),
            "request_timeout_s": float(
                LaunchConfiguration("request_timeout_s").perform(context)),
            "state_stale_s": float(
                LaunchConfiguration("state_stale_s").perform(context)),
            **dashboard_topology_parameters(topology),
            "left_camera_url": LaunchConfiguration(
                "left_camera_url").perform(context),
            "right_camera_url": LaunchConfiguration(
                "right_camera_url").perform(context),
            "camera_timeout_s": float(
                LaunchConfiguration("camera_timeout_s").perform(context)),
            "camera_poll_period_s": float(LaunchConfiguration(
                "camera_poll_period_s").perform(context)),
        }],
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("topology", default_value="dual"),
        DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="8770"),
        DeclareLaunchArgument("request_timeout_s", default_value="3.0"),
        DeclareLaunchArgument("state_stale_s", default_value="1.0"),
        DeclareLaunchArgument("left_camera_url", default_value="http://127.0.0.1:8010"),
        DeclareLaunchArgument("right_camera_url", default_value="http://127.0.0.1:8011"),
        DeclareLaunchArgument("camera_timeout_s", default_value="1.0"),
        DeclareLaunchArgument("camera_poll_period_s", default_value="2.0"),
        OpaqueFunction(function=_dashboard_node),
    ])
