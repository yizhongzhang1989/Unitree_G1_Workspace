"""Launch the dual-bus four-device bringup and its unified web dashboard."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("robot_bringup")
    devices = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(bringup_share, "launch", "dual_bus.launch.py")))
    dashboard = Node(
        package="robot_bringup",
        executable="web_dashboard",
        name="robot_web_dashboard",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "host": ParameterValue(
                LaunchConfiguration("web_host"), value_type=str),
            "port": ParameterValue(
                LaunchConfiguration("web_port"), value_type=int),
            "request_timeout_s": ParameterValue(
                LaunchConfiguration("request_timeout_s"), value_type=float),
            "state_stale_s": ParameterValue(
                LaunchConfiguration("state_stale_s"), value_type=float),
            "left_bus": "can0",
            "left_sensor_node": "/ft_arm0",
            "left_wrench_topic": "/arm0/wrench_raw",
            "left_gripper_node": "/grip_arm0",
            "right_bus": "can1",
            "right_sensor_node": "/ft_arm1",
            "right_wrench_topic": "/arm1/wrench_raw",
            "right_gripper_node": "/grip_arm1",
            "left_camera_url": LaunchConfiguration("left_camera_url"),
            "right_camera_url": LaunchConfiguration("right_camera_url"),
            "camera_timeout_s": ParameterValue(
                LaunchConfiguration("camera_timeout_s"), value_type=float),
            "camera_poll_period_s": ParameterValue(
                LaunchConfiguration("camera_poll_period_s"),
                value_type=float),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="8770"),
        DeclareLaunchArgument("request_timeout_s", default_value="3.0"),
        DeclareLaunchArgument("state_stale_s", default_value="1.0"),
        DeclareLaunchArgument(
            "left_camera_url", default_value="http://127.0.0.1:8010"),
        DeclareLaunchArgument(
            "right_camera_url", default_value="http://127.0.0.1:8011"),
        DeclareLaunchArgument("camera_timeout_s", default_value="1.0"),
        DeclareLaunchArgument("camera_poll_period_s", default_value="2.0"),
        devices,
        dashboard,
    ])
