"""Launch the G1 MIT position adapter and controller test dashboard.

Start unitree_g1_description/g1_data.launch.py first. The MIT adapter remains
inactive until its controller is engaged from the dashboard.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

_CONTROL_DEFAULTS = {
    "use_mit_controller": "true",
    "controller_name": "whole_body_controller",
    "lowstate_topic": "/lowstate",
    "lowcmd_topic": "/lowcmd",
    "left_gripper_command_topic": "/grip_arm0/mit_command",
    "right_gripper_command_topic": "/grip_arm1/mit_command",
    "left_gripper_node": "/grip_arm0",
    "right_gripper_node": "/grip_arm1",
    "g1_command_rate_hz": "500.0",
    "gripper_command_rate_hz": "100.0",
    "command_timeout_s": "0.25",
    "state_timeout_s": "0.25",
    "max_initial_position_error": "0.2",
    "max_command_step": "0.1",
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


def generate_launch_description() -> LaunchDescription:
    description_share = get_package_share_directory("unitree_g1_description")
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in {**_DEFAULTS, **_CONTROL_DEFAULTS}.items()
    ]
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            description_share, "launch", "mit_control.launch.py")),
        condition=IfCondition(LaunchConfiguration("use_mit_controller")),
        launch_arguments={
            "controller_manager": LaunchConfiguration("controller_manager"),
            "controller_name": LaunchConfiguration("controller_name"),
            "lowstate_topic": LaunchConfiguration("lowstate_topic"),
            "lowcmd_topic": LaunchConfiguration("lowcmd_topic"),
            "joint_states_topic": LaunchConfiguration("joint_states_topic"),
            "left_gripper_command_topic": LaunchConfiguration(
                "left_gripper_command_topic"),
            "right_gripper_command_topic": LaunchConfiguration(
                "right_gripper_command_topic"),
            "left_gripper_node": LaunchConfiguration(
                "left_gripper_node"),
            "right_gripper_node": LaunchConfiguration(
                "right_gripper_node"),
            "g1_command_rate_hz": LaunchConfiguration(
                "g1_command_rate_hz"),
            "gripper_command_rate_hz": LaunchConfiguration(
                "gripper_command_rate_hz"),
            "command_timeout_s": LaunchConfiguration("command_timeout_s"),
            "state_timeout_s": LaunchConfiguration("state_timeout_s"),
            "max_initial_position_error": LaunchConfiguration(
                "max_initial_position_error"),
            "max_command_step": LaunchConfiguration("max_command_step"),
            "require_pr_mode": LaunchConfiguration("require_pr_mode"),
            "gripper_kp": LaunchConfiguration("gripper_kp"),
            "gripper_kd": LaunchConfiguration("gripper_kd"),
            "gripper_service_timeout_s": LaunchConfiguration(
                "gripper_service_timeout_s"),
            "manage_motion_mode": LaunchConfiguration("manage_motion_mode"),
            "restore_motion_mode": LaunchConfiguration(
                "restore_motion_mode"),
            "fallback_motion_mode": LaunchConfiguration(
                "fallback_motion_mode"),
            "motion_switch_timeout_s": LaunchConfiguration(
                "motion_switch_timeout_s"),
            "motion_select_timeout_s": LaunchConfiguration(
                "motion_select_timeout_s"),
            "motion_release_attempts": LaunchConfiguration(
                "motion_release_attempts"),
            "motion_release_retry_s": LaunchConfiguration(
                "motion_release_retry_s"),
            "lowcmd_quiet_period_s": LaunchConfiguration(
                "lowcmd_quiet_period_s"),
            "lowcmd_quiet_timeout_s": LaunchConfiguration(
                "lowcmd_quiet_timeout_s"),
        }.items(),
    )
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
    return LaunchDescription([*arguments, controller, dashboard])