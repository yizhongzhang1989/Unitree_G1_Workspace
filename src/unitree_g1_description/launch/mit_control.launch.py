"""Expose one dashboard controller and distribute positions as MIT commands."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_DEFAULTS = {
    "controller_manager": "/controller_manager",
    "controller_name": "whole_body_controller",
    "lowstate_topic": "/lowstate",
    "lowcmd_topic": "/lowcmd",
    "joint_states_topic": "/joint_states",
    "left_gripper_command_topic": "/grip_arm0/mit_command",
    "right_gripper_command_topic": "/grip_arm1/mit_command",
    "g1_command_rate_hz": "500.0",
    "gripper_command_rate_hz": "100.0",
    "command_timeout_s": "0.25",
    "state_timeout_s": "0.25",
    "max_initial_position_error": "0.2",
    "max_command_step": "0.1",
    "require_pr_mode": "true",
    "gripper_kp": "10.0",
    "gripper_kd": "5.0",
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
    package_share = get_package_share_directory("unitree_g1_description")
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in _DEFAULTS.items()
    ]
    controller = Node(
        package="unitree_g1_description",
        executable="mit_position_controller",
        name="g1_mit_position_controller",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "controller_manager": ParameterValue(
                LaunchConfiguration("controller_manager"), value_type=str),
            "controller_name": ParameterValue(
                LaunchConfiguration("controller_name"), value_type=str),
            "lowstate_topic": ParameterValue(
                LaunchConfiguration("lowstate_topic"), value_type=str),
            "lowcmd_topic": ParameterValue(
                LaunchConfiguration("lowcmd_topic"), value_type=str),
            "joint_states_topic": ParameterValue(
                LaunchConfiguration("joint_states_topic"), value_type=str),
            "left_gripper_command_topic": ParameterValue(
                LaunchConfiguration("left_gripper_command_topic"),
                value_type=str),
            "right_gripper_command_topic": ParameterValue(
                LaunchConfiguration("right_gripper_command_topic"),
                value_type=str),
            "gain_file": os.path.join(
                package_share, "config", "default_29dof_param.yaml"),
            "joint_limits_file": os.path.join(
                package_share, "model", "final.urdf"),
            "g1_command_rate_hz": ParameterValue(
                LaunchConfiguration("g1_command_rate_hz"), value_type=float),
            "gripper_command_rate_hz": ParameterValue(
                LaunchConfiguration("gripper_command_rate_hz"),
                value_type=float),
            "command_timeout_s": ParameterValue(
                LaunchConfiguration("command_timeout_s"), value_type=float),
            "state_timeout_s": ParameterValue(
                LaunchConfiguration("state_timeout_s"), value_type=float),
            "max_initial_position_error": ParameterValue(
                LaunchConfiguration("max_initial_position_error"),
                value_type=float),
            "max_command_step": ParameterValue(
                LaunchConfiguration("max_command_step"), value_type=float),
            "require_pr_mode": ParameterValue(
                LaunchConfiguration("require_pr_mode"), value_type=bool),
            "gripper_kp": ParameterValue(
                LaunchConfiguration("gripper_kp"), value_type=float),
            "gripper_kd": ParameterValue(
                LaunchConfiguration("gripper_kd"), value_type=float),
            "manage_motion_mode": ParameterValue(
                LaunchConfiguration("manage_motion_mode"), value_type=bool),
            "restore_motion_mode": ParameterValue(
                LaunchConfiguration("restore_motion_mode"), value_type=bool),
            "fallback_motion_mode": ParameterValue(
                LaunchConfiguration("fallback_motion_mode"), value_type=str),
            "motion_switch_timeout_s": ParameterValue(
                LaunchConfiguration("motion_switch_timeout_s"),
                value_type=float),
            "motion_select_timeout_s": ParameterValue(
                LaunchConfiguration("motion_select_timeout_s"),
                value_type=float),
            "motion_release_attempts": ParameterValue(
                LaunchConfiguration("motion_release_attempts"),
                value_type=int),
            "motion_release_retry_s": ParameterValue(
                LaunchConfiguration("motion_release_retry_s"),
                value_type=float),
            "lowcmd_quiet_period_s": ParameterValue(
                LaunchConfiguration("lowcmd_quiet_period_s"),
                value_type=float),
            "lowcmd_quiet_timeout_s": ParameterValue(
                LaunchConfiguration("lowcmd_quiet_timeout_s"),
                value_type=float),
        }],
    )
    return LaunchDescription([*arguments, controller])