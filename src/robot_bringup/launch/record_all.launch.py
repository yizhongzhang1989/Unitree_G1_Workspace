"""Launch hardware, tasks, sensors, localization and recording together."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_STAGES = (
    ('hardware', 'robot_bringup', 'all_data.launch.py', 0.0),
    ('sensors', 'head_sensors', 'head_sensors.launch.py', 0.0),
    ('motion', 'robot_bringup', 'motion_tasks.launch.py', 6.0),
    ('localization', 'g1_localization', 'localization.launch.py', 8.0),
    ('record', 'record', 'record.launch.py', 12.0),
)

_OVERRIDES = {
    'hardware': ('scope', 'topology'),
    'sensors': ('color_profile', 'color_format'),
    'motion': ('lower_task', 'upper_task'),
}


def generate_launch_description() -> LaunchDescription:
    stages = [
        TimerAction(
            period=delay,
            condition=IfCondition(LaunchConfiguration(name)),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory(package), 'launch', launch_file)),
                launch_arguments=[
                    (key, LaunchConfiguration(key))
                    for key in _OVERRIDES.get(name, ())
                ],
            )],
        )
        for name, package, launch_file, delay in _STAGES
    ]

    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value='true'),
        DeclareLaunchArgument('sensors', default_value='true'),
        DeclareLaunchArgument('motion', default_value='true'),
        DeclareLaunchArgument('localization', default_value='true'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('sigterm_timeout', default_value='40'),
        DeclareLaunchArgument('sigkill_timeout', default_value='10'),
        DeclareLaunchArgument('scope', default_value='whole_body'),
        DeclareLaunchArgument('topology', default_value='dual'),
        DeclareLaunchArgument('lower_task', default_value='velocity'),
        DeclareLaunchArgument('upper_task', default_value='vr'),
        DeclareLaunchArgument('color_profile', default_value='1280x720x30'),
        DeclareLaunchArgument('color_format', default_value='YUYV'),
        *stages,
    ])
