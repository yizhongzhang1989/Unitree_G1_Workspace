"""Compose the session, VR arm task and VR-driven velocity tracking."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    source = Path(get_package_share_directory('robot_bringup')) / 'launch/motion_tasks.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('safety_checks_enabled', default_value=''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(source)),
            launch_arguments={
                'lower_task': 'velocity',
                'upper_task': 'vr',
                'lower_config': 'lower_vr.yaml',
                'safety_checks_enabled': LaunchConfiguration('safety_checks_enabled'),
            }.items())])
