"""Run the manual gripper-gate experiment with the existing CLI."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    source = os.path.join(get_package_share_directory('g1_vla_bridge'),
                          'launch', 'vla_bridge.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(source),
            launch_arguments={
                'bridge_executable': 'gripper_gate_node',
                'execution_mode': 'manual',
                'skip_intermediate_waypoints': 'false',
            }.items()),
    ])
