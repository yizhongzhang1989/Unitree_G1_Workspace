"""Launch only the motion session coordinator."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nodes(context):
    directory = Path(get_package_share_directory('g1_motion_session')) / 'config'
    hardware = Path(get_package_share_directory('unitree_g1_ros2_control')) / 'config/default_31dof_param.yaml'
    files = [str(directory / 'session.yaml')]
    config = LaunchConfiguration('config').perform(context)
    if config and str(directory / config) not in files:
        files.append(str(directory / config))
    overrides = {}
    safety = LaunchConfiguration('safety_checks_enabled', default='').perform(context)
    if safety:
        overrides['safety_checks_enabled'] = ParameterValue(
            LaunchConfiguration('safety_checks_enabled'), value_type=bool)
    return [Node(package='g1_motion_session', executable='motion_session',
                 name='motion_control', output='screen',
                 parameters=[*files, str(hardware), overrides])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='session.yaml'),
        DeclareLaunchArgument('safety_checks_enabled', default_value='',
                              description='Override session safety checks; empty uses YAML'),
        OpaqueFunction(function=_nodes),
    ])
