"""Exclusive, hardware-free PICO -> G1 CSV capture source."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    config = Path(get_package_share_directory('g1_mocap')) / 'config' / 'mocap.yaml'
    document = yaml.safe_load(config.read_text(encoding='utf-8'))
    parameters = dict(document['/mocap']['ros__parameters'])
    for name in ('output_dir', 'category', 'action', 'host', 'token'):
        parameters[name] = LaunchConfiguration(name).perform(context)
    urdf = LaunchConfiguration('urdf_path').perform(context)
    if urdf:
        parameters['urdf_path'] = urdf
    parameters['port'] = int(LaunchConfiguration('port').perform(context))
    for name in ('max_duration_s', 'max_gap_s', 'max_root_speed_m_s', 'max_root_angular_speed_rad_s'):
        parameters[name] = float(LaunchConfiguration(name).perform(context))
    for name in ('model_confirmed', 'controller_buttons'):
        value = LaunchConfiguration(name).perform(context).lower()
        if value not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        parameters[name] = value == 'true'
    return [Node(
        package='g1_mocap', executable='motion_capture_node', name='motion_capture',
        parameters=[parameters], output='screen', emulate_tty=True,
        additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
    )]


def generate_launch_description():
    defaults = dict(
        output_dir='~/motions_dataset', category='locomotion', action='walk_forward',
        urdf_path='', model_confirmed='false',
        host='0.0.0.0', port='18001', token='', controller_buttons='true',
        max_duration_s='60.0', max_gap_s='0.1', max_root_speed_m_s='8.0',
        max_root_angular_speed_rad_s='15.0')
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        OpaqueFunction(function=_nodes),
    ])
