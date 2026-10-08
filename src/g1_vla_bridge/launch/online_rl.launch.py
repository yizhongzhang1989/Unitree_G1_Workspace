"""Opt-in manual online RL bridge; never run alongside another command bridge."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


OPTIONAL = {'server_url': str, 'proxy': str, 'task_description': str,
            'request_timeout_s': float, 'history_length': int, 'action_horizon': int}


def _node(context):
    directory = Path(get_package_share_directory('g1_vla_bridge')) / 'config'
    parameters = {}
    for path in (directory / 'vla_bridge.yaml', directory / 'backends/cogact_unitree.yaml'):
        with path.open(encoding='utf-8') as handle:
            parameters.update(yaml.safe_load(handle)['/vla_bridge']['ros__parameters'])
    parameters.update(vla_backend='cogact_unitree', execution_mode='manual',
                      skip_intermediate_waypoints=False, task_description='',
                      rl_directory=LaunchConfiguration('rl_directory').perform(context),
                      rl_max_observation_age_s=float(
                          LaunchConfiguration('rl_max_observation_age_s').perform(context)))
    for name, convert in OPTIONAL.items():
        value = LaunchConfiguration(name).perform(context)
        if value:
            parameters[name] = convert(value)
    return [Node(package='g1_vla_bridge', executable='online_rl_bridge',
                 name='online_rl_bridge', output='screen', emulate_tty=True,
                 parameters=[parameters])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rl_directory', description='Persistent client experiment directory'),
        DeclareLaunchArgument('rl_max_observation_age_s', default_value='3.0'),
        *(DeclareLaunchArgument(name, default_value='') for name in OPTIONAL),
        OpaqueFunction(function=_node),
    ])
