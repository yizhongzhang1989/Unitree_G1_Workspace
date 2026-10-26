"""Compose independently launched session, lower and upper tasks."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_LOWER = {
    'velocity': ('lower_velocity_tracking', 'velocity_tracking.launch.py'),
    'footstep': ('lower_footstep_tracking', 'footstep_tracking.launch.py'),
}
_UPPER = {
    'vr': ('upper_vr', 'vr_teleop.launch.py'),
    'vla': ('upper_vla', 'vla_bridge.launch.py'),
    'float': ('upper_float', 'float.launch.py'),
    'force_feedback': ('upper_force_feedback', 'force_feedback.launch.py'),
}


def _include(package, filename, config='', *, safety_checks_enabled=''):
    source = PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory(package)) / 'launch' / filename))
    arguments = {}
    if config:
        arguments['config'] = config
    if safety_checks_enabled:
        arguments['safety_checks_enabled'] = safety_checks_enabled
    # 子任务拥有配置默认值，分组隔离各自的 launch 参数
    return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        source, launch_arguments=arguments.items())])


def _tasks(context):
    lower = LaunchConfiguration('lower_task').perform(context)
    upper = LaunchConfiguration('upper_task').perform(context)
    if lower not in (*_LOWER, 'none') or upper not in (*_UPPER, 'none'):
        raise ValueError('invalid lower_task or upper_task')
    safety = LaunchConfiguration('safety_checks_enabled', default='').perform(context)
    actions = [_include('g1_motion_session', 'session.launch.py', safety_checks_enabled=safety)]
    if lower != 'none':
        config = LaunchConfiguration('lower_config', default='').perform(context)
        actions.append(_include(*_LOWER[lower], config))
    if upper != 'none':
        config = LaunchConfiguration('upper_config', default='').perform(context)
        actions.append(_include(*_UPPER[upper], config))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lower_task', default_value='velocity',
                              choices=['velocity', 'footstep', 'none']),
        DeclareLaunchArgument('upper_task', default_value='vr',
                              choices=['vr', 'vla', 'float', 'force_feedback', 'none']),
        DeclareLaunchArgument('lower_config', default_value=''),
        DeclareLaunchArgument('upper_config', default_value=''),
        DeclareLaunchArgument('safety_checks_enabled', default_value=''),
        OpaqueFunction(function=_tasks),
    ])
