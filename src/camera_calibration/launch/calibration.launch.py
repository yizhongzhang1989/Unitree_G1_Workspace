"""标定 dashboard + 外参 TF 发布。

机器人本体、头部相机、腕相机都由各自的 launch 起，这里只管标定。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# LaunchConfiguration 求值出来一律是字符串，不显式给 value_type 的话
# 传给 declare_parameter(name, 8300) 这种整型参数会被拒
_DEFAULTS = {
    'dashboard_port': ('8300', int),
    'data_root': ('~/camera_calib_data', str),
    'board_config': ('', str),
    'cameras_config': ('', str),
    'calib_file': ('', str),
    'base_frame': ('torso_link', str),
    'preview_width': ('720', int),
    'jpeg_quality': ('75', int),
    'detect_period_s': ('0.5', float),
}


def generate_launch_description() -> LaunchDescription:
    arguments = [DeclareLaunchArgument(name, default_value=default)
                 for name, (default, _) in _DEFAULTS.items()]
    arguments.append(DeclareLaunchArgument(
        'publish_tf', default_value='true',
        description='同时把已标定的外参发成 static TF'))

    parameters = {name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                  for name, (_, kind) in _DEFAULTS.items()}

    dashboard = Node(
        package='camera_calibration',
        executable='calib_node',
        name='camera_calibration',
        output='screen',
        parameters=[parameters],
    )
    tf = Node(
        package='camera_calibration',
        executable='calib_tf_node',
        name='camera_calib_tf',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_tf')),
        parameters=[{'calib_file': ParameterValue(
            LaunchConfiguration('calib_file'), value_type=str)}],
    )
    return LaunchDescription(arguments + [dashboard, tf])
