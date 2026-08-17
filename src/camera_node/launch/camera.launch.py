"""单台 IP 相机。`server_port` 留 0 就不开预览页。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_ARGS = {
    'rtsp_url': ('rtsp://admin:123456@192.168.123.97/stream1', str),
    'image_topic': ('~/image_raw', str),
    'image_width': ('0', int),
    'image_height': ('0', int),
    'fps': ('0', int),
    'server_port': ('0', int),
}


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='camera'),
        *(DeclareLaunchArgument(name, default_value=default)
          for name, (default, _) in _ARGS.items()),
        Node(
            package='camera_node', executable='camera_node',
            name=LaunchConfiguration('name'),
            output='screen', emulate_tty=True,
            parameters=[{
                name: ParameterValue(
                    LaunchConfiguration(name), value_type=value_type)
                for name, (_, value_type) in _ARGS.items()
            }],
        ),
    ])
