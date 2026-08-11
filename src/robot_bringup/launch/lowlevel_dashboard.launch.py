"""Launch the read-only low-level monitor (LowState + secondary IMU).

Nothing else is started here: the page only reads what the G1 firmware already
publishes, so it is safe to run at any time, with or without the control stack.

Defaults to the firmware's 20 Hz ``/lf/*`` topics; pass
``lowstate_topic:=/lowstate secondary_imu_topic:=/secondary_imu`` for the 1040 Hz
streams, which costs close to a full core instead of ~1%.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_DEFAULTS = {
    'web_host': '0.0.0.0',
    'web_port': '8210',
    'lowstate_topic': '/lf/lowstate',
    'secondary_imu_topic': '/lf/secondary_imu',
    'idle_release_s': '3.0',
}


def _dashboard_node(context):
    def value(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    return [Node(
        package='robot_bringup',
        executable='lowlevel_dashboard',
        name='lowlevel_dashboard',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'host': value('web_host'),
            'port': int(value('web_port')),
            'lowstate_topic': value('lowstate_topic'),
            'secondary_imu_topic': value('secondary_imu_topic'),
            'idle_release_s': float(value('idle_release_s')),
        }],
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default)
          for name, default in _DEFAULTS.items()),
        OpaqueFunction(function=_dashboard_node),
    ])
