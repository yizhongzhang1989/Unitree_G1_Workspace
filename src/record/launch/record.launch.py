"""采集节点 + 面板。

    ros2 launch record record.launch.py

前置：``all_data.launch.py`` 与 ``motion_control.launch.py`` 已起；头部相机建议用
``color_format:=YUYV`` 起（省掉一次色彩转换，720p 编码从 98% 降到 60% 单核）::

    ros2 launch head_sensors head_camera.launch.py \\
        color_profile:=1280x720x30 color_format:=YUYV
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_ARGS = {
    'output_root': str(Path.home() / '.ros' / 'record' / 'sessions'),
    'dashboard_port': '8220',
    'head_image_topic': '/head/camera/color/image_raw',
    'head_camera_info_topic': '/head/camera/color/camera_info',
    'head_fps': '30',
    # 腕部走主码流原画 -c copy：1.33% 单核/路，比解码转 720p 便宜 78 倍且画质更好，
    # 720p 留到导出机上离线降。
    'wrist_left_url': 'rtsp://admin:123456@192.168.123.97/stream0',
    'wrist_right_url': 'rtsp://admin:123456@192.168.123.98/stream0',
    # 可达域只有对面 A2D 的 1/4，物品数要跟着降（他们是 5-10 件）
    'round_items': '4',
    'round_moves': '6',
}


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in _ARGS.items()],
        Node(
            package='record', executable='recorder', name='recorder',
            output='screen', emulate_tty=True,
            parameters=[{k: LaunchConfiguration(k) for k in _ARGS}],
        ),
    ])
