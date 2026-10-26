"""采集节点 + 面板 + 数据管理及内置回放。

    ros2 launch record record.launch.py                 # 采集 + 数据管理与回放
    ros2 launch record record.launch.py data:=false     # 只起采集

**录制与数据管理分开进程但一起启动**：采集必须最简最稳，数据管理要读全盘、解码取帧、
删目录，负载画像完全不同，不能塞进同一个进程。但操作者不该记两条命令，所以放在
一个 launch 里。DataManager 在自身进程内创建独立回放节点；未封口的数据不能删除或回放。

采集前置：``all_data.launch.py`` 与所需运动任务已起。真机回放前 FPC 必须已经 active，
并由操作者停下其他目标源；回放不检查目标竞争。头部相机建议用
``color_format:=YUYV`` 起（省掉一次色彩转换，720p 编码从 98% 降到 60% 单核）::

    ros2 launch head_sensors head_camera.launch.py \\
        color_profile:=1280x720x30 color_format:=YUYV
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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

_DATA_ARGS = {
    'data': 'true',
    'data_port': '8221',
    'replay_config': '',
}


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument('output_root', default_value=_ARGS['output_root']),
        DeclareLaunchArgument('dashboard_port', default_value=_ARGS['dashboard_port']),
        DeclareLaunchArgument('head_image_topic', default_value=_ARGS['head_image_topic']),
        DeclareLaunchArgument('head_camera_info_topic', default_value=_ARGS['head_camera_info_topic']),
        DeclareLaunchArgument('head_fps', default_value=_ARGS['head_fps']),
        DeclareLaunchArgument('wrist_left_url', default_value=_ARGS['wrist_left_url']),
        DeclareLaunchArgument('wrist_right_url', default_value=_ARGS['wrist_right_url']),
        DeclareLaunchArgument('round_items', default_value=_ARGS['round_items']),
        DeclareLaunchArgument('round_moves', default_value=_ARGS['round_moves']),
        DeclareLaunchArgument('data', default_value=_DATA_ARGS['data']),
        DeclareLaunchArgument('data_port', default_value=_DATA_ARGS['data_port']),
        DeclareLaunchArgument('replay_config', default_value=_DATA_ARGS['replay_config']),
    ]
    return LaunchDescription([
        *arguments,
        Node(
            package='record', executable='recorder', name='recorder',
            output='screen', emulate_tty=True,
            parameters=[{**{k: LaunchConfiguration(k) for k in _ARGS},
                         'peer_port': LaunchConfiguration('data_port')}],
        ),
        Node(
            package='record', executable='data_manager', name='data_manager',
            output='screen', emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('data')),
            parameters=[{
                'sessions_root': LaunchConfiguration('output_root'),
                'dashboard_port': LaunchConfiguration('data_port'),
                'peer_port': LaunchConfiguration('dashboard_port'),
                'replay_config': LaunchConfiguration('replay_config'),
            }],
        ),
    ])
