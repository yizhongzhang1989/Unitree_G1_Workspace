"""VLA 推理桥。

    ros2 launch g1_vla_bridge vla_bridge.launch.py
    #   换服务端：  server_url:=http://10.172.100.47:5509/api/inference
    #   换 VLA：     vla_backend:=<backends/ 下的模块名>
    #   显式代理：  proxy:=socks5h://127.0.0.1:1080
    #   直接带指令：task_description:='Pick up the bottled grape juice using the right arm.'

参数分两层：``config/vla_bridge.yaml`` 是与 VLA 无关的那一份，``config/backends/<name>.yaml``
是选中的那家 VLA 自己的；后者按 ``vla_backend`` 自动挂上，后面的盖前面的。

启动后**不会自己动**，要显式发令：

    ros2 service call /vla_bridge/start std_srvs/srv/Trigger
    ros2 service call /vla_bridge/stop  std_srvs/srv/Trigger

前置条件是 ``motion_control`` 已经起来且 ``~/engage`` 过（``arms_live=true``），
否则 ``~/start`` 会直接拒绝。
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 只暴露现场最常改的这几个，其余走 config/*.yaml。
_ARGUMENTS = ('vla_backend', 'server_url', 'proxy', 'task_description')


def _node(context):
    config_dir = os.path.join(
        get_package_share_directory('g1_vla_bridge'), 'config')
    common = os.path.join(config_dir, 'vla_bridge.yaml')
    # 空值不进 override：字典参数无条件盖过 yaml，不筛掉的话「不传这个 arg」
    # 会把 yaml 里写好的值清成空串。
    overrides = {}
    for name in _ARGUMENTS:
        value = LaunchConfiguration(name).perform(context)
        if value:
            overrides[name] = value

    with open(common, 'r', encoding='utf-8') as handle:
        backend = yaml.safe_load(handle)['/vla_bridge']['ros__parameters']['vla_backend']
    backend = overrides.get('vla_backend', backend)
    backend_config = os.path.join(config_dir, 'backends', f'{backend}.yaml')
    if not os.path.exists(backend_config):
        raise RuntimeError(
            f'backend {backend!r} 没有配置文件 {backend_config}；'
            '每个 backends/<名字>.py 都要配一份同名 yaml')

    return [Node(
        package='g1_vla_bridge',
        executable='vla_node',
        name='vla_bridge',
        output='screen',
        emulate_tty=True,
        # 后面的盖前面的：通用 -> 这家 VLA 的 -> 命令行。
        parameters=[common, backend_config, overrides],
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value='') for name in _ARGUMENTS),
        OpaqueFunction(function=_node),
    ])
