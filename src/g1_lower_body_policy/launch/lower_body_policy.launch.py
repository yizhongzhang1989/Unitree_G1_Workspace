"""启动下肢 ONNX 策略层。

只启这一层。控制栈要先起来：

    ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual

那条 launch 会把 forward_position_controller 加载成 inactive；本层在 ``~/engage``
的时候才去激活它，急停时再反激活。所以这里刻意不做任何 switch_controllers，
和 gravity_float_demo.launch.py 的"启动即激活"是两种模式：那个是演示，这个要人确认。
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parameters(path: Path) -> dict:
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    return next(iter(document.values()))['ros__parameters']


def _nodes(context):
    share = Path(get_package_share_directory('g1_lower_body_policy'))
    config = share / 'config' / 'lower_body_policy.yaml'
    # 31 轴的顺序只能有一个来源，就是控制器自己的参数文件；抄一份到本包的 config
    # 里，早晚会有一次改了这边忘了那边。
    controller = _parameters(
        Path(get_package_share_directory('unitree_g1_ros2_control')) /
        'config' / 'forward_position_controller.yaml')
    overrides = {'joints': controller['joints']}

    policy_path = LaunchConfiguration('policy_path').perform(context)
    if policy_path:
        overrides['policy_path'] = policy_path

    return [Node(
        package='g1_lower_body_policy',
        executable='policy_node',
        name='lower_body_policy',
        output='screen',
        parameters=[str(config), overrides],
        emulate_tty=True,
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # 留空则用 config/lower_body_policy.yaml 里的 policy_path，即本包自带的
        # config/policy.onnx。换策略时可以直接指到 logs/ 里的绝对路径试跑。
        DeclareLaunchArgument('policy_path', default_value=''),
        OpaqueFunction(function=_nodes),
    ])
