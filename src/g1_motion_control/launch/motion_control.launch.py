"""启动整机运动控制层。

只启这一层。控制栈要先起来：

    ros2 launch robot_bringup all_data.launch.py scope:=whole_body topology:=dual

那条 launch 会把 forward_position_controller 加载成 inactive；本层在 ``~/engage``
的时候才去激活它，急停时再反激活。所以这里刻意不做任何 switch_controllers，
和 gravity_float_demo.launch.py 的"启动即激活"是两种模式：那个是演示，这个要人确认。

``arm_mode`` 选上肢怎么接指令，下肢策略不受影响：

    arm_mode:=ik           （默认）臂块是末端位姿，VR / VLA 走这个
    arm_mode:=passthrough  臂块就是 14 个关节目标，键盘的手臂浮动靠它
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


# IK 解的是 6x6 / 7x7 的小矩阵，OpenBLAS 多线程在这个尺度上是纯开销：每次
# np.linalg.solve 都要调度整个线程池，一次 IK 有 20 次 solve。实测机器有负载时
# 跑满 10 次迭代要 8.29 ms，锁成单线程只要 2.58 ms（3.2 倍），而且省下 8 个
# 自旋线程——它们本来还在和 100 Hz 的状态回调抢 CPU。
_SINGLE_THREADED_BLAS = {'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}


def _nodes(context):
    share = Path(get_package_share_directory('g1_motion_control'))
    config = share / 'config' / 'motion_control.yaml'
    # 31 轴的顺序只能有一个来源，就是控制栈的公共参数文件；抄一份到本包的 config
    # 里，早晚会有一次改了这边忘了那边。
    common = _parameters(
        Path(get_package_share_directory('unitree_g1_ros2_control')) /
        'config' / 'default_31dof_param.yaml')
    overrides = {'joints': common['joints']}

    policy_path = LaunchConfiguration('policy_path').perform(context)
    if policy_path:
        overrides['policy_path'] = policy_path

    overrides['arm_mode'] = LaunchConfiguration('arm_mode').perform(context)

    return [Node(
        package='g1_motion_control',
        executable='policy_node',
        name='motion_control',
        output='screen',
        parameters=[str(config), overrides],
        additional_env=_SINGLE_THREADED_BLAS,
        emulate_tty=True,
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # 留空则用 config/motion_control.yaml 里的 policy_path，即本包自带的
        # config/policy.onnx。换策略时可以直接指到 logs/ 里的绝对路径试跑。
        DeclareLaunchArgument('policy_path', default_value=''),
        # ik：臂块是末端位姿，节点解 IK。passthrough：臂块就是 14 个关节目标。
        DeclareLaunchArgument('arm_mode', default_value='ik',
                             choices=['ik', 'passthrough']),
        OpaqueFunction(function=_nodes),
    ])
