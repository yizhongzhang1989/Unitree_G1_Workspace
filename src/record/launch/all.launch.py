"""一条命令起齐采集所需的全部进程，Ctrl-C 一起退出。

    ros2 launch record all.launch.py

分段起，每段都能单独关掉（控制栈已经在跑时就只补上缺的那几段）::

    ros2 launch record all.launch.py hardware:=false motion:=false   # 只补传感器和采集
    ros2 launch record all.launch.py teleop:=false                   # 不要 VR，用键盘
    ros2 launch record all.launch.py data:=false                     # 不起数据管理面板

子 launch 自己的参数直接往上传即可（``output_root:=/data/sessions``、``round_items:=5``
……），靠 launch configuration 继承落到对应那一段。

**为什么必须是 include 而不是各开一个终端。** ``IncludeLaunchDescription`` 把子
launch 的动作内联进**同一个 launch 进程**，Ctrl-C 于是同时送达全部子进程。这是
硬性要求，不是图省事：

* ``G1TopicSystem::stop()`` 的卸力斜坡**只挂在 SIGINT 上**。跳过它的话关节会保持
  最后一帧命令持续吃电流 —— 实测肩部绕组升到 97 °C。
* ``recorder`` 在 ``finally`` 里封口 session（等 ffmpeg 冲完缓冲、算全目录 sha256）。
  没跑完就没有 ``DONE``，那个 session 既删不掉也同步不走。

⚠️ **非交互场景要把 SIGINT 发给整个进程组，不是发给 launch 进程。** Ctrl-C 之所以能用，
是因为内核把 SIGINT 送给**前台进程组的每一个成员**；launch 收到之后只负责编排和等待，
**它不会替你转发给子进程**。只 ``kill -INT <launch pid>`` 的话子进程一个都收不到，
launch 干等满 ``sigterm_timeout`` 再升级成 SIGTERM（实测默认 5 s，调大就按调大的等）::

    kill -INT -- -"$(ps -o pgid= -p "$(pgrep -f 'record all.launch.py' | head -1)" | tr -d ' ')"

实测对比（录着 session 时打断）：进程组 SIGINT **1 s 全部退出且 session 封口**；
只发给 launch 进程要等满超时才靠 SIGTERM 收尾。

⚠️ 别用 ``(ros2 launch ... &)`` 起：子 shell 会给后台作业把 SIGINT 设成 ``SIG_IGN``，
launch 自己就聋了，组信号直接打死子进程 —— 日志里是一片 ``exit code -2``。
脚本里要后台起就先 ``set -m`` 打开作业控制。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

#: 段名 -> (包, launch 文件, 启动延迟秒)。
#:
#: 没有硬性依赖 —— ROS 2 的发现是动态的，晚到的话题照样接得上，各节点也都自带重试
#: （``localization_node`` 每帧重查外参，``point_lio`` 本身就是个订阅者）。错开只为
#: 两件事：Orin NX 上同时冷启 realsense 枚举 + point_lio + 四个 spawner 会把 CPU 打满，
#: 以及让「等不到数据」的告警不在启动阶段刷屏。
_STAGES = (
    # ros2_control + robot_state_publisher + 夹爪 + 腕相机
    ('hardware', 'robot_bringup', 'all_data.launch.py', 0.0),
    # 雷达节点 + D435i。与控制栈无依赖，并行起
    ('sensors', 'head_sensors', 'head_sensors.launch.py', 0.0),
    # 要 /joint_states 和 controller_manager
    ('motion', 'g1_motion_control', 'motion_control.launch.py', 6.0),
    # point_lio 要 head_lidar 的点云和 IMU 先在
    ('localization', 'g1_localization', 'localization.launch.py', 8.0),
    # 要 motion_control 的 ~/command
    ('teleop', 'g1_motion_control', 'vr_teleop.launch.py', 10.0),
    # 开录时要调 g1_localization 的 ~/set_origin
    ('record', 'record', 'record.launch.py', 12.0),
)

#: 只覆盖「子 launch 的默认值不适合采集」的那几个，其余一律不抄第二份 —— 抄了早晚
#: 会像以前的 vx_max / height 那样两边悄悄跑偏。没列出来的参数靠继承：命令行给了就
#: 往下传，没给就用子 launch 自己的默认值。
_OVERRIDES = {
    'hardware': ('scope', 'topology'),
    'sensors': ('color_profile', 'color_format'),
}


def generate_launch_description() -> LaunchDescription:
    stages = [
        TimerAction(
            period=delay,
            condition=IfCondition(LaunchConfiguration(name)),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory(package), 'launch', launch_file)),
                launch_arguments=[(k, LaunchConfiguration(k)) for k in _OVERRIDES.get(name, ())],
            )],
        )
        for name, package, launch_file, delay in _STAGES
    ]

    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value='true')
          for name, _, _, _ in _STAGES],

        # 收尾要多久。launch 默认 SIGINT 之后只等 5 s 就升级到 SIGTERM，而 recorder
        # 封口要等 ffmpeg 冲缓冲（最长 15 s）再算全目录 sha256（一小时素材 5.4 GB，
        # 约 11 s），控制栈的卸力斜坡另有 2 s。正常退出的进程不受这个值影响，
        # 只有真卡住的才会等满。
        DeclareLaunchArgument('sigterm_timeout', default_value='40'),
        DeclareLaunchArgument('sigkill_timeout', default_value='10'),

        DeclareLaunchArgument('scope', default_value='whole_body'),
        DeclareLaunchArgument('topology', default_value='dual'),
        # head_sensors 的默认档是 424x240 RGB8，采集要 720p YUYV：YUYV 直出省掉一次
        # 色彩转换，实测 150% -> 114% 单核且不再丢帧。
        DeclareLaunchArgument('color_profile', default_value='1280x720x30'),
        DeclareLaunchArgument('color_format', default_value='YUYV'),

        *stages,
    ])
