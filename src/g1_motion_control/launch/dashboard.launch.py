"""只读监控页：双臂末端目标 + 29 个本体关节状态。

    ros2 launch g1_motion_control dashboard.launch.py
    #   换端口：    bind_port:=8182
    #   只收本机：  bind_host:=127.0.0.1
    #   换参考系：  base_frame:=torso_link   （必须和策略层的 base_frame 一致）

浏览器打开 ``http://<机器人IP>:8181/``。绿色实心球是统一命令总线的上层末端目标，
黄色菱形是 IK + 关节限速后的末端指令，橙色空心环绑定在实测模型末端；左下
可勾选 29 个本体关节，查看 ``/lowstate`` 角度曲线、状态码和两路温度。

这是个**独立的只读进程**，不在控制链路上：不发任何指令、不调任何服务，不开就是
零开销。控制栈没起来时页面会一直等 ``/robot_description``，不影响别的东西。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_ARGUMENTS = {
    'bind_host': '0.0.0.0',
    'bind_port': '8181',
    # 末端位姿目标就是相对它发布的，改这里必须同步改 motion_control.yaml。
    'base_frame': 'torso_link',
    'command_topic': '/motion_control/command',
    'status_topic': '/motion_control/status',
    'joint_states_topic': '/joint_states',
    'lowstate_topic': '/lowstate',
    'robot_description_topic': '/robot_description',
}


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value)
          for name, value in _ARGUMENTS.items()),
        Node(
            package='g1_motion_control',
            executable='dashboard_node',
            name='motion_control_dashboard',
            output='screen',
            parameters=[{name: LaunchConfiguration(name) for name in _ARGUMENTS}],
        ),
    ])
