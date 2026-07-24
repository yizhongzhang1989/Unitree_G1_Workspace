"""单总线接线：所有设备都在 CANalyst-II 的 CAN1（/can0）。

一个 bridge + 2 力传感器 + 2 夹爪，全部在 /can0。
设备清单同时生成 bridge 的 CAN ID 路由和各设备节点参数。
⚠️ 同一条总线上的非共享活动 CAN ID 必须互不冲突：
  - 力传感器用 examples/set_id.py 改（ft_left 0x15、ft_right 0x18）；
  - 夹爪用其上位机/手册改（grip_left 0x01/0x101、grip_right 0x02/0x102）。
        夹爪可共享状态兼容 ID 0x000，但 command_id 低 4 位必须不同；0x7FF 为固定请求 ID。

    ros2 launch robot_bringup end_effectors_single_bus.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from robot_bringup.end_effectors.nodes import end_effector_actions
from robot_bringup.end_effectors.topology import deployed_topology


def generate_launch_description() -> LaunchDescription:
    buses, kwr57_devices, gloria_devices = deployed_topology("single")

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_grippers_on_start", default_value="true"),
        *end_effector_actions(
            buses,
            kwr57_devices,
            gloria_devices,
            LaunchConfiguration("enable_grippers_on_start"),
        ),
    ])
