"""双总线接线：一个力传感器 + 一个夹爪为一组（一个手臂），分别接两条总线。

CANalyst-II 同时用 CAN1+CAN2（一个 bridge 进程独占设备、桥接两通道）：
  - 臂0 在 /can0（CAN1）：力传感器 + 夹爪
  - 臂1 在 /can1（CAN2）：力传感器 + 夹爪
不同总线相互独立，两臂设备的 CAN ID **可以相同**（无需改 ID）。
设备清单同时生成 bridge 的 CAN ID 路由和各设备节点参数。

    ros2 launch robot_bringup end_effectors_dual_bus.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from robot_bringup.end_effectors.nodes import end_effector_actions
from robot_bringup.end_effectors.topology import deployed_topology


def generate_launch_description() -> LaunchDescription:
    buses, kwr57_devices, gloria_devices = deployed_topology("dual")

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
