"""双总线接线：一个力传感器 + 一个夹爪为一组（一个手臂），分别接两条总线。

CANalyst-II 同时用 CAN1+CAN2（一个 bridge 进程独占设备、桥接两通道）：
  - 臂0 在 /can0（CAN1）：力传感器 + 夹爪
  - 臂1 在 /can1（CAN2）：力传感器 + 夹爪
不同总线相互独立，两臂设备的 CAN ID **可以相同**（无需改 ID）。

  ros2 launch robot_bringup dual_bus.launch.py
"""

from launch import LaunchDescription

from robot_bringup.nodes import bridge, ft_sensor, gripper


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        bridge("dual_bus.yaml"),
        # --- 臂0：/can0 ---
        ft_sensor("ft_arm0", "can0", cmd_id=0x10, data_base_id=0x15,
                  topic="/arm0/wrench_raw", frame_id="arm0_ft_link"),
        gripper("grip_arm0", "can0", command_id=0x01, feedback_id=0x101,
                joint_name="arm0_gripper"),
        # --- 臂1：/can1（不同总线，CAN ID 可与臂0相同）---
        ft_sensor("ft_arm1", "can1", cmd_id=0x10, data_base_id=0x15,
                  topic="/arm1/wrench_raw", frame_id="arm1_ft_link"),
        gripper("grip_arm1", "can1", command_id=0x01, feedback_id=0x101,
                joint_name="arm1_gripper"),
    ])
