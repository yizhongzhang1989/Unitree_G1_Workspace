from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("port", default_value="8310"),
        DeclareLaunchArgument("lowstate_topic", default_value="/lowstate"),
        DeclareLaunchArgument("imu_topic", default_value="/secondary_imu"),
        DeclareLaunchArgument("lowcmd_topic", default_value="/lowcmd"),
        DeclareLaunchArgument("allow_torque_output", default_value="false"),
        # 早期参数文件按整份 URDF 做摘要，连改渲染颜色都会让它认不出模型，
        # 而那种摘要又没法反推出动力学部分变没变，只能人工接受一次。
        # 接受后写入的是模型摘要，以后再改渲染就不会再撞。
        DeclareLaunchArgument("rebind_urdf", default_value="false"),
        # 采样前退开多远再回来。必须大于死区 2*tau_s/kp（本臂实测最大
        # 0.107 rad），否则两侧停在同一点，平均不掉摩擦。0 = 回到单向采样。
        DeclareLaunchArgument("approach_offset_rad", default_value="0.12"),
        DeclareLaunchArgument(
            "urdf_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("unitree_g1_description"),
                "model", "final.urdf",
            ])),
        DeclareLaunchArgument(
            "parameter_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("arm_gravity_compensation"),
                "config", "parameters.json",
            ])),
        DeclareLaunchArgument(
            "calibrated_urdf",
            default_value=PathJoinSubstitution([
                FindPackageShare("arm_gravity_compensation"),
                "config", "calibrated.urdf",
            ])),
        DeclareLaunchArgument(
            "gravity_table",
            default_value=PathJoinSubstitution([
                FindPackageShare("arm_gravity_compensation"),
                "config", "gravity_table.yaml",
            ])),
    ]
    node = Node(
        package="arm_gravity_compensation",
        executable="gravity_calibration",
        name="arm_gravity_compensation",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("port"),
            "lowstate_topic": LaunchConfiguration("lowstate_topic"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "lowcmd_topic": LaunchConfiguration("lowcmd_topic"),
            "allow_torque_output": LaunchConfiguration("allow_torque_output"),
            "rebind_urdf": LaunchConfiguration("rebind_urdf"),
            "approach_offset_rad": LaunchConfiguration("approach_offset_rad"),
            "urdf_path": LaunchConfiguration("urdf_path"),
            "parameter_file": LaunchConfiguration("parameter_file"),
            "calibrated_urdf": LaunchConfiguration("calibrated_urdf"),
            "gravity_table": LaunchConfiguration("gravity_table"),
        }],
    )
    return LaunchDescription([*arguments, node])