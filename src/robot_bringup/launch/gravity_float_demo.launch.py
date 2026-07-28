"""Activate gravity compensation together with FPC and let the arms float.

`all_data.launch.py scope:=whole_body` already loads both controllers but leaves
them inactive. This entry point only flips them on and starts the demo node, so
it must be launched next to a running control stack, never instead of one.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


CONTROLLERS = ["arm_gravity_compensation", "forward_position_controller"]


def _parameters(path: Path) -> dict:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    return next(iter(document.values()))["ros__parameters"]


def _resolve(reference: str) -> Path:
    """Accept the same ``package://`` form the controller resolves in C++."""
    prefix = "package://"
    if not reference.startswith(prefix):
        return Path(reference).expanduser()
    package, _, relative = reference[len(prefix):].partition("/")
    return Path(get_package_share_directory(package)) / relative


def generate_launch_description() -> LaunchDescription:
    # The demo target must use exactly the controller's joint order, so read it
    # from the same file the controller is configured with.
    controller = _parameters(
        Path(get_package_share_directory("unitree_g1_ros2_control")) /
        "config" / "arm_gravity_compensation.yaml")
    joints = controller["joints"]
    # Only the joints the controller actually compensates may float; anything
    # else would be commanded to its own measurement and go limp.
    table = _parameters(_resolve(controller["gravity_table"]))
    floating = [name for side in ("left", "right") for name in table[side]["joints"]]

    activate = ExecuteProcess(
        cmd=["ros2", "control", "switch_controllers",
             "--controller-manager", LaunchConfiguration("controller_manager"),
             "--start", *CONTROLLERS],
        output="screen",
    )
    demo = Node(
        package="robot_bringup",
        executable="gravity_float_demo",
        name="gravity_float_demo",
        output="screen",
        parameters=[{
            "joints": joints,
            "floating_joints": floating,
            "publish_rate_hz": ParameterValue(LaunchConfiguration("publish_rate_hz"), value_type=float),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "controller_manager", default_value="/controller_manager"),
        DeclareLaunchArgument("publish_rate_hz", default_value="100.0"),
        activate,
        # Only start streaming targets once both controllers are actually
        # active, otherwise the first samples are dropped.
        RegisterEventHandler(
            OnProcessExit(target_action=activate, on_exit=[demo])),
    ])
