"""Activate the position controller and let the arms float weightlessly.

`all_data.launch.py scope:=whole_body` already loads the controller but leaves
it inactive. This entry point only flips it on and starts the demo node, so it
must be launched next to a running control stack, never instead of one.

Shutdown is symmetric: the controller goes back to inactive and the robot is
faded out and handed back to `ai`. Pass `release_on_exit:=false` to leave the
robot in low-level mode -- it then keeps holding the last frame and heats up.
"""

import subprocess
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


CONTROLLERS = ["forward_position_controller"]


def _parameters(path: Path, node: str | None = None) -> dict:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    parameters = document[node] if node else next(iter(document.values()))
    return parameters["ros__parameters"]


def _resolve(reference: str) -> Path:
    """Accept the same ``package://`` form the controller resolves in C++."""
    prefix = "package://"
    if not reference.startswith(prefix):
        return Path(reference).expanduser()
    package, _, relative = reference[len(prefix):].partition("/")
    return Path(get_package_share_directory(package)) / relative


def _release(context, *args, **kwargs) -> list:
    """Stop the controller and wind the holding torque down to zero.

    The firmware has no watchdog, so the last frame the controller wrote keeps
    drawing current after the demo stops -- an arm left raised overheats. The
    controller has to be deactivated first, otherwise its 1 kHz stream restores
    the gains the fade-out is trying to remove.

    Run inline rather than as launch actions: a process spawned while launch is
    already shutting down is not reliably waited for.
    """
    manager = LaunchConfiguration("controller_manager").perform(context)
    commands = (
        ["ros2", "control", "switch_controllers",
         "--controller-manager", manager, "--deactivate", *CONTROLLERS],
        ["ros2", "run", "robot_bringup", "exit_debug_mode"],
    )
    for command in commands:
        try:
            # Own session: a second Ctrl-C reaches the terminal's process group
            # and would abort the fade-out half way, leaving torque on.
            subprocess.run(command, timeout=60.0, check=False,
                           start_new_session=True)
        except (subprocess.TimeoutExpired, OSError) as error:
            print("Release step failed (%s): %s" % (error, " ".join(command)))
    return []


def generate_launch_description() -> LaunchDescription:
    # The demo target must use exactly the controller's canonical joint order.
    common = _parameters(
        Path(get_package_share_directory("unitree_g1_ros2_control")) /
        "config" / "default_31dof_param.yaml", "/**")
    controller = _parameters(
        Path(get_package_share_directory("unitree_g1_ros2_control")) /
        "config" / "forward_position_controller.yaml")
    joints = common["joints"]
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
    release = OpaqueFunction(
        function=_release,
        condition=IfCondition(LaunchConfiguration("release_on_exit")),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "controller_manager", default_value="/controller_manager"),
        DeclareLaunchArgument("publish_rate_hz", default_value="100.0"),
        DeclareLaunchArgument(
            "release_on_exit", default_value="true",
            description="when the demo stops, deactivate the controller, fade "
                        "the joints out and hand the robot back to 'ai'"),
        activate,
        # Only start streaming targets once the controller is actually active,
        # otherwise the first samples are dropped.
        RegisterEventHandler(
            OnProcessExit(target_action=activate, on_exit=[demo])),
        RegisterEventHandler(
            OnProcessExit(target_action=demo, on_exit=[release])),
    ])
