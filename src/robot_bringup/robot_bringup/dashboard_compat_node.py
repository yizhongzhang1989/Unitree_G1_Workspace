#!/usr/bin/env python3
"""Run robot_test_dashboard with ROS 2 controller-manager field aliases."""

import importlib
from typing import Optional, Sequence

from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import SwitchController


def _forwarding_property(field_name: str) -> property:
    return property(
        lambda request: getattr(request, field_name),
        lambda request, value: setattr(request, field_name, value),
    )


def install_controller_manager_field_aliases() -> None:
    """Expose post-Foxy controller-manager names on Foxy message objects."""
    request_type = SwitchController.Request
    request = request_type()
    aliases = (
        ("activate_controllers", "start_controllers"),
        ("deactivate_controllers", "stop_controllers"),
        ("activate_asap", "start_asap"),
    )
    for alias, field_name in aliases:
        if hasattr(request, alias):
            continue
        if not hasattr(request, field_name):
            raise RuntimeError(
                f"SwitchController.Request provides neither {alias!r} "
                f"nor {field_name!r}")
        setattr(request_type, alias, _forwarding_property(field_name))

    state_type = ControllerState
    state = state_type()
    if not hasattr(state, "required_command_interfaces"):
        if not hasattr(state, "claimed_interfaces"):
            raise RuntimeError(
                "ControllerState provides neither "
                "'required_command_interfaces' nor 'claimed_interfaces'")
        setattr(
            state_type,
            "required_command_interfaces",
            _forwarding_property("claimed_interfaces"),
        )


def main(args: Optional[Sequence[str]] = None) -> None:
    install_controller_manager_field_aliases()
    dashboard_main = importlib.import_module(
        "robot_test_dashboard.dashboard_node").main
    dashboard_main(args=args)


if __name__ == "__main__":
    main()