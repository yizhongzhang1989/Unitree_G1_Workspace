from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import SwitchController

from robot_bringup.dashboard_compat_node import (
    install_controller_manager_field_aliases,
)


def test_installs_foxy_controller_manager_field_aliases():
    install_controller_manager_field_aliases()
    install_controller_manager_field_aliases()

    request = SwitchController.Request()
    request.activate_controllers = ["whole_body_controller"]
    request.deactivate_controllers = ["old_controller"]
    request.activate_asap = True

    assert request.start_controllers == ["whole_body_controller"]
    assert request.stop_controllers == ["old_controller"]
    assert request.start_asap is True

    state = ControllerState()
    state.claimed_interfaces = ["left_hip_pitch_joint/position"]
    assert state.required_command_interfaces == [
        "left_hip_pitch_joint/position"]