#include "unitree_g1_ros2_control/gravity_joint_trajectory_controller.hpp"

#include <cstddef>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace unitree_g1_ros2_control {

controller_interface::CallbackReturn GravityJointTrajectoryController::on_init() {
    const auto result = JointTrajectoryController::on_init();
    if (result != controller_interface::CallbackReturn::SUCCESS) return result;
    try {
        GravityFeedforward::declare_parameters(get_node());
    } catch (const std::exception& error) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", error.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
GravityJointTrajectoryController::state_interface_configuration() const {
    // The base class matches its own interfaces by name, so the extra ones are
    // simply ignored by it.
    auto configuration = JointTrajectoryController::state_interface_configuration();
    gravity_.append_state_interfaces(params_.joints, configuration.names);
    return configuration;
}

controller_interface::CallbackReturn GravityJointTrajectoryController::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
    const auto result = JointTrajectoryController::on_configure(previous_state);
    if (result != controller_interface::CallbackReturn::SUCCESS) return result;

    if (!gravity_.configure(get_node(), params_.joints)) {
        return controller_interface::CallbackReturn::ERROR;
    }
    if (gravity_.loaded()) {
        // The offset is written through the position command interfaces, in the
        // command joint order, so both lists have to describe the same joints.
        if (command_joint_names_ != params_.joints) {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "gravity_table needs command_joints to match joints");
            return controller_interface::CallbackReturn::ERROR;
        }
        // Open loop control seeds the base class from the command interfaces,
        // which by then already carry our offset: it would fold back in.
        if (params_.open_loop_control) {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "gravity_table cannot be combined with open_loop_control");
            return controller_interface::CallbackReturn::ERROR;
        }
    }
    target_.assign(params_.joints.size(), 0.0);
    command_.assign(params_.joints.size(), 0.0);
    target_valid_ = false;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityJointTrajectoryController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
    const auto result = JointTrajectoryController::on_activate(previous_state);
    if (result != controller_interface::CallbackReturn::SUCCESS) return result;

    if (gravity_.loaded() && !has_position_command_interface_) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "gravity_table needs a position command interface to apply its offset");
        return controller_interface::CallbackReturn::ERROR;
    }
    // The base class claimed its own interfaces first, so ours are the trailing
    // ones it never looked at.
    if (!gravity_.activate(state_interfaces_, get_node()->get_logger())) {
        return controller_interface::CallbackReturn::ERROR;
    }
    // Nothing has been commanded yet: until the base class writes a setpoint,
    // the interfaces keep whatever the previous controller left there and must
    // not be touched, or the hand-over would step by a whole offset.
    target_valid_ = false;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityJointTrajectoryController::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
    target_valid_ = false;
    gravity_.reset();
    return JointTrajectoryController::on_deactivate(previous_state);
}

controller_interface::return_type GravityJointTrajectoryController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
    if (!gravity_.loaded()) return JointTrajectoryController::update(time, period);

    // Slot 0 is position: the base class orders them by allowed_interface_types_.
    auto& commands = joint_command_interface_[0];
    // Hand the base class back the bare setpoints it wrote last cycle. It never
    // reads them, but it does leave them untouched while holding position, and
    // reading a compensated value back as a new target would add the offset a
    // second time on every cycle.
    if (target_valid_) {
        for (std::size_t index = 0; index < commands.size(); ++index) {
            commands[index].get().set_value(target_[index]);
        }
    }

    const auto result = JointTrajectoryController::update(time, period);

    // Whatever stands in the interfaces now is a bare setpoint: either the one
    // just written by the base class, or the one restored above.
    if (has_active_trajectory()) {
        for (std::size_t index = 0; index < commands.size(); ++index) {
            target_[index] = commands[index].get().get_value();
        }
        target_valid_ = true;
    }
    if (!target_valid_) return result;

    if (!gravity_.apply(state_interfaces_, period, target_, command_)) return result;
    for (std::size_t index = 0; index < commands.size(); ++index) {
        commands[index].get().set_value(command_[index]);
    }
    return result;
}

}  // namespace unitree_g1_ros2_control

PLUGINLIB_EXPORT_CLASS(
    unitree_g1_ros2_control::GravityJointTrajectoryController,
    controller_interface::ControllerInterface)
