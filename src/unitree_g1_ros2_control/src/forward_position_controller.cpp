#include "unitree_g1_ros2_control/forward_position_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/qos.hpp"

namespace unitree_g1_ros2_control {

controller_interface::return_type ForwardPositionController::init(
    const std::string& controller_name) {
    const auto result = ControllerInterface::init(controller_name);
    if (result != controller_interface::return_type::OK) return result;

    try {
        auto_declare<std::vector<std::string>>("joints", {});
    } catch (const std::exception& error) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", error.what());
        return controller_interface::return_type::ERROR;
    }
    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
ForwardPositionController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration configuration;
    configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& joint_name : joint_names_) {
        configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    }
    return configuration;
}

controller_interface::InterfaceConfiguration
ForwardPositionController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration configuration;
    configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& joint_name : joint_names_) {
        configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    }
    return configuration;
}

ForwardPositionController::CallbackReturn ForwardPositionController::on_configure(
    const rclcpp_lifecycle::State&) {
    joint_names_ = get_node()->get_parameter("joints").as_string_array();

    if (joint_names_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "The joints parameter must not be empty");
        return CallbackReturn::ERROR;
    }
    auto sorted_names = joint_names_;
    std::sort(sorted_names.begin(), sorted_names.end());
    if (std::adjacent_find(sorted_names.begin(), sorted_names.end()) != sorted_names.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "The joints parameter contains duplicate names");
        return CallbackReturn::ERROR;
    }
    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    command_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/commands", command_qos,
        [this](std_msgs::msg::Float64MultiArray::SharedPtr message) {
            auto sample = std::make_shared<CommandSample>();
            sample->positions = message->data;
            sample->sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
            command_buffer_.writeFromNonRT(std::move(sample));
        });
    command_buffer_.writeFromNonRT(std::shared_ptr<CommandSample>());
    return CallbackReturn::SUCCESS;
}

ForwardPositionController::CallbackReturn ForwardPositionController::on_activate(
    const rclcpp_lifecycle::State&) {
    if (command_interfaces_.size() != joint_names_.size() ||
        state_interfaces_.size() != joint_names_.size()) {
        RCLCPP_ERROR(
            get_node()->get_logger(), "Expected %zu command and state interfaces",
            joint_names_.size());
        return CallbackReturn::ERROR;
    }

    std::vector<double> current_positions;
    if (!copy_current_state(current_positions)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Cannot activate with non-finite joint state");
        return CallbackReturn::ERROR;
    }
    for (std::size_t index = 0; index < current_positions.size(); ++index) {
        command_interfaces_[index].set_value(current_positions[index]);
    }
    processed_sequence_ = 0;
    command_buffer_.writeFromNonRT(std::shared_ptr<CommandSample>());
    return CallbackReturn::SUCCESS;
}

ForwardPositionController::CallbackReturn ForwardPositionController::on_deactivate(
    const rclcpp_lifecycle::State&) {
    command_buffer_.writeFromNonRT(std::shared_ptr<CommandSample>());
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type ForwardPositionController::update() {
    const auto command = *command_buffer_.readFromRT();
    if (!command || command->sequence == processed_sequence_) {
        return controller_interface::return_type::OK;
    }
    processed_sequence_ = command->sequence;
    if (command->positions.size() != command_interfaces_.size() ||
        !std::all_of(command->positions.begin(), command->positions.end(),
                     [](double value) { return std::isfinite(value); })) {
        RCLCPP_WARN(
            get_node()->get_logger(), "Discarding invalid position command: expected %zu finite values",
            command_interfaces_.size());
        return controller_interface::return_type::OK;
    }

    for (std::size_t index = 0; index < command->positions.size(); ++index) {
        command_interfaces_[index].set_value(command->positions[index]);
    }
    return controller_interface::return_type::OK;
}

bool ForwardPositionController::copy_current_state(std::vector<double>& positions) const {
    positions.clear();
    positions.reserve(state_interfaces_.size());
    for (const auto& state_interface : state_interfaces_) {
        const double value = state_interface.get_value();
        if (!std::isfinite(value)) return false;
        positions.push_back(value);
    }
    return true;
}

}  // namespace unitree_g1_ros2_control

PLUGINLIB_EXPORT_CLASS(
    unitree_g1_ros2_control::ForwardPositionController,
    controller_interface::ControllerInterface)