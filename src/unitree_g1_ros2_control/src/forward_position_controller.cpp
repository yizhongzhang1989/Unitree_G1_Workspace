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
    if (result != controller_interface::return_type::OK) {
        return result;
    }

    try {
        auto_declare<std::vector<std::string>>("joints", {});
        auto_declare<double>("command_timeout_s", 0.25);
        auto_declare<double>("max_initial_position_error", 0.2);
        auto_declare<double>("max_command_step", 0.1);
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
    command_timeout_s_ = get_node()->get_parameter("command_timeout_s").as_double();
    max_initial_position_error_ =
        get_node()->get_parameter("max_initial_position_error").as_double();
    max_command_step_ = get_node()->get_parameter("max_command_step").as_double();

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
    if (!std::isfinite(command_timeout_s_) || command_timeout_s_ <= 0.0 ||
        !std::isfinite(max_initial_position_error_) || max_initial_position_error_ <= 0.0 ||
        !std::isfinite(max_command_step_) || max_command_step_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "Controller safety limits must be finite and positive");
        return CallbackReturn::ERROR;
    }

    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    command_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/commands", command_qos,
        [this](std_msgs::msg::Float64MultiArray::SharedPtr message) {
            auto sample = std::make_shared<CommandSample>();
            sample->positions = message->data;
            sample->received_at = Clock::now();
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
    hold_current_state(current_positions);
    received_external_command_ = false;
    timeout_reported_ = false;
    processed_sequence_ = 0;
    last_accepted_at_ = Clock::now();
    command_buffer_.writeFromNonRT(std::shared_ptr<CommandSample>());
    return CallbackReturn::SUCCESS;
}

ForwardPositionController::CallbackReturn ForwardPositionController::on_deactivate(
    const rclcpp_lifecycle::State&) {
    command_buffer_.writeFromNonRT(std::shared_ptr<CommandSample>());
    accepted_positions_.clear();
    received_external_command_ = false;
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type ForwardPositionController::update() {
    std::vector<double> current_positions;
    if (!copy_current_state(current_positions)) {
        RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "Joint state contains a non-finite position");
        return controller_interface::return_type::OK;
    }

    const auto now = Clock::now();
    const auto command = *command_buffer_.readFromRT();
    if (!timeout_reported_ &&
        std::chrono::duration<double>(now - last_accepted_at_).count() > command_timeout_s_) {
        hold_current_state(current_positions);
        received_external_command_ = false;
        timeout_reported_ = true;
        RCLCPP_WARN(
            get_node()->get_logger(),
            "No acceptable position command before timeout; latched the current feedback pose and awaiting a fresh command");
    }

    if (!command || command->sequence == processed_sequence_) {
        return controller_interface::return_type::OK;
    }
    processed_sequence_ = command->sequence;
    if (std::chrono::duration<double>(now - command->received_at).count() > command_timeout_s_) {
        RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "Discarding stale position command");
        return controller_interface::return_type::OK;
    }
    if (command->positions.size() != command_interfaces_.size() ||
        !std::all_of(command->positions.begin(), command->positions.end(),
                     [](double value) { return std::isfinite(value); })) {
        RCLCPP_WARN(
            get_node()->get_logger(), "Discarding invalid position command: expected %zu finite values",
            command_interfaces_.size());
        return controller_interface::return_type::OK;
    }

    double largest_delta = 0.0;
    std::size_t largest_delta_index = 0;
    const auto& reference = received_external_command_ ? accepted_positions_ : current_positions;
    for (std::size_t index = 0; index < command->positions.size(); ++index) {
        const double delta = std::abs(command->positions[index] - reference[index]);
        if (delta > largest_delta) {
            largest_delta = delta;
            largest_delta_index = index;
        }
    }
    const double limit = received_external_command_ ? max_command_step_ : max_initial_position_error_;
    if (largest_delta > limit) {
        RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "Discarding position command for %s: delta %.4f rad exceeds %.4f rad",
            joint_names_[largest_delta_index].c_str(), largest_delta, limit);
        return controller_interface::return_type::OK;
    }

    for (std::size_t index = 0; index < command->positions.size(); ++index) {
        command_interfaces_[index].set_value(command->positions[index]);
    }
    accepted_positions_ = command->positions;
    received_external_command_ = true;
    last_accepted_at_ = command->received_at;
    timeout_reported_ = false;
    return controller_interface::return_type::OK;
}

bool ForwardPositionController::copy_current_state(std::vector<double>& positions) const {
    positions.clear();
    positions.reserve(state_interfaces_.size());
    for (const auto& state_interface : state_interfaces_) {
        const double value = state_interface.get_value();
        if (!std::isfinite(value)) {
            return false;
        }
        positions.push_back(value);
    }
    return true;
}

void ForwardPositionController::hold_current_state(const std::vector<double>& positions) {
    for (std::size_t index = 0; index < positions.size(); ++index) {
        command_interfaces_[index].set_value(positions[index]);
    }
}

}  // namespace unitree_g1_ros2_control

PLUGINLIB_EXPORT_CLASS(
    unitree_g1_ros2_control::ForwardPositionController,
    controller_interface::ControllerInterface)