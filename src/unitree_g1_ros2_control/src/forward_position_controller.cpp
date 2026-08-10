#include "unitree_g1_ros2_control/forward_position_controller.hpp"

#include <algorithm>
#include <array>
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

controller_interface::CallbackReturn ForwardPositionController::on_init() {
    try {
        auto_declare<std::vector<std::string>>("joints", {});
        GravityFeedforward::declare_parameters(get_node());
    } catch (const std::exception& error) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", error.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
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
    gravity_.append_state_interfaces(joint_names_, configuration.names);
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
    if (!gravity_.configure(get_node(), joint_names_)) return CallbackReturn::ERROR;

    target_.assign(joint_names_.size(), 0.0);
    command_.assign(joint_names_.size(), 0.0);
    target_valid_ = false;

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
    const std::size_t expected_states = joint_names_.size() + gravity_.state_interface_count();
    if (command_interfaces_.size() != joint_names_.size() || state_interfaces_.size() != expected_states) {
        RCLCPP_ERROR(
            get_node()->get_logger(), "Expected %zu command and %zu state interfaces",
            joint_names_.size(), expected_states);
        return CallbackReturn::ERROR;
    }
    if (!gravity_.activate(state_interfaces_, get_node()->get_logger())) {
        return CallbackReturn::ERROR;
    }
    // Start from where the joints physically are, so activation cannot jump.
    for (std::size_t index = 0; index < joint_names_.size(); ++index) {
        const double position = state_interfaces_[index].get_value();
        if (!std::isfinite(position)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Cannot activate with non-finite joint state");
            return CallbackReturn::ERROR;
        }
        command_interfaces_[index].set_value(position);
    }
    processed_sequence_ = 0;
    target_valid_ = false;
    command_buffer_.writeFromNonRT(std::shared_ptr<CommandSample>());
    return CallbackReturn::SUCCESS;
}

ForwardPositionController::CallbackReturn ForwardPositionController::on_deactivate(
    const rclcpp_lifecycle::State&) {
    target_valid_ = false;
    gravity_.reset();
    command_buffer_.writeFromNonRT(std::shared_ptr<CommandSample>());
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type ForwardPositionController::update(
    const rclcpp::Time&, const rclcpp::Duration& period) {
    const auto sample = *command_buffer_.readFromRT();
    if (sample && sample->sequence != processed_sequence_) {
        processed_sequence_ = sample->sequence;
        if (sample->positions.size() != command_interfaces_.size() ||
            !std::all_of(sample->positions.begin(), sample->positions.end(),
                         [](double value) { return std::isfinite(value); })) {
            RCLCPP_WARN(
                get_node()->get_logger(),
                "Discarding invalid position command: expected %zu finite values",
                command_interfaces_.size());
        } else {
            target_ = sample->positions;
            target_valid_ = true;
        }
    }
    if (!target_valid_) return controller_interface::return_type::OK;

    // Rewritten every cycle rather than only on a new target: the offset has to
    // keep following the torso attitude and the activation ramp even while the
    // target stands still, and dropping the offset has to take effect at once
    // instead of waiting for the next setpoint.
    write_command(gravity_.apply(state_interfaces_, period, target_, command_) ? command_ : target_);
    return controller_interface::return_type::OK;
}

void ForwardPositionController::write_command(const std::vector<double>& positions) {
    for (std::size_t index = 0; index < positions.size(); ++index) {
        command_interfaces_[index].set_value(positions[index]);
    }
}

}  // namespace unitree_g1_ros2_control

PLUGINLIB_EXPORT_CLASS(
    unitree_g1_ros2_control::ForwardPositionController,
    controller_interface::ControllerInterface)