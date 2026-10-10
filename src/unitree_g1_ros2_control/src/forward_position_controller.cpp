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
        AdaptiveStiffness::declare_parameters(get_node());
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
    stiffness_.append_interfaces(configuration.names);
    return configuration;
}

controller_interface::InterfaceConfiguration
ForwardPositionController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration configuration;
    configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& joint_name : joint_names_) {
        configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    }
    gravity_.append_state_interfaces(configuration.names);
    // Last: the gain interfaces are located by counting back from the end.
    stiffness_.append_interfaces(configuration.names);
    return configuration;
}

ForwardPositionController::CallbackReturn ForwardPositionController::on_configure(
    const rclcpp_lifecycle::State&) {
    joint_names_ = get_node()->get_parameter("joints").as_string_array();

    if (joint_names_.empty() || joint_names_.size() > 31) {
        RCLCPP_ERROR(get_node()->get_logger(), "The joints parameter must contain 1 to 31 joints");
        return CallbackReturn::ERROR;
    }
    auto sorted_names = joint_names_;
    std::sort(sorted_names.begin(), sorted_names.end());
    if (std::adjacent_find(sorted_names.begin(), sorted_names.end()) != sorted_names.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "The joints parameter contains duplicate names");
        return CallbackReturn::ERROR;
    }
    if (!gravity_.configure(get_node(), joint_names_)) return CallbackReturn::ERROR;
    // Only the joints the gravity table binds have the extra gain interfaces,
    // and it reports them in the order both sides index.
    if (!stiffness_.configure(get_node(), joint_names_, gravity_.compensated_indices())) {
        return CallbackReturn::ERROR;
    }

    target_.assign(joint_names_.size(), 0.0);
    measured_.assign(joint_names_.size(), 0.0);
    command_.assign(joint_names_.size(), 0.0);
    target_valid_ = false;
    joint_mask_ = (std::uint32_t{1} << joint_names_.size()) - 1;

    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    command_subscription_ = get_node()->create_subscription<msg::JointTargets>(
        "~/joint_targets", rclcpp::QoS(rclcpp::KeepLast(4)).best_effort(),
        [this](msg::JointTargets::SharedPtr message) {
            receive_command(message->mask, message->positions.data());
        });
    full_command_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/commands", command_qos,
        [this](std_msgs::msg::Float64MultiArray::SharedPtr message) {
            if (message->data.size() == joint_names_.size()) {
                receive_command(joint_mask_, message->data.data());
            }
        });
    command_buffer_.writeFromNonRT(CommandSample{});
    return CallbackReturn::SUCCESS;
}

ForwardPositionController::CallbackReturn ForwardPositionController::on_activate(
    const rclcpp_lifecycle::State&) {
    const std::size_t expected_states = joint_names_.size() +
                                        gravity_.state_interface_count() +
                                        stiffness_.interface_count();
    const std::size_t expected_commands = joint_names_.size() + stiffness_.interface_count();
    if (command_interfaces_.size() != expected_commands ||
        state_interfaces_.size() != expected_states) {
        RCLCPP_ERROR(
            get_node()->get_logger(), "Expected %zu command and %zu state interfaces",
            expected_commands, expected_states);
        return CallbackReturn::ERROR;
    }
    if (!gravity_.activate(state_interfaces_, get_node()->get_logger()) ||
        !stiffness_.activate(
            state_interfaces_, command_interfaces_, get_node()->get_logger())) {
        return CallbackReturn::ERROR;
    }
    stiffness_.reset();
    std::lock_guard<std::mutex> lock(command_mutex_);
    // Start from where the joints physically are, so activation cannot jump.
    for (std::size_t index = 0; index < joint_names_.size(); ++index) {
        const double position = state_interfaces_[index].get_value();
        if (!std::isfinite(position)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Cannot activate with non-finite joint state");
            return CallbackReturn::ERROR;
        }
        command_interfaces_[index].set_value(position);
        pending_command_.positions[index] = position;
    }
    processed_sequence_ = 0;
    target_valid_ = false;
    pending_command_.sequence = 0;
    command_buffer_.writeFromNonRT(pending_command_);
    return CallbackReturn::SUCCESS;
}

ForwardPositionController::CallbackReturn ForwardPositionController::on_deactivate(
    const rclcpp_lifecycle::State&) {
    target_valid_ = false;
    gravity_.reset();
    stiffness_.reset();
    std::lock_guard<std::mutex> lock(command_mutex_);
    pending_command_.sequence = 0;
    command_buffer_.writeFromNonRT(pending_command_);
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type ForwardPositionController::update(
    const rclcpp::Time&, const rclcpp::Duration& period) {
    const auto* sample = command_buffer_.readFromRT();
    if (sample->sequence != processed_sequence_) {
        processed_sequence_ = sample->sequence;
        std::copy_n(sample->positions.begin(), target_.size(), target_.begin());
        target_valid_ = true;
    }
    if (!target_valid_) return controller_interface::return_type::OK;

    // Rewritten every cycle rather than only on a new target: the offset has to
    // keep following the torso attitude and the activation ramp even while the
    // target stands still, and dropping the offset has to take effect at once
    // instead of waiting for the next setpoint.
    // This controller's state interfaces are one position per joint, in order.
    for (std::size_t index = 0; index < measured_.size(); ++index) {
        measured_[index] = state_interfaces_[index].get_value();
    }
    // Before the feed-forward, which sizes its offset against the gains the
    // motor will really be given.
    const auto& stiffness = stiffness_.update(
        target_, measured_, state_interfaces_, command_interfaces_, period.seconds());
    write_command(
        gravity_.apply(state_interfaces_, period, target_, measured_, stiffness, command_)
            ? command_
            : target_);
    return controller_interface::return_type::OK;
}

void ForwardPositionController::receive_command(std::uint32_t mask, const double* positions) {
    mask &= joint_mask_;
    if (!mask) return;
    for (auto bits = mask; bits; bits &= bits - 1) {
        if (!std::isfinite(positions[__builtin_ctz(bits)])) {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                                 "Discarding invalid position command");
            return;
        }
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    for (auto bits = mask; bits; bits &= bits - 1) {
        const auto index = __builtin_ctz(bits);
        pending_command_.positions[index] = positions[index];
    }
    ++pending_command_.sequence;
    command_buffer_.writeFromNonRT(pending_command_);
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