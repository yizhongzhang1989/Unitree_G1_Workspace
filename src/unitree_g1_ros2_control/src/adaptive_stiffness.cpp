#include "unitree_g1_ros2_control/adaptive_stiffness.hpp"

#include <algorithm>
#include <cmath>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter.hpp"

namespace unitree_g1_ros2_control {

namespace {

/// `ros2 param set ... adaptive_stiffness_b 1` arrives as an integer, and
/// as_double() on one of those throws out of the parameter service and takes
/// the process with it.
double as_number(const rclcpp::Parameter& parameter) {
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        return static_cast<double>(parameter.as_int());
    }
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return parameter.as_double();
    }
    return std::nan("");
}

// pow() is around fifty multiplies, and this runs 14 times a cycle at 500 Hz.
double raised(double value, double power) {
    if (power == 1.0) return value;
    if (power == 2.0) return value * value;
    return std::pow(value, power);
}

}  // namespace

void AdaptiveStiffness::declare_parameters(const LifecycleNode::SharedPtr& node) {
    const std::pair<const char*, double> defaults[] = {
        {"adaptive_stiffness_scale", 0.0},
        {"adaptive_stiffness_b", 0.05},
        {"adaptive_stiffness_power", 1.0},
    };
    for (const auto& [name, value] : defaults) {
        if (!node->has_parameter(name)) node->declare_parameter<double>(name, value);
    }
}

bool AdaptiveStiffness::configure(
    const LifecycleNode::SharedPtr& node, const std::vector<std::string>& joint_names,
    const std::vector<std::size_t>& indices) {
    indices_ = indices;
    stiffness_.assign(indices_.size(), 0.0);
    names_.clear();
    names_.reserve(2 * indices_.size());
    for (const std::size_t index : indices_) names_.push_back(joint_names[index] + "/kp");
    for (const std::size_t index : indices_) names_.push_back(joint_names[index] + "/kd");

    ramp_duration_ = node->get_parameter("offset_ramp_s").as_double();
    ramp_ = 0.0;
    for (const auto& parameter : node->get_parameters({"adaptive_stiffness_scale",
                                                       "adaptive_stiffness_b",
                                                       "adaptive_stiffness_power"})) {
        std::string reason;
        if (!store_parameter(parameter, reason)) {
            RCLCPP_ERROR(node->get_logger(), "%s", reason.c_str());
            return false;
        }
    }
    parameter_callback_ = node->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto& parameter : parameters) {
                if (!store_parameter(parameter, result.reason)) result.successful = false;
            }
            return result;
        });
    return true;
}

bool AdaptiveStiffness::store_parameter(
    const rclcpp::Parameter& parameter, std::string& reason) {
    // b is a divisor and power an exponent, so zero is illegal for those two
    // even though it is exactly what turns the scale off.
    const std::tuple<const char*, std::atomic<double>*, bool> slots[] = {
        {"adaptive_stiffness_scale", &scale_, false},
        {"adaptive_stiffness_b", &b_, true},
        {"adaptive_stiffness_power", &power_, true},
    };
    for (const auto& [name, slot, positive] : slots) {
        if (parameter.get_name() != name) continue;
        const double value = as_number(parameter);
        if (!std::isfinite(value) || value < 0.0 || (positive && value == 0.0)) {
            reason = parameter.get_name() +
                     (positive ? " must be finite and positive"
                               : " must be finite and non-negative");
            return false;
        }
        slot->store(value, std::memory_order_relaxed);
        return true;
    }
    return true;
}

double AdaptiveStiffness::stiffness_scale(double position_error) const {
    const double amount = scale_.load(std::memory_order_relaxed);
    if (amount <= 0.0) return 1.0;
    const double ratio = std::abs(position_error) / b_.load(std::memory_order_relaxed);
    return 1.0 + amount / (raised(ratio, power_.load(std::memory_order_relaxed)) + 1.0);
}

void AdaptiveStiffness::append_interfaces(std::vector<std::string>& names) const {
    names.insert(names.end(), names_.begin(), names_.end());
}

bool AdaptiveStiffness::activate(
    const std::vector<hardware_interface::LoanedStateInterface>& states,
    const std::vector<hardware_interface::LoanedCommandInterface>& commands,
    const rclcpp::Logger& logger) const {
    if (states.size() < names_.size() || commands.size() < names_.size()) {
        RCLCPP_ERROR(
            logger, "adaptive stiffness needs %zu interfaces in each direction",
            names_.size());
        return false;
    }
    const std::size_t table = states.size() - names_.size();
    const std::size_t command = commands.size() - names_.size();
    for (std::size_t slot = 0; slot < names_.size(); ++slot) {
        if (states[table + slot].get_name() == names_[slot] &&
            commands[command + slot].get_name() == names_[slot]) {
            continue;
        }
        RCLCPP_ERROR(
            logger, "expected %s in the trailing interfaces, found %s and %s",
            names_[slot].c_str(), states[table + slot].get_name().c_str(),
            commands[command + slot].get_name().c_str());
        return false;
    }
    return true;
}

const std::vector<double>& AdaptiveStiffness::update(
    const std::vector<double>& target, const std::vector<double>& measured,
    const std::vector<hardware_interface::LoanedStateInterface>& states,
    std::vector<hardware_interface::LoanedCommandInterface>& commands, double elapsed) {
    if (states.size() < names_.size() || commands.size() < names_.size()) return stiffness_;
    ramp_ = ramp_duration_ > 0.0 ? std::min(1.0, ramp_ + elapsed / ramp_duration_) : 1.0;
    const std::size_t joints = indices_.size();
    const std::size_t table = states.size() - names_.size();
    const std::size_t command = commands.size() - names_.size();
    for (std::size_t slot = 0; slot < joints; ++slot) {
        const std::size_t index = indices_[slot];
        const double error = index < target.size() && index < measured.size()
                                 ? target[index] - measured[index]
                                 : 0.0;
        const double scale = 1.0 + ramp_ * (stiffness_scale(error) - 1.0);
        stiffness_[slot] = states[table + slot].get_value() * scale;
        commands[command + slot].set_value(stiffness_[slot]);
        commands[command + joints + slot].set_value(
            states[table + joints + slot].get_value() * std::sqrt(scale));
    }
    return stiffness_;
}

}  // namespace unitree_g1_ros2_control
