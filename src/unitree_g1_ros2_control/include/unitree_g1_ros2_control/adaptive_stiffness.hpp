#ifndef UNITREE_G1_ROS2_CONTROL__ADAPTIVE_STIFFNESS_HPP_
#define UNITREE_G1_ROS2_CONTROL__ADAPTIVE_STIFFNESS_HPP_

#include <atomic>
#include <cstddef>
#include <string>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace unitree_g1_ros2_control {

/// Raises the joint stiffness as the tracking error shrinks.
///
///     kp' = kp * (1 + scale / ((|e| / b)^power + 1)),  e = q_target - q
///     kd' = kd * sqrt(kp' / kp)
///
/// `scale` is the extra multiple at zero error, `b` the error where half of it
/// is left; the two are independent, and the tail returns to the nominal `kp`
/// without ever dropping below. Normalising by `b` rather than adding it to
/// `|e|^power` is what keeps them so - otherwise `b` carries units of rad^power
/// and the half-strength point moves whenever `power` does. Rationale and
/// tuning live in the README.
///
/// `kp`/`kd` exist in both directions: the state interfaces carry the gain
/// table, the command interfaces of the same name carry what the motor gets.
/// Re-reading the table every cycle is what stops the scale compounding.
class AdaptiveStiffness {
public:
    using LifecycleNode = rclcpp_lifecycle::LifecycleNode;

    /// The fade-in shares the gravity offset's `offset_ramp_s`: the two have to
    /// arrive together.
    static void declare_parameters(const LifecycleNode::SharedPtr& node);

    /// `indices` is the order everything here is indexed in.
    bool configure(
        const LifecycleNode::SharedPtr& node, const std::vector<std::string>& joint_names,
        const std::vector<std::size_t>& indices);

    /// Appended to the state and the command list alike, and in both cases it
    /// has to go **last**: `update` counts back from the end.
    void append_interfaces(std::vector<std::string>& names) const;
    std::size_t interface_count() const noexcept { return names_.size(); }

    /// Checks the trailing interfaces really are the ones asked for. Getting
    /// this wrong is otherwise silent: a `kd` landing in a `kp` slot just
    /// scales every torque by the ratio of the two.
    bool activate(
        const std::vector<hardware_interface::LoanedStateInterface>& states,
        const std::vector<hardware_interface::LoanedCommandInterface>& commands,
        const rclcpp::Logger& logger) const;

    /// Reads the gain table, applies the law, writes the result to the command
    /// interfaces, and returns the `kp` the motor will get - which the gravity
    /// offset has to divide by, or its torque comes out wrong by that ratio.
    const std::vector<double>& update(
        const std::vector<double>& target, const std::vector<double>& measured,
        const std::vector<hardware_interface::LoanedStateInterface>& states,
        std::vector<hardware_interface::LoanedCommandInterface>& commands, double elapsed);

    /// Rearm the fade-in. Arriving at full strength on the activation cycle
    /// would step the torque by the whole extra `kp * e`.
    void reset() noexcept { ramp_ = 0.0; }

private:
    /// The control law, and the only place the shape is decided.
    double stiffness_scale(double position_error) const;
    bool store_parameter(const rclcpp::Parameter& parameter, std::string& reason);

    // "<joint>/kp" for every slot, then "<joint>/kd".
    std::vector<std::string> names_;
    std::vector<std::size_t> indices_;
    std::vector<double> stiffness_;

    // Read from the update loop, written from the parameter callback.
    std::atomic<double> scale_{0.0};
    std::atomic<double> b_{0.05};
    std::atomic<double> power_{1.0};

    double ramp_duration_{2.0};
    double ramp_{0.0};
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

}  // namespace unitree_g1_ros2_control

#endif  // UNITREE_G1_ROS2_CONTROL__ADAPTIVE_STIFFNESS_HPP_
