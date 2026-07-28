#ifndef UNITREE_G1_ROS2_CONTROL__FORWARD_POSITION_CONTROLLER_HPP_
#define UNITREE_G1_ROS2_CONTROL__FORWARD_POSITION_CONTROLLER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "unitree_g1_ros2_control/gravity_feedforward.hpp"

namespace unitree_g1_ros2_control {

/// Writes the latest full finite position command to the hardware, with the
/// calibrated arm gravity folded into the arm slots.
///
/// The feed-forward lives inside this loop rather than in a controller in front
/// of it: the offset tracks the torso attitude and has to be re-evaluated every
/// cycle, which a topic hop between controllers resamples at executor jitter.
/// Leave `gravity_table` empty, or set `compensation_scale` to zero, for plain
/// forwarding.
class ForwardPositionController : public controller_interface::ControllerInterface {
public:
    controller_interface::CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    struct CommandSample {
        std::vector<double> positions;
        std::uint64_t sequence;
    };

    void write_command(const std::vector<double>& positions);

    std::vector<std::string> joint_names_;
    std::uint64_t processed_sequence_{0};
    std::atomic<std::uint64_t> next_sequence_{1};
    realtime_tools::RealtimeBuffer<std::shared_ptr<CommandSample>> command_buffer_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_subscription_;

    GravityFeedforward gravity_;
    // Scratch buffers, sized once at configure so the loop never allocates.
    std::vector<double> target_;
    std::vector<double> command_;
    bool target_valid_{false};
};

}  // namespace unitree_g1_ros2_control

#endif  // UNITREE_G1_ROS2_CONTROL__FORWARD_POSITION_CONTROLLER_HPP_