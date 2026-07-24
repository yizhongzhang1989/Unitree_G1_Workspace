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
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace unitree_g1_ros2_control {

class ForwardPositionController : public controller_interface::ControllerInterface {
public:
    controller_interface::return_type init(const std::string& controller_name) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type update() override;

private:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    struct CommandSample {
        std::vector<double> positions;
        std::uint64_t sequence;
    };

    bool copy_current_state(std::vector<double>& positions) const;

    std::vector<std::string> joint_names_;
    std::uint64_t processed_sequence_{0};
    std::atomic<std::uint64_t> next_sequence_{1};
    realtime_tools::RealtimeBuffer<std::shared_ptr<CommandSample>> command_buffer_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_subscription_;
};

}  // namespace unitree_g1_ros2_control

#endif  // UNITREE_G1_ROS2_CONTROL__FORWARD_POSITION_CONTROLLER_HPP_