#ifndef UNITREE_G1_ROS2_CONTROL__THROTTLED_BROADCASTERS_HPP_
#define UNITREE_G1_ROS2_CONTROL__THROTTLED_BROADCASTERS_HPP_

#include <chrono>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "imu_sensor_broadcaster/imu_sensor_broadcaster.hpp"
#include "joint_state_broadcaster/joint_state_broadcaster.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace unitree_g1_ros2_control {

class ThrottledJointStateBroadcaster :
    public joint_state_broadcaster::JointStateBroadcaster {
public:
    controller_interface::return_type init(const std::string& controller_name) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::return_type update() override;

private:
    using Clock = std::chrono::steady_clock;
    Clock::duration publish_period_{};
    Clock::time_point next_publish_{};
};

class ThrottledImuSensorBroadcaster :
    public imu_sensor_broadcaster::IMUSensorBroadcaster {
public:
    controller_interface::return_type init(const std::string& controller_name) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::return_type update() override;

private:
    using Clock = std::chrono::steady_clock;
    Clock::duration publish_period_{};
    Clock::time_point next_publish_{};
};

}  // namespace unitree_g1_ros2_control

#endif  // UNITREE_G1_ROS2_CONTROL__THROTTLED_BROADCASTERS_HPP_