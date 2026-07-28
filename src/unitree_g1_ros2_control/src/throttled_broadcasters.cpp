#include "unitree_g1_ros2_control/throttled_broadcasters.hpp"

#include <cmath>
#include <exception>

#include "pluginlib/class_list_macros.hpp"

namespace unitree_g1_ros2_control {

namespace {

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

template <typename ClockDuration>
bool configure_period(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, ClockDuration& period) {
    const double rate = node->get_parameter("publish_rate").as_double();
    if (!std::isfinite(rate) || rate <= 0.0) {
        RCLCPP_ERROR(node->get_logger(), "publish_rate must be finite and positive");
        return false;
    }
    period = std::chrono::duration_cast<ClockDuration>(
        std::chrono::duration<double>(1.0 / rate));
    return period > ClockDuration::zero();
}

}  // namespace

controller_interface::CallbackReturn ThrottledJointStateBroadcaster::on_init() {
    const auto result = joint_state_broadcaster::JointStateBroadcaster::on_init();
    if (result != CallbackReturn::SUCCESS) {
        return result;
    }
    try {
        auto_declare<double>("publish_rate", 100.0);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare publish_rate: %s", error.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn ThrottledJointStateBroadcaster::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
    if (!configure_period(get_node(), publish_period_)) {
        return CallbackReturn::ERROR;
    }
    return joint_state_broadcaster::JointStateBroadcaster::on_configure(previous_state);
}

CallbackReturn ThrottledJointStateBroadcaster::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
    const auto result =
        joint_state_broadcaster::JointStateBroadcaster::on_activate(previous_state);
    if (result == CallbackReturn::SUCCESS) {
        next_publish_ = Clock::now();
    }
    return result;
}

controller_interface::return_type ThrottledJointStateBroadcaster::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
    const auto now = Clock::now();
    if (now < next_publish_) {
        return controller_interface::return_type::OK;
    }
    do {
        next_publish_ += publish_period_;
    } while (next_publish_ <= now);
    return joint_state_broadcaster::JointStateBroadcaster::update(time, period);
}

controller_interface::CallbackReturn ThrottledImuSensorBroadcaster::on_init() {
    const auto result = imu_sensor_broadcaster::IMUSensorBroadcaster::on_init();
    if (result != CallbackReturn::SUCCESS) {
        return result;
    }
    try {
        auto_declare<double>("publish_rate", 100.0);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare publish_rate: %s", error.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn ThrottledImuSensorBroadcaster::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
    if (!configure_period(get_node(), publish_period_)) {
        return CallbackReturn::ERROR;
    }
    return imu_sensor_broadcaster::IMUSensorBroadcaster::on_configure(previous_state);
}

CallbackReturn ThrottledImuSensorBroadcaster::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
    const auto result = imu_sensor_broadcaster::IMUSensorBroadcaster::on_activate(previous_state);
    if (result == CallbackReturn::SUCCESS) {
        next_publish_ = Clock::now();
    }
    return result;
}

controller_interface::return_type ThrottledImuSensorBroadcaster::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
    const auto now = Clock::now();
    if (now < next_publish_) {
        return controller_interface::return_type::OK;
    }
    do {
        next_publish_ += publish_period_;
    } while (next_publish_ <= now);
    return imu_sensor_broadcaster::IMUSensorBroadcaster::update(time, period);
}

}  // namespace unitree_g1_ros2_control

PLUGINLIB_EXPORT_CLASS(
    unitree_g1_ros2_control::ThrottledJointStateBroadcaster,
    controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(
    unitree_g1_ros2_control::ThrottledImuSensorBroadcaster,
    controller_interface::ControllerInterface)