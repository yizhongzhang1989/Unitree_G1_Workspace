#include "unitree_g1_ros2_control/throttled_broadcasters.hpp"

#include "unitree_g1_ros2_control/periodic_deadline.hpp"

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

template <typename Broadcaster>
controller_interface::CallbackReturn ThrottledBroadcaster<Broadcaster>::on_init() {
    const auto result = Broadcaster::on_init();
    if (result != CallbackReturn::SUCCESS) {
        return result;
    }
    try {
        this->template auto_declare<double>("publish_rate", 100.0);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(
            this->get_node()->get_logger(),
            "Failed to declare publish_rate: %s", error.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

template <typename Broadcaster>
CallbackReturn ThrottledBroadcaster<Broadcaster>::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
    if (!configure_period(this->get_node(), publish_period_)) {
        return CallbackReturn::ERROR;
    }
    return Broadcaster::on_configure(previous_state);
}

template <typename Broadcaster>
CallbackReturn ThrottledBroadcaster<Broadcaster>::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
    const auto result = Broadcaster::on_activate(previous_state);
    if (result == CallbackReturn::SUCCESS) {
        next_publish_ = Clock::now();
    }
    return result;
}

template <typename Broadcaster>
controller_interface::return_type ThrottledBroadcaster<Broadcaster>::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
    const auto now = Clock::now();
    if (now < next_publish_) {
        return controller_interface::return_type::OK;
    }
    next_publish_ = advance_periodic_deadline(next_publish_, now, publish_period_);
    return Broadcaster::update(time, period);
}

template class ThrottledBroadcaster<joint_state_broadcaster::JointStateBroadcaster>;
template class ThrottledBroadcaster<imu_sensor_broadcaster::IMUSensorBroadcaster>;

}  // namespace unitree_g1_ros2_control

PLUGINLIB_EXPORT_CLASS(
    unitree_g1_ros2_control::ThrottledJointStateBroadcaster,
    controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(
    unitree_g1_ros2_control::ThrottledImuSensorBroadcaster,
    controller_interface::ControllerInterface)