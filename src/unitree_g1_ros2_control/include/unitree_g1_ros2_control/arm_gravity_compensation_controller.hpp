#ifndef UNITREE_G1_ROS2_CONTROL__ARM_GRAVITY_COMPENSATION_CONTROLLER_HPP_
#define UNITREE_G1_ROS2_CONTROL__ARM_GRAVITY_COMPENSATION_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace unitree_g1_ros2_control {

/// Folds arm gravity compensation into the position target of another
/// controller.
///
/// The G1 arm motors close their own position loop as
/// `tau = kp * (q_cmd - q) - kd * dq`, so holding a pose against gravity only
/// needs the command to be offset by `G(q_target) / kp`. This controller
/// therefore claims no command interface: it reads the torso IMU, evaluates the
/// calibrated gravity torque at the requested pose and republishes the target
/// with the offset applied, leaving the actual write to the position
/// controller downstream.
class ArmGravityCompensationController : public controller_interface::ControllerInterface {
public:
    static constexpr std::size_t kArmJointCount = 7;

    controller_interface::CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type update(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using Vector3 = std::array<double, 3>;
    using Matrix3 = std::array<double, 9>;

    /// One lumped rigid body of the reduced arm chain, as exported by the
    /// calibration package. Every link welded to this body is already merged
    /// into `mass` and `com`.
    struct RigidBody {
        Vector3 axis{};
        Vector3 origin{};
        Matrix3 rotation{};
        Vector3 com{};
        double mass{0.0};
        std::size_t command_index{0};
    };

    bool load_gravity_table(const std::string& path);
    bool update_torso_gravity(double elapsed);
    void arm_offsets(
        std::size_t side, const std::vector<double>& target, double gain,
        std::vector<double>& command) const;

    std::vector<std::string> joint_names_;
    std::array<std::vector<RigidBody>, 2> arms_;
    Matrix3 imu_to_torso_{};

    std::string command_topic_;
    double filter_cutoff_hz_{2.0};
    double ramp_duration_{2.0};
    double ramp_{0.0};
    // Trims residual model error by feel. Read from the update loop, written
    // from the parameter callback, so it has to be lock free.
    std::atomic<double> compensation_scale_{1.0};
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;

    Vector3 gravity_{};
    bool gravity_valid_{false};

    realtime_tools::RealtimeBuffer<std::shared_ptr<std::vector<double>>> target_buffer_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_subscription_;
    std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>
        command_publisher_;
};

}  // namespace unitree_g1_ros2_control

#endif  // UNITREE_G1_ROS2_CONTROL__ARM_GRAVITY_COMPENSATION_CONTROLLER_HPP_
