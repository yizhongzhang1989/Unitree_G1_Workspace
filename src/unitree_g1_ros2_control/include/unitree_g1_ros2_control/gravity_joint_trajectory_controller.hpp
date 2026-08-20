#ifndef UNITREE_G1_ROS2_CONTROL__GRAVITY_JOINT_TRAJECTORY_CONTROLLER_HPP_
#define UNITREE_G1_ROS2_CONTROL__GRAVITY_JOINT_TRAJECTORY_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "unitree_g1_ros2_control/adaptive_stiffness.hpp"
#include "unitree_g1_ros2_control/gravity_feedforward.hpp"

namespace unitree_g1_ros2_control {

/// The upstream trajectory controller with the same arm gravity feed-forward the position controller applies.
///
/// Without it the arms settle BELOW every trajectory point, so a hand-over from
/// the position controller to this one visibly drops them. The upstream class
/// cannot be edited, so the offset is applied around its `update()`: the bare
/// setpoints it wrote last cycle are restored first, so reading them back after
/// it runs can never accumulate our own offset - including on the cycles where
/// it holds position and writes nothing at all.
///
/// Leave `gravity_table` empty, or set `compensation_scale` to zero, to get the stock upstream behaviour.
class GravityJointTrajectoryController
    : public joint_trajectory_controller::JointTrajectoryController {
public:
    controller_interface::CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type update(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    GravityFeedforward gravity_;
    AdaptiveStiffness stiffness_;
    // Scratch buffers, sized once at configure so the loop never allocates.
    std::vector<double> target_;
    std::vector<double> measured_;
    std::vector<double> command_;
    bool target_valid_{false};
};

}  // namespace unitree_g1_ros2_control

#endif  // UNITREE_G1_ROS2_CONTROL__GRAVITY_JOINT_TRAJECTORY_CONTROLLER_HPP_
