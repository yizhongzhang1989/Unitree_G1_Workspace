#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "unitree_g1_ros2_control/gravity_joint_trajectory_controller.hpp"

namespace unitree_g1_ros2_control {
namespace {

constexpr double kMass = 0.2;
constexpr double kLever = 0.5;
constexpr double kStiffness = 10.0;
constexpr double kDamping = 2.0;
constexpr double kGravity = 9.81;

const std::vector<std::string> kJoints = {
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint",          "left_wrist_roll_joint",    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",      "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint",    "right_wrist_pitch_joint",  "right_wrist_yaw_joint",
    "waist_yaw_joint",
};

/// One body per side at `kLever` along +X, hinged about +Y, so the shoulder
/// torque is exactly `-m * g * l` with the torso upright.
std::string write_single_body_table() {
    const std::string path = "/tmp/trajectory_gravity_table_test.yaml";
    std::ofstream stream(path);
    stream << "arm_gravity_compensation:\n  ros__parameters:\n";
    stream << "    imu_to_torso: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]\n";
    for (const char* side : {"left", "right"}) {
        stream << "    " << side << ":\n      joints: [";
        for (std::size_t index = 0; index < 7; ++index) {
            stream << (index ? ", " : "") << side
                   << (index == 0   ? "_shoulder_pitch_joint"
                       : index == 1 ? "_shoulder_roll_joint"
                       : index == 2 ? "_shoulder_yaw_joint"
                       : index == 3 ? "_elbow_joint"
                       : index == 4 ? "_wrist_roll_joint"
                       : index == 5 ? "_wrist_pitch_joint"
                                    : "_wrist_yaw_joint");
        }
        stream << "]\n      axis: [";
        for (std::size_t index = 0; index < 7; ++index) stream << (index ? ", " : "") << "0, 1, 0";
        stream << "]\n      origin_xyz: [";
        for (std::size_t index = 0; index < 7; ++index) stream << (index ? ", " : "") << "0, 0, 0";
        stream << "]\n      origin_rotation: [";
        for (std::size_t index = 0; index < 7; ++index) {
            stream << (index ? ", " : "") << "1, 0, 0, 0, 1, 0, 0, 0, 1";
        }
        stream << "]\n      mass: [" << kMass << ", 0, 0, 0, 0, 0, 0]\n";
        stream << "      com: [" << kLever << ", 0, 0";
        for (std::size_t index = 1; index < 7; ++index) stream << ", 0, 0, 0";
        stream << "]\n";
    }
    return path;
}

/// Owns the fake hardware so the loaned handles stay valid for the whole test.
class Hardware {
public:
    Hardware() : positions_(kJoints.size(), 0.0),
                 velocities_(kJoints.size(), 0.0),
                 commands_(kJoints.size(), std::nan("")),
                 // Torso upright: the fused attitude is identity, so gravity
                 // points at -Z.
                 imu_{0.0, 0.0, 0.0, 1.0},
                 gains_(2 * GravityFeedforward::kArmJointCount, kStiffness),
                 damping_(2 * GravityFeedforward::kArmJointCount, kDamping),
                 commanded_gains_(2 * gains_.size(), std::nan("")) {
        command_handles_.reserve(kJoints.size() + commanded_gains_.size());
        state_handles_.reserve(3 * kJoints.size() + imu_.size() + 2 * gains_.size());
        for (std::size_t index = 0; index < kJoints.size(); ++index) {
            command_handles_.emplace_back(
                kJoints[index], hardware_interface::HW_IF_POSITION, &commands_[index]);
            // Joint major, exactly the order the base class asks for them in.
            state_handles_.emplace_back(
                kJoints[index], hardware_interface::HW_IF_POSITION, &positions_[index]);
            state_handles_.emplace_back(
                kJoints[index], hardware_interface::HW_IF_VELOCITY, &velocities_[index]);
        }
        for (std::size_t index = 0; index < imu_.size(); ++index) {
            state_handles_.emplace_back(
                GravityFeedforward::torso_imu_sensor(),
                GravityFeedforward::imu_interface_names()[index], &imu_[index]);
        }
        for (std::size_t index = 0; index < gains_.size(); ++index) {
            state_handles_.emplace_back(kJoints[index], "kp", &gains_[index]);
        }
        for (std::size_t index = 0; index < gains_.size(); ++index) {
            state_handles_.emplace_back(kJoints[index], "kd", &damping_[index]);
        }
        for (std::size_t index = 0; index < gains_.size(); ++index) {
            command_handles_.emplace_back(
                kJoints[index], "kp", &commanded_gains_[index]);
        }
        for (std::size_t index = 0; index < gains_.size(); ++index) {
            command_handles_.emplace_back(
                kJoints[index], "kd", &commanded_gains_[gains_.size() + index]);
        }
    }

    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands() {
        std::vector<hardware_interface::LoanedCommandInterface> loaned;
        for (auto& handle : command_handles_) loaned.emplace_back(handle);
        return loaned;
    }

    std::vector<hardware_interface::LoanedStateInterface> loaned_states() {
        std::vector<hardware_interface::LoanedStateInterface> loaned;
        for (auto& handle : state_handles_) loaned.emplace_back(handle);
        return loaned;
    }

    std::vector<double> positions_;
    std::vector<double> velocities_;
    std::vector<double> commands_;
    std::vector<double> imu_;
    std::vector<double> gains_;
    std::vector<double> damping_;
    std::vector<double> commanded_gains_;

private:
    std::vector<hardware_interface::CommandInterface> command_handles_;
    std::vector<hardware_interface::StateInterface> state_handles_;
};

class GravityJointTrajectoryControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            int argc = 0;
            char** argv = nullptr;
            rclcpp::init(argc, argv);
        }
        table_ = write_single_body_table();
    }

    void TearDown() override {
        std::remove(table_.c_str());
        if (rclcpp::ok()) rclcpp::shutdown();
    }

    /// The upstream controller reads its parameters when its node is built, so
    /// they have to arrive as overrides rather than through `set_parameter`.
    rclcpp::NodeOptions options(std::vector<rclcpp::Parameter> extra = {}) const {
        std::vector<rclcpp::Parameter> overrides = {
            rclcpp::Parameter("joints", kJoints),
            rclcpp::Parameter("command_interfaces", std::vector<std::string>{"position"}),
            rclcpp::Parameter(
                "state_interfaces", std::vector<std::string>{"position", "velocity"}),
            rclcpp::Parameter("gravity_table", table_),
            rclcpp::Parameter("offset_ramp_s", 0.0),
            // A cut-off this far above the loop rate makes the low pass a
            // pass-through, so the expected offset is exact.
            rclcpp::Parameter("gravity_filter_cutoff_hz", 1.0e6),
        };
        overrides.insert(overrides.end(), extra.begin(), extra.end());
        return rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
            .parameter_overrides(overrides);
    }

    std::string table_;
};
/// The upstream controller writes nothing while it holds a point, so a naive
/// read-modify-write of the command interfaces would add the offset again on
/// every single cycle and run the arm away.
TEST_F(GravityJointTrajectoryControllerTest, holds_a_point_without_accumulating_the_offset) {
    Hardware hardware;
    GravityJointTrajectoryController controller;
    ASSERT_EQ(
        controller.init("gravity_jtc", "", options()),
        controller_interface::return_type::OK);
    ASSERT_EQ(controller.configure().label(), "inactive");
    controller.assign_interfaces(hardware.loaned_commands(), hardware.loaned_states());
    ASSERT_EQ(controller.get_node()->activate().label(), "active");
    // Nothing commanded yet: the interfaces must keep what the previous
    // controller left there, or the hand-over would step by a whole offset.
    EXPECT_TRUE(std::isnan(hardware.commands_[0]));

    auto publisher_node = std::make_shared<rclcpp::Node>("gravity_jtc_test_publisher");
    auto publisher = publisher_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/gravity_jtc/joint_trajectory", rclcpp::QoS(rclcpp::KeepLast(1)));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controller.get_node()->get_node_base_interface());
    executor.add_node(publisher_node);

    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = kJoints;
    trajectory.points.resize(1);
    trajectory.points[0].positions.assign(kJoints.size(), 0.0);
    trajectory.points[0].positions.back() = 0.25;  // waist must pass through
    trajectory.points[0].time_from_start = rclcpp::Duration::from_seconds(0.0);

    const double expected = -kMass * kGravity * kLever / kStiffness;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!(hardware.commands_[0] == expected) &&
           std::chrono::steady_clock::now() < deadline) {
        publisher->publish(trajectory);
        executor.spin_some();
        controller.update(
            controller.get_node()->now(), rclcpp::Duration::from_seconds(0.002));
        std::this_thread::yield();
    }
    ASSERT_DOUBLE_EQ(hardware.commands_[0], expected);
    EXPECT_DOUBLE_EQ(hardware.commands_[7], expected);
    EXPECT_DOUBLE_EQ(hardware.commands_[1], 0.0);
    EXPECT_DOUBLE_EQ(hardware.commands_.back(), 0.25);

    // Hold the very same point for a while: the offset must stay put instead of
    // being folded into its own input.
    for (int cycle = 0; cycle < 200; ++cycle) {
        controller.update(
            controller.get_node()->now(), rclcpp::Duration::from_seconds(0.002));
    }
    EXPECT_DOUBLE_EQ(hardware.commands_[0], expected);
    EXPECT_DOUBLE_EQ(hardware.commands_.back(), 0.25);

    // Roll the torso onto its back without sending anything: gravity now points
    // the other way and the very next cycle has to reflect it.
    hardware.imu_ = {1.0, 0.0, 0.0, 0.0};
    controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(0.002));
    EXPECT_DOUBLE_EQ(hardware.commands_[0], -expected);
    EXPECT_DOUBLE_EQ(hardware.commands_.back(), 0.25);

    EXPECT_EQ(controller.get_node()->deactivate().label(), "inactive");
    controller.release_interfaces();
}

/// Open loop control seeds the upstream controller from the command interfaces,
/// which by then already carry the offset, so it would fold back in.
TEST_F(GravityJointTrajectoryControllerTest, refuses_gravity_together_with_open_loop_control) {
    GravityJointTrajectoryController controller;
    ASSERT_EQ(
        controller.init(
            "gravity_jtc_open_loop", "", options({rclcpp::Parameter("open_loop_control", true)})),
        controller_interface::return_type::OK);

    EXPECT_EQ(
        controller.on_configure(rclcpp_lifecycle::State()),
        controller_interface::CallbackReturn::ERROR);
}

}  // namespace
}  // namespace unitree_g1_ros2_control
