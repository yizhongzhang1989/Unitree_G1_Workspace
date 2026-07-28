#include <gtest/gtest.h>

#include <array>
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
#include "std_msgs/msg/float64_multi_array.hpp"
#include "unitree_g1_ros2_control/forward_position_controller.hpp"

namespace unitree_g1_ros2_control {
namespace {

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

constexpr double kMass = 0.2;
constexpr double kLever = 0.5;
constexpr double kStiffness = 10.0;
constexpr double kGravity = 9.81;

/// Fifteen joints so the whole table fits, with the waist last to prove the
/// uncompensated slots are still forwarded verbatim.
const std::vector<std::string> kGravityJoints = {
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
    const std::string path = "/tmp/forward_position_gravity_table_test.yaml";
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

class ForwardPositionControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            int argc = 0;
            char** argv = nullptr;
            rclcpp::init(argc, argv);
        }
    }

    void TearDown() override {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }
};

TEST_F(ForwardPositionControllerTest, forwards_finite_position_commands_unchanged) {
    double command_position = std::nan("");
    double state_position = 0.0;
    hardware_interface::CommandInterface command_interface(
        "joint", hardware_interface::HW_IF_POSITION, &command_position);
    hardware_interface::StateInterface state_interface(
        "joint", hardware_interface::HW_IF_POSITION, &state_position);

    ForwardPositionController controller;
    ASSERT_EQ(controller.init("test_forward_position_controller"),
              controller_interface::return_type::OK);
    ASSERT_TRUE(controller.get_node()->set_parameter(
        rclcpp::Parameter("joints", std::vector<std::string>{"joint"})).successful);
    ASSERT_EQ(controller.configure().label(), "inactive");

    std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
    command_interfaces.emplace_back(command_interface);
    std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
    state_interfaces.emplace_back(state_interface);
    controller.assign_interfaces(
        std::move(command_interfaces), std::move(state_interfaces));
    ASSERT_EQ(controller.get_node()->activate().label(), "active");
    ASSERT_DOUBLE_EQ(command_position, 0.0);

    auto publisher_node = std::make_shared<rclcpp::Node>("fpc_test_publisher");
    auto publisher = publisher_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/test_forward_position_controller/commands",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controller.get_node()->get_node_base_interface());
    executor.add_node(publisher_node);

    const auto discovery_deadline = std::chrono::steady_clock::now() +
                                    std::chrono::seconds(2);
    while (publisher->get_subscription_count() == 0 &&
           std::chrono::steady_clock::now() < discovery_deadline) {
        executor.spin_some();
        std::this_thread::yield();
    }
    ASSERT_EQ(publisher->get_subscription_count(), 1U);

    const auto publish_and_expect = [&](double target, double expected) {
        std_msgs::msg::Float64MultiArray message;
        message.data = {target};
        publisher->publish(message);
        const auto command_deadline = std::chrono::steady_clock::now() +
                                      std::chrono::seconds(2);
        while (command_position != expected &&
               std::chrono::steady_clock::now() < command_deadline) {
            executor.spin_some();
            controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.002));
            std::this_thread::yield();
        }
        EXPECT_DOUBLE_EQ(command_position, expected);
    };

    publish_and_expect(0.08, 0.08);
    publish_and_expect(-0.08, -0.08);
    publish_and_expect(0.2, 0.2);

    state_position = -0.1;
    ASSERT_EQ(controller.get_node()->deactivate().label(), "inactive");
    ASSERT_EQ(controller.get_node()->activate().label(), "active");
    EXPECT_DOUBLE_EQ(command_position, -0.1);

    EXPECT_EQ(controller.get_node()->deactivate().label(), "inactive");
    controller.release_interfaces();
}

/// The point of folding the feed-forward into this controller: the offset has
/// to track the torso attitude on every cycle, not only on the cycles the
/// executor happens to deliver a new setpoint on.
TEST_F(ForwardPositionControllerTest, reapplies_gravity_offset_without_a_new_command) {
    const std::string table = write_single_body_table();
    std::vector<double> command_positions(kGravityJoints.size(), std::nan(""));
    std::vector<double> state_positions(kGravityJoints.size(), 0.0);
    // Torso upright: the fused attitude is identity, so gravity points at -Z.
    std::vector<double> imu = {0.0, 0.0, 0.0, 1.0};
    std::vector<double> gains(2 * GravityFeedforward::kArmJointCount, kStiffness);

    std::vector<hardware_interface::CommandInterface> commands;
    std::vector<hardware_interface::StateInterface> states;
    commands.reserve(kGravityJoints.size());
    states.reserve(kGravityJoints.size() + imu.size() + gains.size());
    for (std::size_t index = 0; index < kGravityJoints.size(); ++index) {
        commands.emplace_back(
            kGravityJoints[index], hardware_interface::HW_IF_POSITION,
            &command_positions[index]);
        states.emplace_back(
            kGravityJoints[index], hardware_interface::HW_IF_POSITION,
            &state_positions[index]);
    }
    // Exactly the order state_interface_configuration() asks for them in.
    for (std::size_t index = 0; index < imu.size(); ++index) {
        states.emplace_back(
            GravityFeedforward::torso_imu_sensor(),
            GravityFeedforward::imu_interface_names()[index], &imu[index]);
    }
    for (std::size_t index = 0; index < gains.size(); ++index) {
        states.emplace_back(kGravityJoints[index], "kp", &gains[index]);
    }

    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
    for (auto& interface : commands) loaned_commands.emplace_back(interface);
    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    for (auto& interface : states) loaned_states.emplace_back(interface);

    ForwardPositionController controller;
    ASSERT_EQ(controller.init("gravity_forward_position_controller"),
              controller_interface::return_type::OK);
    controller.get_node()->set_parameter({"joints", kGravityJoints});
    controller.get_node()->set_parameter({"gravity_table", table});
    controller.get_node()->set_parameter({"offset_ramp_s", 0.0});
    // A cut-off this far above the loop rate makes the low pass a pass-through,
    // so the expected offset is exact instead of a settling transient.
    controller.get_node()->set_parameter({"gravity_filter_cutoff_hz", 1.0e6});
    ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    controller.assign_interfaces(std::move(loaned_commands), std::move(loaned_states));
    ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

    auto publisher_node = std::make_shared<rclcpp::Node>("gravity_fpc_test_publisher");
    auto publisher = publisher_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/gravity_forward_position_controller/commands",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controller.get_node()->get_node_base_interface());
    executor.add_node(publisher_node);

    std_msgs::msg::Float64MultiArray message;
    message.data.assign(kGravityJoints.size(), 0.0);
    message.data.back() = 0.25;  // waist_yaw_joint must be forwarded untouched
    const double expected = -kMass * kGravity * kLever / kStiffness;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (command_positions[0] != expected &&
           std::chrono::steady_clock::now() < deadline) {
        publisher->publish(message);
        executor.spin_some();
        controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.002));
        std::this_thread::yield();
    }
    EXPECT_DOUBLE_EQ(command_positions[0], expected);
    EXPECT_DOUBLE_EQ(command_positions[7], expected);
    EXPECT_DOUBLE_EQ(command_positions[1], 0.0);
    EXPECT_DOUBLE_EQ(command_positions.back(), 0.25);

    // Roll the torso onto its back without publishing anything: gravity now
    // points the other way and the very next cycle has to reflect it.
    imu = {1.0, 0.0, 0.0, 0.0};
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.002));
    EXPECT_DOUBLE_EQ(command_positions[0], -expected);
    EXPECT_DOUBLE_EQ(command_positions[7], -expected);
    EXPECT_DOUBLE_EQ(command_positions.back(), 0.25);

    // compensation_scale is the runtime off switch: the offset has to vanish
    // exactly, not merely shrink, so the whole evaluation can be skipped.
    ASSERT_TRUE(controller.get_node()->set_parameter({"compensation_scale", 0.0}).successful);
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.002));
    EXPECT_DOUBLE_EQ(command_positions[0], 0.0);
    EXPECT_DOUBLE_EQ(command_positions.back(), 0.25);

    EXPECT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    controller.release_interfaces();
    std::remove(table.c_str());
}

}  // namespace
}  // namespace unitree_g1_ros2_control