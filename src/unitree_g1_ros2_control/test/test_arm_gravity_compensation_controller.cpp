#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "unitree_g1_ros2_control/arm_gravity_compensation_controller.hpp"

namespace unitree_g1_ros2_control {
namespace {

constexpr double kMass = 0.2;
constexpr double kLever = 0.5;
constexpr double kStiffness = 10.0;
constexpr double kGravity = 9.81;

const std::vector<std::string> kJointNames = {
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint",          "left_wrist_roll_joint",    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",      "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint",    "right_wrist_pitch_joint",  "right_wrist_yaw_joint",
    "waist_yaw_joint",
};

/// Seven bodies in a row, every joint rotating about +Y with an identity link
/// rotation, so at zero pose the whole chain lies along +X and each torque is
/// exactly `-g * sum(m_i * x_i)` over the bodies distal to that joint.
std::string write_table(
    const std::array<double, 7>& masses, const std::array<double, 7>& coms,
    const std::array<double, 7>& origins) {
    const std::string path = "/tmp/arm_gravity_table_test.yaml";
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
        for (std::size_t index = 0; index < 7; ++index) {
            stream << (index ? ", " : "") << origins[index] << ", 0, 0";
        }
        stream << "]\n      origin_rotation: [";
        for (std::size_t index = 0; index < 7; ++index) {
            stream << (index ? ", " : "") << "1, 0, 0, 0, 1, 0, 0, 0, 1";
        }
        stream << "]\n      mass: [";
        for (std::size_t index = 0; index < 7; ++index) {
            stream << (index ? ", " : "") << masses[index];
        }
        stream << "]\n      com: [";
        for (std::size_t index = 0; index < 7; ++index) {
            stream << (index ? ", " : "") << coms[index] << ", 0, 0";
        }
        stream << "]\n";
    }
    return path;
}

std::string write_single_body_table() {
    return write_table({kMass, 0, 0, 0, 0, 0, 0}, {kLever, 0, 0, 0, 0, 0, 0}, {});
}

class ArmGravityCompensationControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            int argc = 0;
            char** argv = nullptr;
            rclcpp::init(argc, argv);
        }
    }
    void TearDown() override {
        if (rclcpp::ok()) rclcpp::shutdown();
    }

    /// Configure, activate and spin the controller until it publishes once.
    std::vector<double> run(const std::string& table, const std::vector<double>& target_values) {
        // Torso upright: the fused attitude is identity, so gravity points at -Z.
        imu_ = {0.0, 0.0, 0.0, 1.0};
        gains_.assign(14, kStiffness);
        std::vector<hardware_interface::StateInterface> interfaces;
        const std::vector<std::string> names = {
            "orientation.x", "orientation.y", "orientation.z", "orientation.w",
        };
        for (std::size_t index = 0; index < names.size(); ++index) {
            interfaces.emplace_back("torso_imu", names[index], &imu_[index]);
        }
        // The hardware publishes the stiffness it really applies; the controller
        // must use those values instead of a duplicated parameter.
        for (std::size_t index = 0; index < gains_.size(); ++index) {
            interfaces.emplace_back(kJointNames[index], "kp", &gains_[index]);
        }
        std::vector<hardware_interface::LoanedStateInterface> loaned;
        for (auto& interface : interfaces) loaned.emplace_back(interface);

        ArmGravityCompensationController controller;
        EXPECT_EQ(controller.init("arm_gravity"), controller_interface::return_type::OK);
        controller.get_node()->set_parameter({"joints", kJointNames});
        controller.get_node()->set_parameter({"gravity_table", table});
        controller.get_node()->set_parameter({"command_topic", "/test_commands"});
        controller.get_node()->set_parameter({"offset_ramp_s", 0.0});
        EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State()),
                  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
        controller.assign_interfaces({}, std::move(loaned));
        EXPECT_EQ(controller.on_activate(rclcpp_lifecycle::State()),
                  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(controller.get_node()->get_node_base_interface());
        auto listener = std::make_shared<rclcpp::Node>("listener");
        std_msgs::msg::Float64MultiArray received;
        bool got = false;
        auto subscription = listener->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/test_commands", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
            [&](std_msgs::msg::Float64MultiArray::SharedPtr message) {
                received = *message;
                got = true;
            });
        executor.add_node(listener);
        auto publisher = listener->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/arm_gravity/target", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

        std_msgs::msg::Float64MultiArray target;
        target.data = target_values;
        for (int attempt = 0; attempt < 60 && !got; ++attempt) {
            publisher->publish(target);
            executor.spin_some();
            controller.update();
            executor.spin_some();
        }
        EXPECT_TRUE(got);
        return received.data;
    }

    std::vector<double> imu_;
    std::vector<double> gains_;
};

TEST_F(ArmGravityCompensationControllerTest, offsets_arm_targets_and_passes_others_through) {
    const std::string table = write_single_body_table();
    std::vector<double> target(kJointNames.size(), 0.0);
    target.back() = 0.25;  // waist_yaw_joint must be forwarded untouched
    const std::vector<double> command = run(table, target);
    ASSERT_EQ(command.size(), kJointNames.size());

    const double expected = -kMass * kGravity * kLever / kStiffness;
    EXPECT_NEAR(command[0], expected, 1e-9);
    EXPECT_NEAR(command[7], expected, 1e-9);
    for (std::size_t index = 1; index < 7; ++index) {
        EXPECT_NEAR(command[index], 0.0, 1e-9) << "left index " << index;
        EXPECT_NEAR(command[index + 7], 0.0, 1e-9) << "right index " << index;
    }
    EXPECT_NEAR(command.back(), 0.25, 1e-12);
    // Ramping in over a finite time must never overshoot the steady offset.
    EXPECT_LE(std::abs(command[0]), std::abs(expected) + 1e-9);
    std::remove(table.c_str());
}

/// A single body cannot catch an error in how a joint accumulates everything
/// distal to it, which is the part that actually walks the chain.
TEST_F(ArmGravityCompensationControllerTest, inner_joint_carries_every_outer_body) {
    constexpr double kMassA = 0.3;
    constexpr double kComA = 0.2;
    constexpr double kMassB = 0.5;
    constexpr double kComB = 0.15;
    constexpr double kLinkB = 0.4;
    const std::string table = write_table(
        {kMassA, kMassB, 0, 0, 0, 0, 0}, {kComA, kComB, 0, 0, 0, 0, 0},
        {0.0, kLinkB, 0, 0, 0, 0, 0});
    const std::vector<double> command = run(table, std::vector<double>(kJointNames.size(), 0.0));
    ASSERT_EQ(command.size(), kJointNames.size());

    // Joint 1 only carries body B; joint 0 carries both, with body B's mass
    // acting at its full distance from the shoulder.
    const double outer = -kMassB * kComB * kGravity / kStiffness;
    const double inner =
        -(kMassA * kComA + kMassB * (kLinkB + kComB)) * kGravity / kStiffness;
    EXPECT_NEAR(command[1], outer, 1e-9);
    EXPECT_NEAR(command[0], inner, 1e-9);
    EXPECT_NEAR(command[8], outer, 1e-9);
    EXPECT_NEAR(command[7], inner, 1e-9);
    for (std::size_t index = 2; index < 7; ++index) {
        EXPECT_NEAR(command[index], 0.0, 1e-9) << "left index " << index;
    }
    std::remove(table.c_str());
}

}  // namespace
}  // namespace unitree_g1_ros2_control
