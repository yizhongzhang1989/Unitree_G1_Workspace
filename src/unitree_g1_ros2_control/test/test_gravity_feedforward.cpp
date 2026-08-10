#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "unitree_g1_ros2_control/gravity_feedforward.hpp"

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

/// Load the table, take one upright IMU sample and apply the offsets at full
/// gain, exactly the way the controller drives it.
std::vector<double> offsets(const std::string& table, const std::vector<double>& target) {
    GravityFeedforward gravity;
    gravity.load_gravity_table(table, kJointNames);
    // Torso upright: the fused attitude is identity, so gravity points at -Z.
    EXPECT_TRUE(gravity.update_torso_gravity({0.0, 0.0, 0.0, 1.0}, 0.002));

    const std::vector<double> stiffness(
        GravityFeedforward::kSideCount * GravityFeedforward::kArmJointCount, kStiffness);
    std::vector<double> command = target;
    for (std::size_t side = 0; side < GravityFeedforward::kSideCount; ++side) {
        gravity.arm_offsets(side, target, 1.0, stiffness, command);
    }
    return command;
}

TEST(GravityFeedforwardTest, offsets_arm_targets_and_passes_others_through) {
    const std::string table = write_single_body_table();
    std::vector<double> target(kJointNames.size(), 0.0);
    target.back() = 0.25;  // waist_yaw_joint must be forwarded untouched
    const std::vector<double> command = offsets(table, target);
    ASSERT_EQ(command.size(), kJointNames.size());

    const double expected = -kMass * kGravity * kLever / kStiffness;
    EXPECT_NEAR(command[0], expected, 1e-9);
    EXPECT_NEAR(command[7], expected, 1e-9);
    for (std::size_t index = 1; index < 7; ++index) {
        EXPECT_NEAR(command[index], 0.0, 1e-9) << "left index " << index;
        EXPECT_NEAR(command[index + 7], 0.0, 1e-9) << "right index " << index;
    }
    EXPECT_NEAR(command.back(), 0.25, 1e-12);
    std::remove(table.c_str());
}

/// A single body cannot catch an error in how a joint accumulates everything
/// distal to it, which is the part that actually walks the chain.
TEST(GravityFeedforwardTest, inner_joint_carries_every_outer_body) {
    constexpr double kMassA = 0.3;
    constexpr double kComA = 0.2;
    constexpr double kMassB = 0.5;
    constexpr double kComB = 0.15;
    constexpr double kLinkB = 0.4;
    const std::string table = write_table(
        {kMassA, kMassB, 0, 0, 0, 0, 0}, {kComA, kComB, 0, 0, 0, 0, 0},
        {0.0, kLinkB, 0, 0, 0, 0, 0});
    const std::vector<double> command =
        offsets(table, std::vector<double>(kJointNames.size(), 0.0));
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

/// The stiffness the controller reads off the hardware is indexed by this list,
/// so a wrong order would scale every joint by another joint's gain.
TEST(GravityFeedforwardTest, reports_the_command_slots_its_stiffness_belongs_to) {
    const std::string table = write_single_body_table();
    GravityFeedforward gravity;
    gravity.load_gravity_table(table, kJointNames);

    std::vector<std::size_t> expected(2 * GravityFeedforward::kArmJointCount);
    for (std::size_t index = 0; index < expected.size(); ++index) expected[index] = index;
    EXPECT_TRUE(gravity.loaded());
    EXPECT_EQ(gravity.compensated_indices(), expected);
    std::remove(table.c_str());
}

TEST(GravityFeedforwardTest, rejects_a_table_naming_an_uncommanded_joint) {
    const std::string table = write_single_body_table();
    GravityFeedforward gravity;

    EXPECT_THROW(
        gravity.load_gravity_table(table, {"waist_yaw_joint"}), std::runtime_error);
    EXPECT_FALSE(gravity.loaded());
    std::remove(table.c_str());
}

/// Without an attitude there is no gravity direction, so the caller has to be
/// told to fall back to the bare target instead of applying a stale offset.
TEST(GravityFeedforwardTest, refuses_a_non_unit_quaternion_until_one_arrives) {
    const std::string table = write_single_body_table();
    GravityFeedforward gravity;
    gravity.load_gravity_table(table, kJointNames);

    EXPECT_FALSE(gravity.update_torso_gravity({0.0, 0.0, 0.0, 0.0}, 0.002));
    EXPECT_FALSE(gravity.gravity_valid());

    EXPECT_TRUE(gravity.update_torso_gravity({0.0, 0.0, 0.0, 1.0}, 0.002));
    // A later bad sample keeps the last good direction rather than dropping it.
    EXPECT_TRUE(gravity.update_torso_gravity({std::nan(""), 0.0, 0.0, 1.0}, 0.002));
    gravity.reset();
    EXPECT_FALSE(gravity.gravity_valid());
    std::remove(table.c_str());
}



}  // namespace
}  // namespace unitree_g1_ros2_control
