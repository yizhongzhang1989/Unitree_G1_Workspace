#include <gtest/gtest.h>

#include <algorithm>
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
// Friction coefficients written into the test table. Arbitrary but distinct
// from anything the real calibration produces, so a test can only pass by
// actually reading the table.
constexpr double kFrictionRatio = 0.2;
constexpr double kFrictionFloor = 0.15;

const std::vector<std::string> kJointNames = {
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint",          "left_wrist_roll_joint",    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",      "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint",    "right_wrist_pitch_joint",  "right_wrist_yaw_joint",
    "waist_yaw_joint",
};

const char* joint_suffix(std::size_t index) {
    return index == 0   ? "_shoulder_pitch_joint"
           : index == 1 ? "_shoulder_roll_joint"
           : index == 2 ? "_shoulder_yaw_joint"
           : index == 3 ? "_elbow_joint"
           : index == 4 ? "_wrist_roll_joint"
           : index == 5 ? "_wrist_pitch_joint"
                        : "_wrist_yaw_joint";
}

/// Seven bodies in a row, every joint rotating about +Y with an identity link
/// rotation, so at zero pose the whole chain lies along +X and each torque is
/// exactly `-g * sum(m_i * x_i)` over the bodies distal to that joint.
std::string write_table(
    const std::array<double, 7>& masses, const std::array<double, 7>& coms,
    const std::array<double, 7>& origins,
    const std::array<double, 3>& mount = {0.0, 0.0, 0.0},
    const std::array<double, 9>& mount_rotation =
        {1, 0, 0, 0, 1, 0, 0, 0, 1}) {
    const std::string path = "/tmp/arm_gravity_table_test.yaml";
    std::ofstream stream(path);
    stream << "arm_gravity_compensation:\n  ros__parameters:\n";
    stream << "    imu_to_torso: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]\n";
    for (const char* side : {"left", "right"}) {
        stream << "    " << side << ":\n      joints: [";
        for (std::size_t index = 0; index < 7; ++index) {
            stream << (index ? ", " : "") << side << joint_suffix(index);
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
        stream << "]\n      payload_origin_xyz: [" << mount[0] << ", " << mount[1]
               << ", " << mount[2] << "]\n      payload_origin_rotation: [";
        for (std::size_t index = 0; index < 9; ++index) {
            stream << (index ? ", " : "") << mount_rotation[index];
        }
        stream << "]\n";
    }
    return path;
}

/// The separate file the calibration exports the friction fit to, in the same
/// joint order as the gravity table above.
std::string write_friction_file(
    double ratio, double floor_torque, const char* left_first_joint = nullptr) {
    const std::string path = "/tmp/arm_friction_table_test.yaml";
    std::ofstream stream(path);
    stream << "forward_position_controller:\n  ros__parameters:\n";
    for (const char* side : {"left", "right"}) {
        stream << "    " << side << ":\n      joints: [";
        for (std::size_t index = 0; index < 7; ++index) {
            stream << (index ? ", " : "");
            // Lets one test hand over a deliberately mismatched order.
            if (index == 0 && left_first_joint != nullptr && std::string(side) == "left") {
                stream << left_first_joint;
            } else {
                stream << side << joint_suffix(index);
            }
        }
        stream << "]\n      load_ratio: [";
        for (std::size_t index = 0; index < 7; ++index) {
            stream << (index ? ", " : "") << ratio;
        }
        stream << "]\n      offset: [";
        for (std::size_t index = 0; index < 7; ++index) {
            stream << (index ? ", " : "") << floor_torque;
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
        gravity.arm_offsets(side, target, {}, {}, 1.0, 0.0, stiffness, command);
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

/// A payload is reported in the sensor frame, so it only lands on the right
/// lever arm if both the mount offset and the mount rotation are applied.
TEST(GravityFeedforwardTest, payload_hangs_off_the_sensor_mount) {
    constexpr double kMount = 0.3;
    constexpr double kPayload = 0.8;
    constexpr double kCom = 0.12;
    const std::string table = write_table({}, {}, {}, {kMount, 0.0, 0.0});
    GravityFeedforward gravity;
    gravity.load_gravity_table(table, kJointNames);
    EXPECT_TRUE(gravity.update_torso_gravity({0.0, 0.0, 0.0, 1.0}, 0.002));
    gravity.set_payload(0, kPayload, {kPayload * kCom, 0.0, 0.0});

    const std::vector<double> target(kJointNames.size(), 0.0);
    const std::vector<double> stiffness(
        GravityFeedforward::kSideCount * GravityFeedforward::kArmJointCount, kStiffness);
    std::vector<double> command = target;
    gravity.arm_offsets(0, target, {}, {}, 1.0, 0.0, stiffness, command);

    // Every joint sits at the origin here, so each carries the same lever.
    const double expected = -kPayload * (kMount + kCom) * kGravity / kStiffness;
    for (std::size_t index = 0; index < 7; ++index) {
        EXPECT_NEAR(command[index], expected, 1e-9) << "index " << index;
    }
    for (std::size_t index = 7; index < 14; ++index) {
        EXPECT_NEAR(command[index], 0.0, 1e-12) << "index " << index;
    }
    std::remove(table.c_str());
}

/// A mount rotation applied transposed would keep producing a torque here
/// instead of turning the lever out of the gravity plane.
TEST(GravityFeedforwardTest, payload_lever_follows_the_mount_rotation) {
    const std::string table = write_table(
        {}, {}, {}, {0.0, 0.0, 0.0}, {0, -1, 0, 1, 0, 0, 0, 0, 1});
    GravityFeedforward gravity;
    gravity.load_gravity_table(table, kJointNames);
    EXPECT_TRUE(gravity.update_torso_gravity({0.0, 0.0, 0.0, 1.0}, 0.002));
    gravity.set_payload(0, 0.8, {0.8 * 0.12, 0.0, 0.0});

    const std::vector<double> target(kJointNames.size(), 0.0);
    const std::vector<double> stiffness(
        GravityFeedforward::kSideCount * GravityFeedforward::kArmJointCount, kStiffness);
    std::vector<double> command = target;
    gravity.arm_offsets(0, target, {}, {}, 1.0, 0.0, stiffness, command);

    // The lever now points along +Y, and every joint turns about +Y.
    for (std::size_t index = 0; index < 7; ++index) {
        EXPECT_NEAR(command[index], 0.0, 1e-12) << "index " << index;
    }
    std::remove(table.c_str());
}

/// The stiffness the controller reads off the hardware is indexed by this list,
/// so a wrong order would scale every joint by another joint's gain.
TEST(GravityFeedforwardTest, reports_the_command_slots_its_stiffness_belongs_to) {    const std::string table = write_single_body_table();
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

// ---------------------------------------------------------------------- //
// Friction feed-forward
// ---------------------------------------------------------------------- //

/// Same as `offsets` but with the friction term switched on. `error` is the
/// position error at joint 0 of the left arm; the measured pose is derived from
/// it so the caller only has to think about the one number that matters.
std::vector<double> friction_offsets(
    const std::string& table, const std::vector<double>& target, double error,
    double velocity, double friction_gain) {
    GravityFeedforward gravity;
    gravity.load_gravity_table(table, kJointNames);
    const std::string friction = write_friction_file(kFrictionRatio, kFrictionFloor);
    gravity.load_friction_table(friction);
    std::remove(friction.c_str());
    EXPECT_TRUE(gravity.update_torso_gravity({0.0, 0.0, 0.0, 1.0}, 0.002));

    std::vector<double> measured = target;
    measured[0] -= error;  // error = target - measured
    std::vector<double> velocities(kJointNames.size(), 0.0);
    velocities[0] = velocity;

    const std::vector<double> stiffness(
        GravityFeedforward::kSideCount * GravityFeedforward::kArmJointCount, kStiffness);
    std::vector<double> command = target;
    for (std::size_t side = 0; side < GravityFeedforward::kSideCount; ++side) {
        gravity.arm_offsets(
            side, target, measured, velocities, 1.0, friction_gain, stiffness, command);
    }
    return command;
}

/// A joint that is already where it was asked to be must not be pushed on,
/// however hard the friction term is turned up.
TEST(GravityFeedforwardTest, a_joint_that_is_on_target_and_still_gets_no_friction) {
    const std::string table = write_single_body_table();
    const std::vector<double> target(kJointNames.size(), 0.0);

    const auto plain = offsets(table, target);
    const auto with_friction = friction_offsets(table, target, 0.0, 0.0, 1.0);
    for (std::size_t index = 0; index < target.size(); ++index) {
        EXPECT_DOUBLE_EQ(with_friction[index], plain[index]) << "index " << index;
    }
    std::remove(table.c_str());
}

TEST(GravityFeedforwardTest, friction_is_a_share_of_the_load_plus_a_floor) {
    const std::string table = write_single_body_table();
    const std::vector<double> target(kJointNames.size(), 0.0);

    const auto plain = offsets(table, target);
    // Well past the tanh width, so the term saturates.
    const auto moved = friction_offsets(table, target, 0.5, 0.0, 1.0);

    // Only body 0 has mass, and at zero pose it hangs a lever out along +X.
    const double load = kMass * kLever * kGravity;
    const double expected = (kFrictionRatio * load + kFrictionFloor) / kStiffness;
    EXPECT_NEAR(moved[0] - plain[0], expected, 1e-9);
    // No other joint has an error, so none of them is pushed.
    for (std::size_t index = 1; index < 7; ++index) {
        EXPECT_NEAR(moved[index] - plain[index], 0.0, 1e-12) << "index " << index;
    }
    std::remove(table.c_str());
}

TEST(GravityFeedforwardTest, friction_follows_the_sign_of_the_error) {
    const std::string table = write_single_body_table();
    const std::vector<double> target(kJointNames.size(), 0.0);

    const auto plain = offsets(table, target);
    const auto behind = friction_offsets(table, target, 0.5, 0.0, 1.0);
    const auto ahead = friction_offsets(table, target, -0.5, 0.0, 1.0);

    EXPECT_GT(behind[0] - plain[0], 0.0);
    EXPECT_NEAR(behind[0] - plain[0], plain[0] - ahead[0], 1e-12);
    std::remove(table.c_str());
}

/// The reason the drive is the error and not the target velocity. A 5 mm/s
/// fingertip is about 0.0125 rad/s at the joint; on velocity alone that is
/// tanh(0.625) = 55% of the torque, and 1 mm/s only 12% - least help exactly
/// where the fine motion is. The error reaches the full dead band however
/// slowly the target creeps, so the term stays saturated.
TEST(GravityFeedforwardTest, a_crawling_target_still_gets_the_whole_push) {
    const std::string table = write_single_body_table();
    const std::vector<double> target(kJointNames.size(), 0.0);

    const auto plain = offsets(table, target);
    const double saturated =
        (kFrictionRatio * kMass * kLever * kGravity + kFrictionFloor) / kStiffness;

    // Stuck a dead band behind the target while barely moving at all.
    // 0.1 / 0.05 = 2, and tanh(2) is 96% - near enough the whole push.
    const auto crawling = friction_offsets(table, target, 0.1, 0.0002, 1.0);
    EXPECT_GT(crawling[0] - plain[0], 0.95 * saturated);

    // The same crawl with no error to report would barely register.
    const auto velocity_only = friction_offsets(table, target, 0.0, 0.0002, 1.0);
    EXPECT_LT(velocity_only[0] - plain[0], 0.02 * saturated);
    std::remove(table.c_str());
}

/// A hard sign() would step the command by the full dead band as the error
/// changes sign, which is exactly the excitation that makes a compensated
/// joint ring.
TEST(GravityFeedforwardTest, friction_crosses_zero_smoothly) {
    const std::string table = write_single_body_table();
    const std::vector<double> target(kJointNames.size(), 0.0);

    double previous = 0.0;
    double largest_step = 0.0;
    for (int step = -20; step <= 20; ++step) {
        const double value = friction_offsets(table, target, 0.002 * step, 0.0, 1.0)[0];
        if (step > -20) largest_step = std::max(largest_step, std::abs(value - previous));
        previous = value;
    }
    const double saturated =
        (kFrictionRatio * kMass * kLever * kGravity + kFrictionFloor) / kStiffness;
    EXPECT_LT(largest_step, 0.2 * saturated);
    std::remove(table.c_str());
}

/// Error and target velocity disagree while the target turns around: the joint
/// is still behind where it was asked to be, but is now being asked to come
/// back. Cancelling is the right answer - the direction is genuinely in doubt.
TEST(GravityFeedforwardTest, a_reversal_cancels_the_two_drives) {
    const std::string table = write_single_body_table();
    const std::vector<double> target(kJointNames.size(), 0.0);

    const auto plain = offsets(table, target);
    // 0.05 / 0.05 = 1 from the error, -0.02 / 0.02 = -1 from the velocity.
    const auto opposed = friction_offsets(table, target, 0.05, -0.02, 1.0);
    EXPECT_NEAR(opposed[0] - plain[0], 0.0, 1e-12);
    std::remove(table.c_str());
}

/// Targets arrive at 50 Hz while this runs at 500 Hz. Differencing across the
/// control period would give nine zeroes and one spike ten times too large,
/// and low-passing that yields a 50 Hz ripple rather than the mean - measured
/// at 0.22-0.39 about a 0.3 target. Dividing by the time the target actually
/// held recovers the speed.
TEST(GravityFeedforwardTest, target_velocity_recovers_the_speed_from_a_50hz_staircase) {
    GravityFeedforward gravity;
    constexpr double kPeriod = 0.002;
    constexpr std::size_t kDecimation = 10;
    constexpr double kSpeed = 0.3;

    std::vector<double> target(kJointNames.size(), 0.0);
    gravity.update_target_velocity(target, kPeriod);  // sizes the state
    gravity.update_target_velocity(target, kPeriod);  // seeds the previous target

    double commanded = 0.0;
    for (std::size_t step = 0; step < 2000; ++step) {
        if (step % kDecimation == 0) commanded += kSpeed * kPeriod * kDecimation;
        target[0] = commanded;
        gravity.update_target_velocity(target, kPeriod);
    }
    EXPECT_NEAR(gravity.target_velocity()[0], kSpeed, 0.005);
    // Joints nobody commanded stay at rest.
    EXPECT_DOUBLE_EQ(gravity.target_velocity()[1], 0.0);

    // Holding the same target is the upper layer asking the arm to stand
    // still, so the feed-forward part has to fade out.
    for (std::size_t step = 0; step < 1000; ++step) {
        gravity.update_target_velocity(target, kPeriod);
    }
    EXPECT_NEAR(gravity.target_velocity()[0], 0.0, 1e-6);
}

/// Re-activation seeds from the target it is handed. Differencing against a
/// stale one would report a step the size of the whole pose change.
TEST(GravityFeedforwardTest, activation_does_not_invent_a_velocity) {
    GravityFeedforward gravity;
    std::vector<double> target(kJointNames.size(), 0.0);
    gravity.update_target_velocity(target, 0.002);
    gravity.update_target_velocity(target, 0.002);

    gravity.reset();
    std::vector<double> elsewhere(kJointNames.size(), 0.0);
    elsewhere[0] = 1.5;
    gravity.update_target_velocity(elsewhere, 0.002);
    EXPECT_DOUBLE_EQ(gravity.target_velocity()[0], 0.0);
}

/// Coefficients are per joint, not per arm. Measured mu spans 0.064 to 0.223,
/// so one shared pair over-drives one end while starving the other - which
/// shows up as some joints keeping up and others lagging, pulling the whole
/// path off.
TEST(GravityFeedforwardTest, each_joint_carries_its_own_coefficients) {
    const std::string table = write_single_body_table();
    GravityFeedforward gravity;
    gravity.load_gravity_table(table, kJointNames);
    EXPECT_TRUE(gravity.update_torso_gravity({0.0, 0.0, 0.0, 1.0}, 0.002));

    auto model = gravity.friction_model();
    model.load_ratio.fill(0.0);
    model.offset.fill(0.0);
    constexpr std::size_t kPicked = 2;  // left shoulder yaw
    model.offset[kPicked] = 0.5;
    gravity.set_friction_model(model);

    const std::vector<double> target(kJointNames.size(), 0.0);
    // Every joint lags by the same amount, so only the coefficients can differ.
    std::vector<double> measured(kJointNames.size(), -0.5);
    const std::vector<double> velocity(kJointNames.size(), 0.0);
    const std::vector<double> stiffness(
        GravityFeedforward::kCompensatedCount, kStiffness);

    std::vector<double> command = target;
    for (std::size_t side = 0; side < GravityFeedforward::kSideCount; ++side) {
        gravity.arm_offsets(side, target, measured, velocity, 1.0, 1.0, stiffness, command);
    }

    const auto plain = offsets(table, target);
    EXPECT_NEAR(command[kPicked] - plain[kPicked], 0.5 / kStiffness, 1e-9);
    for (std::size_t index = 0; index < GravityFeedforward::kCompensatedCount; ++index) {
        if (index == kPicked) continue;
        EXPECT_NEAR(command[index] - plain[index], 0.0, 1e-12) << "index " << index;
    }
    std::remove(table.c_str());
}

/// A fitted floor can come out slightly negative on the shoulder yaws, which
/// is the fit extrapolated below the loads that were sampled. Left alone it
/// would push the joint the wrong way once the load drops, widening the dead
/// band instead of closing it.
TEST(GravityFeedforwardTest, a_negative_coefficient_never_pushes_backwards) {
    const std::string table = write_single_body_table();
    GravityFeedforward gravity;
    gravity.load_gravity_table(table, kJointNames);
    EXPECT_TRUE(gravity.update_torso_gravity({0.0, 0.0, 0.0, 1.0}, 0.002));

    auto model = gravity.friction_model();
    model.load_ratio.fill(0.0);
    model.offset.fill(-0.5);
    gravity.set_friction_model(model);

    const std::vector<double> target(kJointNames.size(), 0.0);
    const std::vector<double> measured(kJointNames.size(), -0.5);
    const std::vector<double> velocity(kJointNames.size(), 0.0);
    const std::vector<double> stiffness(
        GravityFeedforward::kCompensatedCount, kStiffness);

    std::vector<double> command = target;
    for (std::size_t side = 0; side < GravityFeedforward::kSideCount; ++side) {
        gravity.arm_offsets(side, target, measured, velocity, 1.0, 1.0, stiffness, command);
    }

    const auto plain = offsets(table, target);
    for (std::size_t index = 0; index < GravityFeedforward::kCompensatedCount; ++index) {
        EXPECT_NEAR(command[index] - plain[index], 0.0, 1e-12) << "index " << index;
    }
    std::remove(table.c_str());
}

/// Without a friction table nothing is compensated - the behaviour from before
/// the term existed. Absence has to read as "not measured" rather than as a
/// licence to guess.
TEST(GravityFeedforwardTest, a_table_without_coefficients_compensates_nothing) {
    const std::string table = write_single_body_table();
    GravityFeedforward gravity;
    gravity.load_gravity_table(table, kJointNames);
    EXPECT_TRUE(gravity.update_torso_gravity({0.0, 0.0, 0.0, 1.0}, 0.002));

    const std::vector<double> target(kJointNames.size(), 0.0);
    const std::vector<double> measured(kJointNames.size(), -0.5);
    const std::vector<double> velocity(kJointNames.size(), 0.0);
    const std::vector<double> stiffness(
        GravityFeedforward::kCompensatedCount, kStiffness);

    std::vector<double> command = target;
    for (std::size_t side = 0; side < GravityFeedforward::kSideCount; ++side) {
        gravity.arm_offsets(side, target, measured, velocity, 1.0, 1.0, stiffness, command);
    }

    const auto plain = offsets(table, target);
    for (std::size_t index = 0; index < GravityFeedforward::kCompensatedCount; ++index) {
        EXPECT_NEAR(command[index] - plain[index], 0.0, 1e-12) << "index " << index;
    }
    std::remove(table.c_str());
}

/// Splitting friction into its own file loses the guarantee that it came from
/// the same calibration as the gravity it is paired with, so the joint names
/// travel with it and a mismatch is refused rather than compensated onto the
/// wrong joint.
TEST(GravityFeedforwardTest, a_friction_table_for_another_arm_is_refused) {
    const std::string table = write_single_body_table();
    GravityFeedforward gravity;
    gravity.load_gravity_table(table, kJointNames);

    const std::string friction = write_friction_file(
        kFrictionRatio, kFrictionFloor, "left_elbow_joint");
    EXPECT_THROW(gravity.load_friction_table(friction), std::runtime_error);
    std::remove(friction.c_str());
    std::remove(table.c_str());
}

}  // namespace
}  // namespace unitree_g1_ros2_control
