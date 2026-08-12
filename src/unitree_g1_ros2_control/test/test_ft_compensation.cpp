#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include "yaml-cpp/yaml.h"

#include "unitree_g1_ros2_control/ft_compensation.hpp"

namespace {

using unitree_g1_ros2_control::FtCalibration;
using unitree_g1_ros2_control::Vector3;
using unitree_g1_ros2_control::Wrench;

std::string fixture(const std::string& name) {
    return std::string(TEST_FIXTURE_DIR) + "/" + name;
}

FtCalibration load_left() {
    FtCalibration calibration;
    EXPECT_TRUE(FtCalibration::load(
        fixture("ft_calibration_fixture.yaml"), "left", calibration));
    return calibration;
}

template <std::size_t N>
std::array<double, N> read(const YAML::Node& node) {
    std::array<double, N> values{};
    for (std::size_t index = 0; index < N; ++index) {
        values[index] = node[index].as<double>();
    }
    return values;
}

}  // namespace

TEST(FtCompensationTest, reads_every_field_of_the_exported_file) {
    const FtCalibration calibration = load_left();

    EXPECT_DOUBLE_EQ(calibration.mass, 0.625);
    EXPECT_DOUBLE_EQ(calibration.polarity, -1.0);
    EXPECT_DOUBLE_EQ(calibration.force_bias[1], -30.125);
    EXPECT_DOUBLE_EQ(calibration.torque_bias[2], 0.3125);
    EXPECT_DOUBLE_EQ(calibration.com[2], 0.121);
    EXPECT_DOUBLE_EQ(calibration.origin[2], 0.053);
    EXPECT_EQ(calibration.frame, "left_kwr57b_link");
}

/// The vendor takes moments about the tool flange, so ignoring `origin` would
/// misplace every payload by a whole sensor height.
TEST(FtCompensationTest, the_torque_reference_point_moves_the_net_torque) {
    FtCalibration calibration = load_left();
    const Vector3 gravity = {0.0, -9.81, 0.0};
    const Wrench raw = {40.0, -25.0, 3.0, 1.0, -0.5, 0.25};

    const Wrench shifted = unitree_g1_ros2_control::net_wrench(
        raw, calibration, gravity);
    calibration.origin = Vector3{};
    const Wrench ignored = unitree_g1_ros2_control::net_wrench(
        raw, calibration, gravity);

    for (std::size_t index = 0; index < 3; ++index) {
        EXPECT_NEAR(shifted[index], ignored[index], 1e-12) << "force " << index;
    }
    EXPECT_GT(std::abs(shifted[3] - ignored[3]) + std::abs(shifted[5] - ignored[5]),
              1e-3);
}

TEST(FtCompensationTest, matches_the_python_reference_for_every_case) {
    const FtCalibration calibration = load_left();
    const YAML::Node cases =
        YAML::LoadFile(fixture("ft_cases_fixture.yaml"))["cases"];
    ASSERT_TRUE(cases && cases.size() > 0);

    for (const auto& entry : cases) {
        const Vector3 gravity = read<3>(entry["gravity"]);
        const Wrench raw = read<6>(entry["raw"]);
        const Wrench expected = read<6>(entry["net"]);
        const Wrench net = unitree_g1_ros2_control::net_wrench(
            raw, calibration, gravity);
        for (std::size_t index = 0; index < 6; ++index) {
            EXPECT_NEAR(net[index], expected[index], 1e-12) << "axis " << index;
        }
    }
}

TEST(FtCompensationTest, rezeroing_cancels_a_drifted_offset_and_keeps_the_tool) {
    const FtCalibration calibration = load_left();
    const Vector3 gravity = {1.3, -4.2, -8.7};
    const Wrench reading = {3.0, -2.0, 1.0, 0.05, -0.02, 0.03};

    const FtCalibration drifted =
        unitree_g1_ros2_control::rezeroed(reading, calibration, gravity);
    const Wrench net =
        unitree_g1_ros2_control::net_wrench(reading, drifted, gravity);

    for (const double value : net) EXPECT_NEAR(value, 0.0, 1e-12);
    EXPECT_DOUBLE_EQ(drifted.mass, calibration.mass);
    EXPECT_DOUBLE_EQ(drifted.com[2], calibration.com[2]);
}

TEST(FtCompensationTest, reports_a_missing_side_without_throwing) {
    FtCalibration calibration;
    EXPECT_FALSE(FtCalibration::load(
        fixture("ft_calibration_fixture.yaml"), "right", calibration));
}
