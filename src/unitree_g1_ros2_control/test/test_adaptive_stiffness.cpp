#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "unitree_g1_ros2_control/adaptive_stiffness.hpp"

namespace unitree_g1_ros2_control {
namespace {

constexpr double kDegree = M_PI / 180.0;
// Distinct from each other and from 1, so a test can only pass by actually
// multiplying the table value rather than reporting a bare ratio.
constexpr double kTableStiffness = 10.0;
constexpr double kTableDamping = 2.0;
// Two joints out of a five-slot command vector, so a test can only pass by
// honouring the index mapping rather than assuming slot == joint.
const std::vector<std::size_t> kIndices = {1, 3};
const std::vector<std::string> kJointNames = {"a", "b", "c", "d", "e"};
// Interfaces the controller claimed for itself, which the gain slots sit behind.
constexpr std::size_t kLeading = 3;

class AdaptiveStiffnessTest : public ::testing::Test {
protected:
    void SetUp() override {
        node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("adaptive_stiffness_test");
        node_->declare_parameter<double>("offset_ramp_s", 0.0);
        AdaptiveStiffness::declare_parameters(node_);
        // Off by default, so every test that wants the law has to say so. Two
        // is the designed strength: a 3x intercept.
        node_->set_parameter({"adaptive_stiffness_scale", 2.0});

        table_.assign(kLeading + 2 * kIndices.size(), 0.0);
        commanded_.assign(kLeading + 2 * kIndices.size(), std::nan(""));
        for (std::size_t slot = 0; slot < kIndices.size(); ++slot) {
            table_[kLeading + slot] = kTableStiffness;
            table_[kLeading + kIndices.size() + slot] = kTableDamping;
        }
        states_.reserve(table_.size());
        commands_.reserve(commanded_.size());
        for (std::size_t index = 0; index < table_.size(); ++index) {
            states_.emplace_back("joint", "slot", &table_[index]);
            commands_.emplace_back("joint", "slot", &commanded_[index]);
        }
        for (auto& handle : states_) loaned_states_.emplace_back(handle);
        for (auto& handle : commands_) loaned_commands_.emplace_back(handle);
    }

    /// Configures, then runs one cycle with a single joint at `error`.
    /// Returns the resulting kp as a multiple of the table value.
    double scale_at(double error) {
        EXPECT_TRUE(stiffness_.configure(node_, kJointNames, kIndices));
        std::vector<double> target(kJointNames.size(), 0.0);
        target[kIndices[0]] = error;
        return run(target)[0] / kTableStiffness;
    }

    const std::vector<double>& run(const std::vector<double>& target, double elapsed = 1.0) {
        static const std::vector<double> measured(kJointNames.size(), 0.0);
        return stiffness_.update(target, measured, loaned_states_, loaned_commands_, elapsed);
    }

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    AdaptiveStiffness stiffness_;
    std::vector<double> table_;
    std::vector<double> commanded_;
    std::vector<hardware_interface::StateInterface> states_;
    std::vector<hardware_interface::CommandInterface> commands_;
    std::vector<hardware_interface::LoanedStateInterface> loaned_states_;
    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands_;
};

TEST_F(AdaptiveStiffnessTest, is_off_until_the_scale_is_raised) {
    node_->set_parameter({"adaptive_stiffness_scale", 0.0});
    ASSERT_TRUE(stiffness_.configure(node_, kJointNames, kIndices));
    std::vector<double> target(kJointNames.size(), 0.0);
    target[kIndices[0]] = 0.2;
    EXPECT_DOUBLE_EQ(run(target)[0], kTableStiffness);
    EXPECT_DOUBLE_EQ(commanded_[kLeading + kIndices.size()], kTableDamping);
}

/// The two knobs are independent: `scale` only sets the strength at zero error,
/// `b` only where half of it is left.
TEST_F(AdaptiveStiffnessTest, scale_sets_the_intercept_and_b_the_reach) {
    EXPECT_DOUBLE_EQ(scale_at(0.0), 3.0);   // 1 + scale
    EXPECT_DOUBLE_EQ(scale_at(0.05), 2.0);  // half of the extra is left at b

    node_->set_parameter({"adaptive_stiffness_b", 0.2});
    EXPECT_DOUBLE_EQ(scale_at(0.0), 3.0);   // intercept untouched
    EXPECT_DOUBLE_EQ(scale_at(0.2), 2.0);   // half-strength moved with b

    node_->set_parameter({"adaptive_stiffness_scale", 4.0});
    EXPECT_DOUBLE_EQ(scale_at(0.0), 5.0);
    EXPECT_DOUBLE_EQ(scale_at(0.2), 3.0);   // still half the extra
}

/// Clearly raised at 10 deg, all but gone at 45, and never below the table
/// gain - so a large error behaves as it did before this existed and the
/// torque stays unbounded. Pinned so a shape change cannot silently move them.
TEST_F(AdaptiveStiffnessTest, decays_across_the_working_range) {
    EXPECT_NEAR(scale_at(10 * kDegree), 1.45, 0.01);
    EXPECT_NEAR(scale_at(45 * kDegree), 1.12, 0.01);
    EXPECT_GT(scale_at(100.0), 1.0);
    EXPECT_NEAR(scale_at(100.0), 1.0, 0.01);
    // Sign of the error must not matter - it is a stiffness, not a torque.
    EXPECT_DOUBLE_EQ(scale_at(-10 * kDegree), scale_at(10 * kDegree));
}

/// power 2 keeps the extra stiffness flat near the origin instead of falling
/// off fastest right there, which is the whole reason the exponent is exposed.
TEST_F(AdaptiveStiffnessTest, power_two_stays_flat_near_the_origin) {
    const double linear_1 = scale_at(1 * kDegree);
    const double linear_45 = scale_at(45 * kDegree);
    node_->set_parameter({"adaptive_stiffness_power", 2.0});
    EXPECT_GT(scale_at(1 * kDegree), linear_1);
    EXPECT_LT(scale_at(45 * kDegree), linear_45);
    EXPECT_DOUBLE_EQ(scale_at(0.05), 2.0);  // half-strength still at b
}

/// Re-reading the table every cycle is what stops the scale compounding; the
/// commanded gain is deliberately never fed back in.
TEST_F(AdaptiveStiffnessTest, settles_instead_of_compounding) {
    ASSERT_TRUE(stiffness_.configure(node_, kJointNames, kIndices));
    std::vector<double> target(kJointNames.size(), 0.0);
    ASSERT_TRUE(node_->set_parameter({"adaptive_stiffness_scale", 2.0}).successful);
    for (int cycle = 0; cycle < 200; ++cycle) run(target);
    EXPECT_DOUBLE_EQ(commanded_[kLeading], 3.0 * kTableStiffness);

    // Raising it at runtime has to land on the new value, not walk away.
    ASSERT_TRUE(node_->set_parameter({"adaptive_stiffness_scale", 4.0}).successful);
    for (int cycle = 0; cycle < 200; ++cycle) run(target);
    EXPECT_DOUBLE_EQ(commanded_[kLeading], 5.0 * kTableStiffness);
}

/// zeta ~ kd / sqrt(kp), so the square root is what holds the damping ratio at
/// its nominal value while kp moves.
TEST_F(AdaptiveStiffnessTest, damping_follows_the_square_root_of_stiffness) {
    ASSERT_TRUE(stiffness_.configure(node_, kJointNames, kIndices));
    std::vector<double> target(kJointNames.size(), 0.0);
    target[kIndices[0]] = 0.05;
    const double scale = run(target)[0] / kTableStiffness;
    EXPECT_DOUBLE_EQ(commanded_[kLeading + kIndices.size()], kTableDamping * std::sqrt(scale));
}

/// Only the joints in `indices` are looked at, and each in its own slot.
TEST_F(AdaptiveStiffnessTest, reads_the_error_through_the_index_map) {
    ASSERT_TRUE(stiffness_.configure(node_, kJointNames, kIndices));
    std::vector<double> target(kJointNames.size(), 0.0);
    // Slot 0 tracks joint 1, slot 1 tracks joint 3. Joint 2 belongs to nobody.
    target[3] = 0.5;
    target[2] = 99.0;
    const auto& stiffness = run(target);
    EXPECT_DOUBLE_EQ(stiffness[0], 3.0 * kTableStiffness);
    EXPECT_LT(stiffness[1], 1.5 * kTableStiffness);
}

/// Arriving at 3x on the activation cycle would step the torque by 2*kp*e at
/// whatever error the hand-over starts with.
TEST_F(AdaptiveStiffnessTest, fades_in_over_the_ramp) {
    node_->set_parameter({"offset_ramp_s", 2.0});
    ASSERT_TRUE(stiffness_.configure(node_, kJointNames, kIndices));
    const std::vector<double> target(kJointNames.size(), 0.0);

    EXPECT_DOUBLE_EQ(run(target, 0.5)[0], 1.5 * kTableStiffness);  // a quarter of the way
    EXPECT_DOUBLE_EQ(run(target, 0.5)[0], 2.0 * kTableStiffness);
    EXPECT_DOUBLE_EQ(run(target, 5.0)[0], 3.0 * kTableStiffness);  // clamped at full

    stiffness_.reset();
    EXPECT_DOUBLE_EQ(run(target, 0.5)[0], 1.5 * kTableStiffness);  // rearmed
}

/// One is a divisor and the other an exponent, so neither may be zero even
/// though zero is the legal "off" for the scale.
TEST_F(AdaptiveStiffnessTest, rejects_a_zero_divisor_or_exponent) {
    node_->set_parameter({"adaptive_stiffness_b", 0.0});
    EXPECT_FALSE(stiffness_.configure(node_, kJointNames, kIndices));
    node_->set_parameter({"adaptive_stiffness_b", 0.5});
    node_->set_parameter({"adaptive_stiffness_power", 0.0});
    EXPECT_FALSE(stiffness_.configure(node_, kJointNames, kIndices));
}

/// The callback has to reject the same values configure() would, or a value
/// could be set at runtime that would have been refused at startup.
TEST_F(AdaptiveStiffnessTest, rejects_bad_values_at_runtime_too) {
    ASSERT_TRUE(stiffness_.configure(node_, kJointNames, kIndices));
    EXPECT_FALSE(node_->set_parameter({"adaptive_stiffness_b", -1.0}).successful);
    EXPECT_FALSE(node_->set_parameter({"adaptive_stiffness_scale", -1.0}).successful);
    EXPECT_TRUE(node_->set_parameter({"adaptive_stiffness_power", 2.0}).successful);
}

/// `ros2 param set ... adaptive_stiffness_b 1` arrives as an integer, and
/// as_double() on one of those throws out of the parameter service and takes
/// the process with it. A controller reaches that path because the YAML
/// overrides are declared for it with dynamic typing before the declaration
/// above gets a chance to pin the type.
TEST_F(AdaptiveStiffnessTest, accepts_an_integer_from_the_parameter_service) {
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("dynamically_typed");
    node->declare_parameter<double>("offset_ramp_s", 0.0);
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    node->declare_parameter("adaptive_stiffness_b", rclcpp::ParameterValue(0.5), descriptor);
    AdaptiveStiffness::declare_parameters(node);

    AdaptiveStiffness stiffness;
    ASSERT_TRUE(stiffness.configure(node, kJointNames, kIndices));
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("adaptive_stiffness_b", 1)).successful);
}

/// The interfaces are claimed behind the controller's own, so they are found by
/// counting back from the end - and nothing in front may be touched.
TEST_F(AdaptiveStiffnessTest, names_and_writes_the_trailing_interfaces) {
    ASSERT_TRUE(stiffness_.configure(node_, kJointNames, kIndices));
    std::vector<std::string> names;
    stiffness_.append_interfaces(names);
    ASSERT_EQ(names.size(), stiffness_.interface_count());
    EXPECT_EQ(names, (std::vector<std::string>{"b/kp", "d/kp", "b/kd", "d/kd"}));

    run(std::vector<double>(kJointNames.size(), 0.0));
    for (std::size_t index = 0; index < kLeading; ++index) {
        EXPECT_TRUE(std::isnan(commanded_[index])) << "leading slot " << index;
    }
    EXPECT_DOUBLE_EQ(commanded_[kLeading + 0], 3.0 * kTableStiffness);
    EXPECT_DOUBLE_EQ(commanded_[kLeading + 1], 3.0 * kTableStiffness);
    EXPECT_DOUBLE_EQ(commanded_[kLeading + 2], std::sqrt(3.0) * kTableDamping);
    EXPECT_DOUBLE_EQ(commanded_[kLeading + 3], std::sqrt(3.0) * kTableDamping);
}

/// Interleaving kp and kd instead of grouping them would otherwise pass
/// silently and scale every torque by kd/kp.
TEST_F(AdaptiveStiffnessTest, refuses_to_activate_on_misordered_interfaces) {
    ASSERT_TRUE(stiffness_.configure(node_, kJointNames, kIndices));
    std::vector<std::string> names;
    stiffness_.append_interfaces(names);

    std::vector<double> values(kLeading + names.size(), 0.0);
    std::vector<hardware_interface::StateInterface> states;
    std::vector<hardware_interface::CommandInterface> commands;
    states.reserve(values.size());
    commands.reserve(values.size());
    for (std::size_t index = 0; index < kLeading; ++index) {
        states.emplace_back("other", "position", &values[index]);
        commands.emplace_back("other", "position", &values[index]);
    }
    // b/kp, b/kd, d/kp, d/kd instead of the grouped order.
    for (const std::size_t slot : {0U, 2U, 1U, 3U}) {
        const std::size_t slash = names[slot].rfind('/');
        states.emplace_back(
            names[slot].substr(0, slash), names[slot].substr(slash + 1), &values[slot]);
        commands.emplace_back(
            names[slot].substr(0, slash), names[slot].substr(slash + 1), &values[slot]);
    }
    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
    for (auto& handle : states) loaned_states.emplace_back(handle);
    for (auto& handle : commands) loaned_commands.emplace_back(handle);

    EXPECT_FALSE(stiffness_.activate(loaned_states, loaned_commands, node_->get_logger()));
}

}  // namespace
}  // namespace unitree_g1_ros2_control

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
