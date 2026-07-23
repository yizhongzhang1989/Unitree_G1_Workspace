#include "unitree_g1_ros2_control/periodic_deadline.hpp"

#include <gtest/gtest.h>

#include <chrono>

namespace unitree_g1_ros2_control
{
namespace
{

using Clock = std::chrono::steady_clock;

TEST(PeriodicDeadline, PreservesPhaseAfterSmallDelay)
{
  const auto start = Clock::time_point{};
  const auto period = std::chrono::milliseconds(10);
  EXPECT_EQ(
    advance_periodic_deadline(start, start + std::chrono::milliseconds(11), period),
    start + std::chrono::milliseconds(20));
}

TEST(PeriodicDeadline, SkipsMissedSlotsWithoutBursting)
{
  const auto start = Clock::time_point{};
  const auto period = std::chrono::milliseconds(10);
  EXPECT_EQ(
    advance_periodic_deadline(start, start + std::chrono::milliseconds(35), period),
    start + std::chrono::milliseconds(40));
}

TEST(PeriodicDeadline, LeavesFutureDeadlineUnchanged)
{
  const auto start = Clock::time_point{};
  const auto period = std::chrono::milliseconds(10);
  EXPECT_EQ(
    advance_periodic_deadline(
      start + std::chrono::milliseconds(20), start + std::chrono::milliseconds(15), period),
    start + std::chrono::milliseconds(20));
}

}  // namespace
}  // namespace unitree_g1_ros2_control