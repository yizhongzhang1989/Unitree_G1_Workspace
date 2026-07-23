#ifndef UNITREE_G1_ROS2_CONTROL__PERIODIC_DEADLINE_HPP_
#define UNITREE_G1_ROS2_CONTROL__PERIODIC_DEADLINE_HPP_

#include <chrono>

namespace unitree_g1_ros2_control {

inline std::chrono::steady_clock::time_point advance_periodic_deadline(
    std::chrono::steady_clock::time_point deadline,
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::duration period) {
    if (deadline > now) {
        return deadline;
    }
    return deadline + period * ((now - deadline) / period + 1);
}

}  // namespace unitree_g1_ros2_control

#endif  // UNITREE_G1_ROS2_CONTROL__PERIODIC_DEADLINE_HPP_