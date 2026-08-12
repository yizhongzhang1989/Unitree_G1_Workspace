#ifndef UNITREE_G1_ROS2_CONTROL__FT_COMPENSATION_HPP_
#define UNITREE_G1_ROS2_CONTROL__FT_COMPENSATION_HPP_

#include <array>
#include <string>

#include "unitree_g1_ros2_control/gravity_table.hpp"

namespace unitree_g1_ros2_control {

using Wrench = std::array<double, 6>;

/// What `arm_gravity_compensation` measured for one KWR57.
///
/// `*_bias` lives in the sensor's own measurement frame, `mass` and `com`
/// describe everything hanging off it - gripper, camera, cables - in the mount
/// link frame, taking moments about the link origin. `origin` is where the
/// sensor's own torque reference point sits in that frame: the vendor puts it
/// on the tool flange, a whole sensor height away from the link origin.
/// `polarity` absorbs the vendor's sign convention. The maths is the same as
/// `arm_gravity_compensation/ft_model.py`, which is what produced these numbers.
struct FtCalibration {
    Vector3 force_bias{};
    Vector3 torque_bias{};
    double mass{0.0};
    Vector3 com{};
    Matrix3 rotation{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    double polarity{1.0};
    Vector3 origin{};
    std::string frame;

    /// Read one side out of the exported parameter file. Throws
    /// `std::runtime_error` describing what is wrong with the file; returns
    /// false when the file simply has no entry for this side.
    static bool load(
        const std::string& path, const std::string& side, FtCalibration& calibration);
};

/// The tool's own weight, in the mount link frame.
Wrench tool_wrench(const FtCalibration& calibration, const Vector3& gravity);

/// What a payload or the environment applies to the tool side, in the mount
/// link frame and about its origin: the raw reading with the offset, the
/// tool's weight and the torque reference point all taken out.
Wrench net_wrench(
    const Wrench& raw, const FtCalibration& calibration, const Vector3& gravity);

/// Re-estimate only the offset from one unloaded reading. A large offset
/// drifts with temperature, and re-solving the whole calibration for that is
/// far too heavy when every other term is already known.
FtCalibration rezeroed(
    const Wrench& raw, const FtCalibration& calibration, const Vector3& gravity);

}  // namespace unitree_g1_ros2_control

#endif  // UNITREE_G1_ROS2_CONTROL__FT_COMPENSATION_HPP_
