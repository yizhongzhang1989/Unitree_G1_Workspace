#ifndef UNITREE_G1_ROS2_CONTROL__GRAVITY_TABLE_HPP_
#define UNITREE_G1_ROS2_CONTROL__GRAVITY_TABLE_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace unitree_g1_ros2_control {

using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<double, 9>;

Vector3 rotate(const Matrix3& rotation, const Vector3& vector);
Matrix3 multiply(const Matrix3& left, const Matrix3& right);
/// Rodrigues rotation of `angle` about the unit `axis`.
Matrix3 axis_rotation(const Vector3& axis, double angle);
Matrix3 transpose(const Matrix3& matrix);
Vector3 cross(const Vector3& left, const Vector3& right);
double dot(const Vector3& left, const Vector3& right);
double norm(const Vector3& vector);

/// Resolve a leading ``~`` or a ``package://<name>/<relative>`` reference. The
/// calibration files ship inside a package, so a configured path has to
/// survive the workspace being moved or installed elsewhere.
std::string resolve_path(const std::string& path);

/// Gravity in the torso frame from an IMU quaternion ordered `x, y, z, w`.
/// Returns false for a quaternion that is not usable yet.
bool torso_gravity(
    const std::array<double, 4>& orientation, const Matrix3& imu_to_torso,
    Vector3& gravity);

/// One lumped rigid body of the reduced arm chain, as exported by the
/// calibration package. Every link welded to this body is already merged into
/// `mass` and `com`.
struct RigidBody {
    Vector3 axis{};
    Vector3 origin{};
    Matrix3 rotation{};
    Vector3 com{};
    double mass{0.0};
    std::size_t command_index{0};
};

/// The `gravity_table.yaml` exported by `arm_gravity_compensation`.
///
/// Both the gravity feed-forward and the force sensor compensation walk this
/// same chain, so the file is parsed once here instead of twice with two
/// slightly different conventions.
class GravityTable {
public:
    static constexpr std::size_t kArmJointCount = 7;
    static constexpr std::size_t kSideCount = 2;

    static const std::array<const char*, kSideCount>& side_names();

    /// Throws `std::runtime_error` describing what is wrong with the file.
    void load(const std::string& path);
    bool loaded() const noexcept { return loaded_; }
    /// Whether the file carries the force sensor mount, which older exports
    /// predate.
    bool has_sensor() const noexcept { return has_sensor_; }

    const std::vector<RigidBody>& arm(std::size_t side) const { return arms_[side]; }
    std::vector<RigidBody>& arm(std::size_t side) { return arms_[side]; }
    const std::vector<std::string>& joints(std::size_t side) const {
        return joints_[side];
    }
    const Matrix3& imu_to_torso() const noexcept { return imu_to_torso_; }
    const Vector3& sensor_origin(std::size_t side) const { return sensor_origin_[side]; }
    const Matrix3& sensor_rotation(std::size_t side) const {
        return sensor_rotation_[side];
    }

    /// Rotation from the force sensor frame to the torso frame, given the
    /// seven joint angles of `side` in table order.
    Matrix3 sensor_orientation(
        std::size_t side, const std::array<double, kArmJointCount>& angles) const;

private:
    std::array<std::vector<RigidBody>, kSideCount> arms_;
    std::array<std::vector<std::string>, kSideCount> joints_;
    std::array<Vector3, kSideCount> sensor_origin_{};
    std::array<Matrix3, kSideCount> sensor_rotation_{};
    Matrix3 imu_to_torso_{};
    bool loaded_{false};
    bool has_sensor_{false};
};

}  // namespace unitree_g1_ros2_control

#endif  // UNITREE_G1_ROS2_CONTROL__GRAVITY_TABLE_HPP_
