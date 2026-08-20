#include "unitree_g1_ros2_control/gravity_table.hpp"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

namespace unitree_g1_ros2_control {

namespace {

std::vector<double> read_doubles(
    const YAML::Node& node, const std::string& key, std::size_t expected) {
    const YAML::Node entry = node[key];
    if (!entry || !entry.IsSequence() || entry.size() != expected) {
        throw std::runtime_error(
            key + " must hold " + std::to_string(expected) + " values");
    }
    std::vector<double> values;
    values.reserve(expected);
    for (const auto& item : entry) {
        const double value = item.as<double>();
        if (!std::isfinite(value)) {
            throw std::runtime_error(key + " holds a non-finite value");
        }
        values.push_back(value);
    }
    return values;
}

}  // namespace

Vector3 rotate(const Matrix3& rotation, const Vector3& vector) {
    return {
        rotation[0] * vector[0] + rotation[1] * vector[1] + rotation[2] * vector[2],
        rotation[3] * vector[0] + rotation[4] * vector[1] + rotation[5] * vector[2],
        rotation[6] * vector[0] + rotation[7] * vector[1] + rotation[8] * vector[2],
    };
}

Matrix3 multiply(const Matrix3& left, const Matrix3& right) {
    Matrix3 result{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            double sum = 0.0;
            for (std::size_t inner = 0; inner < 3; ++inner) {
                sum += left[3 * row + inner] * right[3 * inner + column];
            }
            result[3 * row + column] = sum;
        }
    }
    return result;
}

Matrix3 axis_rotation(const Vector3& axis, double angle) {
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    const double complement = 1.0 - cosine;
    return {
        cosine + axis[0] * axis[0] * complement,
        axis[0] * axis[1] * complement - axis[2] * sine,
        axis[0] * axis[2] * complement + axis[1] * sine,
        axis[1] * axis[0] * complement + axis[2] * sine,
        cosine + axis[1] * axis[1] * complement,
        axis[1] * axis[2] * complement - axis[0] * sine,
        axis[2] * axis[0] * complement - axis[1] * sine,
        axis[2] * axis[1] * complement + axis[0] * sine,
        cosine + axis[2] * axis[2] * complement,
    };
}

Matrix3 transpose(const Matrix3& matrix) {
    return {
        matrix[0], matrix[3], matrix[6],
        matrix[1], matrix[4], matrix[7],
        matrix[2], matrix[5], matrix[8],
    };
}

Vector3 cross(const Vector3& left, const Vector3& right) {
    return {
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    };
}

double dot(const Vector3& left, const Vector3& right) {
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
}

double norm(const Vector3& vector) { return std::sqrt(dot(vector, vector)); }

std::string resolve_path(const std::string& path) {
    constexpr const char* kPackagePrefix = "package://";
    const std::size_t prefix_length = std::strlen(kPackagePrefix);
    if (path.compare(0, prefix_length, kPackagePrefix) == 0) {
        const std::size_t separator = path.find('/', prefix_length);
        if (separator == std::string::npos) {
            throw std::runtime_error(path + " is missing a path after the package name");
        }
        const std::string package =
            path.substr(prefix_length, separator - prefix_length);
        return ament_index_cpp::get_package_share_directory(package) +
               path.substr(separator);
    }
    if (path.empty() || path[0] != '~') return path;
    const char* home = std::getenv("HOME");
    if (home == nullptr) return path;
    return std::string(home) + path.substr(1);
}

bool torso_gravity(
    const std::array<double, 4>& orientation, const Matrix3& imu_to_torso,
    Vector3& gravity) {
    // The IMU reports the world-from-sensor rotation, so gravity in sensor
    // coordinates is its transpose applied to straight down. State order is
    // x, y, z, w.
    const double x = orientation[0];
    const double y = orientation[1];
    const double z = orientation[2];
    const double w = orientation[3];
    const double square = w * w + x * x + y * y + z * z;
    if (!std::isfinite(square) || std::abs(square - 1.0) > 0.1) return false;

    const Vector3 torso = rotate(imu_to_torso, {
        -2.0 * (x * z - w * y),
        -2.0 * (y * z + w * x),
        -(w * w - x * x - y * y + z * z),
    });
    const double length = norm(torso);
    // Only a degenerate imu_to_torso can collapse an already unit quaternion.
    if (length < 1e-6) return false;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        gravity[axis] = torso[axis] / length;
    }
    return true;
}

const std::array<const char*, GravityTable::kSideCount>& GravityTable::side_names() {
    static const std::array<const char*, kSideCount> names = {"left", "right"};
    return names;
}

void GravityTable::load(const std::string& path) {
    loaded_ = false;
    has_sensor_ = false;

    const std::string resolved = resolve_path(path);
    YAML::Node document;
    try {
        document = YAML::LoadFile(resolved);
    } catch (const std::exception& error) {
        throw std::runtime_error(
            "cannot read gravity table " + resolved + ": " + error.what());
    }
    // Accept the exported file verbatim: it is written as a ROS 2 parameter
    // file so that it can also be fed to a node directly.
    YAML::Node table = document;
    if (document.size() == 1 && document.begin()->second["ros__parameters"]) {
        table = document.begin()->second["ros__parameters"];
    }

    const std::vector<double> rotation = read_doubles(table, "imu_to_torso", 9);
    std::copy(rotation.begin(), rotation.end(), imu_to_torso_.begin());

    bool sensors = true;
    for (std::size_t side = 0; side < kSideCount; ++side) {
        const YAML::Node chain = table[side_names()[side]];
        if (!chain) throw std::runtime_error(std::string(side_names()[side]) + " is missing");
        const YAML::Node names = chain["joints"];
        if (!names || names.size() != kArmJointCount) {
            throw std::runtime_error("joints must hold seven names");
        }
        const auto axes = read_doubles(chain, "axis", 3 * kArmJointCount);
        const auto origins = read_doubles(chain, "origin_xyz", 3 * kArmJointCount);
        const auto rotations = read_doubles(chain, "origin_rotation", 9 * kArmJointCount);
        const auto masses = read_doubles(chain, "mass", kArmJointCount);
        const auto centres = read_doubles(chain, "com", 3 * kArmJointCount);

        arms_[side].assign(kArmJointCount, RigidBody{});
        joints_[side].clear();
        for (std::size_t index = 0; index < kArmJointCount; ++index) {
            RigidBody& body = arms_[side][index];
            std::copy_n(axes.begin() + 3 * index, 3, body.axis.begin());
            std::copy_n(origins.begin() + 3 * index, 3, body.origin.begin());
            std::copy_n(rotations.begin() + 9 * index, 9, body.rotation.begin());
            std::copy_n(centres.begin() + 3 * index, 3, body.com.begin());
            body.mass = masses[index];
            if (body.mass < 0.0) throw std::runtime_error("mass must be non-negative");
            joints_[side].push_back(names[index].as<std::string>());
        }

        // Tables exported before the force sensor was calibrated have no
        // mount, and everything except the payload path works without it.
        if (chain["payload_origin_xyz"] && chain["payload_origin_rotation"]) {
            const auto origin = read_doubles(chain, "payload_origin_xyz", 3);
            const auto mount = read_doubles(chain, "payload_origin_rotation", 9);
            std::copy(origin.begin(), origin.end(), sensor_origin_[side].begin());
            std::copy(mount.begin(), mount.end(), sensor_rotation_[side].begin());
        } else {
            sensors = false;
        }
    }
    has_sensor_ = sensors;
    loaded_ = true;
}

void FrictionTable::load(
    const std::string& path,
    const std::array<std::vector<std::string>, kSideCount>& joints) {
    loaded_ = false;
    for (auto& side : load_ratio_) side.fill(0.0);
    for (auto& side : offset_) side.fill(0.0);

    const std::string resolved = resolve_path(path);
    YAML::Node document;
    try {
        document = YAML::LoadFile(resolved);
    } catch (const std::exception& error) {
        throw std::runtime_error(
            "cannot read friction table " + resolved + ": " + error.what());
    }
    YAML::Node table = document;
    if (document.size() == 1 && document.begin()->second["ros__parameters"]) {
        table = document.begin()->second["ros__parameters"];
    }

    for (std::size_t side = 0; side < kSideCount; ++side) {
        const YAML::Node block = table[GravityTable::side_names()[side]];
        // A side with no two-sided samples is left out rather than zero
        // filled, so absence here is "not measured", not "no friction".
        if (!block) continue;
        const YAML::Node names = block["joints"];
        if (!names || names.size() != kArmJointCount) {
            throw std::runtime_error("friction table joints must hold seven names");
        }
        for (std::size_t index = 0; index < kArmJointCount; ++index) {
            if (names[index].as<std::string>() != joints[side][index]) {
                throw std::runtime_error(
                    "friction table joint order does not match the gravity table: " +
                    names[index].as<std::string>() + " where " + joints[side][index] +
                    " was expected");
            }
        }
        const auto ratios = read_doubles(block, "load_ratio", kArmJointCount);
        const auto offsets = read_doubles(block, "offset", kArmJointCount);
        for (std::size_t index = 0; index < kArmJointCount; ++index) {
            if (ratios[index] < 0.0) {
                throw std::runtime_error("load_ratio must be non-negative");
            }
            load_ratio_[side][index] = ratios[index];
            offset_[side][index] = offsets[index];
        }
    }
    loaded_ = true;
}

Matrix3 GravityTable::sensor_orientation(
    std::size_t side, const std::array<double, kArmJointCount>& angles) const {
    Matrix3 rotation = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    for (std::size_t index = 0; index < kArmJointCount; ++index) {
        const RigidBody& body = arms_[side][index];
        rotation = multiply(rotation, body.rotation);
        rotation = multiply(rotation, axis_rotation(body.axis, angles[index]));
    }
    return multiply(rotation, sensor_rotation_[side]);
}

}  // namespace unitree_g1_ros2_control
