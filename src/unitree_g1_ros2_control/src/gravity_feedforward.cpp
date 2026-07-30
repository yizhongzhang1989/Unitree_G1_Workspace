#include "unitree_g1_ros2_control/gravity_feedforward.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter.hpp"
#include "yaml-cpp/yaml.h"

namespace unitree_g1_ros2_control {

namespace {

constexpr double kGravityMagnitude = 9.81;
const std::array<const char*, 2> kSideNames = {"left", "right"};

using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<double, 9>;

template <typename T>
void declare_if_missing(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& name,
    const T& value) {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, value);
}

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

/// Rodrigues rotation of `angle` about the unit `axis`.
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

/// Resolve a leading ``~`` or a ``package://<name>/<relative>`` reference. The
/// gravity table ships inside the calibration package, so the configured path
/// has to survive the workspace being moved or installed elsewhere.
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

/// A controller declares every parameter of its YAML from the overrides, and
/// those declarations are dynamically typed, so `ros2 param set ...
/// compensation_scale 0` passes the type check and arrives here as an integer -
/// where `as_double()` would throw out of the parameter service and abort the
/// whole process. Anything that is not a number becomes NaN and is rejected by
/// the finiteness check the value goes through anyway.
double as_number(const rclcpp::Parameter& parameter) {
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        return static_cast<double>(parameter.as_int());
    }
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return parameter.as_double();
    }
    return std::nan("");
}

std::vector<double> read_doubles(
    const YAML::Node& node, const std::string& key, std::size_t expected) {
    const YAML::Node entry = node[key];
    if (!entry || !entry.IsSequence() || entry.size() != expected) {
        throw std::runtime_error(key + " must hold " + std::to_string(expected) + " values");
    }
    std::vector<double> values;
    values.reserve(expected);
    for (const auto& item : entry) {
        const double value = item.as<double>();
        if (!std::isfinite(value)) throw std::runtime_error(key + " holds a non-finite value");
        values.push_back(value);
    }
    return values;
}

}  // namespace

const char* GravityFeedforward::torso_imu_sensor() { return "torso_imu"; }

const std::array<std::string, 4>& GravityFeedforward::imu_interface_names() {
    // The fused attitude is preferred over the raw accelerometer because it
    // also rejects the linear acceleration of a moving torso.
    static const std::array<std::string, 4> names = {
        "orientation.x", "orientation.y", "orientation.z", "orientation.w",
    };
    return names;
}

void GravityFeedforward::declare_parameters(const LifecycleNode::SharedPtr& node) {
    declare_if_missing<std::string>(node, "gravity_table", "");
    declare_if_missing<double>(node, "gravity_filter_cutoff_hz", 2.0);
    declare_if_missing<double>(node, "offset_ramp_s", 2.0);
    declare_if_missing<double>(node, "compensation_scale", 1.0);
}

bool GravityFeedforward::configure(
    const LifecycleNode::SharedPtr& node, const std::vector<std::string>& joint_names) {
    const auto table_path = node->get_parameter("gravity_table").as_string();
    if (table_path.empty()) return true;

    try {
        load_gravity_table(table_path, joint_names);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(
            node->get_logger(),
            "gravity_table must point at the file exported by arm_gravity_compensation: %s",
            error.what());
        return false;
    }

    const double cutoff = node->get_parameter("gravity_filter_cutoff_hz").as_double();
    ramp_duration_ = node->get_parameter("offset_ramp_s").as_double();
    if (!(cutoff > 0.0) || !(ramp_duration_ >= 0.0)) {
        RCLCPP_ERROR(
            node->get_logger(),
            "gravity_filter_cutoff_hz must be positive and offset_ramp_s must not be negative");
        return false;
    }
    filter_cutoff_hz_ = cutoff;

    const double scale = node->get_parameter("compensation_scale").as_double();
    if (!std::isfinite(scale) || scale < 0.0) {
        RCLCPP_ERROR(node->get_logger(), "compensation_scale must be finite and non-negative");
        return false;
    }
    compensation_scale_.store(scale, std::memory_order_relaxed);
    // Live tuning: the identified table still carries a few percent of common
    // mode error that only shows up as drift once the arm floats, and that is
    // far easier to trim by hand than to re-identify.
    parameter_callback_ = node->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto& parameter : parameters) {
                if (parameter.get_name() != "compensation_scale") continue;
                const double value = as_number(parameter);
                if (!std::isfinite(value) || value < 0.0) {
                    result.successful = false;
                    result.reason = "compensation_scale must be a finite non-negative number";
                    break;
                }
                compensation_scale_.store(value, std::memory_order_relaxed);
            }
            return result;
        });
    stiffness_.assign(compensated_indices_.size(), 0.0);
    return true;
}

void GravityFeedforward::append_state_interfaces(
    const std::vector<std::string>& joint_names, std::vector<std::string>& names) const {
    if (!loaded_) return;
    for (const auto& interface : imu_interface_names()) {
        names.push_back(std::string(torso_imu_sensor()) + "/" + interface);
    }
    // Read the stiffness the hardware really applies instead of copying the
    // gain file: a mismatch would scale every compensation torque by the same
    // wrong ratio without any visible symptom.
    for (const std::size_t index : compensated_indices_) {
        names.push_back(joint_names[index] + "/kp");
    }
}

std::size_t GravityFeedforward::state_interface_count() const {
    if (!loaded_) return 0;
    return imu_interface_names().size() + compensated_indices_.size();
}

bool GravityFeedforward::activate(
    const std::vector<hardware_interface::LoanedStateInterface>& interfaces,
    const rclcpp::Logger& logger) {
    if (!loaded_) return true;
    if (interfaces.size() < state_interface_count()) {
        RCLCPP_ERROR(
            logger, "Expected at least %zu state interfaces for gravity compensation, got %zu",
            state_interface_count(), interfaces.size());
        return false;
    }
    const std::size_t stiffness_offset = interfaces.size() - compensated_indices_.size();
    for (std::size_t index = 0; index < stiffness_.size(); ++index) {
        const double stiffness = interfaces[stiffness_offset + index].get_value();
        if (!std::isfinite(stiffness) || stiffness <= 0.0) {
            RCLCPP_ERROR(
                logger, "%s reports a non-positive kp",
                interfaces[stiffness_offset + index].get_name().c_str());
            return false;
        }
    }
    reset();
    // The steady-state offset reaches ~0.4 rad on a loaded shoulder, so
    // applying it at once would step the command by twice the current droop.
    ramp_ = 0.0;
    return true;
}

bool GravityFeedforward::apply(
    const std::vector<hardware_interface::LoanedStateInterface>& interfaces,
    const rclcpp::Duration& period, const std::vector<double>& target,
    std::vector<double>& command) {
    if (!loaded_) return false;
    const double scale = compensation_scale_.load(std::memory_order_relaxed);
    if (scale <= 0.001) {
        // Switched off at runtime. The point is not the saved arithmetic - that
        // is under a microsecond - but the ramp: letting it climb to full while
        // the offset is multiplied away would apply the whole ~0.4 rad at once
        // the moment the scale comes back.
        ramp_ = 0.0;
        return false;
    }
    // The control loop hands us its own measured period, so the filter and the
    // ramp advance on the same clock the hardware is written with.
    const double elapsed = period.seconds();
    ramp_ = ramp_duration_ > 0.0 ? std::min(1.0, ramp_ + elapsed / ramp_duration_) : 1.0;
    const std::size_t offset = interfaces.size() - state_interface_count();
    const std::array<double, 4> orientation = {
        interfaces[offset].get_value(),
        interfaces[offset + 1].get_value(),
        interfaces[offset + 2].get_value(),
        interfaces[offset + 3].get_value(),
    };
    if (!update_torso_gravity(orientation, elapsed)) return false;

    const std::size_t stiffness_offset = interfaces.size() - compensated_indices_.size();
    for (std::size_t index = 0; index < stiffness_.size(); ++index) {
        stiffness_[index] = interfaces[stiffness_offset + index].get_value();
    }
    command = target;
    for (std::size_t side = 0; side < kSideCount; ++side) {
        arm_offsets(side, target, ramp_ * scale, stiffness_, command);
    }
    return true;
}

void GravityFeedforward::load_gravity_table(
    const std::string& path, const std::vector<std::string>& joint_names) {
    loaded_ = false;
    compensated_indices_.clear();

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

    compensated_indices_.reserve(kSideCount * kArmJointCount);
    for (std::size_t side = 0; side < kSideNames.size(); ++side) {
        const YAML::Node chain = table[kSideNames[side]];
        if (!chain) throw std::runtime_error(std::string(kSideNames[side]) + " is missing");
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
        for (std::size_t index = 0; index < kArmJointCount; ++index) {
            RigidBody& body = arms_[side][index];
            std::copy_n(axes.begin() + 3 * index, 3, body.axis.begin());
            std::copy_n(origins.begin() + 3 * index, 3, body.origin.begin());
            std::copy_n(rotations.begin() + 9 * index, 9, body.rotation.begin());
            std::copy_n(centres.begin() + 3 * index, 3, body.com.begin());
            body.mass = masses[index];
            if (body.mass < 0.0) throw std::runtime_error("mass must be non-negative");
            const auto name = names[index].as<std::string>();
            const auto found = std::find(joint_names.begin(), joint_names.end(), name);
            if (found == joint_names.end()) {
                throw std::runtime_error(name + " is not in the joints parameter");
            }
            body.command_index =
                static_cast<std::size_t>(std::distance(joint_names.begin(), found));
            compensated_indices_.push_back(body.command_index);
        }
    }
    loaded_ = true;
}

bool GravityFeedforward::update_torso_gravity(
    const std::array<double, 4>& orientation, double elapsed) {
    // The IMU reports the world-from-sensor rotation, so gravity in sensor
    // coordinates is its transpose applied to straight down. State order is
    // x, y, z, w.
    const double x = orientation[0];
    const double y = orientation[1];
    const double z = orientation[2];
    const double w = orientation[3];
    const double square = w * w + x * x + y * y + z * z;
    if (!std::isfinite(square) || std::abs(square - 1.0) > 0.1) return gravity_valid_;

    const Vector3 torso = rotate(imu_to_torso_, {
        -2.0 * (x * z - w * y),
        -2.0 * (y * z + w * x),
        -(w * w - x * x - y * y + z * z),
    });
    const double length = norm(torso);
    // Only a degenerate imu_to_torso can collapse an already unit quaternion.
    if (length < 1e-6) return gravity_valid_;

    const double alpha =
        gravity_valid_ ? 1.0 - std::exp(-2.0 * M_PI * filter_cutoff_hz_ * elapsed) : 1.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const double measured = kGravityMagnitude * torso[axis] / length;
        gravity_[axis] += alpha * (measured - gravity_[axis]);
    }
    gravity_valid_ = true;
    return true;
}

void GravityFeedforward::arm_offsets(
    std::size_t side, const std::vector<double>& target, double gain,
    const std::vector<double>& stiffness, std::vector<double>& command) const {
    const std::vector<RigidBody>& chain = arms_[side];
    std::array<Vector3, kArmJointCount> origins{};
    std::array<Vector3, kArmJointCount> axes{};
    std::array<Vector3, kArmJointCount> moments{};

    Matrix3 rotation = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    Vector3 translation{};
    for (std::size_t index = 0; index < kArmJointCount; ++index) {
        const RigidBody& body = chain[index];
        // Evaluating at the requested pose rather than the measured one keeps
        // the feed-forward noise free and makes the equilibrium land on the
        // target instead of trailing behind it.
        const double angle = target[body.command_index];
        const Vector3 offset = rotate(rotation, body.origin);
        for (std::size_t axis = 0; axis < 3; ++axis) translation[axis] += offset[axis];
        rotation = multiply(rotation, body.rotation);
        rotation = multiply(rotation, axis_rotation(body.axis, angle));

        origins[index] = translation;
        axes[index] = rotate(rotation, body.axis);
        const Vector3 centre = rotate(rotation, body.com);
        for (std::size_t axis = 0; axis < 3; ++axis) {
            moments[index][axis] = body.mass * (translation[axis] + centre[axis]);
        }
    }

    // One backward pass: walking inwards, the running sums are already exactly
    // the mass and moment hanging off the next joint, so the whole chain costs
    // seven steps instead of re-summing the tail at every joint.
    double downstream_mass = 0.0;
    Vector3 downstream_moment{};
    for (std::size_t index = kArmJointCount; index-- > 0;) {
        downstream_mass += chain[index].mass;
        Vector3 lever{};
        for (std::size_t axis = 0; axis < 3; ++axis) {
            downstream_moment[axis] += moments[index][axis];
            lever[axis] = downstream_moment[axis] - downstream_mass * origins[index][axis];
        }
        const double torque = -dot(axes[index], cross(lever, gravity_));
        const std::size_t command_index = chain[index].command_index;
        // The stiffness comes straight from the hardware, in the same order
        // `compensated_indices()` reports the slots it belongs to.
        const double gain_value = stiffness[side * kArmJointCount + index];
        // Deliberately unbounded: the offset is pure feed-forward from a
        // validated table and a normalised gravity vector, so it is already
        // bounded by the payload it was calibrated with. Clamping it would only
        // ever under-compensate a heavier load, and nothing downstream clamps
        // to joint limits either - MIT impedance derives its torque from the
        // command-to-feedback offset, so truncating it changes the force.
        // The gains are fixed at hardware configuration time and activation
        // rejected every non-positive one, so the division is always defined.
        command[command_index] = target[command_index] + gain * torque / gain_value;
    }
}

}  // namespace unitree_g1_ros2_control
