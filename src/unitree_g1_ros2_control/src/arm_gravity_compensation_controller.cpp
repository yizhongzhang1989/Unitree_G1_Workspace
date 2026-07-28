#include "unitree_g1_ros2_control/arm_gravity_compensation_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/qos.hpp"
#include "yaml-cpp/yaml.h"

namespace unitree_g1_ros2_control {

namespace {

constexpr const char* kTorsoImuSensor = "torso_imu";
constexpr double kGravityMagnitude = 9.81;
// The fused attitude is preferred over the raw accelerometer because it also
// rejects the linear acceleration of a moving torso.
const std::array<std::string, 4> kImuInterfaceNames = {
    "orientation.x", "orientation.y", "orientation.z", "orientation.w",
};
const std::array<const char*, 2> kSideNames = {"left", "right"};

using Vector3 = std::array<double, 3>;
using Matrix3 = std::array<double, 9>;

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

controller_interface::CallbackReturn ArmGravityCompensationController::on_init() {
    try {
        auto_declare<std::vector<std::string>>("joints", {});
        auto_declare<std::string>("gravity_table", "");
        auto_declare<std::string>(
            "command_topic", "/forward_position_controller/commands");
        auto_declare<double>("gravity_filter_cutoff_hz", 2.0);
        auto_declare<double>("offset_ramp_s", 2.0);
        auto_declare<double>("compensation_scale", 1.0);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", error.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ArmGravityCompensationController::command_interface_configuration() const {
    // The downstream position controller owns the command interfaces; this one
    // only reshapes the target, so it claims nothing and can run alongside it.
    controller_interface::InterfaceConfiguration configuration;
    configuration.type = controller_interface::interface_configuration_type::NONE;
    return configuration;
}

controller_interface::InterfaceConfiguration
ArmGravityCompensationController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration configuration;
    configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& interface : kImuInterfaceNames) {
        configuration.names.push_back(std::string(kTorsoImuSensor) + "/" + interface);
    }
    // Read the stiffness the hardware really applies instead of copying the
    // gain file: a mismatch would scale every compensation torque by the same
    // wrong ratio without any visible symptom.
    for (const auto& chain : arms_) {
        for (const RigidBody& body : chain) {
            configuration.names.push_back(joint_names_[body.command_index] + "/kp");
        }
    }
    return configuration;
}

bool ArmGravityCompensationController::load_gravity_table(const std::string& path) {
    YAML::Node document;
    try {
        document = YAML::LoadFile(path);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(
            get_node()->get_logger(), "Cannot read gravity table %s: %s", path.c_str(),
            error.what());
        return false;
    }
    // Accept the exported file verbatim: it is written as a ROS 2 parameter
    // file so that it can also be fed to a node directly.
    YAML::Node table = document;
    if (document.size() == 1 && document.begin()->second["ros__parameters"]) {
        table = document.begin()->second["ros__parameters"];
    }

    try {
        const std::vector<double> rotation = read_doubles(table, "imu_to_torso", 9);
        std::copy(rotation.begin(), rotation.end(), imu_to_torso_.begin());

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
                const auto found = std::find(joint_names_.begin(), joint_names_.end(), name);
                if (found == joint_names_.end()) {
                    throw std::runtime_error(name + " is not in the joints parameter");
                }
                body.command_index =
                    static_cast<std::size_t>(std::distance(joint_names_.begin(), found));
            }
        }
    } catch (const std::exception& error) {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid gravity table: %s", error.what());
        return false;
    }
    return true;
}

ArmGravityCompensationController::CallbackReturn
ArmGravityCompensationController::on_configure(const rclcpp_lifecycle::State&) {
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    if (joint_names_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "The joints parameter must not be empty");
        return CallbackReturn::ERROR;
    }

    std::string table_path;
    try {
        table_path = resolve_path(get_node()->get_parameter("gravity_table").as_string());
    } catch (const std::exception& error) {
        RCLCPP_ERROR(get_node()->get_logger(), "Cannot resolve gravity_table: %s", error.what());
        return CallbackReturn::ERROR;
    }
    if (table_path.empty() || !load_gravity_table(table_path)) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "gravity_table must point at the file exported by arm_gravity_compensation");
        return CallbackReturn::ERROR;
    }

    filter_cutoff_hz_ = get_node()->get_parameter("gravity_filter_cutoff_hz").as_double();
    ramp_duration_ = get_node()->get_parameter("offset_ramp_s").as_double();
    if (!(filter_cutoff_hz_ > 0.0) || !(ramp_duration_ >= 0.0)) {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "gravity_filter_cutoff_hz must be positive and offset_ramp_s must not be negative");
        return CallbackReturn::ERROR;
    }

    const double scale = get_node()->get_parameter("compensation_scale").as_double();
    if (!std::isfinite(scale) || scale < 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "compensation_scale must be finite and non-negative");
        return CallbackReturn::ERROR;
    }
    compensation_scale_.store(scale, std::memory_order_relaxed);
    // Live tuning: the identified table still carries a few percent of common
    // mode error that only shows up as drift once the arm floats, and that is
    // far easier to trim by hand than to re-identify.
    parameter_callback_ = get_node()->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto& parameter : parameters) {
                if (parameter.get_name() != "compensation_scale") continue;
                const double value = parameter.as_double();
                if (!std::isfinite(value) || value < 0.0) {
                    result.successful = false;
                    result.reason = "compensation_scale must be finite and non-negative";
                    break;
                }
                compensation_scale_.store(value, std::memory_order_relaxed);
            }
            return result;
        });

    command_topic_ = get_node()->get_parameter("command_topic").as_string();
    if (command_topic_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "command_topic must not be empty");
        return CallbackReturn::ERROR;
    }
    const auto stream_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    command_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
            get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
                command_topic_, stream_qos));
    target_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/target", stream_qos,
        [this](std_msgs::msg::Float64MultiArray::SharedPtr message) {
            target_buffer_.writeFromNonRT(
                std::make_shared<std::vector<double>>(message->data));
        });
    target_buffer_.writeFromNonRT(std::shared_ptr<std::vector<double>>());
    return CallbackReturn::SUCCESS;
}

ArmGravityCompensationController::CallbackReturn
ArmGravityCompensationController::on_activate(const rclcpp_lifecycle::State&) {
    const std::size_t expected = kImuInterfaceNames.size() + 2 * kArmJointCount;
    if (state_interfaces_.size() != expected) {
        RCLCPP_ERROR(
            get_node()->get_logger(), "Expected %zu state interfaces, got %zu", expected,
            state_interfaces_.size());
        return CallbackReturn::ERROR;
    }
    for (std::size_t index = kImuInterfaceNames.size(); index < expected; ++index) {
        const double stiffness = state_interfaces_[index].get_value();
        if (!std::isfinite(stiffness) || stiffness <= 0.0) {
            RCLCPP_ERROR(
                get_node()->get_logger(), "%s reports a non-positive kp",
                state_interfaces_[index].get_name().c_str());
            return CallbackReturn::ERROR;
        }
    }
    gravity_valid_ = false;
    // The steady-state offset reaches ~0.4 rad on a loaded shoulder, so
    // applying it at once would step the command by twice the current droop.
    ramp_ = 0.0;
    target_buffer_.writeFromNonRT(std::shared_ptr<std::vector<double>>());
    return CallbackReturn::SUCCESS;
}

ArmGravityCompensationController::CallbackReturn
ArmGravityCompensationController::on_deactivate(const rclcpp_lifecycle::State&) {
    gravity_valid_ = false;
    return CallbackReturn::SUCCESS;
}

bool ArmGravityCompensationController::update_torso_gravity(double elapsed) {
    // The IMU reports the world-from-sensor rotation, so gravity in sensor
    // coordinates is its transpose applied to straight down. State order is
    // x, y, z, w.
    const double x = state_interfaces_[0].get_value();
    const double y = state_interfaces_[1].get_value();
    const double z = state_interfaces_[2].get_value();
    const double w = state_interfaces_[3].get_value();
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

void ArmGravityCompensationController::arm_offsets(
    std::size_t side, const std::vector<double>& target, double gain,
    std::vector<double>& command) const {
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
        // The stiffness comes straight from the hardware, in the same order the
        // state interfaces were requested.
        const double stiffness =
            state_interfaces_[kImuInterfaceNames.size() + side * kArmJointCount + index]
                .get_value();
        // Deliberately unbounded: the offset is pure feed-forward from a
        // validated table and a normalised gravity vector, so it is already
        // bounded by the payload it was calibrated with. Clamping it would only
        // ever under-compensate a heavier load, and nothing downstream clamps
        // to joint limits either - MIT impedance derives its torque from the
        // command-to-feedback offset, so truncating it changes the force.
        // The gains are fixed at hardware configuration time and on_activate
        // rejected every non-positive one, so the division is always defined.
        command[command_index] = target[command_index] + gain * torque / stiffness;
    }
}

controller_interface::return_type ArmGravityCompensationController::update(
    const rclcpp::Time&, const rclcpp::Duration& period) {
    const auto sample = *target_buffer_.readFromRT();
    if (!sample) return controller_interface::return_type::OK;

    // The control loop hands us its own measured period, so the filter and the
    // ramp advance on the same clock the hardware is written with.
    const double elapsed = period.seconds();
    ramp_ = ramp_duration_ > 0.0 ? std::min(1.0, ramp_ + elapsed / ramp_duration_) : 1.0;
    if (!update_torso_gravity(elapsed)) return controller_interface::return_type::OK;

    if (sample->size() != joint_names_.size() ||
        !std::all_of(sample->begin(), sample->end(), [](double value) {
            return std::isfinite(value);
        })) {
        RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 2000,
            "Discarding invalid target: expected %zu finite values", joint_names_.size());
        return controller_interface::return_type::OK;
    }

    // Republished every cycle rather than only on a new target: the offset has
    // to keep following the torso attitude and the activation ramp even while
    // the upstream target stands still. Built straight inside the realtime
    // message, so the 31 values are only copied once per cycle.
    if (!command_publisher_->trylock()) return controller_interface::return_type::OK;
    std::vector<double>& command = command_publisher_->msg_.data;
    command = *sample;
    const double gain = ramp_ * compensation_scale_.load(std::memory_order_relaxed);
    for (std::size_t side = 0; side < arms_.size(); ++side) {
        arm_offsets(side, *sample, gain, command);
    }
    command_publisher_->unlockAndPublish();
    return controller_interface::return_type::OK;
}

}  // namespace unitree_g1_ros2_control

PLUGINLIB_EXPORT_CLASS(
    unitree_g1_ros2_control::ArmGravityCompensationController,
    controller_interface::ControllerInterface)
