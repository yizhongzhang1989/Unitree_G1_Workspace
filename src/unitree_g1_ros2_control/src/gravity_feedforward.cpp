#include "unitree_g1_ros2_control/gravity_feedforward.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter.hpp"

namespace unitree_g1_ros2_control {

namespace {

constexpr double kGravityMagnitude = 9.81;

template <typename T>
void declare_if_missing(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& name,
    const T& value) {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, value);
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
    declare_if_missing<std::string>(node, "left_payload_topic", "/arm0/payload");
    declare_if_missing<std::string>(node, "right_payload_topic", "/arm1/payload");
    declare_if_missing<double>(node, "maximum_payload_mass", 3.0);
    declare_if_missing<double>(node, "payload_filter_tau_s", 1.0);
    declare_if_missing<double>(node, "payload_timeout_s", 2.0);
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
    subscribe_payload(node);
    return true;
}

void GravityFeedforward::subscribe_payload(const LifecycleNode::SharedPtr& node) {
    maximum_payload_mass_ = node->get_parameter("maximum_payload_mass").as_double();
    payload_filter_tau_ = node->get_parameter("payload_filter_tau_s").as_double();
    payload_timeout_ = node->get_parameter("payload_timeout_s").as_double();
    // Without the mount there is nowhere to hang a payload; the rest of the
    // feed-forward is unaffected.
    if (!table_.has_sensor()) return;

    const std::array<std::string, kSideCount> parameters = {
        "left_payload_topic", "right_payload_topic"};
    for (std::size_t side = 0; side < kSideCount; ++side) {
        const auto topic = node->get_parameter(parameters[side]).as_string();
        if (topic.empty()) continue;
        payload_subscription_[side] =
            node->create_subscription<geometry_msgs::msg::InertiaStamped>(
                topic, rclcpp::SensorDataQoS().keep_last(1),
                [this, side](geometry_msgs::msg::InertiaStamped::ConstSharedPtr message) {
                    accept_payload(side, *message);
                });
    }
}

void GravityFeedforward::accept_payload(
    std::size_t side, const geometry_msgs::msg::InertiaStamped& message) {
    const double mass = message.inertia.m;
    const Vector3 centre = {
        message.inertia.com.x, message.inertia.com.y, message.inertia.com.z};
    if (!std::isfinite(mass) || mass < 0.0 || mass > maximum_payload_mass_) return;
    if (!std::isfinite(centre[0]) || !std::isfinite(centre[1]) ||
        !std::isfinite(centre[2]) || norm(centre) > 1.0) {
        return;
    }
    payload_buffer_[side].writeFromNonRT(PayloadCommand{
        mass, {mass * centre[0], mass * centre[1], mass * centre[2]}});
    payload_sequence_[side].fetch_add(1, std::memory_order_relaxed);
}

void GravityFeedforward::advance_payload(double elapsed) {
    for (std::size_t side = 0; side < kSideCount; ++side) {
        const uint64_t sequence = payload_sequence_[side].load(std::memory_order_relaxed);
        if (sequence != payload_seen_[side]) {
            payload_seen_[side] = sequence;
            payload_age_[side] = 0.0;
        } else {
            payload_age_[side] += elapsed;
        }
        PayloadCommand target{};
        // A publisher that died must not leave a phantom load holding the arm
        // up, so an expired payload fades out instead of latching.
        if (payload_age_[side] <= payload_timeout_) {
            target = *payload_buffer_[side].readFromRT();
        }
        const double alpha = payload_filter_tau_ > 0.0
            ? 1.0 - std::exp(-elapsed / payload_filter_tau_)
            : 1.0;
        payload_mass_[side] += alpha * (target.mass - payload_mass_[side]);
        for (std::size_t axis = 0; axis < 3; ++axis) {
            payload_moment_[side][axis] +=
                alpha * (target.first_moment[axis] - payload_moment_[side][axis]);
        }
    }
}

void GravityFeedforward::set_payload(
    std::size_t side, double mass, const Vector3& first_moment) {
    payload_mass_[side] = mass;
    payload_moment_[side] = first_moment;
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
    for (std::size_t side = 0; side < kSideCount; ++side) {
        payload_mass_[side] = 0.0;
        payload_moment_[side] = Vector3{};
        payload_age_[side] = payload_timeout_ + 1.0;
    }
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
    advance_payload(elapsed);

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

    table_.load(path);
    compensated_indices_.reserve(kSideCount * kArmJointCount);
    for (std::size_t side = 0; side < kSideCount; ++side) {
        for (std::size_t index = 0; index < kArmJointCount; ++index) {
            const std::string& name = table_.joints(side)[index];
            const auto found = std::find(joint_names.begin(), joint_names.end(), name);
            if (found == joint_names.end()) {
                throw std::runtime_error(name + " is not in the joints parameter");
            }
            table_.arm(side)[index].command_index =
                static_cast<std::size_t>(std::distance(joint_names.begin(), found));
            compensated_indices_.push_back(table_.arm(side)[index].command_index);
        }
    }
    loaded_ = true;
}

bool GravityFeedforward::update_torso_gravity(
    const std::array<double, 4>& orientation, double elapsed) {
    Vector3 direction{};
    if (!torso_gravity(orientation, table_.imu_to_torso(), direction)) {
        return gravity_valid_;
    }
    const double alpha =
        gravity_valid_ ? 1.0 - std::exp(-2.0 * M_PI * filter_cutoff_hz_ * elapsed) : 1.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        gravity_[axis] += alpha * (kGravityMagnitude * direction[axis] - gravity_[axis]);
    }
    gravity_valid_ = true;
    return true;
}

void GravityFeedforward::arm_offsets(
    std::size_t side, const std::vector<double>& target, double gain,
    const std::vector<double>& stiffness, std::vector<double>& command) const {
    const std::vector<RigidBody>& chain = table_.arm(side);
    std::array<Vector3, kArmJointCount> origins{};
    std::array<Vector3, kArmJointCount> axes{};
    std::array<Vector3, kArmJointCount> moments{};

    // The payload rides on the force sensor, which is welded to the last body,
    // so merging the two keeps the chain seven bodies long and every joint
    // upstream picks it up through the same backward pass.
    constexpr std::size_t last = kArmJointCount - 1;
    RigidBody carrier = chain[last];
    if (payload_mass_[side] > 0.0) {
        const Vector3 mount = rotate(
            table_.sensor_rotation(side), payload_moment_[side]);
        const double total = carrier.mass + payload_mass_[side];
        for (std::size_t axis = 0; axis < 3; ++axis) {
            carrier.com[axis] =
                (carrier.mass * carrier.com[axis] + mount[axis] +
                 payload_mass_[side] * table_.sensor_origin(side)[axis]) / total;
        }
        carrier.mass = total;
    }

    Matrix3 rotation = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    Vector3 translation{};
    for (std::size_t index = 0; index < kArmJointCount; ++index) {
        const RigidBody& body = index == last ? carrier : chain[index];
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
        downstream_mass += index == last ? carrier.mass : chain[index].mass;
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
