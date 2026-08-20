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

/// Same dynamic typing hazard as `as_number`: a YAML array written without
/// decimal points arrives as an integer array. An empty result means the
/// parameter was not a numeric array at all.
std::vector<double> as_number_array(const rclcpp::Parameter& parameter) {
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
        const auto values = parameter.as_integer_array();
        return std::vector<double>(values.begin(), values.end());
    }
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        return parameter.as_double_array();
    }
    return {};
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
    declare_if_missing<std::string>(node, "friction_table", "");
    declare_if_missing<double>(node, "gravity_filter_cutoff_hz", 2.0);
    declare_if_missing<double>(node, "offset_ramp_s", 2.0);
    declare_if_missing<double>(node, "compensation_scale", 1.0);
    declare_if_missing<double>(node, "friction_scale", 0.0);
    // Never write these as an empty list in YAML: an untyped override aborts
    // the whole node when rclcpp tries to convert it to vector<double>.
    declare_if_missing<std::vector<double>>(node, "friction_load_ratio", {});
    declare_if_missing<std::vector<double>>(node, "friction_offset_nm", {});
    declare_if_missing<double>(node, "friction_error_epsilon", 0.05);
    declare_if_missing<double>(node, "friction_velocity_epsilon", 0.02);
    declare_if_missing<double>(node, "target_velocity_cutoff_hz", 5.0);
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

    // Empty leaves every coefficient at zero, which is no friction
    // compensation - the behaviour before the term existed.
    const auto friction_path = node->get_parameter("friction_table").as_string();
    if (!friction_path.empty()) {
        try {
            load_friction_table(friction_path);
        } catch (const std::exception& error) {
            RCLCPP_ERROR(
                node->get_logger(),
                "friction_table must point at the file exported by "
                "arm_gravity_compensation: %s",
                error.what());
            return false;
        }
    }

    const double cutoff = node->get_parameter("gravity_filter_cutoff_hz").as_double();    ramp_duration_ = node->get_parameter("offset_ramp_s").as_double();
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

    const double velocity_cutoff = node->get_parameter("target_velocity_cutoff_hz").as_double();
    if (!(velocity_cutoff > 0.0)) {
        RCLCPP_ERROR(node->get_logger(), "target_velocity_cutoff_hz must be positive");
        return false;
    }
    velocity_cutoff_hz_ = velocity_cutoff;

    for (const char* name : {"friction_scale", "friction_load_ratio", "friction_offset_nm",
                             "friction_error_epsilon", "friction_velocity_epsilon"}) {
        std::string reason;
        if (!store_tuning_parameter(node->get_parameter(name), reason)) {
            RCLCPP_ERROR(node->get_logger(), "%s", reason.c_str());
            return false;
        }
    }

    // Live tuning: the identified table still carries a few percent of common
    // mode error that only shows up as drift once the arm floats, and that is
    // far easier to trim by hand than to re-identify. The friction gains are
    // here for a different reason - over-compensating friction rings, so they
    // have to be raised a notch at a time against the real arm.
    parameter_callback_ = node->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto& parameter : parameters) {
                if (!store_tuning_parameter(parameter, result.reason)) {
                    result.successful = false;
                    break;
                }
            }
            return result;
        });
    previous_target_.assign(joint_names.size(), 0.0);
    target_velocity_.assign(joint_names.size(), 0.0);
    target_velocity_valid_ = false;
    subscribe_payload(node);
    return true;
}

bool GravityFeedforward::store_tuning_parameter(
    const rclcpp::Parameter& parameter, std::string& reason) {
    const std::string& name = parameter.get_name();
    const bool is_ratio = name == "friction_load_ratio";
    if (is_ratio || name == "friction_offset_nm") {
        const std::vector<double> values = as_number_array(parameter);
        // Empty leaves the calibrated coefficients in place. Setting it back
        // to empty at runtime cannot restore them, so it is only honoured as a
        // "nothing to override" at startup.
        if (values.empty()) return true;
        if (values.size() != kCompensatedCount) {
            reason = name + " must be empty or " + std::to_string(kCompensatedCount) +
                     " numbers, left arm then right";
            return false;
        }
        // A negative ratio has no reading at all; a negative floor is just the
        // fit extrapolated below the loads that were sampled, and the torque is
        // clamped at zero before it is used.
        for (double value : values) {
            if (!std::isfinite(value) || (is_ratio && value < 0.0)) {
                reason = name + (is_ratio ? " must be finite and non-negative"
                                          : " must be finite");
                return false;
            }
        }
        FrictionModel model = *friction_buffer_.readFromNonRT();
        auto& slot = is_ratio ? model.load_ratio : model.offset;
        std::copy(values.begin(), values.end(), slot.begin());
        friction_buffer_.writeFromNonRT(model);
        return true;
    }

    std::atomic<double>* slot = nullptr;
    if (name == "compensation_scale") slot = &compensation_scale_;
    else if (name == "friction_scale") slot = &friction_scale_;
    else if (name == "friction_error_epsilon") slot = &friction_error_epsilon_;
    else if (name == "friction_velocity_epsilon") slot = &friction_velocity_epsilon_;
    if (slot == nullptr) return true;

    const double value = as_number(parameter);
    if (!std::isfinite(value) || value < 0.0) {
        reason = name + " must be a finite non-negative number";
        return false;
    }
    slot->store(value, std::memory_order_relaxed);
    return true;
}

void GravityFeedforward::set_friction_model(const FrictionModel& model) {
    friction_buffer_.writeFromNonRT(model);
    // Also the live copy: `arm_offsets` is callable without `apply`, which is
    // what would otherwise be the only thing that refreshes it.
    friction_ = model;
}

GravityFeedforward::FrictionModel GravityFeedforward::friction_model() const {
    return friction_;
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

void GravityFeedforward::append_state_interfaces(std::vector<std::string>& names) const {
    if (!loaded_) return;
    for (const auto& interface : imu_interface_names()) {
        names.push_back(std::string(torso_imu_sensor()) + "/" + interface);
    }
}

std::size_t GravityFeedforward::state_interface_count() const {
    return loaded_ ? imu_interface_names().size() : 0;
}

bool GravityFeedforward::activate(
    const std::vector<hardware_interface::LoanedStateInterface>& interfaces,
    const rclcpp::Logger& logger) {
    if (!loaded_) return true;
    const std::string first =
        std::string(torso_imu_sensor()) + "/" + imu_interface_names().front();
    const auto found = std::find_if(
        interfaces.begin(), interfaces.end(),
        [&first](const hardware_interface::LoanedStateInterface& interface) {
            return interface.get_name() == first;
        });
    if (found == interfaces.end() ||
        static_cast<std::size_t>(std::distance(interfaces.begin(), found)) +
                imu_interface_names().size() >
            interfaces.size()) {
        RCLCPP_ERROR(logger, "%s was not claimed", first.c_str());
        return false;
    }
    imu_offset_ = static_cast<std::size_t>(std::distance(interfaces.begin(), found));
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
    const std::vector<double>& measured, const std::vector<double>& stiffness,
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
    // Kept up to date even on the cycles that bail out below, so the estimate
    // never sees a gap in time it cannot account for.
    update_target_velocity(target, elapsed);
    const std::array<double, 4> orientation = {
        interfaces[imu_offset_].get_value(),
        interfaces[imu_offset_ + 1].get_value(),
        interfaces[imu_offset_ + 2].get_value(),
        interfaces[imu_offset_ + 3].get_value(),
    };
    if (!update_torso_gravity(orientation, elapsed)) return false;
    advance_payload(elapsed);

    command = target;
    friction_ = *friction_buffer_.readFromRT();
    const double friction_gain = ramp_ * friction_scale_.load(std::memory_order_relaxed);
    for (std::size_t side = 0; side < kSideCount; ++side) {
        arm_offsets(
            side, target, measured, target_velocity_, ramp_ * scale, friction_gain,
            stiffness, command);
    }
    return true;
}

void GravityFeedforward::update_target_velocity(
    const std::vector<double>& target, double elapsed) {
    if (target_velocity_.size() != target.size()) {
        target_velocity_.assign(target.size(), 0.0);
        previous_target_ = target;
        target_velocity_valid_ = false;
        target_hold_time_ = 0.0;
        return;
    }
    // First cycle after activation: the previous target is whatever was left
    // over, so differencing it would report a step the size of the whole pose.
    if (!target_velocity_valid_) {
        previous_target_ = target;
        std::fill(target_velocity_.begin(), target_velocity_.end(), 0.0);
        target_velocity_valid_ = true;
        target_hold_time_ = 0.0;
        return;
    }
    if (!(elapsed > 0.0)) return;
    target_hold_time_ += elapsed;

    // Difference across the step, not across the cycle. Commands arrive at
    // 50 Hz and this runs at 500 Hz, so the target is a staircase: dividing by
    // the control period would give nine zeroes and one spike ten times too
    // big, and a low pass turns that into a 50 Hz ripple rather than the mean.
    // Measured on the 0.3 rad/s staircase below: a 5 Hz filter settles into a
    // 0.22-0.39 swing about a 0.3 target. Dividing by the time the target
    // actually stood still recovers the commanded speed exactly, on the cycle
    // it changes, with no ripple to feed into the friction sign.
    const bool moved = !std::equal(target.begin(), target.end(), previous_target_.begin());
    const double alpha = 1.0 - std::exp(-2.0 * M_PI * velocity_cutoff_hz_ * elapsed);
    if (moved) {
        for (std::size_t index = 0; index < target.size(); ++index) {
            const double raw = (target[index] - previous_target_[index]) / target_hold_time_;
            target_velocity_[index] += alpha * (raw - target_velocity_[index]);
            previous_target_[index] = target[index];
        }
        target_hold_time_ = 0.0;
        return;
    }
    // Nothing new for longer than the publisher's period: the arm is being
    // asked to stand still, so let the estimate fall away and take the
    // friction push with it. Two missed 50 Hz frames of tolerance.
    if (target_hold_time_ > kTargetStaleSeconds) {
        for (double& velocity : target_velocity_) velocity -= alpha * velocity;
    }
}

double GravityFeedforward::friction_torque(
    std::size_t slot, double gravity_torque, double position_error,
    double target_velocity) const {
    const double error_epsilon = friction_error_epsilon_.load(std::memory_order_relaxed);
    const double velocity_epsilon = friction_velocity_epsilon_.load(std::memory_order_relaxed);
    double drive = 0.0;
    if (error_epsilon > 0.0) drive += position_error / error_epsilon;
    if (velocity_epsilon > 0.0) drive += target_velocity / velocity_epsilon;
    if (drive == 0.0 || slot >= kCompensatedCount) return 0.0;

    // Clamped at zero: a joint whose fitted floor is negative would otherwise
    // get a backwards push once its load drops below the sampled range, which
    // widens the dead band instead of closing it.
    const double magnitude = std::max(
        0.0, friction_.load_ratio[slot] * std::abs(gravity_torque) + friction_.offset[slot]);
    return magnitude * std::tanh(drive);
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

void GravityFeedforward::load_friction_table(const std::string& path) {
    FrictionTable table;
    std::array<std::vector<std::string>, kSideCount> joints;
    for (std::size_t side = 0; side < kSideCount; ++side) joints[side] = table_.joints(side);
    table.load(path, joints);

    FrictionModel model;
    for (std::size_t side = 0; side < kSideCount; ++side) {
        for (std::size_t index = 0; index < kArmJointCount; ++index) {
            const std::size_t slot = side * kArmJointCount + index;
            model.load_ratio[slot] = table.load_ratio(side, index);
            model.offset[slot] = table.offset(side, index);
        }
    }
    set_friction_model(model);
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
    std::size_t side, const std::vector<double>& target,
    const std::vector<double>& measured, const std::vector<double>& target_velocity,
    double gain, double friction_gain, const std::vector<double>& stiffness,
    std::vector<double>& command) const {
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
        double applied = gain * torque;
        // The friction term needs no bound of its own: it is a fraction of a
        // torque that is already bounded, and it vanishes wherever the joint is
        // already sitting where it was asked to.
        if (friction_gain > 0.0) {
            const double error = command_index < measured.size()
                                     ? target[command_index] - measured[command_index]
                                     : 0.0;
            const double velocity = command_index < target_velocity.size()
                                        ? target_velocity[command_index]
                                        : 0.0;
            applied += friction_gain *
                       friction_torque(
                           side * kArmJointCount + index, torque, error, velocity);
        }
        command[command_index] = target[command_index] + applied / gain_value;
    }
}

}  // namespace unitree_g1_ros2_control
