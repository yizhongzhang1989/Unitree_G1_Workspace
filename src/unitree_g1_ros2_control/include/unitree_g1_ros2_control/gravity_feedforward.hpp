#ifndef UNITREE_G1_ROS2_CONTROL__GRAVITY_FEEDFORWARD_HPP_
#define UNITREE_G1_ROS2_CONTROL__GRAVITY_FEEDFORWARD_HPP_

#include <array>
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/inertia_stamped.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "unitree_g1_ros2_control/gravity_table.hpp"

namespace unitree_g1_ros2_control {

/// Calibrated arm gravity, expressed as the position offset that holds a pose.
///
/// The G1 arm motors close their own position loop as
/// `tau = kp * (q_cmd - q) - kd * dq`, so holding a pose against gravity only
/// needs the command to be offset by `G(q_target) / kp`. Every controller that
/// writes joint positions wants the same offset, so the parameters and the
/// torso IMU interfaces live here too, next to the maths, instead of being
/// copied into each controller. The `kp` it divides by is passed in rather
/// than read here - `AdaptiveStiffness` owns that channel.
///
/// The numeric half - `load_gravity_table`, `update_torso_gravity`,
/// `arm_offsets` - touches no ROS type and can be exercised on its own.
class GravityFeedforward {
public:
    static constexpr std::size_t kArmJointCount = GravityTable::kArmJointCount;
    static constexpr std::size_t kSideCount = GravityTable::kSideCount;
    /// Both arms end to end, in the order `compensated_indices()` reports.
    static constexpr std::size_t kCompensatedCount = kSideCount * kArmJointCount;

    /// Per-joint friction model, loaded from the same table as the gravity it
    /// is paired with. The load ratio and the floor differ enough between
    /// joints that one pair for the whole arm mis-serves both ends: measured
    /// mu spans 0.064 to 0.223, and the two wrists with the weakest load
    /// correlation are exactly the ones a shoulder-sized coefficient
    /// over-drives.
    struct FrictionModel {
        std::array<double, kCompensatedCount> load_ratio{};
        std::array<double, kCompensatedCount> offset{};
    };

    using LifecycleNode = rclcpp_lifecycle::LifecycleNode;

    /// Sensor the torso attitude is read from, and its interfaces in the order
    /// `update_torso_gravity` expects its argument.
    static const char* torso_imu_sensor();
    static const std::array<std::string, 4>& imu_interface_names();

    // ---------------------------------------------------------------- //
    // Controller side
    // ---------------------------------------------------------------- //

    /// Declares `gravity_table`, `gravity_filter_cutoff_hz`, `offset_ramp_s`
    /// and `compensation_scale` unless they already exist.
    static void declare_parameters(const LifecycleNode::SharedPtr& node);
    /// Loads the table named by `gravity_table` and binds it to `joint_names`.
    /// An empty name leaves the feed-forward disabled; a bad one is logged and
    /// reported as a failure.
    bool configure(
        const LifecycleNode::SharedPtr& node, const std::vector<std::string>& joint_names);

    /// Interface names to append after the controller's own state interfaces.
    void append_state_interfaces(std::vector<std::string>& names) const;

    /// How many entries `append_state_interfaces` adds.
    std::size_t state_interface_count() const;

    /// Locates the torso IMU by interface name and arms the activation ramp.
    /// Resolving by name rather than by position lets other components append
    /// to the same list without this one having to know about them.
    bool activate(
        const std::vector<hardware_interface::LoanedStateInterface>& interfaces,
        const rclcpp::Logger& logger);

    /// Fills `command` with `target + s * G(target) / kp`. Returns false, and
    /// leaves `command` untouched, when there is nothing to add: no table,
    /// `compensation_scale` at zero, or no usable torso attitude yet.
    /// `measured` is the joint positions in `target`'s order; the two
    /// controllers lay their state interfaces out differently, so it is read
    /// by the caller rather than looked up here.
    ///
    /// `stiffness` is the gain the motor will really be given this cycle,
    /// ordered like `compensated_indices()`. Whoever commands the gains has to
    /// pass the same numbers here: the offset is sized as `G / kp`, so a
    /// mismatch scales every compensation torque by that ratio with no visible
    /// symptom.
    bool apply(
        const std::vector<hardware_interface::LoanedStateInterface>& interfaces,
        const rclcpp::Duration& period, const std::vector<double>& target,
        const std::vector<double>& measured, const std::vector<double>& stiffness,
        std::vector<double>& command);

    // ---------------------------------------------------------------- //
    // Maths, usable without a node
    // ---------------------------------------------------------------- //

    /// Reads the table exported by `arm_gravity_compensation` and binds every
    /// body to its slot in `joint_names`, including the friction coefficients
    /// when the table carries them. `path` may be absolute, `~`-relative
    /// or a `package://<name>/<relative>` reference. Throws
    /// `std::runtime_error` describing what is wrong with the file.
    void load_gravity_table(
        const std::string& path, const std::vector<std::string>& joint_names);
    bool loaded() const noexcept { return loaded_; }

    /// Reads the friction coefficients exported alongside the gravity table.
    /// Must follow `load_gravity_table`: the joint order it reports is what
    /// the friction file is checked against. Throws `std::runtime_error`.
    void load_friction_table(const std::string& path);

    /// Command slots whose `kp` is needed, in the order `arm_offsets` indexes
    /// the stiffness values it is handed.
    const std::vector<std::size_t>& compensated_indices() const noexcept {
        return compensated_indices_;
    }

    /// Forget the filtered direction, so the next sample is taken as it is.
    void reset() noexcept {
        gravity_valid_ = false;
        target_velocity_valid_ = false;
        target_hold_time_ = 0.0;
        std::fill(target_velocity_.begin(), target_velocity_.end(), 0.0);
    }
    bool gravity_valid() const noexcept { return gravity_valid_; }

    /// Folds one IMU sample, ordered `x, y, z, w`, into the filtered gravity
    /// direction. Returns false while no usable direction is known yet.
    bool update_torso_gravity(
        const std::array<double, 4>& orientation, double elapsed);

    /// Differentiates the target and low-passes it into `target_velocity()`.
    ///
    /// The friction term needs to know which way the joint is being asked to
    /// go. It has to come from the target and not from the measurement: the
    /// measured velocity is the controlled quantity, so driving the
    /// compensation with it closes a positive feedback loop that breaks the
    /// joint out, overshoots, breaks it back and rings.
    ///
    /// The difference is taken across the step rather than across the cycle,
    /// because commands arrive at 50 Hz while this runs at 500 Hz.
    void update_target_velocity(const std::vector<double>& target, double elapsed);
    const std::vector<double>& target_velocity() const noexcept { return target_velocity_; }

    /// Writes `target + gain * G / kp + friction_gain * F / kp` into the arm
    /// slots of `command` and leaves every other slot untouched. `stiffness` is
    /// indexed the way `compensated_indices()` is ordered; `measured` and
    /// `target_velocity` are indexed like `target` and only steer the friction
    /// term, which is zero wherever the joint is already where it was asked to
    /// be. Either may be empty to drop that part of the drive.
    void arm_offsets(
        std::size_t side, const std::vector<double>& target,
        const std::vector<double>& measured, const std::vector<double>& target_velocity,
        double gain, double friction_gain, const std::vector<double>& stiffness,
        std::vector<double>& command) const;

    /// Friction torque at the joint in `slot` of `compensated_indices()`,
    /// carrying `gravity_torque`.
    ///
    /// Measured 2026-08-19 over 11 poses per side: friction tracks joint load
    /// with a correlation of 0.71-0.96 on the shoulders, `mu` landing in
    /// 0.064-0.223 - the signature of load-dependent gearbox loss. Modelling
    /// it as a constant is not merely less accurate but unsafe: the same joint
    /// spans 11x between poses (0.106-1.205 N.m on the left shoulder roll), so
    /// a constant set from the median over-compensates a lightly loaded pose
    /// tenfold, which is negative damping.
    ///
    /// The direction is driven by the position error first and the target
    /// velocity second. Error is what actually reports the stiction: the joint
    /// stays put until `kp * e` exceeds the friction, so `e` grows to the full
    /// dead band no matter how slowly the target creeps. Target velocity alone
    /// fails exactly where it is needed - a 5 mm/s fingertip is 0.0125 rad/s at
    /// the joint, which `tanh` at eps = 0.02 answers with 55% of the torque,
    /// and 1 mm/s with 12%. Adding the two lets a reversal cancel them against
    /// each other, which is the right thing to do while the direction is in
    /// doubt. Either epsilon may be zero to drop that term.
    double friction_torque(
        std::size_t slot, double gravity_torque, double position_error,
        double target_velocity) const;

    /// Replaces the per-joint coefficients. Lock free against the update loop.
    void set_friction_model(const FrictionModel& model);
    FrictionModel friction_model() const;

    /// Extra mass hanging on the force sensor, reported in the sensor frame.
    /// It is merged into the last body of the chain, which is why the table
    /// carries the sensor mount. Applied as given - the ramp and the limits
    /// live where the message is received.
    void set_payload(std::size_t side, double mass, const Vector3& first_moment);
    double payload_mass(std::size_t side) const { return payload_mass_[side]; }

private:
    /// Validates and stores one live-tunable gain. Shared by `configure` and
    /// the parameter callback so a value can never be accepted at startup that
    /// would be rejected at runtime. Returns false and fills `reason` when the
    /// value is out of range; unknown names are ignored and return true.
    bool store_tuning_parameter(const rclcpp::Parameter& parameter, std::string& reason);

    /// Fold the newest payload message in over `payload_filter_tau_s`, and let
    /// it decay back to nothing once its publisher goes quiet. A payload that
    /// appears in one cycle would otherwise step the command by the whole
    /// offset it is worth.
    void advance_payload(double elapsed);
    void subscribe_payload(const LifecycleNode::SharedPtr& node);
    void accept_payload(
        std::size_t side, const geometry_msgs::msg::InertiaStamped& message);

    struct PayloadCommand {
        double mass{0.0};
        Vector3 first_moment{};
    };

    GravityTable table_;
    std::vector<std::size_t> compensated_indices_;
    std::array<double, 3> gravity_{};
    double filter_cutoff_hz_{2.0};
    bool gravity_valid_{false};
    bool loaded_{false};

    // Activation fade-in: `ramp_` climbs 0 -> 1 over `ramp_duration_` seconds
    // (`offset_ramp_s`) and multiplies the offset on top of
    // `compensation_scale`, so the command is
    //     q_cmd = q_target + ramp_ * compensation_scale * G(q_target) / kp
    // At activation the arm already hangs a full offset below the target, so
    // applying it in one cycle would step the position error from zero to twice
    // the droop - around 0.8 rad on a loaded shoulder, a torque spike near the
    // motor limit. It is reset to zero whenever the compensation is switched
    // off, so switching it back on eases the offset in again. Unrelated to the
    // low pass above: that one filters the gravity DIRECTION and runs forever,
    // this one only matters for the first `offset_ramp_s` after activation.
    double ramp_duration_{2.0};
    double ramp_{0.0};
    // Trims residual model error by feel. Read from the update loop, written
    // from the parameter callback, so it has to be lock free.
    std::atomic<double> compensation_scale_{1.0};

    // Coulomb friction as a fraction of the joint load, plus a floor. Off by
    // default: over-compensating friction is destabilising, so it has to be
    // switched on deliberately and raised a notch at a time.
    std::atomic<double> friction_scale_{0.0};
    // Per-joint, so the buffer rather than an atomic. Copied into `friction_`
    // once per cycle; `arm_offsets` is const and reads that copy.
    realtime_tools::RealtimeBuffer<FrictionModel> friction_buffer_;
    FrictionModel friction_{};
    // Widths of the tanh that replaces sign(). A hard sign would swing the
    // command by the whole dead band every time the drive crosses zero. The
    // error width also sets how much stiffness the term adds below it -
    // `tau_f / eps` on top of `kp` - so shrinking it trades dead band for
    // damping ratio. Either may be zero to drop that term.
    std::atomic<double> friction_error_epsilon_{0.05};
    std::atomic<double> friction_velocity_epsilon_{0.02};

    double velocity_cutoff_hz_{5.0};
    // How long a target may stand still before it is read as "hold here"
    // rather than "the next command is late". Two frames of the 50 Hz
    // publisher.
    static constexpr double kTargetStaleSeconds = 0.05;
    std::vector<double> previous_target_;
    std::vector<double> target_velocity_;
    double target_hold_time_{0.0};
    bool target_velocity_valid_{false};

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
    // Where the four torso IMU orientation slots start, resolved by name at
    // activation so the loop can index straight into them.
    std::size_t imu_offset_{0};

    std::array<rclcpp::Subscription<geometry_msgs::msg::InertiaStamped>::SharedPtr,
               kSideCount> payload_subscription_;
    std::array<realtime_tools::RealtimeBuffer<PayloadCommand>, kSideCount> payload_buffer_;
    std::array<std::atomic<uint64_t>, kSideCount> payload_sequence_{};
    std::array<uint64_t, kSideCount> payload_seen_{};
    std::array<double, kSideCount> payload_age_{};
    std::array<double, kSideCount> payload_mass_{};
    std::array<Vector3, kSideCount> payload_moment_{};
    double payload_filter_tau_{1.0};
    double payload_timeout_{2.0};
    double maximum_payload_mass_{3.0};
};

}  // namespace unitree_g1_ros2_control

#endif  // UNITREE_G1_ROS2_CONTROL__GRAVITY_FEEDFORWARD_HPP_
