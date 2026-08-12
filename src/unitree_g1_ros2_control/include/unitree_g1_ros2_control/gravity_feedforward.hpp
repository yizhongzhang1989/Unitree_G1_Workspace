#ifndef UNITREE_G1_ROS2_CONTROL__GRAVITY_FEEDFORWARD_HPP_
#define UNITREE_G1_ROS2_CONTROL__GRAVITY_FEEDFORWARD_HPP_

#include <array>
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
/// writes joint positions wants the same offset, so the four parameters and the
/// eighteen extra state interfaces live here too, next to the maths, instead of
/// being copied into each controller.
///
/// The numeric half - `load_gravity_table`, `update_torso_gravity`,
/// `arm_offsets` - touches no ROS type and can be exercised on its own.
class GravityFeedforward {
public:
    static constexpr std::size_t kArmJointCount = GravityTable::kArmJointCount;
    static constexpr std::size_t kSideCount = GravityTable::kSideCount;

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
    void append_state_interfaces(
        const std::vector<std::string>& joint_names, std::vector<std::string>& names) const;

    /// How many entries `append_state_interfaces` adds. They are always the
    /// trailing ones, which is how `activate` and `apply` find them.
    std::size_t state_interface_count() const;

    /// Rejects a hardware that reports a non-positive stiffness and arms the
    /// activation ramp.
    bool activate(
        const std::vector<hardware_interface::LoanedStateInterface>& interfaces,
        const rclcpp::Logger& logger);

    /// Fills `command` with `target + s * G(target) / kp`. Returns false, and
    /// leaves `command` untouched, when there is nothing to add: no table,
    /// `compensation_scale` at zero, or no usable torso attitude yet.
    bool apply(
        const std::vector<hardware_interface::LoanedStateInterface>& interfaces,
        const rclcpp::Duration& period, const std::vector<double>& target,
        std::vector<double>& command);

    // ---------------------------------------------------------------- //
    // Maths, usable without a node
    // ---------------------------------------------------------------- //

    /// Reads the table exported by `arm_gravity_compensation` and binds every
    /// body to its slot in `joint_names`. `path` may be absolute, `~`-relative
    /// or a `package://<name>/<relative>` reference. Throws
    /// `std::runtime_error` describing what is wrong with the file.
    void load_gravity_table(
        const std::string& path, const std::vector<std::string>& joint_names);
    bool loaded() const noexcept { return loaded_; }

    /// Command slots whose `kp` is needed, in the order `arm_offsets` indexes
    /// the stiffness values it is handed.
    const std::vector<std::size_t>& compensated_indices() const noexcept {
        return compensated_indices_;
    }

    /// Forget the filtered direction, so the next sample is taken as it is.
    void reset() noexcept { gravity_valid_ = false; }
    bool gravity_valid() const noexcept { return gravity_valid_; }

    /// Folds one IMU sample, ordered `x, y, z, w`, into the filtered gravity
    /// direction. Returns false while no usable direction is known yet.
    bool update_torso_gravity(
        const std::array<double, 4>& orientation, double elapsed);

    /// Writes `target + gain * G(target) / kp` into the arm slots of `command`
    /// and leaves every other slot untouched. `stiffness` is indexed the way
    /// `compensated_indices()` is ordered.
    void arm_offsets(
        std::size_t side, const std::vector<double>& target, double gain,
        const std::vector<double>& stiffness, std::vector<double>& command) const;

    /// Extra mass hanging on the force sensor, reported in the sensor frame.
    /// It is merged into the last body of the chain, which is why the table
    /// carries the sensor mount. Applied as given - the ramp and the limits
    /// live where the message is received.
    void set_payload(std::size_t side, double mass, const Vector3& first_moment);
    double payload_mass(std::size_t side) const { return payload_mass_[side]; }

private:
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
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
    std::vector<double> stiffness_;

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
