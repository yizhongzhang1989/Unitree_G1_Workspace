/// Turns the raw KWR57 stream into the net wrench a payload or the
/// environment applies.
///
/// The sensor sits past every arm joint, so its own reading is dominated by
/// the offset and by the weight of the gripper hanging off it. Both are known
/// after calibration; the only thing that has to be computed live is where
/// gravity points in the sensor frame, which is the same chain the gravity
/// feed-forward already walks. Deliberately a separate process from the CAN
/// bridge: a generic device driver has no business knowing where the arm is.
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "unitree_g1_ros2_control/ft_compensation.hpp"
#include "unitree_g1_ros2_control/gravity_table.hpp"
#include "unitree_hg/msg/imu_state.hpp"

namespace unitree_g1_ros2_control {

namespace {

constexpr double kGravityMagnitude = 9.81;
constexpr double kKgfToNewton = 9.80665;

}  // namespace

class FtWrenchCompensator : public rclcpp::Node {
public:
    FtWrenchCompensator() : rclcpp::Node("ft_wrench_compensator") {
        const auto table_path = declare_parameter<std::string>(
            "gravity_table",
            "package://arm_gravity_compensation/config/gravity_table.yaml");
        const auto calibration_path = declare_parameter<std::string>(
            "ft_calibration",
            "package://arm_gravity_compensation/config/ft_calibration.yaml");
        const auto unit = declare_parameter<std::string>("input_unit", "si");
        input_scale_ = unit == "kgf" ? kKgfToNewton : 1.0;
        publish_period_ = 1.0 / std::max(
            declare_parameter<double>("publish_rate", 200.0), 1e-9);
        state_timeout_ = declare_parameter<double>("state_timeout_s", 0.5);
        tare_timeout_ = declare_parameter<double>("tare_timeout_s", 30.0);

        table_.load(table_path);
        if (!table_.has_sensor()) {
            throw std::runtime_error(
                table_path + " predates the force sensor mount; re-export it "
                "from the calibration page");
        }

        const std::array<std::string, GravityTable::kSideCount> defaults_in = {
            "/arm0/wrench_raw", "/arm1/wrench_raw"};
        const std::array<std::string, GravityTable::kSideCount> defaults_out = {
            "/arm0/wrench_net", "/arm1/wrench_net"};
        const std::array<std::string, GravityTable::kSideCount> defaults_gravity = {
            "/arm0/gravity", "/arm1/gravity"};
        const std::array<std::string, GravityTable::kSideCount> defaults_tare = {
            "/ft_arm0/reset_tare", "/ft_arm1/reset_tare"};

        const rclcpp::QoS sensor_qos =
            rclcpp::SensorDataQoS().keep_last(1);
        for (std::size_t side = 0; side < GravityTable::kSideCount; ++side) {
            const std::string name = GravityTable::side_names()[side];
            if (!FtCalibration::load(calibration_path, name, calibration_[side])) {
                RCLCPP_WARN(
                    get_logger(), "%s has no calibration in %s; it stays quiet",
                    name.c_str(), calibration_path.c_str());
                continue;
            }
            const auto input = declare_parameter<std::string>(
                name + "_input_topic", defaults_in[side]);
            const auto output = declare_parameter<std::string>(
                name + "_output_topic", defaults_out[side]);
            reset_tare_[side] = declare_parameter<std::string>(
                name + "_reset_tare_service", defaults_tare[side]);
            publisher_[side] =
                create_publisher<geometry_msgs::msg::WrenchStamped>(output, sensor_qos);
            // Removing the tool's weight already needs gravity in the sensor
            // frame, and a payload estimator needs exactly that and nothing
            // else. Publishing it keeps the kinematic model in one place.
            gravity_publisher_[side] =
                create_publisher<geometry_msgs::msg::Vector3Stamped>(
                    declare_parameter<std::string>(
                        name + "_gravity_topic", defaults_gravity[side]),
                    sensor_qos);
            subscription_[side] = create_subscription<geometry_msgs::msg::WrenchStamped>(
                input, sensor_qos,
                [this, side](geometry_msgs::msg::WrenchStamped::ConstSharedPtr message) {
                    on_wrench(side, *message);
                });
            active_[side] = true;
            RCLCPP_INFO(
                get_logger(), "%s: %s -> %s, tool %.4f kg", name.c_str(),
                input.c_str(), output.c_str(), calibration_[side].mass);
        }
        if (std::none_of(active_.begin(), active_.end(), [](bool value) { return value; })) {
            throw std::runtime_error(
                "no side is calibrated in " + calibration_path);
        }

        joint_states_ = create_subscription<sensor_msgs::msg::JointState>(
            declare_parameter<std::string>("joint_states_topic", "/joint_states"),
            sensor_qos,
            [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
                on_joint_state(*message);
            });
        imu_ = create_subscription<unitree_hg::msg::IMUState>(
            declare_parameter<std::string>("imu_topic", "/secondary_imu"), sensor_qos,
            [this](unitree_hg::msg::IMUState::ConstSharedPtr message) {
                on_imu(*message);
            });
        rezero_ = create_service<std_srvs::srv::Trigger>(
            "~/rezero",
            [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
                   std_srvs::srv::Trigger::Response::SharedPtr response) {
                on_rezero(*response);
            });

        // The driver keeps its own software tare, and a stale one would shift
        // every reading out from under a calibration that was captured
        // without it.
        release_driver_tare();
    }

private:
    using Angles = std::array<double, GravityTable::kArmJointCount>;

    void on_imu(const unitree_hg::msg::IMUState& message) {
        // Unitree orders the quaternion w, x, y, z.
        const std::array<double, 4> orientation = {
            message.quaternion[1], message.quaternion[2], message.quaternion[3],
            message.quaternion[0]};
        Vector3 direction{};
        if (!torso_gravity(orientation, table_.imu_to_torso(), direction)) return;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            torso_gravity_[axis] = kGravityMagnitude * direction[axis];
        }
        imu_stamp_ = now();
    }

    void on_joint_state(const sensor_msgs::msg::JointState& message) {
        // The broadcaster iterates an unordered map, so the order is arbitrary
        // and only the names may be trusted.
        if (message.name != names_) {
            names_ = message.name;
            for (std::size_t side = 0; side < GravityTable::kSideCount; ++side) {
                for (std::size_t index = 0; index < GravityTable::kArmJointCount; ++index) {
                    const auto& joint = table_.joints(side)[index];
                    const auto found = std::find(names_.begin(), names_.end(), joint);
                    lookup_[side][index] = found == names_.end()
                        ? names_.size()
                        : static_cast<std::size_t>(std::distance(names_.begin(), found));
                }
            }
        }
        for (std::size_t side = 0; side < GravityTable::kSideCount; ++side) {
            for (std::size_t index = 0; index < GravityTable::kArmJointCount; ++index) {
                const std::size_t slot = lookup_[side][index];
                if (slot >= message.position.size()) return;
                angles_[side][index] = message.position[slot];
            }
        }
        joint_stamp_ = now();
    }

    /// Gravity in the sensor frame, or false while the pose is unknown.
    bool sensor_gravity(std::size_t side, Vector3& gravity) const {
        const rclcpp::Time stamp = now();
        if (joint_stamp_.nanoseconds() == 0 || imu_stamp_.nanoseconds() == 0) return false;
        if ((stamp - joint_stamp_).seconds() > state_timeout_ ||
            (stamp - imu_stamp_).seconds() > state_timeout_) {
            return false;
        }
        gravity = rotate(
            transpose(table_.sensor_orientation(side, angles_[side])), torso_gravity_);
        return true;
    }

    void on_wrench(std::size_t side, const geometry_msgs::msg::WrenchStamped& message) {
        const rclcpp::Time stamp = now();
        if ((stamp - published_[side]).seconds() < publish_period_) return;
        Vector3 gravity{};
        if (!sensor_gravity(side, gravity)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "%s: no fresh arm pose or torso attitude, holding output",
                GravityTable::side_names()[side]);
            return;
        }
        published_[side] = stamp;
        latest_[side] = raw_wrench(message);
        const Wrench net = net_wrench(latest_[side], calibration_[side], gravity);

        geometry_msgs::msg::WrenchStamped output;
        output.header.stamp = message.header.stamp;
        output.header.frame_id = calibration_[side].frame;
        output.wrench.force.x = net[0];
        output.wrench.force.y = net[1];
        output.wrench.force.z = net[2];
        output.wrench.torque.x = net[3];
        output.wrench.torque.y = net[4];
        output.wrench.torque.z = net[5];
        publisher_[side]->publish(output);

        geometry_msgs::msg::Vector3Stamped direction;
        direction.header = output.header;
        direction.vector.x = gravity[0];
        direction.vector.y = gravity[1];
        direction.vector.z = gravity[2];
        gravity_publisher_[side]->publish(direction);
    }

    Wrench raw_wrench(const geometry_msgs::msg::WrenchStamped& message) const {
        return {
            message.wrench.force.x * input_scale_,
            message.wrench.force.y * input_scale_,
            message.wrench.force.z * input_scale_,
            message.wrench.torque.x * input_scale_,
            message.wrench.torque.y * input_scale_,
            message.wrench.torque.z * input_scale_,
        };
    }

    void on_rezero(std_srvs::srv::Trigger::Response& response) {
        std::string report;
        for (std::size_t side = 0; side < GravityTable::kSideCount; ++side) {
            Vector3 gravity{};
            if (!active_[side] || !sensor_gravity(side, gravity)) continue;
            if (published_[side].nanoseconds() == 0) continue;
            calibration_[side] = rezeroed(latest_[side], calibration_[side], gravity);
            report += std::string(report.empty() ? "" : ", ") +
                      GravityTable::side_names()[side];
            RCLCPP_INFO(
                get_logger(), "%s offset now [%.3f %.3f %.3f] [%.4f %.4f %.4f]",
                GravityTable::side_names()[side], calibration_[side].force_bias[0],
                calibration_[side].force_bias[1], calibration_[side].force_bias[2],
                calibration_[side].torque_bias[0], calibration_[side].torque_bias[1],
                calibration_[side].torque_bias[2]);
        }
        response.success = !report.empty();
        response.message = report.empty()
            ? "no side had a fresh unloaded reading"
            : "offset re-estimated for " + report + " (in memory only)";
    }

    void release_driver_tare() {
        for (std::size_t side = 0; side < GravityTable::kSideCount; ++side) {
            if (!active_[side] || reset_tare_[side].empty()) continue;
            tare_clients_[side] =
                create_client<std_srvs::srv::Trigger>(reset_tare_[side]);
        }
        // 和 bridge 同一个 launch 起来时，KWR57 要等 USB/CAN 开完才出现。
        tare_attempts_ = static_cast<int>(std::ceil(tare_timeout_ / 0.2));
        tare_timer_ = create_wall_timer(
            std::chrono::milliseconds(200), [this] { poll_driver_tare(); });
    }

    void poll_driver_tare() {
        const bool expired = --tare_attempts_ <= 0;
        bool pending = false;
        for (auto& client : tare_clients_) {
            if (!client) continue;
            if (client->service_is_ready()) {
                client->async_send_request(
                    std::make_shared<std_srvs::srv::Trigger::Request>());
                RCLCPP_INFO(
                    get_logger(), "%s: software tare cleared",
                    client->get_service_name());
            } else if (expired) {
                RCLCPP_WARN(
                    get_logger(),
                    "%s never showed up; make sure its software tare is off",
                    client->get_service_name());
            } else {
                pending = true;
                continue;
            }
            served_.push_back(std::move(client));
        }
        if (!pending) tare_timer_->cancel();
    }

    GravityTable table_;
    std::array<FtCalibration, GravityTable::kSideCount> calibration_;
    std::array<bool, GravityTable::kSideCount> active_{false, false};
    std::array<std::string, GravityTable::kSideCount> reset_tare_;
    std::array<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr,
               GravityTable::kSideCount> publisher_;
    std::array<rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr,
               GravityTable::kSideCount> gravity_publisher_;
    std::array<rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr,
               GravityTable::kSideCount> subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_;
    rclcpp::Subscription<unitree_hg::msg::IMUState>::SharedPtr imu_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr rezero_;
    std::array<rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr,
               GravityTable::kSideCount> tare_clients_;
    // 发完也要握着 client，否则请求还没出去就跟着它一起析了。
    std::vector<rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> served_;
    rclcpp::TimerBase::SharedPtr tare_timer_;
    int tare_attempts_{0};

    std::vector<std::string> names_;
    std::array<std::array<std::size_t, GravityTable::kArmJointCount>,
               GravityTable::kSideCount> lookup_{};
    std::array<Angles, GravityTable::kSideCount> angles_{};
    std::array<Wrench, GravityTable::kSideCount> latest_{};
    std::array<rclcpp::Time, GravityTable::kSideCount> published_{
        rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Time(0, 0, RCL_ROS_TIME)};
    Vector3 torso_gravity_{0.0, 0.0, -kGravityMagnitude};
    rclcpp::Time joint_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time imu_stamp_{0, 0, RCL_ROS_TIME};
    double input_scale_{1.0};
    double publish_period_{0.005};
    double state_timeout_{0.5};
    double tare_timeout_{30.0};
};

}  // namespace unitree_g1_ros2_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(
            std::make_shared<unitree_g1_ros2_control::FtWrenchCompensator>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(
            rclcpp::get_logger("ft_wrench_compensator"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
