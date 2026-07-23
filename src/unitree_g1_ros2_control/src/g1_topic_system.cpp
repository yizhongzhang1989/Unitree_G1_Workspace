#include "unitree_g1_ros2_control/g1_topic_system.hpp"

#include "unitree_g1_ros2_control/periodic_deadline.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <exception>
#include <future>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/qos.hpp"
#include "yaml-cpp/yaml.h"

namespace {

constexpr std::int64_t kCheckModeApiId = 1001;
constexpr std::int64_t kSelectModeApiId = 1002;
constexpr std::int64_t kReleaseModeApiId = 1003;
constexpr std::size_t kLowCmdMotorCount = 35;
constexpr std::size_t kLowCmdPayloadSize = 1000;
constexpr std::uint32_t kCrcPolynomial = 0x04C11DB7;
const std::array<std::string, 6> kWrenchInterfaceNames = {
    "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"};
const std::array<std::string, 10> kImuInterfaceNames = {
    "orientation.x",
    "orientation.y",
    "orientation.z",
    "orientation.w",
    "angular_velocity.x",
    "angular_velocity.y",
    "angular_velocity.z",
    "linear_acceleration.x",
    "linear_acceleration.y",
    "linear_acceleration.z",
};
constexpr std::uint32_t kBodyClaimMask = (1U << 29U) - 1U;
constexpr std::uint32_t kLeftGripperClaim = 1U << 29U;
constexpr std::uint32_t kRightGripperClaim = 1U << 30U;
constexpr std::uint32_t kGripperClaimMask = kLeftGripperClaim | kRightGripperClaim;
constexpr std::size_t kFirstArmJointIndex = 15;

const std::array<std::string, 31> kExpectedJointNames = {
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    "left_eccentric_joint",
    "right_eccentric_joint",
};

template <typename Value>
void append_value(std::array<std::uint8_t, kLowCmdPayloadSize>& payload,
                  std::size_t& offset, const Value& value) {
    std::memcpy(payload.data() + offset, &value, sizeof(Value));
    offset += sizeof(Value);
}

std::uint32_t lowcmd_crc(const unitree_hg::msg::LowCmd& message) {
    std::array<std::uint8_t, kLowCmdPayloadSize> payload{};
    std::size_t offset = 0;
    append_value(payload, offset, message.mode_pr);
    append_value(payload, offset, message.mode_machine);
    offset += 2;

    for (const auto& command : message.motor_cmd) {
        append_value(payload, offset, command.mode);
        offset += 3;
        append_value(payload, offset, command.q);
        append_value(payload, offset, command.dq);
        append_value(payload, offset, command.tau);
        append_value(payload, offset, command.kp);
        append_value(payload, offset, command.kd);
        append_value(payload, offset, command.reserve);
    }
    for (const auto value : message.reserve) {
        append_value(payload, offset, value);
    }
    if (offset != payload.size()) {
        throw std::logic_error("unexpected Unitree LowCmd payload size");
    }

    std::uint32_t checksum = 0xFFFFFFFF;
    for (std::size_t byte = 0; byte < payload.size(); byte += 4) {
        const std::uint32_t data =
            static_cast<std::uint32_t>(payload[byte]) |
            (static_cast<std::uint32_t>(payload[byte + 1]) << 8U) |
            (static_cast<std::uint32_t>(payload[byte + 2]) << 16U) |
            (static_cast<std::uint32_t>(payload[byte + 3]) << 24U);
        std::uint32_t bit = 1U << 31U;
        for (std::uint32_t index = 0; index < 32; ++index) {
            checksum = (checksum & 0x80000000U)
                           ? (checksum << 1U) ^ kCrcPolynomial
                           : checksum << 1U;
            if (data & bit) {
                checksum ^= kCrcPolynomial;
            }
            bit >>= 1U;
        }
    }
    return checksum;
}

bool finite_positive(double value) {
    return std::isfinite(value) && value > 0.0;
}

std::string join_errors(const std::vector<std::string>& errors) {
    std::ostringstream stream;
    for (std::size_t index = 0; index < errors.size(); ++index) {
        if (index != 0) {
            stream << "; ";
        }
        stream << errors[index];
    }
    return stream.str();
}

bool safe_mode_name(const std::string& mode) {
    return !mode.empty() && std::all_of(mode.begin(), mode.end(), [](char value) {
        return std::isalnum(static_cast<unsigned char>(value)) || value == '_' ||
               value == '-' || value == '.';
    });
}

}  // namespace

namespace unitree_g1_ros2_control {

G1TopicSystem::~G1TopicSystem() {
    if (executor_) {
        stop();
    }
}

hardware_interface::return_type G1TopicSystem::configure(
    const hardware_interface::HardwareInfo& info) {
    if (configure_default(info) != hardware_interface::return_type::OK) {
        return hardware_interface::return_type::ERROR;
    }
    try {
        if (!configure_interfaces() || !configure_parameters()) {
            return hardware_interface::return_type::ERROR;
        }
        create_ros_interfaces();
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("G1TopicSystem"), "Configuration failed: %s", error.what());
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

bool G1TopicSystem::configure_interfaces() {
    if (info_.joints.size() != kControlledJointCount) {
        RCLCPP_ERROR(
            rclcpp::get_logger("G1TopicSystem"), "Expected %zu joints, got %zu",
            kControlledJointCount, info_.joints.size());
        return false;
    }

    const double nan = std::numeric_limits<double>::quiet_NaN();
    state_position_.assign(kControlledJointCount, 0.0);
    state_velocity_.assign(kControlledJointCount, 0.0);
    state_effort_.assign(kControlledJointCount, 0.0);
    command_position_.assign(kControlledJointCount, nan);
    lower_limits_.resize(kControlledJointCount);
    upper_limits_.resize(kControlledJointCount);

    for (std::size_t index = 0; index < info_.joints.size(); ++index) {
        const auto& joint = info_.joints[index];
        if (joint.name != kExpectedJointNames[index]) {
            RCLCPP_ERROR(
                rclcpp::get_logger("G1TopicSystem"),
                "Joint %zu is %s; expected %s", index, joint.name.c_str(),
                kExpectedJointNames[index].c_str());
            return false;
        }
        if (joint.command_interfaces.size() != 1 ||
            joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_ERROR(
                rclcpp::get_logger("G1TopicSystem"),
                "%s must expose exactly one position command interface", joint.name.c_str());
            return false;
        }
        std::array<bool, 3> found_state{};
        for (const auto& interface : joint.state_interfaces) {
            if (interface.name == hardware_interface::HW_IF_POSITION) {
                found_state[0] = true;
            } else if (interface.name == hardware_interface::HW_IF_VELOCITY) {
                found_state[1] = true;
            } else if (interface.name == hardware_interface::HW_IF_EFFORT) {
                found_state[2] = true;
            }
        }
        if (!std::all_of(found_state.begin(), found_state.end(), [](bool value) { return value; })) {
            RCLCPP_ERROR(
                rclcpp::get_logger("G1TopicSystem"),
                "%s must expose position, velocity and effort state interfaces",
                joint.name.c_str());
            return false;
        }
        try {
            lower_limits_[index] = std::stod(joint.command_interfaces[0].min);
            upper_limits_[index] = std::stod(joint.command_interfaces[0].max);
        } catch (const std::exception&) {
            RCLCPP_ERROR(
                rclcpp::get_logger("G1TopicSystem"),
                "%s position command interface requires finite min/max", joint.name.c_str());
            return false;
        }
        if (!std::isfinite(lower_limits_[index]) || !std::isfinite(upper_limits_[index]) ||
            lower_limits_[index] > upper_limits_[index]) {
            RCLCPP_ERROR(
                rclcpp::get_logger("G1TopicSystem"), "%s has invalid position limits",
                joint.name.c_str());
            return false;
        }
    }
    if (info_.sensors.size() != kForceTorqueSensorCount + 1) {
        RCLCPP_ERROR(
            rclcpp::get_logger("G1TopicSystem"), "Expected two force-torque sensors and one IMU, got %zu",
            info_.sensors.size());
        return false;
    }
    wrench_sensor_names_ = {info_.sensors[0].name, info_.sensors[1].name};
    for (std::size_t sensor_index = 0; sensor_index < kForceTorqueSensorCount; ++sensor_index) {
        const auto& sensor = info_.sensors[sensor_index];
        if (sensor.state_interfaces.size() != kWrenchAxisCount) {
            RCLCPP_ERROR(
                rclcpp::get_logger("G1TopicSystem"),
                "%s must expose six force/torque state interfaces", sensor.name.c_str());
            return false;
        }
        for (std::size_t axis = 0; axis < kWrenchAxisCount; ++axis) {
            if (sensor.state_interfaces[axis].name != kWrenchInterfaceNames[axis]) {
                RCLCPP_ERROR(
                    rclcpp::get_logger("G1TopicSystem"),
                    "%s interface %zu is %s; expected %s", sensor.name.c_str(), axis,
                    sensor.state_interfaces[axis].name.c_str(),
                    kWrenchInterfaceNames[axis].c_str());
                return false;
            }
        }
    }
    const auto& imu = info_.sensors[kForceTorqueSensorCount];
    imu_sensor_name_ = imu.name;
    if (imu.state_interfaces.size() != kImuAxisCount) {
        RCLCPP_ERROR(
            rclcpp::get_logger("G1TopicSystem"), "%s must expose ten IMU state interfaces",
            imu.name.c_str());
        return false;
    }
    for (std::size_t axis = 0; axis < kImuAxisCount; ++axis) {
        if (imu.state_interfaces[axis].name != kImuInterfaceNames[axis]) {
            RCLCPP_ERROR(
                rclcpp::get_logger("G1TopicSystem"),
                "%s interface %zu is %s; expected %s", imu.name.c_str(), axis,
                imu.state_interfaces[axis].name.c_str(), kImuInterfaceNames[axis].c_str());
            return false;
        }
    }
    const double wrench_nan = std::numeric_limits<double>::quiet_NaN();
    for (auto& sensor : wrench_state_) {
        sensor.fill(wrench_nan);
    }
    imu_state_.fill(0.0);
    imu_state_[3] = 1.0;
    pending_imu_ = imu_state_;
    return true;
}

bool G1TopicSystem::configure_parameters() {
    const auto string_parameter = [this](const std::string& name, const std::string& fallback) {
        const auto found = info_.hardware_parameters.find(name);
        return found == info_.hardware_parameters.end() ? fallback : found->second;
    };
    const auto double_parameter = [&string_parameter](const std::string& name, double fallback) {
        const std::string text = string_parameter(name, "");
        return text.empty() ? fallback : std::stod(text);
    };
    const auto int_parameter = [&string_parameter](const std::string& name, int fallback) {
        const std::string text = string_parameter(name, "");
        return text.empty() ? fallback : std::stoi(text);
    };
    const auto bool_parameter = [&string_parameter](const std::string& name, bool fallback) {
        const std::string text = string_parameter(name, "");
        if (text.empty()) {
            return fallback;
        }
        if (text == "true" || text == "1") {
            return true;
        }
        if (text == "false" || text == "0") {
            return false;
        }
        throw std::invalid_argument(name + " must be true or false");
    };

    lowstate_topic_ = string_parameter("lowstate_topic", "/lowstate");
    lowcmd_topic_ = string_parameter("lowcmd_topic", "/lowcmd");
    gripper_state_topics_ = {
        string_parameter("left_gripper_state_topic", "/grip_arm0/joint_states"),
        string_parameter("right_gripper_state_topic", "/grip_arm1/joint_states"),
    };
    gripper_command_topics_ = {
        string_parameter("left_gripper_command_topic", "/grip_arm0/mit_command"),
        string_parameter("right_gripper_command_topic", "/grip_arm1/mit_command"),
    };
    gripper_nodes_ = {
        string_parameter("left_gripper_node", "/grip_arm0"),
        string_parameter("right_gripper_node", "/grip_arm1"),
    };
    wrench_topics_ = {
        string_parameter("left_wrench_topic", "/arm0/wrench_raw"),
        string_parameter("right_wrench_topic", "/arm1/wrench_raw"),
    };
    wrench_scales_ = {
        double_parameter("left_wrench_scale", wrench_scales_[0]),
        double_parameter("right_wrench_scale", wrench_scales_[1]),
    };
    state_timeout_s_ = double_parameter("state_timeout_s", state_timeout_s_);
    gripper_state_timeout_s_ =
        double_parameter("gripper_state_timeout_s", gripper_state_timeout_s_);
    gripper_command_rate_hz_ =
        double_parameter("gripper_command_rate_hz", gripper_command_rate_hz_);
    arm_stiffness_scale_ =
        double_parameter("arm_stiffness_scale", arm_stiffness_scale_);
    gripper_kp_ = double_parameter("gripper_kp", gripper_kp_);
    gripper_kd_ = double_parameter("gripper_kd", gripper_kd_);
    gripper_service_timeout_s_ =
        double_parameter("gripper_service_timeout_s", gripper_service_timeout_s_);
    motion_switch_timeout_s_ =
        double_parameter("motion_switch_timeout_s", motion_switch_timeout_s_);
    motion_select_timeout_s_ =
        double_parameter("motion_select_timeout_s", motion_select_timeout_s_);
    motion_release_attempts_ =
        int_parameter("motion_release_attempts", motion_release_attempts_);
    motion_release_retry_s_ =
        double_parameter("motion_release_retry_s", motion_release_retry_s_);
    lowcmd_quiet_period_s_ =
        double_parameter("lowcmd_quiet_period_s", lowcmd_quiet_period_s_);
    lowcmd_quiet_timeout_s_ =
        double_parameter("lowcmd_quiet_timeout_s", lowcmd_quiet_timeout_s_);
    require_pr_mode_ = bool_parameter("require_pr_mode", require_pr_mode_);
    manage_motion_mode_ = bool_parameter("manage_motion_mode", manage_motion_mode_);
    restore_motion_mode_ = bool_parameter("restore_motion_mode", restore_motion_mode_);
    fallback_motion_mode_ = string_parameter("fallback_motion_mode", fallback_motion_mode_);

    const std::array<double, 10> positive_values = {
        state_timeout_s_,          gripper_state_timeout_s_,  gripper_command_rate_hz_,
        gripper_service_timeout_s_, motion_switch_timeout_s_, motion_select_timeout_s_,
        lowcmd_quiet_period_s_,    lowcmd_quiet_timeout_s_,   gripper_kp_ + 1.0,
        gripper_kd_ + 1.0,
    };
    if (!std::all_of(positive_values.begin(), positive_values.end(), finite_positive) ||
        !finite_positive(arm_stiffness_scale_) || arm_stiffness_scale_ > 4.0 ||
        motion_release_attempts_ <= 0 || !std::isfinite(motion_release_retry_s_) ||
        motion_release_retry_s_ < 0.0 || gripper_kp_ < 0.0 || gripper_kp_ > 500.0 ||
        gripper_kd_ < 0.0 || gripper_kd_ > 5.0) {
        throw std::invalid_argument("invalid ros2_control hardware safety parameter");
    }
    if (lowstate_topic_.empty() || lowcmd_topic_.empty() ||
        std::any_of(gripper_state_topics_.begin(), gripper_state_topics_.end(),
                    [](const std::string& value) { return value.empty(); }) ||
        std::any_of(gripper_command_topics_.begin(), gripper_command_topics_.end(),
                    [](const std::string& value) { return value.empty(); }) ||
        std::any_of(gripper_nodes_.begin(), gripper_nodes_.end(),
                    [](const std::string& value) { return value.empty() || value.front() != '/'; }) ||
        std::any_of(wrench_topics_.begin(), wrench_topics_.end(),
                    [](const std::string& value) { return value.empty(); }) ||
        std::any_of(wrench_scales_.begin(), wrench_scales_.end(),
                    [](double value) { return !finite_positive(value); })) {
        throw std::invalid_argument("hardware topic and gripper node names must not be empty");
    }
    if (restore_motion_mode_ && !safe_mode_name(fallback_motion_mode_)) {
        throw std::invalid_argument("fallback_motion_mode contains unsupported characters");
    }

    const std::string gain_file = string_parameter("gain_file", "");
    if (gain_file.empty() || !load_gains(gain_file)) {
        throw std::invalid_argument("gain_file is missing or invalid");
    }
    RCLCPP_INFO(
        rclcpp::get_logger("G1TopicSystem"),
        "Loaded G1 gains with arm stiffness scale %.2f (arm kd unchanged)",
        arm_stiffness_scale_);
    return true;
}

bool G1TopicSystem::load_gains(const std::string& path) {
    const YAML::Node config = YAML::LoadFile(path);
    const auto names = config["joint_names"];
    const auto stiffness = config["stiffness"];
    const auto damping = config["damping"];
    if (!names || !stiffness || !damping || names.size() != kG1JointCount ||
        stiffness.size() != kG1JointCount || damping.size() != kG1JointCount) {
        return false;
    }
    for (std::size_t index = 0; index < kG1JointCount; ++index) {
        if (names[index].as<std::string>() != kExpectedJointNames[index]) {
            return false;
        }
        stiffness_[index] = stiffness[index].as<double>();
        damping_[index] = damping[index].as<double>();
        if (index >= kFirstArmJointIndex) {
            stiffness_[index] *= arm_stiffness_scale_;
        }
        if (!std::isfinite(stiffness_[index]) || stiffness_[index] < 0.0 ||
            !std::isfinite(damping_[index]) || damping_[index] < 0.0) {
            return false;
        }
    }
    return true;
}

void G1TopicSystem::create_ros_interfaces() {
    node_ = std::make_shared<rclcpp::Node>(info_.name + "_topic_bridge");

    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    lowcmd_publisher_ =
        node_->create_publisher<unitree_hg::msg::LowCmd>(lowcmd_topic_, reliable_qos);
    for (std::size_t side = 0; side < 2; ++side) {
        gripper_publishers_[side] = node_->create_publisher<gloria_ros::msg::MitCommand>(
            gripper_command_topics_[side], reliable_qos);
    }
    auto wrench_qos = rclcpp::QoS(rclcpp::KeepLast(64)).best_effort();
    for (std::size_t side = 0; side < kForceTorqueSensorCount; ++side) {
        wrench_subscriptions_[side] =
            node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
                wrench_topics_[side], wrench_qos,
                [this, side](geometry_msgs::msg::WrenchStamped::SharedPtr message) {
                    on_wrench(side, std::move(message));
                });
    }
    lowstate_subscription_ = node_->create_subscription<unitree_hg::msg::LowState>(
        lowstate_topic_, sensor_qos,
        [this](unitree_hg::msg::LowState::SharedPtr message) { on_lowstate(std::move(message)); });
    for (std::size_t side = 0; side < 2; ++side) {
        gripper_state_subscriptions_[side] =
            node_->create_subscription<sensor_msgs::msg::JointState>(
                gripper_state_topics_[side], sensor_qos,
                [this, side](sensor_msgs::msg::JointState::SharedPtr message) {
                    on_gripper_state(side, std::move(message));
                });
    }
    lowcmd_subscription_ = node_->create_subscription<unitree_hg::msg::LowCmd>(
        lowcmd_topic_, sensor_qos,
        [this](unitree_hg::msg::LowCmd::SharedPtr message) {
            on_lowcmd(std::move(message));
        });
    motion_request_publisher_ = node_->create_publisher<unitree_api::msg::Request>(
        "/api/motion_switcher/request", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    motion_response_subscription_ = node_->create_subscription<unitree_api::msg::Response>(
        "/api/motion_switcher/response", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        [this](unitree_api::msg::Response::SharedPtr message) {
            on_motion_response(std::move(message));
        });

    for (std::size_t side = 0; side < 2; ++side) {
        gripper_clients_[side][0] =
            node_->create_client<Trigger>(gripper_nodes_[side] + "/enable");
        gripper_clients_[side][1] =
            node_->create_client<Trigger>(gripper_nodes_[side] + "/disable");
    }
}

std::vector<hardware_interface::StateInterface> G1TopicSystem::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> interfaces;
    interfaces.reserve(kControlledJointCount * 3);
    for (std::size_t index = 0; index < info_.joints.size(); ++index) {
        interfaces.emplace_back(
            info_.joints[index].name, hardware_interface::HW_IF_POSITION,
            &state_position_[index]);
        interfaces.emplace_back(
            info_.joints[index].name, hardware_interface::HW_IF_VELOCITY,
            &state_velocity_[index]);
        interfaces.emplace_back(
            info_.joints[index].name, hardware_interface::HW_IF_EFFORT,
            &state_effort_[index]);
    }
    for (std::size_t sensor = 0; sensor < kForceTorqueSensorCount; ++sensor) {
        for (std::size_t axis = 0; axis < kWrenchAxisCount; ++axis) {
            interfaces.emplace_back(
                wrench_sensor_names_[sensor], kWrenchInterfaceNames[axis],
                &wrench_state_[sensor][axis]);
        }
    }
    for (std::size_t axis = 0; axis < kImuAxisCount; ++axis) {
        interfaces.emplace_back(
            imu_sensor_name_, kImuInterfaceNames[axis], &imu_state_[axis]);
    }
    return interfaces;
}

std::vector<hardware_interface::CommandInterface> G1TopicSystem::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.reserve(kControlledJointCount);
    for (std::size_t index = 0; index < info_.joints.size(); ++index) {
        interfaces.emplace_back(
            info_.joints[index].name, hardware_interface::HW_IF_POSITION,
            &command_position_[index]);
    }
    return interfaces;
}

hardware_interface::return_type G1TopicSystem::start() {
    if (status_ == hardware_interface::status::STARTED) {
        return hardware_interface::return_type::OK;
    }
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_thread_ = std::thread([executor = executor_]() { executor->spin(); });
    status_ = hardware_interface::status::STARTED;
    RCLCPP_INFO(node_->get_logger(), "G1 topic system started with output disabled");
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type G1TopicSystem::stop() {
    if (status_ == hardware_interface::status::STARTED) {
        clear_output();
        if (rclcpp::ok() && control_acquired_.load(std::memory_order_acquire)) {
            std::string error;
            if (!release_control(true, error)) {
                RCLCPP_ERROR(node_->get_logger(), "Shutdown cleanup failed: %s", error.c_str());
            }
        }
    }
    if (executor_) {
        executor_->cancel();
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
        executor_.reset();
    }
    status_ = hardware_interface::status::STOPPED;
    return hardware_interface::return_type::OK;
}

void G1TopicSystem::on_lowstate(const unitree_hg::msg::LowState::SharedPtr message) {
    if (message->motor_state.size() < kG1JointCount) {
        return;
    }
    for (std::size_t index = 0; index < kG1JointCount; ++index) {
        if (!std::isfinite(message->motor_state[index].q)) {
            return;
        }
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (std::size_t index = 0; index < kG1JointCount; ++index) {
        const auto& motor = message->motor_state[index];
        pending_state_.position[index] = motor.q;
        pending_state_.velocity[index] = std::isfinite(motor.dq) ? motor.dq : 0.0;
        pending_state_.effort[index] = std::isfinite(motor.tau_est) ? motor.tau_est : 0.0;
        pending_state_.received[index] = true;
    }
    pending_state_.mode_pr = message->mode_pr;
    pending_state_.mode_machine = message->mode_machine;
    const std::array<double, 4> orientation = {
        message->imu_state.quaternion[1],
        message->imu_state.quaternion[2],
        message->imu_state.quaternion[3],
        message->imu_state.quaternion[0],
    };
    const double orientation_norm = std::sqrt(
        orientation[0] * orientation[0] + orientation[1] * orientation[1] +
        orientation[2] * orientation[2] + orientation[3] * orientation[3]);
    if (std::isfinite(orientation_norm) && orientation_norm > 0.0) {
        for (std::size_t axis = 0; axis < orientation.size(); ++axis) {
            pending_imu_[axis] = orientation[axis] / orientation_norm;
        }
    } else {
        pending_imu_[0] = 0.0;
        pending_imu_[1] = 0.0;
        pending_imu_[2] = 0.0;
        pending_imu_[3] = 1.0;
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const double angular_velocity = message->imu_state.gyroscope[axis];
        const double linear_acceleration = message->imu_state.accelerometer[axis];
        pending_imu_[4 + axis] = std::isfinite(angular_velocity) ? angular_velocity : 0.0;
        pending_imu_[7 + axis] = std::isfinite(linear_acceleration) ? linear_acceleration : 0.0;
    }
    pending_state_.g1_received_at = Clock::now();
}

void G1TopicSystem::on_gripper_state(
    std::size_t side, const sensor_msgs::msg::JointState::SharedPtr message) {
    if (message->position.empty()) {
        return;
    }
    const std::string& expected_name = kExpectedJointNames[kG1JointCount + side];
    std::size_t source_index = 0;
    const auto named = std::find(message->name.begin(), message->name.end(), expected_name);
    if (named != message->name.end()) {
        source_index = static_cast<std::size_t>(std::distance(message->name.begin(), named));
    }
    if (source_index >= message->position.size() || !std::isfinite(message->position[source_index])) {
        return;
    }
    const std::size_t target_index = kG1JointCount + side;
    std::lock_guard<std::mutex> lock(state_mutex_);
    pending_state_.position[target_index] = message->position[source_index];
    pending_state_.velocity[target_index] =
        source_index < message->velocity.size() && std::isfinite(message->velocity[source_index])
            ? message->velocity[source_index]
            : 0.0;
    pending_state_.effort[target_index] =
        source_index < message->effort.size() && std::isfinite(message->effort[source_index])
            ? message->effort[source_index]
            : 0.0;
    pending_state_.received[target_index] = true;
    pending_state_.gripper_received_at[side] = Clock::now();
}

void G1TopicSystem::on_wrench(
    std::size_t side, const geometry_msgs::msg::WrenchStamped::SharedPtr message) {
    const double scale = wrench_scales_[side];
    const std::array<double, kWrenchAxisCount> values = {
        message->wrench.force.x * scale,
        message->wrench.force.y * scale,
        message->wrench.force.z * scale,
        message->wrench.torque.x * scale,
        message->wrench.torque.y * scale,
        message->wrench.torque.z * scale,
    };
    if (!std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); })) {
        return;
    }
    wrench_sequence_[side].fetch_add(1, std::memory_order_acq_rel);
    for (std::size_t axis = 0; axis < kWrenchAxisCount; ++axis) {
        pending_wrench_[side][axis].store(values[axis], std::memory_order_relaxed);
    }
    wrench_sequence_[side].fetch_add(1, std::memory_order_release);
    wrench_received_[side].store(true, std::memory_order_release);
}

void G1TopicSystem::on_lowcmd(const unitree_hg::msg::LowCmd::SharedPtr) {
    {
        std::lock_guard<std::mutex> lock(lowcmd_observer_mutex_);
        last_observed_lowcmd_ = Clock::now();
    }
    lowcmd_observer_condition_.notify_all();
}

void G1TopicSystem::on_motion_response(const unitree_api::msg::Response::SharedPtr message) {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (!pending_motion_id_ || message->header.identity.id != *pending_motion_id_) {
        return;
    }
    motion_response_ = *message;
    motion_condition_.notify_all();
}

hardware_interface::return_type G1TopicSystem::read() {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (std::size_t index = 0; index < kControlledJointCount; ++index) {
            if (!pending_state_.received[index]) {
                continue;
            }
            state_position_[index] = pending_state_.position[index];
            state_velocity_[index] = pending_state_.velocity[index];
            state_effort_[index] = pending_state_.effort[index];
        }
        imu_state_ = pending_imu_;
    }
    for (std::size_t sensor = 0; sensor < kForceTorqueSensorCount; ++sensor) {
        if (wrench_received_[sensor].load(std::memory_order_acquire)) {
            std::array<double, kWrenchAxisCount> snapshot{};
            std::uint64_t before = 0;
            std::uint64_t after = 0;
            do {
                before = wrench_sequence_[sensor].load(std::memory_order_acquire);
                if ((before & 1U) != 0U) {
                    continue;
                }
                for (std::size_t axis = 0; axis < kWrenchAxisCount; ++axis) {
                    snapshot[axis] =
                        pending_wrench_[sensor][axis].load(std::memory_order_relaxed);
                }
                after = wrench_sequence_[sensor].load(std::memory_order_acquire);
            } while (before != after || (after & 1U) != 0U);
            wrench_state_[sensor] = snapshot;
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type G1TopicSystem::write() {
    if (output_inhibited_.load(std::memory_order_acquire)) {
        return hardware_interface::return_type::OK;
    }

    const std::uint32_t claimed_mask =
        claimed_joint_mask_.load(std::memory_order_acquire);
    if (claimed_mask == 0U) {
        return hardware_interface::return_type::OK;
    }

    for (std::size_t index = 0; index < command_position_.size(); ++index) {
        if ((claimed_mask & (1U << index)) == 0U) {
            command_position_[index] = state_position_[index];
        }
    }

    if ((claimed_mask & kBodyClaimMask) != 0U) {
        std::string state_error;
        if (!state_ready(claimed_mask & kBodyClaimMask, state_error)) {
            clear_output();
            RCLCPP_ERROR_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 1000,
                "Stopping G1 body output: %s", state_error.c_str());
            return hardware_interface::return_type::ERROR;
        }
        for (std::size_t index = 0; index < kG1JointCount; ++index) {
            if ((claimed_mask & (1U << index)) == 0U) {
                continue;
            }
            const double value = command_position_[index];
            if (!std::isfinite(value) || value < lower_limits_[index] ||
                value > upper_limits_[index]) {
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 1000,
                    "Ignoring out-of-range command for %s",
                    info_.joints[index].name.c_str());
                return hardware_interface::return_type::OK;
            }
        }
        std::uint8_t mode_machine = 0;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            mode_machine = pending_state_.mode_machine;
        }
        lowcmd_publisher_->publish(make_lowcmd(mode_machine));
    }

    const auto now = Clock::now();
    if ((claimed_mask & kGripperClaimMask) != 0U && now >= next_gripper_publish_) {
        const auto period = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1.0 / gripper_command_rate_hz_));
        next_gripper_publish_ = advance_periodic_deadline(
            next_gripper_publish_, now, period);
        for (std::size_t side = 0; side < 2; ++side) {
            const std::uint32_t side_claim = 1U << (kG1JointCount + side);
            if ((claimed_mask & side_claim) == 0U) {
                continue;
            }
            std::string state_error;
            if (!state_ready(side_claim, state_error)) {
                RCLCPP_ERROR_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 1000,
                    "Skipping %s gripper output: %s",
                    side == 0 ? "left" : "right", state_error.c_str());
                continue;
            }
            const std::size_t index = kG1JointCount + side;
            const double value = command_position_[index];
            if (!std::isfinite(value) || value < lower_limits_[index] ||
                value > upper_limits_[index]) {
                RCLCPP_ERROR_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 1000,
                    "Skipping %s gripper output: invalid command %.6f",
                    side == 0 ? "left" : "right", value);
                continue;
            }
            gloria_ros::msg::MitCommand command;
            command.q = value;
            command.dq = 0.0;
            command.kp = gripper_kp_;
            command.kd = gripper_kd_;
            command.tau = 0.0;
            gripper_publishers_[side]->publish(command);
        }
    }
    return hardware_interface::return_type::OK;
}

bool G1TopicSystem::state_ready(std::uint32_t claim_mask, std::string& reason) const {
    const auto now = Clock::now();
    std::lock_guard<std::mutex> lock(state_mutex_);
    if ((claim_mask & kBodyClaimMask) != 0U) {
        for (std::size_t index = 0; index < kG1JointCount; ++index) {
            if (!pending_state_.received[index]) {
                reason = "G1 body feedback is incomplete";
                return false;
            }
        }
    }
    if ((claim_mask & kBodyClaimMask) != 0U &&
        std::chrono::duration<double>(now - pending_state_.g1_received_at).count() >
        state_timeout_s_) {
        reason = "LowState is stale";
        return false;
    }
    for (std::size_t side = 0; side < 2; ++side) {
        if ((claim_mask & (1U << (kG1JointCount + side))) == 0U) {
            continue;
        }
        if (std::chrono::duration<double>(now - pending_state_.gripper_received_at[side]).count() >
            gripper_state_timeout_s_) {
            reason = side == 0 ? "left gripper state is stale" : "right gripper state is stale";
            return false;
        }
    }
    if ((claim_mask & kBodyClaimMask) != 0U && require_pr_mode_ && pending_state_.mode_pr != 0) {
        reason = "PR mode 0 is required";
        return false;
    }
    return true;
}

std::uint32_t G1TopicSystem::next_claim_mask(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) const {
    std::uint32_t result = claimed_joint_mask_.load(std::memory_order_acquire);
    for (const auto& interface : stop_interfaces) {
        const auto index = control_interface_index(interface);
        if (index) {
            result &= ~(1U << *index);
        }
    }
    for (const auto& interface : start_interfaces) {
        const auto index = control_interface_index(interface);
        if (index) {
            result |= 1U << *index;
        }
    }
    return result;
}

std::optional<std::size_t> G1TopicSystem::control_interface_index(
    const std::string& name) const {
    for (std::size_t index = 0; index < info_.joints.size(); ++index) {
        if (name == info_.joints[index].name + "/" + hardware_interface::HW_IF_POSITION) {
            return index;
        }
    }
    return std::nullopt;
}

hardware_interface::return_type G1TopicSystem::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
    std::lock_guard<std::mutex> switch_lock(switch_mutex_);
    const std::uint32_t current = claimed_joint_mask_.load(std::memory_order_acquire);
    const std::uint32_t next = next_claim_mask(start_interfaces, stop_interfaces);
    const bool acquiring = current == 0U && next != 0U;
    const bool releasing = current != 0U && next == 0U;
    prepared_joint_mask_ = next;
    prepared_output_enabled_ = next != 0U;
    std::string error;
    if (acquiring && !acquire_control(error)) {
        RCLCPP_ERROR(node_->get_logger(), "Cannot acquire low-level control: %s", error.c_str());
        return hardware_interface::return_type::ERROR;
    }
    if (releasing) {
        clear_output();
        if (!release_control(true, error)) {
            RCLCPP_ERROR(node_->get_logger(), "Cannot finish low-level release: %s", error.c_str());
            return hardware_interface::return_type::ERROR;
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type G1TopicSystem::perform_command_mode_switch(
    const std::vector<std::string>&, const std::vector<std::string>&) {
    std::lock_guard<std::mutex> switch_lock(switch_mutex_);
    claimed_joint_mask_.store(prepared_joint_mask_, std::memory_order_release);
    output_inhibited_.store(!prepared_output_enabled_, std::memory_order_release);
    next_gripper_publish_ = Clock::now();
    return hardware_interface::return_type::OK;
}

bool G1TopicSystem::acquire_control(std::string& error) {
    const std::uint32_t claim_mask = prepared_joint_mask_;
    if (!state_ready(claim_mask, error)) {
        std::string disable_error;
        call_grippers("disable", claim_mask & kGripperClaimMask, disable_error);
        if (!disable_error.empty()) {
            error += "; " + disable_error;
        }
        return false;
    }

    previous_motion_mode_.clear();
    if ((claim_mask & kBodyClaimMask) != 0U && manage_motion_mode_) {
        std::string mode;
        if (!check_motion_mode(mode, error)) {
            std::string disable_error;
            call_grippers(
                "disable", claim_mask & kGripperClaimMask, disable_error);
            return false;
        }
        previous_motion_mode_ = mode;
        for (int attempt = 0; !mode.empty() && attempt < motion_release_attempts_; ++attempt) {
            if (!release_motion_mode(error)) {
                std::string rollback_error;
                release_control(true, rollback_error);
                if (!rollback_error.empty()) {
                    error += "; rollback failed: " + rollback_error;
                }
                return false;
            }
            if (motion_release_retry_s_ > 0.0) {
                std::this_thread::sleep_for(std::chrono::duration<double>(motion_release_retry_s_));
            }
            if (!check_motion_mode(mode, error)) {
                std::string rollback_error;
                release_control(true, rollback_error);
                return false;
            }
        }
        if (!mode.empty()) {
            error = "motion mode remains active after release attempts: " + mode;
            std::string rollback_error;
            release_control(true, rollback_error);
            return false;
        }
        if (!wait_for_lowcmd_quiet()) {
            error = "existing /lowcmd stream did not become quiet";
            std::string rollback_error;
            release_control(true, rollback_error);
            return false;
        }
    }

    if (!call_grippers("enable", claim_mask & kGripperClaimMask, error)) {
        std::string rollback_error;
        release_control(true, rollback_error);
        if (!rollback_error.empty()) {
            error += "; rollback failed: " + rollback_error;
        }
        return false;
    }
    if (!state_ready(claim_mask, error)) {
        std::string rollback_error;
        release_control(true, rollback_error);
        return false;
    }
    control_acquired_.store(true, std::memory_order_release);
    return true;
}

bool G1TopicSystem::release_control(bool restore_mode, std::string& error) {
    std::vector<std::string> errors;
    std::string detail;
    const std::uint32_t claim_mask =
        claimed_joint_mask_.load(std::memory_order_acquire) | prepared_joint_mask_;
    if (!call_grippers("disable", claim_mask & kGripperClaimMask, detail)) {
        errors.push_back(detail);
    }
    if (restore_mode && !restore_motion_mode(detail)) {
        errors.push_back(detail);
    }
    control_acquired_.store(false, std::memory_order_release);
    error = join_errors(errors);
    return errors.empty();
}

bool G1TopicSystem::call_grippers(
    const std::string& action, std::uint32_t side_mask, std::string& error) {
    if (side_mask == 0U) {
        error.clear();
        return true;
    }
    const std::size_t action_index = action == "enable" ? 0 : 1;
    const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                           std::chrono::duration<double>(gripper_service_timeout_s_));
    std::array<bool, 2> ready{};
    while (Clock::now() < deadline && (!ready[0] || !ready[1])) {
        for (std::size_t side = 0; side < 2; ++side) {
            if ((side_mask & (1U << (kG1JointCount + side))) == 0U) {
                ready[side] = true;
                continue;
            }
            ready[side] = ready[side] || gripper_clients_[side][action_index]->service_is_ready();
        }
        if (!ready[0] || !ready[1]) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    using Future = rclcpp::Client<Trigger>::SharedFuture;
    std::vector<std::pair<std::size_t, Future>> futures;
    std::vector<std::string> errors;
    for (std::size_t side = 0; side < 2; ++side) {
        if ((side_mask & (1U << (kG1JointCount + side))) == 0U) {
            continue;
        }
        if (!ready[side]) {
            errors.push_back(gripper_nodes_[side] + "/" + action + " is unavailable");
            continue;
        }
        futures.emplace_back(
            side, gripper_clients_[side][action_index]->async_send_request(
                      std::make_shared<Trigger::Request>()));
    }
    for (auto& pending : futures) {
        if (pending.second.wait_until(deadline) != std::future_status::ready) {
            errors.push_back(gripper_nodes_[pending.first] + "/" + action + " timed out");
            continue;
        }
        try {
            const auto response = pending.second.get();
            if (!response->success) {
                errors.push_back(
                    gripper_nodes_[pending.first] + "/" + action + " failed: " +
                    response->message);
            }
        } catch (const std::exception& exception) {
            errors.push_back(
                gripper_nodes_[pending.first] + "/" + action + " failed: " + exception.what());
        }
    }
    error = join_errors(errors);
    return errors.empty();
}

bool G1TopicSystem::call_motion(
    std::int64_t api_id, const std::string& parameter, double timeout_s,
    unitree_api::msg::Response& response, std::string& error) {
    std::lock_guard<std::mutex> call_lock(motion_call_mutex_);
    const auto identity = static_cast<std::int64_t>(Clock::now().time_since_epoch().count());
    unitree_api::msg::Request request;
    request.header.identity.id = identity;
    request.header.identity.api_id = api_id;
    request.parameter = parameter;
    {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        pending_motion_id_ = identity;
        motion_response_.reset();
    }
    motion_request_publisher_->publish(request);

    std::unique_lock<std::mutex> lock(motion_mutex_);
    const bool received = motion_condition_.wait_for(
        lock, std::chrono::duration<double>(timeout_s), [this]() { return motion_response_.has_value(); });
    if (!received) {
        pending_motion_id_.reset();
        error = "motion switcher request timed out";
        return false;
    }
    response = *motion_response_;
    pending_motion_id_.reset();
    motion_response_.reset();
    if (response.header.status.code != 0) {
        error = "motion switcher returned status " +
                std::to_string(response.header.status.code);
        return false;
    }
    return true;
}

bool G1TopicSystem::check_motion_mode(std::string& mode, std::string& error) {
    unitree_api::msg::Response response;
    if (!call_motion(kCheckModeApiId, "", motion_switch_timeout_s_, response, error)) {
        return false;
    }
    try {
        const YAML::Node data = YAML::Load(response.data.empty() ? "{}" : response.data);
        const auto name = data["name"];
        mode = name ? name.as<std::string>() : "";
    } catch (const std::exception& exception) {
        error = std::string("invalid CheckMode response: ") + exception.what();
        return false;
    }
    return true;
}

bool G1TopicSystem::release_motion_mode(std::string& error) {
    unitree_api::msg::Response response;
    return call_motion(kReleaseModeApiId, "", motion_switch_timeout_s_, response, error);
}

bool G1TopicSystem::restore_motion_mode(std::string& error) {
    if (!manage_motion_mode_ || !restore_motion_mode_) {
        return true;
    }
    const std::string target =
        previous_motion_mode_.empty() ? fallback_motion_mode_ : previous_motion_mode_;
    if (!safe_mode_name(target)) {
        error = "invalid motion mode to restore";
        return false;
    }
    const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                           std::chrono::duration<double>(motion_select_timeout_s_));
    std::string last_error;
    while (Clock::now() < deadline) {
        std::string selected;
        if (check_motion_mode(selected, last_error)) {
            if (selected == target) {
                return true;
            }
            if (!selected.empty()) {
                error = "motion mode " + selected + " became active while restoring " + target;
                return false;
            }
        }
        unitree_api::msg::Response response;
        const std::string parameter = "{\"name\":\"" + target + "\"}";
        call_motion(kSelectModeApiId, parameter, motion_switch_timeout_s_, response, last_error);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    error = "motion mode " + target + " was not restored: " + last_error;
    return false;
}

bool G1TopicSystem::wait_for_lowcmd_quiet() {
    const auto deadline = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                           std::chrono::duration<double>(lowcmd_quiet_timeout_s_));
    std::unique_lock<std::mutex> lock(lowcmd_observer_mutex_);
    while (Clock::now() < deadline) {
        const auto quiet_for = std::chrono::duration<double>(Clock::now() - last_observed_lowcmd_).count();
        if (quiet_for >= lowcmd_quiet_period_s_) {
            return true;
        }
        lowcmd_observer_condition_.wait_for(
            lock, std::chrono::duration<double>(lowcmd_quiet_period_s_ - quiet_for));
    }
    return false;
}

unitree_hg::msg::LowCmd G1TopicSystem::make_lowcmd(std::uint8_t mode_machine) const {
    unitree_hg::msg::LowCmd message;
    message.mode_pr = 0;
    message.mode_machine = mode_machine;
    for (std::size_t index = 0; index < kG1JointCount; ++index) {
        auto& command = message.motor_cmd[index];
        command.mode = 1;
        command.q = static_cast<float>(command_position_[index]);
        command.dq = 0.0F;
        command.tau = 0.0F;
        command.kp = static_cast<float>(stiffness_[index]);
        command.kd = static_cast<float>(damping_[index]);
    }
    message.crc = lowcmd_crc(message);
    return message;
}

void G1TopicSystem::clear_output() {
    output_inhibited_.store(true, std::memory_order_release);
}

}  // namespace unitree_g1_ros2_control

PLUGINLIB_EXPORT_CLASS(
    unitree_g1_ros2_control::G1TopicSystem,
    hardware_interface::SystemInterface)