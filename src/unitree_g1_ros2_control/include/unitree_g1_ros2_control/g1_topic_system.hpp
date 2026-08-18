#ifndef UNITREE_G1_ROS2_CONTROL__G1_TOPIC_SYSTEM_HPP_
#define UNITREE_G1_ROS2_CONTROL__G1_TOPIC_SYSTEM_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "gloria_ros/msg/mit_command.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "unitree_api/msg/response.hpp"

#include "rclcpp/client.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "realtime_tools/mutex.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_hg/msg/imu_state.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"

namespace unitree_g1_ros2_control {

class G1TopicSystem : public hardware_interface::SystemInterface {
public:
    ~G1TopicSystem() override;

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;
    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

private:
    using Clock = std::chrono::steady_clock;
    using Trigger = std_srvs::srv::Trigger;

    static constexpr std::size_t kG1JointCount = 29;
    static constexpr std::size_t kControlledJointCount = 31;
    static constexpr std::size_t kForceTorqueSensorCount = 2;
    static constexpr std::size_t kWrenchAxisCount = 6;
    static constexpr std::size_t kImuAxisCount = 10;

    struct PendingState {
        std::array<double, kControlledJointCount> position{};
        std::array<double, kControlledJointCount> velocity{};
        std::array<double, kControlledJointCount> effort{};
        std::array<bool, kControlledJointCount> received{};
        Clock::time_point g1_received_at{};
        std::array<Clock::time_point, 2> gripper_received_at{};
        std::uint8_t mode_pr{0};
        std::uint8_t mode_machine{0};
    };

    bool configure_interfaces();
    void configure_parameters();
    bool load_gains(const std::string& path);
    void create_ros_interfaces();
    void on_lowstate(const unitree_hg::msg::LowState::SharedPtr message);
    void on_torso_imu(const unitree_hg::msg::IMUState::SharedPtr message);
    static void decode_imu(
        const unitree_hg::msg::IMUState& imu,
        std::array<double, kImuAxisCount>& target);
    void on_gripper_state(
        std::size_t side, const sensor_msgs::msg::JointState::SharedPtr message);
    void on_wrench(
        std::size_t side, const geometry_msgs::msg::WrenchStamped::SharedPtr message);
    void on_lowcmd(const unitree_hg::msg::LowCmd::SharedPtr message);
    void on_motion_response(const unitree_api::msg::Response::SharedPtr message);
    bool state_ready(std::uint32_t claim_mask, std::string& reason) const;
    bool acquire_control(std::string& error);
    bool release_control(bool restore_mode, std::string& error);
    bool call_grippers(
        const std::string& action, std::uint32_t side_mask, std::string& error);
    bool call_motion(
        std::int64_t api_id, const std::string& parameter, double timeout_s,
        unitree_api::msg::Response& response, std::string& error);
    bool check_motion_mode(std::string& mode, std::string& error);
    bool release_motion_mode(std::string& error);
    bool restore_motion_mode(std::string& error);
    bool wait_for_lowcmd_quiet();
    std::optional<std::size_t> control_interface_index(const std::string& name) const;
    std::uint32_t next_claim_mask(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) const;
    void shutdown();
    void clear_output();
    void release_body();
    void restore_motion_on_shutdown();
    void start_release_channel();
    void stop_release_channel();

    std::vector<double> state_position_;
    std::vector<double> state_velocity_;
    std::vector<double> state_effort_;
    std::vector<double> command_position_;
    // 停止时把 `kp` 降到零的时长；`kd` 全程保留，只在最后一帧归零。
    double release_ramp_s_{2.0};
    // The gains actually written into every command, exported as state so that
    // controllers never have to duplicate the gain file.
    std::array<double, kControlledJointCount> gain_stiffness_{};
    std::array<double, kControlledJointCount> gain_damping_{};
    double arm_stiffness_scale_{1.0};   // 双臂 15–28 号关节的 `kp` 的缩放
    std::array<std::array<double, kWrenchAxisCount>, kForceTorqueSensorCount>
        wrench_state_{};
    std::array<std::array<std::atomic<double>, kWrenchAxisCount>, kForceTorqueSensorCount>
        pending_wrench_{};
    std::array<std::atomic<bool>, kForceTorqueSensorCount> wrench_received_{};
    std::array<std::atomic<std::uint64_t>, kForceTorqueSensorCount> wrench_sequence_{};
    std::array<double, kImuAxisCount> imu_state_{};
    std::array<double, kImuAxisCount> pending_imu_{};
    // Second IMU, physically in the torso. ``LowState.imu_state`` sits in the
    // pelvis, which is separated from ``torso_link`` by the three waist
    // joints, so torso-rooted models must use this one instead.
    std::array<double, kImuAxisCount> torso_imu_state_{};
    std::array<double, kImuAxisCount> pending_torso_imu_{};
    std::atomic<bool> torso_imu_received_{false};

    mutable realtime_tools::prio_inherit_mutex state_mutex_;
    PendingState pending_state_;

    // ros2_control tracks the lifecycle state itself, but the destructor still
    // needs to know whether there is anything to unwind.
    bool active_{false};
    std::atomic<bool> output_inhibited_{true};
    Clock::time_point next_gripper_publish_{};

    std::mutex switch_mutex_;
    std::atomic<std::uint32_t> claimed_joint_mask_{0};
    std::uint32_t prepared_joint_mask_{0};
    std::atomic<bool> control_acquired_{false};
    std::string previous_motion_mode_;

    std::string lowstate_topic_;
    std::string torso_imu_topic_;
    std::string torso_imu_sensor_name_;
    std::string lowcmd_topic_;
    std::array<std::string, 2> gripper_state_topics_;
    std::array<std::string, 2> gripper_command_topics_;
    std::array<std::string, 2> gripper_nodes_;
    std::array<std::string, kForceTorqueSensorCount> wrench_topics_;
    std::array<std::string, kForceTorqueSensorCount> wrench_sensor_names_;
    std::string imu_sensor_name_;
    std::array<double, kForceTorqueSensorCount> wrench_scales_{{9.80665, 9.80665}};
    double state_timeout_s_{0.25};
    double gripper_state_timeout_s_{0.75};
    double gripper_command_rate_hz_{100.0};
    double gripper_service_timeout_s_{3.0};
    double motion_switch_timeout_s_{1.0};
    double motion_select_timeout_s_{10.0};
    double motion_release_retry_s_{0.2};
    double lowcmd_quiet_period_s_{0.1};
    double lowcmd_quiet_timeout_s_{2.0};
    int motion_release_attempts_{3};
    bool require_pr_mode_{true};
    bool manage_motion_mode_{true};
    bool restore_motion_mode_{true};
    std::string fallback_motion_mode_{"ai"};

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread executor_thread_;
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_publisher_;
    std::array<rclcpp::Publisher<gloria_ros::msg::MitCommand>::SharedPtr, 2>
        gripper_publishers_;
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_subscription_;
    rclcpp::Subscription<unitree_hg::msg::IMUState>::SharedPtr torso_imu_subscription_;
    std::array<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr, 2>
        gripper_state_subscriptions_;
    std::array<
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr,
        kForceTorqueSensorCount>
        wrench_subscriptions_;
    rclcpp::Subscription<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_subscription_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr motion_request_publisher_;
    // stop() only runs once the global context is already shutting down, and a
    // publisher on a dead context silently drops every message. These live on a
    // private context that is kept out of the signal handler, so the release
    // ramp and the motion hand-back actually reach the robot.
    rclcpp::Context::SharedPtr release_context_;
    rclcpp::Node::SharedPtr release_node_;
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr release_lowcmd_publisher_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr release_motion_publisher_;
    rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr motion_response_subscription_;
    std::array<std::array<rclcpp::Client<Trigger>::SharedPtr, 2>, 2> gripper_clients_;

    std::mutex lowcmd_observer_mutex_;
    std::condition_variable lowcmd_observer_condition_;
    Clock::time_point last_observed_lowcmd_{};

    std::mutex motion_mutex_;
    std::mutex motion_call_mutex_;
    std::condition_variable motion_condition_;
    std::optional<std::int64_t> pending_motion_id_;
    std::optional<unitree_api::msg::Response> motion_response_;
};

}  // namespace unitree_g1_ros2_control

#endif  // UNITREE_G1_ROS2_CONTROL__G1_TOPIC_SYSTEM_HPP_