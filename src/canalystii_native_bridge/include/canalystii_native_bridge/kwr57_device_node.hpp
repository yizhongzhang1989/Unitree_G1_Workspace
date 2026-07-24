#pragma once

#include "canalystii_native_bridge/config.hpp"
#include "canalystii_native_bridge/protocol.hpp"

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <thread>

namespace canalystii_native_bridge
{

class Kwr57DeviceNode : public rclcpp::Node
{
public:
  using SendFrame = std::function<bool(int, const CanFrame &)>;

  Kwr57DeviceNode(Kwr57Config config, SendFrame send_frame);
  ~Kwr57DeviceNode() override;

  void handle_frame(const CanFrame & frame);
  void activate();
  void stop_device();
  void report_statistics();

private:
  void start_async(bool tare);
  void start_sequence(bool tare);
  void cancel_start_sequence();
  bool wait_or_cancel(std::chrono::milliseconds duration);
  bool send_command(const CanFrame & frame);
  void publish_sample(const WrenchSample & sample);
  void finish_start_sequence();

  void command_callback(const std_msgs::msg::String::SharedPtr message);
  void start_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void stop_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void tare_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void reset_tare_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void clear_tare();

  Kwr57Config config_;
  SendFrame send_frame_;
  Kwr57Assembler assembler_;
  std::mutex assembler_mutex_;
  std::array<std::atomic<float>, 6> offsets_{};
  std::atomic<bool> tare_pending_{false};
  std::atomic<std::uint64_t> frames_received_{0};
  std::atomic<std::uint64_t> samples_published_{0};
  std::atomic<bool> stopped_{false};
  std::chrono::steady_clock::time_point last_publish_{};
  std::chrono::duration<double> minimum_publish_period_{0.0};
  geometry_msgs::msg::WrenchStamped wrench_message_;

  std::mutex start_api_mutex_;
  std::mutex start_state_mutex_;
  std::condition_variable start_condition_;
  std::thread start_thread_;
  bool start_cancelled_{false};
  bool start_running_{false};

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tare_service_;
};

}  // namespace canalystii_native_bridge
