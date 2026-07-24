#include "canalystii_native_bridge/kwr57_device_node.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>
#include <utility>

namespace canalystii_native_bridge
{
namespace
{

constexpr float kKgfToNewton = 9.80665F;
constexpr auto kCommandSettleTime = std::chrono::milliseconds(100);
constexpr auto kStartConfirmationTime = std::chrono::milliseconds(200);
constexpr auto kStartPollTime = std::chrono::milliseconds(5);

std::string normalized_command(std::string value)
{
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), [](unsigned char character) {
    return !std::isspace(character);
  }));
  value.erase(std::find_if(value.rbegin(), value.rend(), [](unsigned char character) {
    return !std::isspace(character);
  }).base(), value.end());
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

}  // namespace

Kwr57DeviceNode::Kwr57DeviceNode(Kwr57Config config, SendFrame send_frame)
: Node(config.node_name, rclcpp::NodeOptions().use_global_arguments(false)),
  config_(std::move(config)),
  send_frame_(std::move(send_frame)),
  assembler_(config_.data_base_id)
{
  if (!send_frame_) {
    throw std::invalid_argument("KWR57 requires a CAN send callback");
  }
  for (auto & offset : offsets_) {
    offset.store(0.0F);
  }
  if (config_.publish_rate > 0.0) {
    minimum_publish_period_ = std::chrono::duration<double>(1.0 / config_.publish_rate);
  }

  const auto wrench_qos = rclcpp::QoS(rclcpp::KeepLast(32)).best_effort().durability_volatile();
  wrench_publisher_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
    config_.topic, wrench_qos);
  wrench_message_.header.frame_id = config_.frame_id;

  command_subscription_ = create_subscription<std_msgs::msg::String>(
    "~/command", rclcpp::QoS(10),
    std::bind(&Kwr57DeviceNode::command_callback, this, std::placeholders::_1));
  start_service_ = create_service<std_srvs::srv::Trigger>(
    "~/start", std::bind(
      &Kwr57DeviceNode::start_service, this,
      std::placeholders::_1, std::placeholders::_2));
  stop_service_ = create_service<std_srvs::srv::Trigger>(
    "~/stop", std::bind(
      &Kwr57DeviceNode::stop_service, this,
      std::placeholders::_1, std::placeholders::_2));
  tare_service_ = create_service<std_srvs::srv::Trigger>(
    "~/tare", std::bind(
      &Kwr57DeviceNode::tare_service, this,
      std::placeholders::_1, std::placeholders::_2));
  reset_tare_service_ = create_service<std_srvs::srv::Trigger>(
    "~/reset_tare", std::bind(
      &Kwr57DeviceNode::reset_tare_service, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    get_logger(),
    "KWR57 native: channel=%d cmd=0x%X data=0x%X/0x%X/0x%X "
    "rate=%dHz period=%dms -> %s (si=%s)",
    config_.channel_id, config_.cmd_id, config_.data_base_id,
    config_.data_base_id + 1U, config_.data_base_id + 2U,
    config_.sample_rate_hz, config_.period_ms,
    config_.topic.c_str(), config_.use_si ? "true" : "false");
}

Kwr57DeviceNode::~Kwr57DeviceNode()
{
  cancel_start_sequence();
}

void Kwr57DeviceNode::handle_frame(const CanFrame & frame)
{
  frames_received_.fetch_add(1, std::memory_order_relaxed);
  WrenchSample sample;
  AssembleResult result;
  {
    std::lock_guard<std::mutex> lock(assembler_mutex_);
    result = assembler_.push(frame, sample);
  }
  if (result == AssembleResult::kComplete) {
    publish_sample(sample);
  }
}

void Kwr57DeviceNode::activate()
{
  if (config_.autostart) {
    start_async(config_.tare_on_start);
  } else {
    RCLCPP_INFO(get_logger(), "waiting for start command");
  }
}

void Kwr57DeviceNode::stop_device()
{
  if (stopped_.exchange(true)) {
    return;
  }
  cancel_start_sequence();
  send_command(make_kwr57_stop_command(config_.cmd_id));
  RCLCPP_INFO(get_logger(), "stream stopped");
}

void Kwr57DeviceNode::report_statistics()
{
  std::lock_guard<std::mutex> lock(assembler_mutex_);
  RCLCPP_INFO(
    get_logger(),
    "final KWR57 RX frames=%lu samples=%lu malformed=%lu dropped_sequences=%lu",
    static_cast<unsigned long>(frames_received_.load(std::memory_order_relaxed)),
    static_cast<unsigned long>(samples_published_.load(std::memory_order_relaxed)),
    static_cast<unsigned long>(assembler_.malformed_frames()),
    static_cast<unsigned long>(assembler_.dropped_sequences()));
}

void Kwr57DeviceNode::start_async(bool tare)
{
  std::lock_guard<std::mutex> api_lock(start_api_mutex_);
  {
    std::lock_guard<std::mutex> state_lock(start_state_mutex_);
    if (start_running_) {
      RCLCPP_INFO(get_logger(), "stream start already in progress");
      return;
    }
  }
  if (start_thread_.joinable()) {
    start_thread_.join();
  }
  stopped_.store(false);
  {
    std::lock_guard<std::mutex> state_lock(start_state_mutex_);
    start_cancelled_ = false;
    start_running_ = true;
  }
  start_thread_ = std::thread(&Kwr57DeviceNode::start_sequence, this, tare);
}

void Kwr57DeviceNode::start_sequence(bool tare)
{
  if (!send_command(make_kwr57_stop_command(config_.cmd_id)) ||
    wait_or_cancel(kCommandSettleTime))
  {
    finish_start_sequence();
    return;
  }
  {
    std::lock_guard<std::mutex> lock(assembler_mutex_);
    assembler_.reset();
  }
  if (!send_command(make_kwr57_sample_rate_command(
      config_.cmd_id, config_.sample_rate_hz)) ||
    wait_or_cancel(kCommandSettleTime))
  {
    finish_start_sequence();
    return;
  }

  const double requested_rate = config_.period_ms > 0 ?
    std::min(
    static_cast<double>(config_.sample_rate_hz),
    1000.0 / static_cast<double>(config_.period_ms)) : 0.0;
  const auto expected_samples = static_cast<std::uint64_t>(
    requested_rate *
    std::chrono::duration<double>(kStartConfirmationTime).count());
  const std::uint64_t minimum_samples = std::max<std::uint64_t>(
    1, expected_samples / 2);

  for (int attempt = 0; attempt < 3; ++attempt) {
    const std::uint64_t sample_baseline =
      samples_published_.load(std::memory_order_relaxed);
    if (!send_command(make_kwr57_realtime_command(config_.cmd_id, config_.period_ms))) {
      break;
    }
    const auto deadline = std::chrono::steady_clock::now() + kStartConfirmationTime;
    while (std::chrono::steady_clock::now() < deadline) {
      const std::uint64_t observed_samples =
        samples_published_.load(std::memory_order_relaxed) - sample_baseline;
      if (observed_samples >= minimum_samples) {
        if (tare) {
          tare_pending_.store(true);
        }
        RCLCPP_INFO(
          get_logger(), "stream started at requested rate (%lu samples in probe)",
          static_cast<unsigned long>(observed_samples));
        finish_start_sequence();
        return;
      }
      if (wait_or_cancel(kStartPollTime)) {
        finish_start_sequence();
        return;
      }
    }
    const std::uint64_t observed_samples =
      samples_published_.load(std::memory_order_relaxed) - sample_baseline;
    RCLCPP_WARN(
      get_logger(),
      "stream rate not confirmed (%lu/%lu samples in %ld ms); "
      "resending period=%d ms",
      static_cast<unsigned long>(observed_samples),
      static_cast<unsigned long>(minimum_samples),
      static_cast<long>(kStartConfirmationTime.count()), config_.period_ms);
  }
  if (tare) {
    tare_pending_.store(true);
  }
  RCLCPP_WARN(get_logger(), "stream start not confirmed (no frames)");
  finish_start_sequence();
}

void Kwr57DeviceNode::finish_start_sequence()
{
  std::lock_guard<std::mutex> lock(start_state_mutex_);
  start_running_ = false;
  start_condition_.notify_all();
}

void Kwr57DeviceNode::cancel_start_sequence()
{
  std::lock_guard<std::mutex> api_lock(start_api_mutex_);
  {
    std::lock_guard<std::mutex> state_lock(start_state_mutex_);
    start_cancelled_ = true;
    start_condition_.notify_all();
  }
  if (start_thread_.joinable() && start_thread_.get_id() != std::this_thread::get_id()) {
    start_thread_.join();
  }
}

bool Kwr57DeviceNode::wait_or_cancel(std::chrono::milliseconds duration)
{
  std::unique_lock<std::mutex> lock(start_state_mutex_);
  return start_condition_.wait_for(lock, duration, [this]() {return start_cancelled_;});
}

bool Kwr57DeviceNode::send_command(const CanFrame & frame)
{
  try {
    if (send_frame_(config_.channel_id, frame)) {
      return true;
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_logger(), "CAN command enqueue failed: %s", exception.what());
    return false;
  }
  RCLCPP_ERROR(get_logger(), "CAN command queue is unavailable or full");
  return false;
}

void Kwr57DeviceNode::publish_sample(const WrenchSample & input)
{
  samples_published_.fetch_add(1, std::memory_order_relaxed);
  std::array<float, 6> values = input.values;
  if (config_.use_si) {
    for (float & value : values) {
      value *= kKgfToNewton;
    }
  }
  if (tare_pending_.exchange(false)) {
    for (std::size_t index = 0; index < values.size(); ++index) {
      offsets_[index].store(values[index]);
    }
    RCLCPP_INFO(
      get_logger(), "tare baseline set: Fx=%+.3f Fy=%+.3f Fz=%+.3f",
      values[0], values[1], values[2]);
  }

  const auto steady_now = std::chrono::steady_clock::now();
  if (minimum_publish_period_.count() > 0.0 && last_publish_.time_since_epoch().count() != 0 &&
    steady_now - last_publish_ < minimum_publish_period_)
  {
    return;
  }
  last_publish_ = steady_now;

  auto & message = wrench_message_;
  message.header.stamp = now();
  message.wrench.force.x = values[0] - offsets_[0].load();
  message.wrench.force.y = values[1] - offsets_[1].load();
  message.wrench.force.z = values[2] - offsets_[2].load();
  message.wrench.torque.x = values[3] - offsets_[3].load();
  message.wrench.torque.y = values[4] - offsets_[4].load();
  message.wrench.torque.z = values[5] - offsets_[5].load();
  wrench_publisher_->publish(message);
}

void Kwr57DeviceNode::command_callback(const std_msgs::msg::String::SharedPtr message)
{
  const std::string command = normalized_command(message->data);
  if (command == "start") {
    start_async(false);
  } else if (command == "stop") {
    stop_device();
  } else if (command == "tare" || command == "zero") {
    tare_pending_.store(true);
    RCLCPP_INFO(get_logger(), "tare requested (offset = next sample)");
  } else if (command == "reset_tare" || command == "untare" || command == "clear_tare") {
    clear_tare();
    RCLCPP_INFO(get_logger(), "tare cleared");
  } else {
    RCLCPP_WARN(get_logger(), "ignoring unknown command '%s'", message->data.c_str());
  }
}

void Kwr57DeviceNode::start_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  start_async(false);
  response->success = true;
  response->message = "streaming";
}

void Kwr57DeviceNode::stop_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  stop_device();
  response->success = true;
  response->message = "stopped";
}

void Kwr57DeviceNode::tare_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  tare_pending_.store(true);
  response->success = true;
  response->message = "tare requested (offset captured from next sample)";
}

void Kwr57DeviceNode::reset_tare_service(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  clear_tare();
  response->success = true;
  response->message = "tare cleared";
}

void Kwr57DeviceNode::clear_tare()
{
  tare_pending_.store(false);
  for (auto & offset : offsets_) {
    offset.store(0.0F);
  }
}

}  // namespace canalystii_native_bridge
