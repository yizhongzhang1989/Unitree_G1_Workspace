#include "canalystii_native_bridge/native_bridge_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace canalystii_native_bridge
{
namespace
{

std::vector<int> integer_channels(const std::vector<std::int64_t> & values)
{
  std::vector<int> channels;
  channels.reserve(values.size());
  std::set<int> seen;
  for (const std::int64_t value : values) {
    if (value < 0 || value > 1) {
      throw std::invalid_argument("CANalyst-II channel_ids must contain only 0 or 1");
    }
    const int channel = static_cast<int>(value);
    if (!seen.insert(channel).second) {
      throw std::invalid_argument("channel_ids must not contain duplicates");
    }
    channels.push_back(channel);
  }
  if (channels.empty()) {
    throw std::invalid_argument("at least one channel_id is required");
  }
  return channels;
}

}  // namespace

NativeBridgeNode::NativeBridgeNode(const rclcpp::NodeOptions & options)
: Node("can_bridge_ros", options)
{
  declare_parameter<std::vector<std::int64_t>>("channel_ids", {0});
  declare_parameter<std::vector<std::string>>("bus_names", {"can0"});
  declare_parameter<std::int64_t>("rx_queue_depth", 128);
  declare_parameter<std::vector<std::string>>("rx_routes", {""});
  declare_parameter<std::vector<std::string>>("kwr57_device_specs", {""});
  declare_parameter<bool>("io_diagnostics", false);
  declare_parameter<std::int64_t>("native_rx_transfers_per_channel", 8);
  declare_parameter<std::int64_t>("native_rx_queue_capacity", 8192);
  declare_parameter<std::int64_t>("native_tx_queue_capacity", 2000);

  channel_ids_ = integer_channels(get_parameter("channel_ids").as_integer_array());
  bus_names_ = get_parameter("bus_names").as_string_array();
  const std::int64_t rx_depth = get_parameter("rx_queue_depth").as_int();
  const bool diagnostics = get_parameter("io_diagnostics").as_bool();
  const std::int64_t rx_transfers = get_parameter("native_rx_transfers_per_channel").as_int();
  const std::int64_t rx_capacity = get_parameter("native_rx_queue_capacity").as_int();
  const std::int64_t tx_capacity = get_parameter("native_tx_queue_capacity").as_int();

  if (bus_names_.size() != channel_ids_.size() ||
    std::set<std::string>(bus_names_.begin(), bus_names_.end()).size() != bus_names_.size())
  {
    throw std::invalid_argument("bus_names must be unique and match channel_ids length");
  }
  if (rx_depth < 1 || rx_transfers < 1 || rx_capacity < rx_transfers || tx_capacity < 1) {
    throw std::invalid_argument("native bridge queue and transfer sizes must be positive");
  }

  const auto routes = parse_rx_routes(
    get_parameter("rx_routes").as_string_array(), channel_ids_);
  const auto device_configs = parse_kwr57_device_specs(
    get_parameter("kwr57_device_specs").as_string_array(), channel_ids_);

  const auto rx_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(rx_depth)))
    .best_effort().durability_volatile();
  const auto tx_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile();

  std::map<std::string, FramePublisher::SharedPtr> publishers_by_topic;
  for (std::size_t index = 0; index < channel_ids_.size(); ++index) {
    const int channel_id = channel_ids_[index];
    const std::string rx_topic = "/" + bus_names_[index] + "/rx";
    const std::string tx_topic = "/" + bus_names_[index] + "/tx";
    default_publishers_[channel_id] = create_publisher<can_msgs::msg::Frame>(rx_topic, rx_qos);
    tx_subscriptions_.push_back(create_subscription<can_msgs::msg::Frame>(
      tx_topic, tx_qos,
      [this, channel_id](const can_msgs::msg::Frame::SharedPtr message) {
        transmit_frame(channel_id, message);
      }));
    RCLCPP_INFO(
      get_logger(), "native channel %d <-> %s (BEST_EFFORT), %s (RELIABLE)",
      channel_id, rx_topic.c_str(), tx_topic.c_str());
  }

  for (const auto & route : routes) {
    auto & publishers = route_publishers_[route.first];
    for (const std::string & topic : route.second) {
      auto existing = publishers_by_topic.find(topic);
      if (existing == publishers_by_topic.end()) {
        existing = publishers_by_topic.emplace(
          topic, create_publisher<can_msgs::msg::Frame>(topic, rx_qos)).first;
      }
      publishers.push_back(existing->second);
      RCLCPP_INFO(
        get_logger(), "native RX route channel %d, CAN ID 0x%X -> %s",
        route.first.first, route.first.second, topic.c_str());
    }
  }

  for (const Kwr57Config & config : device_configs) {
    auto device = std::make_shared<Kwr57DeviceNode>(
      config, [this](int channel_id, const CanFrame & frame) {
        return transport_ != nullptr &&
               transport_->enqueue(channel_id, frame);
      });
    for (std::uint32_t can_id = config.data_base_id; can_id <= config.data_base_id + 2U; ++can_id) {
      const RouteKey key{config.channel_id, can_id};
      if (!kwr57_routes_.emplace(key, device.get()).second) {
        throw std::invalid_argument("duplicate native KWR57 CAN route");
      }
    }
    device_nodes_.push_back(std::move(device));
  }

  transport_ = std::make_unique<CanalystiiTransport>(
    channel_ids_,
    [this](int channel_id, const CanFrame & frame) {receive_frame(channel_id, frame);},
    [this](const std::string & message) {transport_error(message);},
    static_cast<std::size_t>(rx_transfers),
    static_cast<std::size_t>(rx_capacity),
    static_cast<std::size_t>(tx_capacity));
  transport_->open();
  for (const auto & device : device_nodes_) {
    device->activate();
  }

  if (diagnostics) {
    statistics_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&NativeBridgeNode::report_statistics, this));
  }
  RCLCPP_INFO(
    get_logger(), "native CANalyst-II bridge started with %zu symmetric RX transfers per channel",
    static_cast<std::size_t>(rx_transfers));
}

NativeBridgeNode::~NativeBridgeNode()
{
  shutdown_transport();
}

const std::vector<std::shared_ptr<Kwr57DeviceNode>> & NativeBridgeNode::device_nodes() const
{
  return device_nodes_;
}

void NativeBridgeNode::receive_frame(int channel_id, const CanFrame & frame)
{
  if (!frame.extended && !frame.remote) {
    const auto device = kwr57_routes_.find({channel_id, frame.id});
    if (device != kwr57_routes_.end()) {
      device->second->handle_frame(frame);
      return;
    }
  }
  publish_can_frame(channel_id, frame);
}

void NativeBridgeNode::transmit_frame(
  int channel_id, const can_msgs::msg::Frame::SharedPtr message)
{
  if (message->dlc > 8) {
    RCLCPP_ERROR(get_logger(), "dropping CAN TX frame with DLC %u", message->dlc);
    return;
  }
  if (message->is_error ||
    (message->is_extended && message->id > 0x1FFFFFFFU) ||
    (!message->is_extended && message->id > 0x7FFU))
  {
    RCLCPP_ERROR(get_logger(), "dropping invalid CAN TX frame ID 0x%X", message->id);
    return;
  }

  CanFrame frame;
  frame.id = message->id;
  frame.remote = message->is_rtr;
  frame.extended = message->is_extended;
  frame.dlc = message->dlc;
  std::copy(message->data.begin(), message->data.end(), frame.data.begin());
  if (transport_ == nullptr || !transport_->enqueue(channel_id, frame)) {
    RCLCPP_ERROR(get_logger(), "CAN TX queue full or unavailable on channel %d", channel_id);
  }
}

void NativeBridgeNode::publish_can_frame(int channel_id, const CanFrame & frame)
{
  can_msgs::msg::Frame message;
  message.header.stamp = now();
  message.id = frame.id;
  message.is_rtr = frame.remote;
  message.is_extended = frame.extended;
  message.is_error = false;
  message.dlc = frame.dlc;
  message.data = frame.data;

  const auto route = (!frame.extended && !frame.remote) ?
    route_publishers_.find({channel_id, frame.id}) : route_publishers_.end();
  if (route != route_publishers_.end()) {
    for (const auto & publisher : route->second) {
      publisher->publish(message);
    }
    return;
  }
  const auto publisher = default_publishers_.find(channel_id);
  if (publisher != default_publishers_.end()) {
    publisher->second->publish(message);
  }
}

void NativeBridgeNode::transport_error(const std::string & message)
{
  RCLCPP_FATAL(get_logger(), "native CANalyst-II transport failed: %s", message.c_str());
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void NativeBridgeNode::report_statistics()
{
  if (transport_ == nullptr) {
    return;
  }
  const TransportStats stats = transport_->stats();
  for (int channel : channel_ids_) {
    RCLCPP_INFO(
      get_logger(),
      "native ch%d RX packets=%lu frames=%lu TX packets=%lu frames=%lu queue_drops=%lu",
      channel,
      static_cast<unsigned long>(stats.rx_packets[channel]),
      static_cast<unsigned long>(stats.rx_frames[channel]),
      static_cast<unsigned long>(stats.tx_packets[channel]),
      static_cast<unsigned long>(stats.tx_frames[channel]),
      static_cast<unsigned long>(stats.rx_queue_drops[channel]));
  }
}

void NativeBridgeNode::shutdown_transport()
{
  if (shutting_down_.exchange(true)) {
    return;
  }
  statistics_timer_.reset();
  for (const auto & device : device_nodes_) {
    device->stop_device();
  }
  if (transport_ != nullptr) {
    if (!transport_->wait_tx_idle(std::chrono::milliseconds(500))) {
      RCLCPP_WARN(get_logger(), "timed out waiting for native CAN TX queue to drain");
    }
    const TransportStats stats = transport_->stats();
    transport_->close();
    for (int channel : channel_ids_) {
      RCLCPP_INFO(
        get_logger(), "native ch%d final RX frames=%lu TX frames=%lu drops=%lu",
        channel,
        static_cast<unsigned long>(stats.rx_frames[channel]),
        static_cast<unsigned long>(stats.tx_frames[channel]),
        static_cast<unsigned long>(stats.rx_queue_drops[channel]));
    }
    for (const auto & device : device_nodes_) {
      device->report_statistics();
    }
  }
}

}  // namespace canalystii_native_bridge
