#pragma once

#include "canalystii_native_bridge/config.hpp"
#include "canalystii_native_bridge/kwr57_device_node.hpp"
#include "canalystii_native_bridge/transport.hpp"

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace canalystii_native_bridge
{

class NativeBridgeNode : public rclcpp::Node
{
public:
  explicit NativeBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~NativeBridgeNode() override;

  const std::vector<std::shared_ptr<Kwr57DeviceNode>> & device_nodes() const;
  void shutdown_transport();

private:
  using FramePublisher = rclcpp::Publisher<can_msgs::msg::Frame>;

  void receive_frame(int channel_id, const CanFrame & frame);
  void transmit_frame(int channel_id, const can_msgs::msg::Frame::SharedPtr message);
  void publish_can_frame(int channel_id, const CanFrame & frame);
  void transport_error(const std::string & message);
  void report_statistics();

  std::vector<int> channel_ids_;
  std::vector<std::string> bus_names_;
  std::map<int, FramePublisher::SharedPtr> default_publishers_;
  std::map<RouteKey, std::vector<FramePublisher::SharedPtr>> route_publishers_;
  std::map<RouteKey, Kwr57DeviceNode *> kwr57_routes_;
  std::vector<rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr> tx_subscriptions_;
  std::vector<std::shared_ptr<Kwr57DeviceNode>> device_nodes_;
  std::unique_ptr<CanalystiiTransport> transport_;
  rclcpp::TimerBase::SharedPtr statistics_timer_;
  std::atomic<bool> shutting_down_{false};
};

}  // namespace canalystii_native_bridge
