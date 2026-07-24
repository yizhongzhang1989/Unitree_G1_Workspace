#include "canalystii_native_bridge/native_bridge_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<canalystii_native_bridge::NativeBridgeNode> bridge;
  try {
    bridge = std::make_shared<canalystii_native_bridge::NativeBridgeNode>();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("can_bridge_ros"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(bridge);
  for (const auto & device : bridge->device_nodes()) {
    executor.add_node(device);
  }
  executor.spin();

  bridge->shutdown_transport();
  for (const auto & device : bridge->device_nodes()) {
    executor.remove_node(device);
  }
  executor.remove_node(bridge);
  bridge.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
