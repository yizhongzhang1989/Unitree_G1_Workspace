#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace canalystii_native_bridge
{

struct Kwr57Config
{
  int channel_id{0};
  std::string node_name{"kwr57_ft_sensor"};
  std::uint32_t cmd_id{0x10};
  std::uint32_t data_base_id{0x15};
  std::string topic{"/kwr57_ft_sensor/wrench_raw"};
  std::string frame_id{"kwr57_ft_sensor_link"};
  int period_ms{1};
  int sample_rate_hz{1000};
  double publish_rate{0.0};
  bool use_si{false};
  bool autostart{true};
  bool tare_on_start{false};
};

using RouteKey = std::pair<int, std::uint32_t>;
using RouteTable = std::map<RouteKey, std::vector<std::string>>;

std::vector<Kwr57Config> parse_kwr57_device_specs(
  const std::vector<std::string> & specs,
  const std::vector<int> & channel_ids);

RouteTable parse_rx_routes(
  const std::vector<std::string> & specs,
  const std::vector<int> & channel_ids);

}  // namespace canalystii_native_bridge
