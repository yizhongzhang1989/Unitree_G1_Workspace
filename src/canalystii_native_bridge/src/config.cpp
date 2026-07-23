#include "canalystii_native_bridge/config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

namespace canalystii_native_bridge
{
namespace
{

bool contains_channel(const std::vector<int> & channel_ids, int channel_id)
{
  return std::find(channel_ids.begin(), channel_ids.end(), channel_id) != channel_ids.end();
}

std::uint32_t parse_unsigned(const std::string & text, const std::string & field)
{
  std::size_t parsed = 0;
  unsigned long value = 0;
  try {
    value = std::stoul(text, &parsed, 0);
  } catch (const std::exception &) {
    throw std::invalid_argument(field + " must be an integer: " + text);
  }
  if (parsed != text.size() || value > 0xFFFFFFFFUL) {
    throw std::invalid_argument(field + " must be an integer: " + text);
  }
  return static_cast<std::uint32_t>(value);
}

template<typename T>
void assign_if_present(const YAML::Node & node, const char * key, T & destination)
{
  if (node[key]) {
    destination = node[key].as<T>();
  }
}

void validate_kwr57(const Kwr57Config & config, const std::vector<int> & channel_ids)
{
  if (!contains_channel(channel_ids, config.channel_id)) {
    throw std::invalid_argument("KWR57 channel_id is not configured by the bridge");
  }
  if (config.node_name.empty() || config.topic.empty() || config.frame_id.empty()) {
    throw std::invalid_argument("KWR57 node_name, topic, and frame_id must be non-empty");
  }
  if (config.topic.front() != '/') {
    throw std::invalid_argument("KWR57 topic must be absolute");
  }
  if (config.cmd_id > 0x7FFU) {
    throw std::invalid_argument("KWR57 cmd_id must be a standard CAN ID");
  }
  if (config.data_base_id > 0x7FDU) {
    throw std::invalid_argument("KWR57 data_base_id must be at most 0x7FD");
  }
  if (config.cmd_id >= config.data_base_id && config.cmd_id <= config.data_base_id + 2U) {
    throw std::invalid_argument("KWR57 cmd_id conflicts with its data IDs");
  }
  if (config.period_ms < 0 || config.period_ms > 0xFFFF) {
    throw std::invalid_argument("KWR57 period_ms must be between 0 and 65535");
  }
  static const std::set<int> sample_rates{100, 200, 400, 500, 600, 1000};
  if (sample_rates.count(config.sample_rate_hz) == 0) {
    throw std::invalid_argument("unsupported KWR57 sample_rate_hz");
  }
  if (config.publish_rate < 0.0) {
    throw std::invalid_argument("KWR57 publish_rate must not be negative");
  }
}

}  // namespace

std::vector<Kwr57Config> parse_kwr57_device_specs(
  const std::vector<std::string> & specs,
  const std::vector<int> & channel_ids)
{
  std::vector<Kwr57Config> result;
  std::set<std::string> node_names;
  std::set<std::string> topics;
  std::map<RouteKey, std::string> owners;

  for (const std::string & raw_spec : specs) {
    if (raw_spec.empty()) {
      continue;
    }
    YAML::Node root;
    try {
      root = YAML::Load(raw_spec);
    } catch (const YAML::Exception & exception) {
      throw std::invalid_argument(std::string("invalid KWR57 device JSON: ") + exception.what());
    }
    if (!root.IsMap()) {
      throw std::invalid_argument("KWR57 device spec must be an object");
    }

    static const std::set<std::string> allowed_fields{
      "channel_id", "node_name", "cmd_id", "data_base_id", "topic", "frame_id",
      "period_ms", "sample_rate_hz", "publish_rate", "use_si", "autostart",
      "tare_on_start"};
    for (const auto & entry : root) {
      const std::string field = entry.first.as<std::string>();
      if (allowed_fields.count(field) == 0) {
        throw std::invalid_argument("unknown KWR57 device field: " + field);
      }
    }

    Kwr57Config config;
    assign_if_present(root, "channel_id", config.channel_id);
    assign_if_present(root, "node_name", config.node_name);
    assign_if_present(root, "cmd_id", config.cmd_id);
    assign_if_present(root, "data_base_id", config.data_base_id);
    assign_if_present(root, "topic", config.topic);
    assign_if_present(root, "frame_id", config.frame_id);
    assign_if_present(root, "period_ms", config.period_ms);
    assign_if_present(root, "sample_rate_hz", config.sample_rate_hz);
    assign_if_present(root, "publish_rate", config.publish_rate);
    assign_if_present(root, "use_si", config.use_si);
    assign_if_present(root, "autostart", config.autostart);
    assign_if_present(root, "tare_on_start", config.tare_on_start);
    validate_kwr57(config, channel_ids);

    if (!node_names.insert(config.node_name).second) {
      throw std::invalid_argument("duplicate KWR57 node_name: " + config.node_name);
    }
    if (!topics.insert(config.topic).second) {
      throw std::invalid_argument("duplicate KWR57 wrench topic: " + config.topic);
    }
    const RouteKey command_key{config.channel_id, config.cmd_id};
    if (owners.count(command_key) != 0) {
      throw std::invalid_argument("KWR57 command or data CAN ID collision");
    }
    owners.emplace(command_key, config.node_name);
    for (std::uint32_t can_id = config.data_base_id; can_id <= config.data_base_id + 2U; ++can_id) {
      const RouteKey key{config.channel_id, can_id};
      if (owners.count(key) != 0) {
        throw std::invalid_argument("KWR57 command or data CAN ID collision");
      }
      owners.emplace(key, config.node_name);
    }
    result.push_back(std::move(config));
  }
  return result;
}

RouteTable parse_rx_routes(
  const std::vector<std::string> & specs,
  const std::vector<int> & channel_ids)
{
  RouteTable result;
  for (const std::string & raw_spec : specs) {
    if (raw_spec.empty()) {
      continue;
    }
    const std::size_t first_separator = raw_spec.find(':');
    const std::size_t second_separator = raw_spec.find(':', first_separator + 1);
    if (first_separator == std::string::npos || second_separator == std::string::npos) {
      throw std::invalid_argument("rx_routes entries must use channel:can_id:topic");
    }
    const std::string channel_text = raw_spec.substr(0, first_separator);
    const std::string can_id_text = raw_spec.substr(
      first_separator + 1, second_separator - first_separator - 1);
    const std::string topic = raw_spec.substr(second_separator + 1);
    const std::uint32_t channel_value = parse_unsigned(channel_text, "route channel");
    const std::uint32_t can_id = parse_unsigned(can_id_text, "route CAN ID");
    if (channel_value > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      !contains_channel(channel_ids, static_cast<int>(channel_value)))
    {
      throw std::invalid_argument("rx_routes channel is not configured by the bridge");
    }
    if (can_id > 0x7FFU) {
      throw std::invalid_argument("rx_routes CAN ID must be standard");
    }
    if (topic.empty() || topic.front() != '/') {
      throw std::invalid_argument("rx_routes topic must be absolute");
    }
    auto & topics = result[{static_cast<int>(channel_value), can_id}];
    if (std::find(topics.begin(), topics.end(), topic) != topics.end()) {
      throw std::invalid_argument("duplicate rx_routes entry");
    }
    topics.push_back(topic);
  }
  return result;
}

}  // namespace canalystii_native_bridge
