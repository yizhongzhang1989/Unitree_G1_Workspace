#include "canalystii_native_bridge/config.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace cb = canalystii_native_bridge;

namespace
{

std::string spec(
  int channel,
  const std::string & name,
  const std::string & topic,
  int cmd_id = 16,
  int data_base_id = 21)
{
  return std::string{"{\"channel_id\":"} + std::to_string(channel) +
         ",\"node_name\":\"" + name + "\",\"cmd_id\":" + std::to_string(cmd_id) +
         ",\"data_base_id\":" + std::to_string(data_base_id) +
         ",\"topic\":\"" + topic +
         "\",\"frame_id\":\"sensor_link\",\"period_ms\":1,"
         "\"sample_rate_hz\":1000,\"publish_rate\":0.0,\"use_si\":true,"
         "\"autostart\":true,\"tare_on_start\":false}";
}

}  // namespace

TEST(NativeConfig, PreservesCompleteKwr57Configuration)
{
  const auto devices = cb::parse_kwr57_device_specs(
    {spec(0, "ft_left", "/left/wrench")}, {0, 1});
  ASSERT_EQ(devices.size(), 1U);
  const auto & device = devices.front();
  EXPECT_EQ(device.channel_id, 0);
  EXPECT_EQ(device.node_name, "ft_left");
  EXPECT_EQ(device.cmd_id, 0x10U);
  EXPECT_EQ(device.data_base_id, 0x15U);
  EXPECT_EQ(device.topic, "/left/wrench");
  EXPECT_EQ(device.frame_id, "sensor_link");
  EXPECT_EQ(device.period_ms, 1);
  EXPECT_EQ(device.sample_rate_hz, 1000);
  EXPECT_DOUBLE_EQ(device.publish_rate, 0.0);
  EXPECT_TRUE(device.use_si);
  EXPECT_TRUE(device.autostart);
  EXPECT_FALSE(device.tare_on_start);
}

TEST(NativeConfig, AllowsIdenticalCanIdsOnDifferentChannels)
{
  const auto devices = cb::parse_kwr57_device_specs(
    {spec(0, "left", "/left/wrench"), spec(1, "right", "/right/wrench")},
    {0, 1});
  ASSERT_EQ(devices.size(), 2U);
  EXPECT_EQ(devices[0].cmd_id, devices[1].cmd_id);
  EXPECT_EQ(devices[0].data_base_id, devices[1].data_base_id);
}

TEST(NativeConfig, RejectsCollisionsOnOneChannel)
{
  EXPECT_THROW(
    cb::parse_kwr57_device_specs(
      {spec(0, "left", "/left/wrench"), spec(0, "right", "/right/wrench")},
      {0, 1}),
    std::invalid_argument);
  EXPECT_THROW(
    cb::parse_kwr57_device_specs({spec(2, "left", "/left/wrench")}, {0, 1}),
    std::invalid_argument);
}

TEST(NativeConfig, RejectsUnknownFieldsAndNonObjects)
{
  EXPECT_THROW(
    cb::parse_kwr57_device_specs({"[]"}, {0}),
    std::invalid_argument);
  EXPECT_THROW(
    cb::parse_kwr57_device_specs({"{\"surprise\":1}"}, {0}),
    std::invalid_argument);
}

TEST(NativeConfig, ParsesRouteFanoutWithoutDuplication)
{
  const auto routes = cb::parse_rx_routes(
    {"0:0x101:/left/rx", "0:0x101:/observer/rx", "1:257:/right/rx"},
    {0, 1});
  ASSERT_EQ(routes.size(), 2U);
  ASSERT_EQ(routes.at({0, 0x101}).size(), 2U);
  EXPECT_EQ(routes.at({0, 0x101})[0], "/left/rx");
  EXPECT_EQ(routes.at({1, 0x101})[0], "/right/rx");
  EXPECT_THROW(
    cb::parse_rx_routes({"0:0x101:/left/rx", "0:0x101:/left/rx"}, {0}),
    std::invalid_argument);
}
