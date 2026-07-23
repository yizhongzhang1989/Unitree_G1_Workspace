#include "canalystii_native_bridge/protocol.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <stdexcept>

namespace cb = canalystii_native_bridge;

namespace
{

cb::CanFrame frame(std::uint32_t id, float first, float second)
{
  cb::CanFrame result;
  result.id = id;
  result.dlc = 8;
  std::uint32_t first_bits = 0;
  std::uint32_t second_bits = 0;
  std::memcpy(&first_bits, &first, sizeof(first));
  std::memcpy(&second_bits, &second, sizeof(second));
  for (std::size_t byte = 0; byte < 4; ++byte) {
    result.data[byte] = static_cast<std::uint8_t>(first_bits >> (byte * 8));
    result.data[byte + 4] = static_cast<std::uint8_t>(second_bits >> (byte * 8));
  }
  return result;
}

}  // namespace

TEST(CanalystiiProtocol, PacksAndParsesExactMessageLayout)
{
  std::array<cb::CanFrame, 3> source{
    frame(0x15, 1.0F, 2.0F),
    frame(0x16, 3.0F, 4.0F),
    frame(0x17, 5.0F, 6.0F),
  };
  source[0].adapter_timestamp = 0x12345678;

  const cb::UsbPacket packet = cb::pack_message_buffer(source.data(), source.size());
  EXPECT_EQ(packet.size(), 64U);
  EXPECT_EQ(packet[0], 3U);
  EXPECT_EQ(packet[1], 0x15U);
  EXPECT_EQ(packet[5], 0x78U);
  EXPECT_EQ(packet[9], 1U);
  EXPECT_EQ(packet[13], 8U);
  EXPECT_EQ(packet[22], 0x16U);
  EXPECT_EQ(packet[43], 0x17U);

  std::array<cb::CanFrame, 3> parsed{};
  std::size_t count = 0;
  std::string error;
  ASSERT_TRUE(cb::parse_message_buffer(packet, parsed, count, &error)) << error;
  ASSERT_EQ(count, 3U);
  EXPECT_EQ(parsed[0].id, 0x15U);
  EXPECT_EQ(parsed[0].adapter_timestamp, 0x12345678U);
  EXPECT_EQ(parsed[2].data, source[2].data);
}

TEST(CanalystiiProtocol, RejectsMalformedMessageBuffers)
{
  cb::UsbPacket packet{};
  packet[0] = 4;
  std::array<cb::CanFrame, 3> frames{};
  std::size_t count = 99;
  std::string error;
  EXPECT_FALSE(cb::parse_message_buffer(packet, frames, count, &error));
  EXPECT_EQ(count, 0U);

  packet[0] = 1;
  packet[13] = 9;
  EXPECT_FALSE(cb::parse_message_buffer(packet, frames, count, &error));
  EXPECT_THROW(cb::pack_message_buffer(nullptr, 0), std::invalid_argument);
}

TEST(CanalystiiProtocol, BuildsFirmwareCommands)
{
  const cb::UsbPacket init = cb::make_init_1mbps_command();
  EXPECT_EQ(init[0], cb::kCommandInit);
  EXPECT_EQ(init[4], 0x01U);
  EXPECT_EQ(init[8], 0xFFU);
  EXPECT_EQ(init[24], 0x00U);
  EXPECT_EQ(init[28], 0x14U);
  EXPECT_EQ(init[36], 0x01U);

  const cb::UsbPacket stop = cb::make_simple_command(cb::kCommandStop);
  EXPECT_EQ(stop[0], cb::kCommandStop);
  for (std::size_t index = 4; index < stop.size(); ++index) {
    EXPECT_EQ(stop[index], 0U);
  }
}

TEST(Kwr57Protocol, BuildsStreamingCommands)
{
  const cb::CanFrame stop = cb::make_kwr57_stop_command(0x10);
  EXPECT_EQ(stop.id, 0x10U);
  EXPECT_EQ(stop.dlc, 3U);
  EXPECT_EQ(stop.data[0], 0x8AU);
  EXPECT_EQ(stop.data[1], 0U);
  EXPECT_EQ(stop.data[2], 0U);

  const cb::CanFrame rate = cb::make_kwr57_sample_rate_command(0x10, 1000);
  EXPECT_EQ(rate.dlc, 2U);
  EXPECT_EQ(rate.data[0], 0x60U);
  EXPECT_EQ(rate.data[1], 0x06U);

  const cb::CanFrame realtime = cb::make_kwr57_realtime_command(0x10, 1);
  EXPECT_EQ(realtime.dlc, 3U);
  EXPECT_EQ(realtime.data[0], 0x8AU);
  EXPECT_EQ(realtime.data[1], 0U);
  EXPECT_EQ(realtime.data[2], 1U);
  EXPECT_THROW(cb::make_kwr57_sample_rate_command(0x10, 999), std::invalid_argument);
  EXPECT_THROW(cb::make_kwr57_realtime_command(0x10, -1), std::invalid_argument);
}

TEST(Kwr57Assembler, CompletesOnlyStrictThreeFrameSequences)
{
  cb::Kwr57Assembler assembler(0x15);
  cb::WrenchSample sample;
  EXPECT_EQ(assembler.push(frame(0x15, 1.0F, 2.0F), sample), cb::AssembleResult::kIncomplete);
  EXPECT_EQ(assembler.push(frame(0x16, 3.0F, 4.0F), sample), cb::AssembleResult::kIncomplete);
  EXPECT_EQ(assembler.push(frame(0x17, 5.0F, 6.0F), sample), cb::AssembleResult::kComplete);
  EXPECT_EQ(sample.values, (std::array<float, 6>{1, 2, 3, 4, 5, 6}));
}

TEST(Kwr57Assembler, ResynchronizesAtARepeatedStartFrame)
{
  cb::Kwr57Assembler assembler(0x15);
  cb::WrenchSample sample;
  assembler.push(frame(0x15, 10.0F, 20.0F), sample);
  assembler.push(frame(0x15, 1.0F, 2.0F), sample);
  assembler.push(frame(0x16, 3.0F, 4.0F), sample);
  EXPECT_EQ(assembler.push(frame(0x17, 5.0F, 6.0F), sample), cb::AssembleResult::kComplete);
  EXPECT_EQ(sample.values, (std::array<float, 6>{1, 2, 3, 4, 5, 6}));
  EXPECT_EQ(assembler.dropped_sequences(), 1U);
}

TEST(Kwr57Assembler, DropsOutOfOrderAndMalformedSequences)
{
  cb::Kwr57Assembler assembler(0x15);
  cb::WrenchSample sample;
  assembler.push(frame(0x15, 1.0F, 2.0F), sample);
  EXPECT_EQ(assembler.push(frame(0x17, 5.0F, 6.0F), sample), cb::AssembleResult::kIncomplete);
  EXPECT_EQ(assembler.dropped_sequences(), 1U);

  assembler.push(frame(0x15, 1.0F, 2.0F), sample);
  cb::CanFrame malformed = frame(0x16, 3.0F, 4.0F);
  malformed.dlc = 7;
  EXPECT_EQ(assembler.push(malformed, sample), cb::AssembleResult::kMalformed);
  EXPECT_EQ(assembler.malformed_frames(), 1U);
  EXPECT_EQ(assembler.dropped_sequences(), 2U);
}

TEST(Kwr57Assembler, KeepsLeftAndRightStateIndependent)
{
  cb::Kwr57Assembler left(0x15);
  cb::Kwr57Assembler right(0x15);
  cb::WrenchSample left_sample;
  cb::WrenchSample right_sample;

  left.push(frame(0x15, 1.0F, 2.0F), left_sample);
  right.push(frame(0x15, 11.0F, 12.0F), right_sample);
  right.push(frame(0x16, 13.0F, 14.0F), right_sample);
  left.push(frame(0x16, 3.0F, 4.0F), left_sample);
  EXPECT_EQ(right.push(frame(0x17, 15.0F, 16.0F), right_sample), cb::AssembleResult::kComplete);
  EXPECT_EQ(left.push(frame(0x17, 5.0F, 6.0F), left_sample), cb::AssembleResult::kComplete);

  EXPECT_EQ(left_sample.values, (std::array<float, 6>{1, 2, 3, 4, 5, 6}));
  EXPECT_EQ(right_sample.values, (std::array<float, 6>{11, 12, 13, 14, 15, 16}));
  EXPECT_EQ(left.dropped_sequences(), right.dropped_sequences());
}
