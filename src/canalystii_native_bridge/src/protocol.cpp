#include "canalystii_native_bridge/protocol.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace canalystii_native_bridge
{
namespace
{

std::uint32_t read_le32(const std::uint8_t * data)
{
  return static_cast<std::uint32_t>(data[0]) |
         (static_cast<std::uint32_t>(data[1]) << 8U) |
         (static_cast<std::uint32_t>(data[2]) << 16U) |
         (static_cast<std::uint32_t>(data[3]) << 24U);
}

void write_le32(std::uint8_t * data, std::uint32_t value)
{
  data[0] = static_cast<std::uint8_t>(value & 0xFFU);
  data[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
  data[2] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
  data[3] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
}

float read_le_float(const std::uint8_t * data)
{
  const std::uint32_t bits = read_le32(data);
  float value = 0.0F;
  static_assert(sizeof(value) == sizeof(bits), "KWR57 requires IEEE754 float32");
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

}  // namespace

bool parse_message_buffer(
  const UsbPacket & packet,
  std::array<CanFrame, kMessagesPerUsbPacket> & frames,
  std::size_t & frame_count,
  std::string * error)
{
  frame_count = packet[0];
  if (frame_count > kMessagesPerUsbPacket) {
    if (error != nullptr) {
      *error = "CANalyst-II message count exceeds three";
    }
    frame_count = 0;
    return false;
  }

  for (std::size_t index = 0; index < frame_count; ++index) {
    const std::size_t offset = 1 + index * kUsbCanMessageSize;
    CanFrame & frame = frames[index];
    frame.id = read_le32(packet.data() + offset);
    frame.adapter_timestamp = read_le32(packet.data() + offset + 4);
    frame.remote = packet[offset + 10] != 0;
    frame.extended = packet[offset + 11] != 0;
    frame.dlc = packet[offset + 12];
    if (frame.dlc > frame.data.size()) {
      if (error != nullptr) {
        *error = "CANalyst-II frame DLC exceeds eight";
      }
      frame_count = 0;
      return false;
    }
    std::copy_n(packet.data() + offset + 13, frame.data.size(), frame.data.begin());
  }
  return true;
}

UsbPacket pack_message_buffer(const CanFrame * frames, std::size_t frame_count)
{
  if (frames == nullptr || frame_count == 0 || frame_count > kMessagesPerUsbPacket) {
    throw std::invalid_argument("CANalyst-II TX packet requires one to three frames");
  }

  UsbPacket packet{};
  packet[0] = static_cast<std::uint8_t>(frame_count);
  for (std::size_t index = 0; index < frame_count; ++index) {
    const CanFrame & frame = frames[index];
    if (frame.dlc > frame.data.size()) {
      throw std::invalid_argument("CAN frame DLC exceeds eight");
    }
    const std::size_t offset = 1 + index * kUsbCanMessageSize;
    write_le32(packet.data() + offset, frame.id);
    write_le32(packet.data() + offset + 4, frame.adapter_timestamp);
    packet[offset + 8] = 1;
    packet[offset + 9] = 0;
    packet[offset + 10] = frame.remote ? 1 : 0;
    packet[offset + 11] = frame.extended ? 1 : 0;
    packet[offset + 12] = frame.dlc;
    std::copy(frame.data.begin(), frame.data.end(), packet.begin() + offset + 13);
  }
  return packet;
}

UsbPacket make_simple_command(std::uint32_t command)
{
  UsbPacket packet{};
  write_le32(packet.data(), command);
  return packet;
}

UsbPacket make_init_1mbps_command()
{
  UsbPacket packet{};
  write_le32(packet.data(), kCommandInit);
  write_le32(packet.data() + 4, 0x01);
  write_le32(packet.data() + 8, 0xFFFFFFFFU);
  write_le32(packet.data() + 16, 0x01);
  write_le32(packet.data() + 24, 0x00);
  write_le32(packet.data() + 28, 0x14);
  write_le32(packet.data() + 32, 0x00);
  write_le32(packet.data() + 36, 0x01);
  return packet;
}

CanFrame make_kwr57_stop_command(std::uint32_t command_id)
{
  return make_kwr57_realtime_command(command_id, 0);
}

CanFrame make_kwr57_sample_rate_command(std::uint32_t command_id, int rate_hz)
{
  std::uint8_t rate_code = 0;
  switch (rate_hz) {
    case 100: rate_code = 0x01; break;
    case 200: rate_code = 0x02; break;
    case 400: rate_code = 0x03; break;
    case 500: rate_code = 0x04; break;
    case 600: rate_code = 0x05; break;
    case 1000: rate_code = 0x06; break;
    default: throw std::invalid_argument("unsupported KWR57 sample rate");
  }
  if (command_id > 0x7FFU) {
    throw std::invalid_argument("KWR57 command ID must be standard");
  }
  CanFrame frame;
  frame.id = command_id;
  frame.dlc = 2;
  frame.data[0] = 0x60;
  frame.data[1] = rate_code;
  return frame;
}

CanFrame make_kwr57_realtime_command(std::uint32_t command_id, int period_ms)
{
  if (command_id > 0x7FFU) {
    throw std::invalid_argument("KWR57 command ID must be standard");
  }
  if (period_ms < 0 || period_ms > 0xFFFF) {
    throw std::invalid_argument("KWR57 period must be between 0 and 65535 ms");
  }
  CanFrame frame;
  frame.id = command_id;
  frame.dlc = 3;
  frame.data[0] = 0x8A;
  frame.data[1] = static_cast<std::uint8_t>((period_ms >> 8) & 0xFF);
  frame.data[2] = static_cast<std::uint8_t>(period_ms & 0xFF);
  return frame;
}

Kwr57Assembler::Kwr57Assembler(std::uint32_t data_base_id)
: data_base_id_(data_base_id)
{
  if (data_base_id > 0x7FDU) {
    throw std::invalid_argument("KWR57 data base ID must be at most 0x7FD");
  }
}

bool Kwr57Assembler::handles(std::uint32_t can_id) const
{
  return can_id >= data_base_id_ && can_id <= data_base_id_ + 2U;
}

AssembleResult Kwr57Assembler::push(const CanFrame & frame, WrenchSample & sample)
{
  if (!handles(frame.id)) {
    return AssembleResult::kIgnored;
  }
  if (frame.extended || frame.remote || frame.dlc != 8) {
    ++malformed_frames_;
    reset(true);
    return AssembleResult::kMalformed;
  }

  const std::size_t position = frame.id - data_base_id_;
  if (position != expected_index_) {
    if (position != 0) {
      ++dropped_sequences_;
      expected_index_ = 0;
      return AssembleResult::kIncomplete;
    }
    if (expected_index_ != 0) {
      ++dropped_sequences_;
    }
  }

  const std::size_t value_offset = position * 2;
  values_[value_offset] = read_le_float(frame.data.data());
  values_[value_offset + 1] = read_le_float(frame.data.data() + 4);
  if (position == 2) {
    expected_index_ = 0;
    sample.values = values_;
    return AssembleResult::kComplete;
  }
  expected_index_ = position + 1;
  return AssembleResult::kIncomplete;
}

void Kwr57Assembler::reset(bool count_drop)
{
  if (count_drop && expected_index_ != 0) {
    ++dropped_sequences_;
  }
  expected_index_ = 0;
}

}  // namespace canalystii_native_bridge
