#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace canalystii_native_bridge
{

constexpr std::size_t kUsbPacketSize = 64;
constexpr std::size_t kUsbCanMessageSize = 21;
constexpr std::size_t kMessagesPerUsbPacket = 3;
constexpr std::uint16_t kUsbVendorId = 0x04D8;
constexpr std::uint16_t kUsbProductId = 0x0053;
constexpr std::uint32_t kCommandInit = 0x01;
constexpr std::uint32_t kCommandStart = 0x02;
constexpr std::uint32_t kCommandStop = 0x03;

using UsbPacket = std::array<std::uint8_t, kUsbPacketSize>;

struct CanFrame
{
  std::uint32_t id{0};
  std::uint32_t adapter_timestamp{0};
  bool remote{false};
  bool extended{false};
  std::uint8_t dlc{0};
  std::array<std::uint8_t, 8> data{};
};

bool parse_message_buffer(
  const UsbPacket & packet,
  std::array<CanFrame, kMessagesPerUsbPacket> & frames,
  std::size_t & frame_count,
  std::string * error = nullptr);

UsbPacket pack_message_buffer(const CanFrame * frames, std::size_t frame_count);
UsbPacket make_simple_command(std::uint32_t command);
UsbPacket make_init_1mbps_command();
CanFrame make_kwr57_stop_command(std::uint32_t command_id);
CanFrame make_kwr57_sample_rate_command(std::uint32_t command_id, int rate_hz);
CanFrame make_kwr57_realtime_command(std::uint32_t command_id, int period_ms);

struct WrenchSample
{
  std::array<float, 6> values{};
};

enum class AssembleResult
{
  kIgnored,
  kIncomplete,
  kComplete,
  kMalformed,
};

class Kwr57Assembler
{
public:
  explicit Kwr57Assembler(std::uint32_t data_base_id);

  AssembleResult push(const CanFrame & frame, WrenchSample & sample);
  bool handles(std::uint32_t can_id) const;
  void reset(bool count_drop = false);

  std::uint64_t malformed_frames() const {return malformed_frames_;}
  std::uint64_t dropped_sequences() const {return dropped_sequences_;}

private:
  std::uint32_t data_base_id_;
  std::array<float, 6> values_{};
  std::size_t expected_index_{0};
  std::uint64_t malformed_frames_{0};
  std::uint64_t dropped_sequences_{0};
};

}  // namespace canalystii_native_bridge
