#pragma once

#include "canalystii_native_bridge/protocol.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace canalystii_native_bridge
{

bool is_control_transaction(const CanFrame & frame);

struct TxBatch
{
  int channel{-1};
  std::array<CanFrame, kMessagesPerUsbPacket> frames{};
  std::size_t count{0};
};

class FairTxQueue
{
public:
  FairTxQueue(std::vector<int> channels, std::size_t capacity);
  ~FairTxQueue();

  bool push(int channel, const CanFrame & frame);
  bool pop(TxBatch & batch);
  bool empty() const;
  std::size_t size() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct TransportStats
{
  std::array<std::uint64_t, 2> rx_packets{};
  std::array<std::uint64_t, 2> rx_frames{};
  std::array<std::uint64_t, 2> tx_packets{};
  std::array<std::uint64_t, 2> tx_frames{};
  std::array<std::uint64_t, 2> rx_queue_drops{};
};

class CanalystiiTransport
{
public:
  using FrameCallback = std::function<void(int, const CanFrame &)>;
  using ErrorCallback = std::function<void(const std::string &)>;

  CanalystiiTransport(
    std::vector<int> channels,
    FrameCallback frame_callback,
    ErrorCallback error_callback,
    std::size_t transfers_per_channel = 8,
    std::size_t rx_queue_capacity = 8192,
    std::size_t tx_queue_capacity = 2000);
  ~CanalystiiTransport();

  CanalystiiTransport(const CanalystiiTransport &) = delete;
  CanalystiiTransport & operator=(const CanalystiiTransport &) = delete;

  void open();
  bool enqueue(int channel, const CanFrame & frame);
  bool wait_tx_idle(std::chrono::milliseconds timeout);
  void close();

  TransportStats stats() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace canalystii_native_bridge
