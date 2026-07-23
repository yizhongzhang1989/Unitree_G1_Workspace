#include "canalystii_native_bridge/transport.hpp"

#include <libusb-1.0/libusb.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace canalystii_native_bridge
{
namespace
{

constexpr std::array<unsigned char, 2> kCommandEndpoints{0x02, 0x04};
constexpr std::array<unsigned char, 2> kMessageOutEndpoints{0x01, 0x03};
constexpr std::array<unsigned char, 2> kMessageInEndpoints{0x81, 0x83};
constexpr int kUsbInterface = 0;
constexpr unsigned int kCommandTimeoutMs = 250;

void check_libusb(int result, const std::string & operation)
{
  if (result != LIBUSB_SUCCESS) {
    throw std::runtime_error(operation + ": " + libusb_error_name(result));
  }
}

bool valid_channel(int channel)
{
  return channel == 0 || channel == 1;
}

}  // namespace

bool is_control_transaction(const CanFrame & frame)
{
  if (frame.id == 0x7FFU) {
    return true;
  }
  if (frame.dlc != 8) {
    return false;
  }
  for (std::size_t index = 0; index < 7; ++index) {
    if (frame.data[index] != 0xFFU) {
      return false;
    }
  }
  return frame.data[7] == 0xFCU || frame.data[7] == 0xFDU || frame.data[7] == 0xFEU;
}

struct FairTxQueue::Impl
{
  struct Entry
  {
    bool priority;
    CanFrame frame;
  };

  Impl(std::vector<int> requested_channels, std::size_t requested_capacity)
  : channels(std::move(requested_channels)), capacity(requested_capacity)
  {
    if (channels.empty() || capacity == 0) {
      throw std::invalid_argument("TX channels and capacity must be non-empty");
    }
    std::array<bool, 2> seen{};
    for (const int channel : channels) {
      if (!valid_channel(channel) || seen[channel]) {
        throw std::invalid_argument("TX channels must be unique CANalyst-II channels 0 or 1");
      }
      seen[channel] = true;
    }
  }

  bool select(bool priority_only, std::size_t & selected_index) const
  {
    for (std::size_t offset = 0; offset < channels.size(); ++offset) {
      const std::size_t index = (next_channel + offset) % channels.size();
      const auto & queue = queues[channels[index]];
      if (!queue.empty() && (!priority_only || queue.front().priority)) {
        selected_index = index;
        return true;
      }
    }
    return false;
  }

  std::vector<int> channels;
  std::size_t capacity;
  std::array<std::deque<Entry>, 2> queues;
  std::size_t total_size{0};
  std::size_t next_channel{0};
  mutable std::mutex mutex;
};

FairTxQueue::FairTxQueue(std::vector<int> channels, std::size_t capacity)
: impl_(std::make_unique<Impl>(std::move(channels), capacity))
{
}

FairTxQueue::~FairTxQueue() = default;

bool FairTxQueue::push(int channel, const CanFrame & frame)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!valid_channel(channel) ||
    std::find(impl_->channels.begin(), impl_->channels.end(), channel) == impl_->channels.end())
  {
    throw std::invalid_argument("unknown CANalyst-II TX channel");
  }
  if (impl_->total_size == impl_->capacity) {
    return false;
  }
  impl_->queues[channel].push_back({is_control_transaction(frame), frame});
  ++impl_->total_size;
  return true;
}

bool FairTxQueue::pop(TxBatch & batch)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  std::size_t selected_index = 0;
  if (!impl_->select(true, selected_index) && !impl_->select(false, selected_index)) {
    batch.count = 0;
    return false;
  }

  batch.channel = impl_->channels[selected_index];
  auto & queue = impl_->queues[batch.channel];
  const bool priority = queue.front().priority;
  batch.count = 0;
  while (batch.count < batch.frames.size() && !queue.empty() && queue.front().priority == priority) {
    batch.frames[batch.count++] = queue.front().frame;
    queue.pop_front();
    --impl_->total_size;
  }
  impl_->next_channel = (selected_index + 1) % impl_->channels.size();
  return true;
}

bool FairTxQueue::empty() const
{
  return size() == 0;
}

std::size_t FairTxQueue::size() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->total_size;
}

struct CanalystiiTransport::Impl
{
  struct RxQueue
  {
    explicit RxQueue(std::size_t capacity) : packets(capacity) {}

    bool push(const UsbPacket & packet)
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (size == packets.size()) {
        return false;
      }
      packets[(head + size) % packets.size()] = packet;
      ++size;
      return true;
    }

    bool pop(UsbPacket & packet)
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (size == 0) {
        return false;
      }
      packet = packets[head];
      head = (head + 1) % packets.size();
      --size;
      return true;
    }

    bool empty() const
    {
      std::lock_guard<std::mutex> lock(mutex);
      return size == 0;
    }

    std::vector<UsbPacket> packets;
    std::size_t head{0};
    std::size_t size{0};
    mutable std::mutex mutex;
  };

  struct RxSlot
  {
    Impl * owner{nullptr};
    int channel{0};
    libusb_transfer * transfer{nullptr};
    UsbPacket buffer{};
    bool submitted{false};
  };

  Impl(
    std::vector<int> requested_channels,
    FrameCallback requested_frame_callback,
    ErrorCallback requested_error_callback,
    std::size_t requested_transfers,
    std::size_t rx_capacity,
    std::size_t tx_capacity)
  : channels(std::move(requested_channels)),
    frame_callback(std::move(requested_frame_callback)),
    error_callback(std::move(requested_error_callback)),
    transfers_per_channel(requested_transfers),
    rx_queues{std::make_unique<RxQueue>(rx_capacity), std::make_unique<RxQueue>(rx_capacity)},
    tx_queue(channels, tx_capacity)
  {
    if (!frame_callback) {
      throw std::invalid_argument("CANalyst-II transport requires a frame callback");
    }
    if (transfers_per_channel == 0 || transfers_per_channel > 64) {
      throw std::invalid_argument("RX transfers per channel must be between 1 and 64");
    }
  }

  ~Impl()
  {
    close();
  }

  void set_error(const std::string & message)
  {
    bool first = false;
    {
      std::lock_guard<std::mutex> lock(error_mutex);
      if (error_message.empty()) {
        error_message = message;
        first = true;
      }
    }
    if (first) {
      stopping.store(true);
      rx_condition.notify_all();
      tx_condition.notify_all();
      if (error_callback) {
        error_callback(message);
      }
    }
  }

  void send_command(int channel, const UsbPacket & packet)
  {
    int actual_length = 0;
    const int result = libusb_bulk_transfer(
      handle,
      kCommandEndpoints[channel],
      const_cast<unsigned char *>(packet.data()),
      static_cast<int>(packet.size()),
      &actual_length,
      kCommandTimeoutMs);
    check_libusb(result, "CANalyst-II command transfer");
    if (actual_length != static_cast<int>(packet.size())) {
      throw std::runtime_error("CANalyst-II command transfer was short");
    }
  }

  void initialize_device()
  {
    check_libusb(libusb_init(&context), "could not initialize libusb");
    handle = libusb_open_device_with_vid_pid(context, kUsbVendorId, kUsbProductId);
    if (handle == nullptr) {
      throw std::runtime_error("CANalyst-II 04d8:0053 was not found or could not be opened");
    }

    libusb_set_auto_detach_kernel_driver(handle, 1);
    int configuration = 0;
    check_libusb(libusb_get_configuration(handle, &configuration), "could not read USB configuration");
    if (configuration != 1) {
      check_libusb(libusb_set_configuration(handle, 1), "could not select USB configuration 1");
    }
    check_libusb(libusb_claim_interface(handle, kUsbInterface), "could not claim CANalyst-II interface");
    interface_claimed = true;

    for (const int channel : channels) {
      send_command(channel, make_init_1mbps_command());
    }
  }

  void start_channels()
  {
    for (const int channel : channels) {
      send_command(channel, make_simple_command(kCommandStart));
    }
  }

  void allocate_transfers()
  {
    for (const int channel : channels) {
      for (std::size_t index = 0; index < transfers_per_channel; ++index) {
        auto slot = std::make_unique<RxSlot>();
        slot->owner = this;
        slot->channel = channel;
        slot->transfer = libusb_alloc_transfer(0);
        if (slot->transfer == nullptr) {
          throw std::bad_alloc();
        }
        libusb_fill_bulk_transfer(
          slot->transfer,
          handle,
          kMessageInEndpoints[channel],
          slot->buffer.data(),
          static_cast<int>(slot->buffer.size()),
          &Impl::rx_completed,
          slot.get(),
          0);
        rx_slots.push_back(std::move(slot));
      }
    }
  }

  void submit_initial_rx()
  {
    for (auto & slot : rx_slots) {
      const int result = libusb_submit_transfer(slot->transfer);
      if (result != LIBUSB_SUCCESS) {
        throw std::runtime_error(
                std::string("could not submit CANalyst-II RX transfer: ") + libusb_error_name(result));
      }
      slot->submitted = true;
      ++active_transfers;
    }
  }

  static void LIBUSB_CALL rx_completed(libusb_transfer * transfer)
  {
    auto * slot = static_cast<RxSlot *>(transfer->user_data);
    Impl * self = slot->owner;
    if (slot->submitted) {
      slot->submitted = false;
      --self->active_transfers;
    }

    if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
      if (transfer->actual_length != static_cast<int>(slot->buffer.size())) {
        self->set_error("CANalyst-II returned a non-64-byte RX packet");
      } else {
        ++self->rx_packets[slot->channel];
        if (!self->rx_queues[slot->channel]->push(slot->buffer)) {
          ++self->rx_queue_drops[slot->channel];
          self->set_error("CANalyst-II native RX queue overflow on channel " +
            std::to_string(slot->channel));
        } else {
          self->rx_condition.notify_one();
        }
      }
    } else if (transfer->status != LIBUSB_TRANSFER_CANCELLED && !self->stopping.load()) {
      self->set_error("CANalyst-II RX transfer failed on channel " +
        std::to_string(slot->channel) + " (status " + std::to_string(transfer->status) + ")");
    }

    if (!self->stopping.load()) {
      const int result = libusb_submit_transfer(transfer);
      if (result == LIBUSB_SUCCESS) {
        slot->submitted = true;
        ++self->active_transfers;
      } else {
        self->set_error(std::string("could not resubmit CANalyst-II RX transfer: ") +
          libusb_error_name(result));
      }
    }
  }

  void send_tx()
  {
    TxBatch batch;
    if (!tx_queue.pop(batch)) {
      return;
    }
    UsbPacket packet = pack_message_buffer(batch.frames.data(), batch.count);
    int actual_length = 0;
    const int result = libusb_bulk_transfer(
      handle,
      kMessageOutEndpoints[batch.channel],
      packet.data(),
      static_cast<int>(packet.size()),
      &actual_length,
      kCommandTimeoutMs);
    outstanding_tx_frames.fetch_sub(batch.count);
    if (result != LIBUSB_SUCCESS) {
      set_error(std::string("CANalyst-II TX transfer failed: ") + libusb_error_name(result));
    } else if (actual_length != static_cast<int>(packet.size())) {
      set_error("CANalyst-II completed a short TX transfer on channel " +
        std::to_string(batch.channel));
    } else {
      ++tx_packets[batch.channel];
      tx_frames[batch.channel] += batch.count;
    }
    tx_condition.notify_all();
  }

  void cancel_transfers()
  {
    for (auto & slot : rx_slots) {
      if (slot->submitted) {
        const int result = libusb_cancel_transfer(slot->transfer);
        if (result != LIBUSB_SUCCESS && result != LIBUSB_ERROR_NOT_FOUND) {
          set_error(std::string("could not cancel CANalyst-II RX transfer: ") +
            libusb_error_name(result));
        }
      }
    }
  }

  void event_loop()
  {
    bool cancellation_issued = false;
    while (true) {
      if (!stopping.load()) {
        send_tx();
      } else if (!cancellation_issued) {
        cancellation_issued = true;
        cancel_transfers();
      }
      if (cancellation_issued && active_transfers.load() == 0) {
        break;
      }

      timeval timeout{};
      timeout.tv_usec = 1000;
      const int result = libusb_handle_events_timeout(context, &timeout);
      if (result != LIBUSB_SUCCESS && result != LIBUSB_ERROR_INTERRUPTED) {
        set_error(std::string("libusb event handling failed: ") + libusb_error_name(result));
      }
    }
    tx_condition.notify_all();
    rx_condition.notify_all();
  }

  void process_packet(int channel, const UsbPacket & packet)
  {
    std::array<CanFrame, kMessagesPerUsbPacket> frames{};
    std::size_t count = 0;
    std::string parse_error;
    if (!parse_message_buffer(packet, frames, count, &parse_error)) {
      set_error("malformed CANalyst-II RX packet on channel " +
        std::to_string(channel) + ": " + parse_error);
      return;
    }
    rx_frames[channel] += count;
    for (std::size_t index = 0; index < count; ++index) {
      try {
        frame_callback(channel, frames[index]);
      } catch (const std::exception & exception) {
        set_error(std::string("CAN RX callback failed: ") + exception.what());
        return;
      } catch (...) {
        set_error("CAN RX callback failed with an unknown exception");
        return;
      }
    }
  }

  void processing_loop()
  {
    std::size_t next_channel = 0;
    while (true) {
      bool processed = false;
      for (std::size_t offset = 0; offset < channels.size(); ++offset) {
        const std::size_t index = (next_channel + offset) % channels.size();
        const int channel = channels[index];
        UsbPacket packet;
        if (rx_queues[channel]->pop(packet)) {
          process_packet(channel, packet);
          next_channel = (index + 1) % channels.size();
          processed = true;
          break;
        }
      }
      if (processed) {
        continue;
      }
      if (stopping.load()) {
        const bool all_empty = std::all_of(
          channels.begin(), channels.end(),
          [this](int channel) {return rx_queues[channel]->empty();});
        if (all_empty) {
          break;
        }
      }
      std::unique_lock<std::mutex> lock(rx_wait_mutex);
      rx_condition.wait_for(lock, std::chrono::milliseconds(5));
    }
  }

  void open()
  {
    if (opened.load()) {
      return;
    }
    stopping.store(false);
    try {
      initialize_device();
      allocate_transfers();
      submit_initial_rx();
      start_channels();
      opened.store(true);
      event_thread = std::thread(&Impl::event_loop, this);
      processing_thread = std::thread(&Impl::processing_loop, this);
    } catch (...) {
      opened.store(false);
      stopping.store(true);
      rx_condition.notify_all();
      tx_condition.notify_all();
      if (event_thread.joinable()) {
        event_thread.join();
      } else if (context != nullptr && active_transfers.load() != 0) {
        cancel_transfers();
        while (active_transfers.load() != 0) {
          timeval timeout{};
          timeout.tv_usec = 1000;
          libusb_handle_events_timeout(context, &timeout);
        }
      }
      if (processing_thread.joinable()) {
        processing_thread.join();
      }
      release_resources();
      throw;
    }
  }

  bool enqueue(int channel, const CanFrame & frame)
  {
    if (!opened.load() || stopping.load()) {
      return false;
    }
    ++outstanding_tx_frames;
    bool accepted = false;
    try {
      accepted = tx_queue.push(channel, frame);
    } catch (...) {
      --outstanding_tx_frames;
      throw;
    }
    if (!accepted) {
      --outstanding_tx_frames;
    }
    tx_condition.notify_all();
    return accepted;
  }

  bool wait_tx_idle(std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::unique_lock<std::mutex> lock(tx_wait_mutex);
    return tx_condition.wait_until(lock, deadline, [this]() {
      return outstanding_tx_frames.load() == 0;
    });
  }

  void release_resources()
  {
    for (auto & slot : rx_slots) {
      if (slot->transfer != nullptr) {
        libusb_free_transfer(slot->transfer);
        slot->transfer = nullptr;
      }
    }
    rx_slots.clear();

    if (interface_claimed && handle != nullptr) {
      libusb_release_interface(handle, kUsbInterface);
      interface_claimed = false;
    }
    if (handle != nullptr) {
      libusb_close(handle);
      handle = nullptr;
    }
    if (context != nullptr) {
      libusb_exit(context);
      context = nullptr;
    }
  }

  void close()
  {
    if (!opened.exchange(false)) {
      release_resources();
      return;
    }
    stopping.store(true);
    rx_condition.notify_all();
    tx_condition.notify_all();
    if (event_thread.joinable()) {
      event_thread.join();
    }
    if (processing_thread.joinable()) {
      processing_thread.join();
    }

    if (handle != nullptr) {
      for (const int channel : channels) {
        try {
          send_command(channel, make_simple_command(kCommandStop));
        } catch (const std::exception & exception) {
          set_error(std::string("CANalyst-II channel stop failed: ") + exception.what());
        }
      }
    }
    release_resources();
  }

  TransportStats stats() const
  {
    TransportStats result;
    for (std::size_t channel = 0; channel < 2; ++channel) {
      result.rx_packets[channel] = rx_packets[channel].load();
      result.rx_frames[channel] = rx_frames[channel].load();
      result.tx_packets[channel] = tx_packets[channel].load();
      result.tx_frames[channel] = tx_frames[channel].load();
      result.rx_queue_drops[channel] = rx_queue_drops[channel].load();
    }
    return result;
  }

  std::vector<int> channels;
  FrameCallback frame_callback;
  ErrorCallback error_callback;
  std::size_t transfers_per_channel;
  std::array<std::unique_ptr<RxQueue>, 2> rx_queues;
  FairTxQueue tx_queue;

  libusb_context * context{nullptr};
  libusb_device_handle * handle{nullptr};
  bool interface_claimed{false};
  std::vector<std::unique_ptr<RxSlot>> rx_slots;
  std::thread event_thread;
  std::thread processing_thread;
  std::atomic<bool> opened{false};
  std::atomic<bool> stopping{false};
  std::atomic<int> active_transfers{0};
  std::atomic<std::uint64_t> outstanding_tx_frames{0};

  std::array<std::atomic<std::uint64_t>, 2> rx_packets{};
  std::array<std::atomic<std::uint64_t>, 2> rx_frames{};
  std::array<std::atomic<std::uint64_t>, 2> tx_packets{};
  std::array<std::atomic<std::uint64_t>, 2> tx_frames{};
  std::array<std::atomic<std::uint64_t>, 2> rx_queue_drops{};

  std::condition_variable rx_condition;
  std::condition_variable tx_condition;
  std::mutex rx_wait_mutex;
  std::mutex tx_wait_mutex;
  mutable std::mutex error_mutex;
  std::string error_message;
};

CanalystiiTransport::CanalystiiTransport(
  std::vector<int> channels,
  FrameCallback frame_callback,
  ErrorCallback error_callback,
  std::size_t transfers_per_channel,
  std::size_t rx_queue_capacity,
  std::size_t tx_queue_capacity)
: impl_(std::make_unique<Impl>(
    std::move(channels), std::move(frame_callback), std::move(error_callback),
    transfers_per_channel, rx_queue_capacity, tx_queue_capacity))
{
}

CanalystiiTransport::~CanalystiiTransport() = default;

void CanalystiiTransport::open()
{
  impl_->open();
}

bool CanalystiiTransport::enqueue(int channel, const CanFrame & frame)
{
  return impl_->enqueue(channel, frame);
}

bool CanalystiiTransport::wait_tx_idle(std::chrono::milliseconds timeout)
{
  return impl_->wait_tx_idle(timeout);
}

void CanalystiiTransport::close()
{
  impl_->close();
}

TransportStats CanalystiiTransport::stats() const
{
  return impl_->stats();
}

}  // namespace canalystii_native_bridge
