#include <libusb-1.0/libusb.h>

#include <algorithm>
#include <cstdint>
#include <deque>
#include <mutex>

struct libusb_context {};
struct libusb_device_handle {};

namespace
{
libusb_context context;
libusb_device_handle device;
std::deque<libusb_transfer *> pending;
std::mutex pending_mutex;
bool failed = false;
}  // namespace

extern "C"
{
int LIBUSB_CALL libusb_init(libusb_context ** result)
{
  *result = &context;
  return LIBUSB_SUCCESS;
}

void LIBUSB_CALL libusb_exit(libusb_context *) {}

libusb_device_handle * LIBUSB_CALL libusb_open_device_with_vid_pid(
  libusb_context *, std::uint16_t, std::uint16_t)
{
  return &device;
}

void LIBUSB_CALL libusb_close(libusb_device_handle *) {}

int LIBUSB_CALL libusb_set_auto_detach_kernel_driver(libusb_device_handle *, int)
{
  return LIBUSB_SUCCESS;
}

int LIBUSB_CALL libusb_get_configuration(libusb_device_handle *, int * value)
{
  *value = 1;
  return LIBUSB_SUCCESS;
}

int LIBUSB_CALL libusb_set_configuration(libusb_device_handle *, int)
{
  return LIBUSB_SUCCESS;
}

int LIBUSB_CALL libusb_claim_interface(libusb_device_handle *, int)
{
  return LIBUSB_SUCCESS;
}

int LIBUSB_CALL libusb_release_interface(libusb_device_handle *, int)
{
  return LIBUSB_SUCCESS;
}

int LIBUSB_CALL libusb_bulk_transfer(
  libusb_device_handle *, unsigned char, unsigned char *, int length,
  int * actual_length, unsigned int)
{
  *actual_length = length;
  return LIBUSB_SUCCESS;
}

int LIBUSB_CALL libusb_submit_transfer(libusb_transfer * transfer)
{
  std::lock_guard<std::mutex> lock(pending_mutex);
  transfer->status = LIBUSB_TRANSFER_COMPLETED;
  pending.push_back(transfer);
  return LIBUSB_SUCCESS;
}

int LIBUSB_CALL libusb_cancel_transfer(libusb_transfer * transfer)
{
  std::lock_guard<std::mutex> lock(pending_mutex);
  if (std::find(pending.begin(), pending.end(), transfer) == pending.end()) {
    return LIBUSB_ERROR_NOT_FOUND;
  }
  transfer->status = LIBUSB_TRANSFER_CANCELLED;
  return LIBUSB_SUCCESS;
}

int LIBUSB_CALL libusb_handle_events_timeout(libusb_context *, timeval *)
{
  libusb_transfer * transfer = nullptr;
  {
    std::lock_guard<std::mutex> lock(pending_mutex);
    if (pending.empty()) {
      return LIBUSB_SUCCESS;
    }
    transfer = pending.front();
    pending.pop_front();
  }
  if (!failed && transfer->endpoint == 0x83) {
    transfer->status = LIBUSB_TRANSFER_ERROR;
    failed = true;
  }
  transfer->actual_length = 0;
  transfer->callback(transfer);
  return LIBUSB_SUCCESS;
}
}
