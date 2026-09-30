#pragma once

// macOS only: capture every attached RealSense from the OS UVC driver once and
// hold it for the life of the process (APP-16649). No-op elsewhere.
//
// librealsense's power cycles make libusb re-enumerate the camera whenever its
// per-device capture count hits zero; a large count here prevents that. Relies
// on libusb 1.0.26 internals (conan.lock): re-check with
// scripts/macos-probe/rs_mac_probe --hold-capture after a bump. Needs root.

#include <cstdint>
#include <cstdio>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <viam/sdk/log/logging.hpp>

#if defined(__APPLE__)
#include <libusb.h>
#endif

namespace realsense {
namespace usb_capture {

// Intel's USB vendor id.
inline constexpr std::uint16_t kRealsenseVendorId = 0x8086;

// About 200k power cycles; held cameras get kTopUp per call.
inline constexpr int kInitialBudget = 1000000;
inline constexpr int kTopUp = 10000;

#if defined(__APPLE__)
namespace detail {
struct Held {
  libusb_device_handle *handle;
  std::uint16_t product_id;
};
struct State {
  std::mutex mutex;
  libusb_context *ctx = nullptr;
  // libusb_device is stable while attached (a replug is a new one). Handles
  // stay open so the capture outlives every librealsense handle.
  std::unordered_map<libusb_device *, Held> held;
};
inline State &state() {
  static State s;
  return s;
}
inline void addBudget(libusb_device_handle *handle, int amount) {
  // No USB traffic once captured.
  for (int i = 0; i < amount; i++) {
    libusb_detach_kernel_driver(handle, 0);
  }
}
inline std::string productIdHex(std::uint16_t id) {
  char buf[8];
  std::snprintf(buf, sizeof buf, "0x%04x", id);
  return buf;
}
} // namespace detail
#endif

// Capture new RealSenses, top up held ones, forget unplugged ones. Thread-safe;
// call before librealsense's first device object and on every device change.
// Returns the number newly captured.
inline int captureRealsenseDevices() {
#if !defined(__APPLE__)
  return 0;
#else
  auto &st = detail::state();
  std::lock_guard<std::mutex> lock(st.mutex);
  if (st.ctx == nullptr) {
    int rc = libusb_init(&st.ctx);
    if (rc != 0) {
      VIAM_SDK_LOG(warn) << "[usb_capture] libusb_init failed: "
                         << libusb_error_name(rc);
      st.ctx = nullptr;
      return 0;
    }
  }

  libusb_device **list = nullptr;
  ssize_t count = libusb_get_device_list(st.ctx, &list);
  if (count < 0) {
    VIAM_SDK_LOG(warn) << "[usb_capture] libusb_get_device_list failed: "
                       << libusb_error_name(static_cast<int>(count));
    return 0;
  }

  int captured = 0;
  std::unordered_set<libusb_device *> present;
  for (ssize_t i = 0; i < count; i++) {
    libusb_device *dev = list[i];
    libusb_device_descriptor desc{};
    if (libusb_get_device_descriptor(dev, &desc) != 0 or
        desc.idVendor != kRealsenseVendorId) {
      continue;
    }
    present.insert(dev);
    auto it = st.held.find(dev);
    if (it != st.held.end()) {
      detail::addBudget(it->second.handle, kTopUp);
      continue;
    }

    libusb_device_handle *handle = nullptr;
    int rc = libusb_open(dev, &handle);
    if (rc != 0) {
      VIAM_SDK_LOG(warn) << "[usb_capture] cannot open RealSense "
                         << detail::productIdHex(desc.idProduct) << ": "
                         << libusb_error_name(rc);
      continue;
    }
    // The interface number is ignored on macOS: capture is per device.
    rc = libusb_detach_kernel_driver(handle, 0);
    if (rc != 0) {
      VIAM_SDK_LOG(warn) << "[usb_capture] cannot capture RealSense "
                         << detail::productIdHex(desc.idProduct) << ": "
                         << libusb_error_name(rc)
                         << (rc == LIBUSB_ERROR_ACCESS
                                 ? " (capturing a USB device needs root)"
                                 : "");
      libusb_close(handle);
      continue;
    }
    detail::addBudget(handle, kInitialBudget - 1);
    st.held[dev] = detail::Held{handle, desc.idProduct};
    captured++;
    VIAM_SDK_LOG(info)
        << "[usb_capture] captured RealSense "
        << detail::productIdHex(desc.idProduct)
        << " from macOS's UVC driver for the life of this process";
  }

  // Unplugged cameras. A replug is a new device, captured on the next call.
  for (auto it = st.held.begin(); it != st.held.end();) {
    if (present.count(it->first) == 0) {
      VIAM_SDK_LOG(info) << "[usb_capture] RealSense "
                         << detail::productIdHex(it->second.product_id)
                         << " left the bus, dropping its capture handle";
      libusb_close(it->second.handle);
      it = st.held.erase(it);
    } else {
      ++it;
    }
  }
  libusb_free_device_list(list, 1);
  return captured;
#endif
}

} // namespace usb_capture
} // namespace realsense
