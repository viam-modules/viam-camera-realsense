#pragma once

// macOS only: capture every attached RealSense from macOS's UVC driver once
// and hold it for the life of the process. No-op elsewhere.
//
// librealsense powers sensors on and off before streaming, and on macOS each
// power-off makes libusb hand the camera back to the OS with a USB
// re-enumeration; init then fails at whatever call loses the race to grab it
// back (APP-16649). libusb only re-enumerates when its per-device capture
// count drops to zero, and this module shares librealsense's static libusb,
// so capturing here with a large count keeps that from ever happening.
//
// Depends on libusb 1.0.26 (conan.lock) internals; check with
// scripts/macos-probe/rs_mac_probe --hold-capture after a libusb bump. Root
// is still required.

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

// Intel's USB vendor id; every RealSense enumerates under it.
inline constexpr std::uint16_t kRealsenseVendorId = 0x8086;

// Capture-count budget: about 200k power cycles (up to five decrements each).
// Held cameras get kTopUp on every call, so the count only grows.
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
  // Keyed by libusb_device, which stays the same object while the camera is
  // attached (a replug is a new device). libusb_open holds a reference to it.
  // Handles are never closed while the camera is attached: the capture has to
  // outlive every librealsense handle.
  std::unordered_map<libusb_device *, Held> held;
};
inline State &state() {
  static State s;
  return s;
}
inline void addBudget(libusb_device_handle *handle, int amount) {
  // Pure bookkeeping once the device is captured: no USB traffic.
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

// Capture every RealSense on the bus that is not captured yet, top up the
// budget of the ones that are, and forget cameras that left. Safe to call
// repeatedly and from any thread; call it before librealsense builds its first
// device object and again on every device-changed event. Returns how many
// cameras were newly captured.
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
    // Interface 0 is the depth UVC control interface on every D400. On macOS
    // the interface number is ignored anyway: capture is per device.
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

  // Cameras that left the bus. A replug comes back as a new libusb device and
  // is captured on the next call (the device-changed callback).
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
