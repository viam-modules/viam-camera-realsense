#pragma once

// macOS only. Takes every attached RealSense away from macOS's own UVC driver
// once and keeps it for the life of the process. No-op on other platforms.
//
// Why (APP-16649): librealsense powers a sensor on and off for every short
// interaction before streaming (building the device object, listing stream
// profiles, writing an option), about seven times per camera init. Its libusb
// backend opens a fresh handle with auto kernel-driver detach on every power-on
// and closes it on every power-off. libusb's darwin backend implements the
// detach as a device capture (root required) and the re-attach on release as a
// full USBDeviceReEnumerate of the camera. So on macOS every power-off is an
// unplug/replug, macOS's UVC driver (UVCAssistant) races to grab the camera
// back after each one, and init fails at whatever call loses that race
// ("failed to set power state", "cannot access depth sensor", "No device
// connected"). Measured on a D435i: 2 of 20 inits succeed.
//
// How: libusb keeps one cached record per physical device, shared by every
// libusb context in the process, with an integer capture count. detach
// increments it (and captures on the 0 -> 1 edge); the re-attach that runs on
// every interface release decrements it and only re-enumerates when it reaches
// 0. This module links the same static libusb librealsense does, so capturing
// the camera here and pushing that count far above zero means librealsense's
// releases never reach 0 and never re-enumerate. Measured: 40 of 40 inits,
// zero re-enumerations, every pre-start step faster.
//
// What it relies on, all pinned by conan.lock (libusb 1.0.26): the per-device
// count is process-global and decremented once per interface release, and a
// captured device shows no kernel driver on its interfaces so librealsense's
// own claims never detach again. scripts/macos-probe/rs_mac_probe
// --hold-capture is the regression check for a libusb bump.
//
// Side effects: the camera is invisible to macOS camera apps while the module
// runs (already true while it streams) and comes back when the process exits.
// Root is still required, this only moves the one capture earlier.

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

// Capture-count budget. A power cycle costs one decrement per released
// interface (three for the depth handle, two for color), so this covers about
// 200k power cycles. Every call for an already-captured camera adds kTopUp, so
// the count only ever grows while the camera stays attached; overflow would
// need more than 200k device-changed callbacks.
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
