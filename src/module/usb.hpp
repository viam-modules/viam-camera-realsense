#pragma once

#include <optional>
#include <string>
#include <utility>

#include <librealsense2/rs.hpp>

namespace realsense {
namespace device {

enum class UsbConnectionType { usb3, usb2, unknown };

// Classifies the USB type descriptor reported by the device (e.g. "2.1",
// "3.2", or "Unknown" on some platforms/firmware). Returns nullopt when the
// device does not report a descriptor or querying it fails.
template <typename DeviceT>
std::optional<std::pair<UsbConnectionType, std::string>>
getUsbConnectionType(DeviceT const &dev) noexcept {
  try {
    if (not dev.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)) {
      return std::nullopt;
    }
    std::string usb_type = dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
    auto type = UsbConnectionType::unknown;
    if (not usb_type.empty() and usb_type[0] == '3') {
      type = UsbConnectionType::usb3;
    } else if (not usb_type.empty() and usb_type[0] == '2') {
      type = UsbConnectionType::usb2;
    }
    return std::make_pair(type, usb_type);
  } catch (...) {
    return std::nullopt;
  }
}

} // namespace device
} // namespace realsense
