#pragma once

#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "download_utils.hpp"
#include "zip_utils.hpp"

#include <librealsense2/rs.hpp>

#include <viam/sdk/log/logging.hpp>

namespace viam {
namespace realsense {
namespace firmware_update {

/**
 * Read firmware data from a local file.
 * @param file_path Path to the firmware file
 * @return The firmware data as a vector of bytes
 * @throws std::runtime_error if file cannot be opened or read
 */
std::vector<uint8_t> readFirmwareFile(const std::string &file_path);

/**
 * Get the firmware URL for a specific version.
 * @param version The firmware version
 * @return The firmware URL as a string, or std::nullopt if not found
 */
std::optional<std::string> getFirmwareURLForVersion(const std::string &version);

/**
 * Update the firmware of a RealSense device.
 * @param rs_device The RealSense device to update
 * @param device_serial_number The serial number of the device
 * @param firmware_url The URL of the firmware file
 * @param logger The logger to use for logging
 * @throws std::runtime_error if firmware update fails
 * @return pair<bool, map> - first is success flag, second contains
 * message/error
 */
template <typename RealsenseContextT>
[[nodiscard]] std::pair<bool, std::unordered_map<std::string, std::string>>
updateFirmware(std::shared_ptr<rs2::device> rs_device,
               std::string const &device_serial_number,
               std::string firmware_url,
               std::shared_ptr<RealsenseContextT> realsense_ctx,
               viam::sdk::LogSource &logger) {

  std::pair<bool, std::unordered_map<std::string, std::string>> response;
  if (firmware_url.empty()) {
    VIAM_SDK_LOG_IMPL(logger, info)
        << "[handleFirmwareUpdate] Auto-detect mode: querying device for "
           "recommended firmware";

    // Check if device supports recommended firmware version
    if (!rs_device->supports(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION)) {
      std::string error_msg = std::string(
          "Auto-detect failed: Device does not provide recommended "
          "firmware version information. Please specify the firmware URL "
          "directly using: {\"firmware_update\": "
          "\"https://your-firmware-url.zip\"}");
      VIAM_SDK_LOG_IMPL(logger, error) << error_msg;
      return {false, {{"error", error_msg}}};
    }

    std::string const recommended_version =
        rs_device->get_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION);
    VIAM_SDK_LOG_IMPL(logger, info)
        << "[handleFirmwareUpdate] Device recommended "
           "firmware version: "
        << recommended_version;

    // Look up URL for recommended version
    auto const firmware_url_opt = getFirmwareURLForVersion(recommended_version);
    if (!firmware_url_opt.has_value()) {
      std::string error_msg =
          std::string("Auto-detect found recommended firmware version ") +
          recommended_version +
          ", but no download URL mapping is available for this version. "
          "Please specify the firmware URL directly using: "
          "{\"firmware_update\": \"https://your-firmware-url.zip\"}";
      VIAM_SDK_LOG_IMPL(logger, error) << error_msg;
      return {false, {{"error", error_msg}}};
    }

    firmware_url = *firmware_url_opt;
    VIAM_SDK_LOG_IMPL(logger, info)
        << "[handleFirmwareUpdate] Auto-detected firmware URL: " << firmware_url
        << " for version " << recommended_version;
  }

  // Download firmware from URL
  VIAM_SDK_LOG_IMPL(logger, info)
      << "[handleFirmwareUpdate] Firmware URL: " << firmware_url;
  std::vector<uint8_t> firmware_data =
      viam::realsense::download_utils::downloadFirmwareFromURL(firmware_url,
                                                               logger);

  // Check if it's a ZIP file and extract if needed
  if (viam::realsense::zip_utils::isZipFile(firmware_data)) {
    VIAM_SDK_LOG_IMPL(logger, info)
        << "[handleFirmwareUpdate] Detected ZIP file, extracting .bin file";
    firmware_data =
        viam::realsense::zip_utils::extractBinFromZip(firmware_data, logger);
  } else {
    VIAM_SDK_LOG_IMPL(logger, error)
        << "[handleFirmwareUpdate] Firmware update "
           "failed: firmware data is not a ZIP file";
    return {
        false,
        {{"error", "Firmware update failed: firmware data is not a ZIP file"}}};
  }

  VIAM_SDK_LOG_IMPL(logger, info)
      << "[handleFirmwareUpdate] Firmware data size: " << firmware_data.size()
      << " bytes";

  VIAM_SDK_LOG_IMPL(logger, info)
      << "[handleFirmwareUpdate] Updating device with "
         "serial number: "
      << device_serial_number;

  // Check if device is already in recovery/DFU mode first
  rs2::update_device update_device;
  bool already_in_recovery = rs_device->is<rs2::update_device>();

  VIAM_SDK_LOG_IMPL(logger, info)
      << "[handleFirmwareUpdate] Device state check: is_update_device="
      << already_in_recovery
      << ", is_updatable=" << rs_device->is<rs2::updatable>();

  // Capture firmware_update_id before entering update state for later matching
  std::string firmware_update_id;
  if (rs_device->supports(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID)) {
    firmware_update_id =
        rs_device->get_info(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID);
    VIAM_SDK_LOG_IMPL(logger, info)
        << "[handleFirmwareUpdate] Device firmware update ID: "
        << firmware_update_id;
  }

  if (already_in_recovery) {
    VIAM_SDK_LOG_IMPL(logger, info)
        << "[handleFirmwareUpdate] Device is already in recovery/DFU mode, "
           "skipping enter_update_state and compatibility checks";
    update_device = rs_device->as<rs2::update_device>();

    if (!update_device) {
      std::string error_msg =
          "Failed to get update device interface from recovery mode device";
      VIAM_SDK_LOG_IMPL(logger, error)
          << "[handleFirmwareUpdate] " << error_msg;
      return {false, {{"error", error_msg}}};
    }
  } else {
    // Normal path: Check if device is updatable, check compatibility, then
    // enter update state

    // Check if device is updatable
    if (!rs_device->is<rs2::updatable>()) {
      throw std::runtime_error(
          "Firmware update failed: Device does not support firmware updates");
    }

    auto updatable_device = rs_device->as<rs2::updatable>();
    if (!updatable_device) {
      std::string error_msg = "Device does not support firmware updates";
      VIAM_SDK_LOG_IMPL(logger, error)
          << "[handleFirmwareUpdate] " << error_msg;
      return {false, {{"error", error_msg}}};
    }

    // Check firmware compatibility
    VIAM_SDK_LOG_IMPL(logger, info)
        << "[handleFirmwareUpdate] Checking firmware compatibility";
    if (!updatable_device.check_firmware_compatibility(firmware_data)) {
      std::string error_msg =
          "Firmware update failed: Firmware is not compatible with "
          "this device";
      VIAM_SDK_LOG_IMPL(logger, error)
          << "[handleFirmwareUpdate] " << error_msg;
      return {false, {{"error", error_msg}}};
    }

    /*
        From realsense source code:
        Places the device in DFU (recovery) mode, where the DFU process
        can continue with update_device_interface.
        Restarts the device!
        */

    VIAM_SDK_LOG_IMPL(logger, info)
        << "[handleFirmwareUpdate] Entering update state";
    updatable_device.enter_update_state();

    // Event-driven device detection using condition variable (similar to
    // rs-fw-update reference)
    VIAM_SDK_LOG_IMPL(logger, info)
        << "[handleFirmwareUpdate] Waiting for device to reconnect in DFU mode "
           "(timeout: 15s)";

    std::mutex mutex;
    std::condition_variable cv;
    bool device_found = false;

    // Set up callback to detect device reconnection
    realsense_ctx->setDevicesChangedCallback([&](rs2::event_information &info) {
      for (auto &&device : info.get_new_devices()) {
        std::lock_guard<std::mutex> lk(mutex);

        // Check if this is an update device (in DFU/recovery mode)
        if (!device.is<rs2::update_device>()) {
          continue;
        }

        // Match by firmware_update_id if available
        if (!firmware_update_id.empty() &&
            device.supports(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID)) {
          std::string device_fw_id =
              device.get_info(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID);
          if (device_fw_id != firmware_update_id) {
            continue;
          }
        }

        // Check USB type and warn if USB 2.x
        if (device.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)) {
          std::string usb_type =
              device.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
          if (usb_type.find("2.") != std::string::npos) {
            VIAM_SDK_LOG_IMPL(logger, warn)
                << "Warning! the camera is connected via USB 2 port, in case "
                   "the "
                   "process fails, connect the camera to a USB 3 port and try "
                   "again";
          }
        }

        // Found the device in DFU mode
        update_device = device.as<rs2::update_device>();
        device_found = true;
        cv.notify_one();
        break;
      }
    });

    // RAII: Automatically clear custom callback on any exit path
    // (success/error/exception) The RAII restorer in the caller will then
    // restore the default callback
    auto callback_clearer =
        std::shared_ptr<void>(nullptr, [realsense_ctx](void *) {
          realsense_ctx->clearDevicesChangedCallback();
        });

    // Wait for device to reconnect with 15 second timeout
    std::unique_lock<std::mutex> lk(mutex);
    constexpr int WAIT_FOR_DEVICE_TIMEOUT = 15;
    if (!cv.wait_for(lk, std::chrono::seconds(WAIT_FOR_DEVICE_TIMEOUT),
                     [&] { return device_found; })) {
      std::string error_msg = "Timeout waiting for device to reconnect in DFU "
                              "mode after 15 seconds";
      VIAM_SDK_LOG_IMPL(logger, error)
          << "[handleFirmwareUpdate] " << error_msg;
      return {false, {{"error", error_msg}}};
    }

    VIAM_SDK_LOG_IMPL(logger, info)
        << "[handleFirmwareUpdate] Device reconnected in DFU mode";

    if (!update_device) {
      std::string error_msg = "Failed to get update device interface";
      VIAM_SDK_LOG_IMPL(logger, error)
          << "[handleFirmwareUpdate] " << error_msg;
      return {false, {{"error", error_msg}}};
    }
  } // End of else block (normal path)

  // Perform the firmware update with progress tracking
  VIAM_SDK_LOG_IMPL(logger, info)
      << "[handleFirmwareUpdate] Starting firmware update process";
  update_device.update(firmware_data, [&logger](const float progress) {
    int percent = static_cast<int>(progress * 100);
    VIAM_SDK_LOG_IMPL(logger, info)
        << "[handleFirmwareUpdate] Progress: " << percent << "%";
  });

  VIAM_SDK_LOG_IMPL(logger, info)
      << "[handleFirmwareUpdate] Firmware update completed successfully";

  return {true, {{"message", "Firmware update completed successfully"}}};
}

} // namespace firmware_update
} // namespace realsense
} // namespace viam
