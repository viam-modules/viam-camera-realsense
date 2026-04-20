#pragma once

#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <iostream>
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
 * Core firmware update logic.
 * @param rs_device The RealSense device to update
 * @param device_serial_number The serial number of the device
 * @param firmware_data The firmware data
 * @param realsense_ctx The RealSense context
 * @param logger The logger to use for logging
 * @return pair<bool, map> - first is success flag, second contains
 * message/error
 */
template <typename RealsenseContextT>
[[nodiscard]] std::tuple<bool, std::unordered_map<std::string, std::string>,
                         rs2::update_device>
prepareDeviceForUpdate(std::shared_ptr<rs2::device> rs_device,
                       std::string const &device_serial_number,
                       std::vector<uint8_t> firmware_data,
                       std::shared_ptr<RealsenseContextT> realsense_ctx,
                       viam::sdk::LogSource &logger) {
  // Validates device is updatable, checks firmware compatibility,
  // enters DFU mode, and waits for device to reconnect

  // Check if device is updatable
  if (!rs_device->is<rs2::updatable>()) {
    std::string error_msg = "Device does not support firmware updates";
    VIAM_SDK_LOG_IMPL(logger, error)
        << "[prepareDeviceForUpdate] " << error_msg;
    return {false, {{"error", error_msg}}, rs2::update_device()};
  }

  auto updatable_device = rs_device->as<rs2::updatable>();

  // Check firmware compatibility
  VIAM_SDK_LOG_IMPL(logger, info)
      << "[prepareDeviceForUpdate] Checking firmware compatibility";
  if (!updatable_device.check_firmware_compatibility(firmware_data)) {
    std::string device_model = "unknown";
    if (rs_device->supports(RS2_CAMERA_INFO_NAME)) {
      device_model = rs_device->get_info(RS2_CAMERA_INFO_NAME);
    }

    std::string error_msg =
        "Firmware update failed: Firmware is incompatible with this device. "
        "Verify the firmware URL/file is correct for your RealSense camera "
        "model (" +
        device_model + ").";
    VIAM_SDK_LOG_IMPL(logger, error)
        << "[prepareDeviceForUpdate] " << error_msg;
    return {false, {{"error", error_msg}}, rs2::update_device()};
  }

  /*
      From realsense source code:
      Places the device in DFU (recovery) mode, where the DFU process
      can continue with update_device_interface.
      Restarts the device!
      */

  VIAM_SDK_LOG_IMPL(logger, info)
      << "[prepareDeviceForUpdate] Entering update state";
  updatable_device.enter_update_state();

  // Event-driven device detection using condition variable (similar to
  // rs-fw-update reference)
  VIAM_SDK_LOG_IMPL(logger, info)
      << "[prepareDeviceForUpdate] Waiting for device to reconnect in DFU mode "
         "(timeout: 15s)";

  // Capture firmware_update_id before entering update state for later matching
  // This is required to safely identify the correct device after DFU reconnect
  std::string firmware_update_id;
  if (rs_device->supports(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID)) {
    firmware_update_id =
        rs_device->get_info(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID);
    VIAM_SDK_LOG_IMPL(logger, info)
        << "[prepareDeviceForUpdate] Device firmware update ID: "
        << firmware_update_id;
  } else {
    std::string error_msg =
        "Device does not support firmware_update_id. Cannot safely identify "
        "device after DFU mode reconnection. This may indicate an older "
        "device or firmware that doesn't support this feature.";
    VIAM_SDK_LOG_IMPL(logger, error)
        << "[prepareDeviceForUpdate] " << error_msg;
    return {false, {{"error", error_msg}}, rs2::update_device()};
  }

  std::mutex mutex;
  std::condition_variable cv;
  bool device_found = false;
  rs2::update_device update_device;

  // Set up callback to detect device reconnection
  // Explicit capture list for thread safety:
  // - firmware_update_id by value (immutable copy, thread-safe)
  // - mutex, cv, device_found, update_device by reference (must be modified)
  // Note: Using std::cout/cerr instead of logger since callback runs on
  // different thread and logger lifetime is uncertain
  realsense_ctx->setDevicesChangedCallback(
      [&mutex, &cv, &device_found, &update_device,
       firmware_update_id](rs2::event_information &info) {
        for (auto &&device : info.get_new_devices()) {
          std::lock_guard<std::mutex> lk(mutex);

          // Check if this is an update device (in DFU/recovery mode)
          if (!device.is<rs2::update_device>()) {
            continue;
          }

          // Match by firmware_update_id (required for multi-device safety)
          // Following Intel RealSense reference implementation pattern
          if (firmware_update_id.empty()) {
            // Cannot safely identify device without firmware_update_id
            std::cout << "[prepareDeviceForUpdate] Skipping device - no "
                         "firmware_update_id to match against"
                      << std::endl;
            continue;
          }

          if (!device.supports(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID)) {
            std::cout << "[prepareDeviceForUpdate] Skipping device - does not "
                         "support firmware_update_id"
                      << std::endl;
            continue;
          }

          std::string device_fw_id =
              device.get_info(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID);
          if (device_fw_id != firmware_update_id) {
            std::cout << "[prepareDeviceForUpdate] Skipping device - "
                         "firmware_update_id mismatch (expected: "
                      << firmware_update_id << ", got: " << device_fw_id << ")"
                      << std::endl;
            continue;
          }

          // Found the correct device in DFU mode
          std::cout << "[prepareDeviceForUpdate] Matched device by "
                       "firmware_update_id: "
                    << firmware_update_id << std::endl;
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
        << "[prepareDeviceForUpdate] " << error_msg;
    return {false, {{"error", error_msg}}, update_device};
  }

  VIAM_SDK_LOG_IMPL(logger, info)
      << "[prepareDeviceForUpdate] Device reconnected in DFU mode";

  if (!update_device) {
    std::string error_msg = "Failed to get update device interface";
    VIAM_SDK_LOG_IMPL(logger, error)
        << "[prepareDeviceForUpdate] " << error_msg;
    return {false, {{"error", error_msg}}, update_device};
  }
  return {true, {}, update_device};
}

/**
 * Update the firmware of a RealSense device.
 * @param rs_device The RealSense device to update
 * @param device_serial_number The serial number of the device
 * @param firmware_url The URL of the firmware file (empty for auto-detect)
 * @param realsense_ctx The RealSense context
 * @param logger The logger to use for logging
 * @return pair<bool, map> - first is success flag, second contains
 * message/error. Returns {false, {{"error", "..."}}} on failure, {true,
 * {{"message", "..."}}} on success.
 */
template <typename RealsenseContextT>
[[nodiscard]] std::pair<bool, std::unordered_map<std::string, std::string>>
updateFirmware(std::shared_ptr<rs2::device> rs_device,
               std::string const &device_serial_number,
               std::string firmware_url,
               std::shared_ptr<RealsenseContextT> realsense_ctx,
               viam::sdk::LogSource &logger) {
  try {

    // Check USB type and warn if USB 2.x
    if (rs_device->supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)) {
      std::string usb_type =
          rs_device->get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
      if (usb_type.find("2.") != std::string::npos) {
        VIAM_SDK_LOG_IMPL(logger, warn)
            << "Warning! the camera is connected via USB 2 port, in case "
               "the "
               "process fails, connect the camera to a USB 3 port and try "
               "again";
      }
    }

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
            "directly using: {\"update_firmware\": "
            "\"https://your-firmware-url.zip\"}. Find firmware URLs at: "
            "https://dev.realsenseai.com/docs/firmware-releases-d400");
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
      auto const firmware_url_opt =
          getFirmwareURLForVersion(recommended_version);
      if (!firmware_url_opt.has_value()) {
        std::string error_msg =
            std::string("Auto-detect found recommended firmware version ") +
            recommended_version +
            ", but no download URL mapping is available for this version. "
            "Please specify the firmware URL directly using: "
            "{\"update_firmware\": \"https://your-firmware-url.zip\"}. "
            "Find firmware URLs at: "
            "https://dev.realsenseai.com/docs/firmware-releases-d400";
        VIAM_SDK_LOG_IMPL(logger, error) << error_msg;
        return {false, {{"error", error_msg}}};
      }

      firmware_url = *firmware_url_opt;
      VIAM_SDK_LOG_IMPL(logger, info)
          << "[handleFirmwareUpdate] Auto-detected firmware URL: "
          << firmware_url << " for version " << recommended_version;
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
      return {false,
              {{"error",
                "Firmware update failed: firmware data is not a ZIP file"}}};
    }

    VIAM_SDK_LOG_IMPL(logger, info)
        << "[handleFirmwareUpdate] Firmware data size: " << firmware_data.size()
        << " bytes";

    VIAM_SDK_LOG_IMPL(logger, info)
        << "[handleFirmwareUpdate] Updating device with "
           "serial number: "
        << device_serial_number;

    // Capture firmware_update_id for logging (if available)
    std::string firmware_update_id;
    if (rs_device->supports(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID)) {
      firmware_update_id =
          rs_device->get_info(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID);
    }

    // Check if device is already in recovery/DFU mode first
    rs2::update_device update_device;
    bool already_in_recovery = rs_device->is<rs2::update_device>();

    VIAM_SDK_LOG_IMPL(logger, info)
        << "[handleFirmwareUpdate] Device state check: is_update_device="
        << already_in_recovery
        << ", is_updatable=" << rs_device->is<rs2::updatable>();

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
    } else // End of else block (normal path)
    {
      auto [success, message, update_device_local] =
          prepareDeviceForUpdate(rs_device, device_serial_number, firmware_data,
                                 realsense_ctx, logger);
      if (!success) {
        return {success, message};
      }
      update_device = update_device_local;
    }

    // Perform the firmware update with progress tracking
    VIAM_SDK_LOG_IMPL(logger, info)
        << "[handleFirmwareUpdate] Starting firmware update process";
    update_device.update(
        firmware_data,
        [device_serial_number, firmware_update_id](const float progress) {
          int percent = static_cast<int>(progress * 100);
          std::cout << "Firmware update progress: " << percent << "%";
          if (!device_serial_number.empty()) {
            std::cout << " (serial: " << device_serial_number;
            if (!firmware_update_id.empty()) {
              std::cout << ", fw_update_id: " << firmware_update_id;
            }
            std::cout << ")";
          }
          std::cout << std::endl;
        });

    VIAM_SDK_LOG_IMPL(logger, info)
        << "[handleFirmwareUpdate] Firmware update completed successfully";

    return {true, {{"message", "Firmware update completed successfully"}}};
  } catch (const rs2::camera_disconnected_error &e) {
    std::string error_msg = std::string("Camera disconnected: ") + e.what();
    VIAM_SDK_LOG_IMPL(logger, error) << "[updateFirmware] " << error_msg;
    return {false, {{"error", error_msg}}};
  } catch (const rs2::backend_error &e) {
    std::string error_msg = std::string("Backend error: ") + e.what();
    VIAM_SDK_LOG_IMPL(logger, error) << "[updateFirmware] " << error_msg;
    return {false, {{"error", error_msg}}};
  } catch (const rs2::invalid_value_error &e) {
    std::string error_msg = std::string("Invalid value: ") + e.what();
    VIAM_SDK_LOG_IMPL(logger, error) << "[updateFirmware] " << error_msg;
    return {false, {{"error", error_msg}}};
  } catch (const rs2::error &e) {
    std::string error_msg = std::string(e.what());

    // Check for the "locked for update" error and provide helpful message
    if (error_msg.find("locked for update") != std::string::npos) {
      std::string friendly_msg =
          "Device is locked and cannot be downgraded or updated to the same "
          "version. Current firmware appears to be at or above the target "
          "version. To update to a newer version, specify the firmware URL "
          "directly using: {\"update_firmware\": "
          "\"https://your-firmware-url.zip\"}. Find firmware URLs at: "
          "https://dev.realsenseai.com/docs/firmware-releases-d400. "
          "Error details: " +
          error_msg;
      VIAM_SDK_LOG_IMPL(logger, error) << "[updateFirmware] " << friendly_msg;
      return {false, {{"error", friendly_msg}}};
    }

    // Generic RealSense error
    error_msg = std::string("RealSense error: ") + error_msg;
    VIAM_SDK_LOG_IMPL(logger, error) << "[updateFirmware] " << error_msg;
    return {false, {{"error", error_msg}}};
  } catch (const std::runtime_error &e) {
    std::string error_msg = std::string("Firmware update failed: ") + e.what();
    VIAM_SDK_LOG_IMPL(logger, error) << "[updateFirmware] " << error_msg;
    return {false, {{"error", error_msg}}};
  } catch (const std::exception &e) {
    std::string error_msg = std::string("Unexpected error: ") + e.what();
    VIAM_SDK_LOG_IMPL(logger, error) << "[updateFirmware] " << error_msg;
    return {false, {{"error", error_msg}}};
  } catch (...) {
    std::string error_msg = "Unknown error during firmware update";
    VIAM_SDK_LOG_IMPL(logger, error) << "[updateFirmware] " << error_msg;
    return {false, {{"error", error_msg}}};
  }
}

} // namespace firmware_update
} // namespace realsense
} // namespace viam
