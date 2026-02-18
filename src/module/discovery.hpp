#pragma once

#include "realsense.hpp"

#include <chrono>
#include <memory>
#include <thread>

#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>
#include <viam/sdk/services/discovery.hpp>

#include <librealsense2/rs.hpp>

namespace realsense {
namespace discovery {

template <typename ContextT>
class RealsenseDiscovery : public viam::sdk::Discovery {
  std::shared_ptr<ContextT> rs_ctx_;

public:
  static inline viam::sdk::Model model{"viam", "realsense", "discovery"};
  RealsenseDiscovery(viam::sdk::Dependencies dependencies,
                     viam::sdk::ResourceConfig configuration,
                     std::shared_ptr<ContextT> ctx)
      : Discovery(configuration.name()), rs_ctx_(ctx) {}

  std::vector<viam::sdk::ResourceConfig>
  discover_resources(const viam::sdk::ProtoStruct &extra) override {
    std::vector<viam::sdk::ResourceConfig> configs;

    try {
      VIAM_SDK_LOG(info) << "[discover_resources] Starting device discovery";

      auto deviceList = rs_ctx_->query_devices();
      int devCount = deviceList.size();

      VIAM_SDK_LOG(info) << "[discover_resources] query_devices returned "
                         << devCount << " devices";

      if (devCount == 0) {
        VIAM_SDK_LOG(warn) << "[discover_resources] No Realsense devices found "
                              "during discovery";
        return {};
      }

      std::vector<std::string> device_errors;
      for (int i = 0; i < devCount; i++) {
        try {
          VIAM_SDK_LOG(debug)
              << "[discover_resources] Attempting to access device at index "
              << i;
          auto dev = deviceList[i];
          VIAM_SDK_LOG(debug)
              << "[discover_resources] Successfully accessed device at index "
              << i;

          std::string device_identifier;
          bool is_recovery_mode = false;

          // Try to get serial number first (normal mode)
          if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
            device_identifier = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
          }
          // If no serial number, check if it's a recovery/DFU mode device
          else if (dev.supports(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID)) {
            device_identifier =
                dev.get_info(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID);
            is_recovery_mode = true;
            VIAM_SDK_LOG(warn) << "[discover_resources] Device at index " << i
                               << " is in recovery/DFU mode with update ID: "
                               << device_identifier;
          } else {
            VIAM_SDK_LOG(warn)
                << "[discover_resources] Device at index " << i
                << " does not support serial number or firmware update ID";
            continue;
          }

          viam::sdk::ProtoStruct attributes;
          attributes["serial_number"] = device_identifier;

          viam::sdk::ProtoList sensors;
          sensors.push_back("color");
          sensors.push_back("depth");
          attributes["sensors"] = sensors;

          std::ostringstream name;
          name << "realsense-" << device_identifier;
          if (is_recovery_mode) {
            name << "-recovery";
          }

          viam::sdk::ResourceConfig config(
              "camera", std::move(name.str()), "viam", attributes,
              "rdk:component:camera", realsense::Realsense<ContextT>::model,
              viam::sdk::LinkConfig{}, viam::sdk::log_level::info);
          configs.push_back(config);
        } catch (const std::exception &e) {
          VIAM_SDK_LOG(error)
              << "[discover_resources] Failed to access device at index " << i
              << ": " << e.what()
              << "Device may be in an invalid state or have firmware "
                 "compatibility issues.";
          std::ostringstream error_msg;
          error_msg << "Device " << i << ": " << e.what();
          device_errors.push_back(error_msg.str());
          // Continue to try other devices
          continue;
        }
      }

      // If we found at least one valid device, return those configs
      if (!configs.empty()) {
        if (!device_errors.empty()) {
          VIAM_SDK_LOG(warn) << "[discover_resources] Successfully discovered "
                             << configs.size() << " device(s), but "
                             << device_errors.size() << " device(s) failed";
          for (const auto &err : device_errors) {
            VIAM_SDK_LOG(error)
                << "[discover_resources] Failed device: " << err;
          }
        }
        VIAM_SDK_LOG(info) << "[discover_resources] Successfully created "
                           << configs.size() << " configs";
        return configs;
      }

      // If all devices failed, throw an error with details
      if (!device_errors.empty()) {
        std::ostringstream error_msg;
        error_msg << "RealSense device(s) detected but inaccessible. "
                  << "Errors: ";
        for (size_t i = 0; i < device_errors.size(); i++) {
          if (i > 0)
            error_msg << "; ";
          error_msg << device_errors[i];
        }
        error_msg << ". Device(s) may be in an invalid state or have firmware "
                     "compatibility issues.";
        throw std::runtime_error(error_msg.str());
      }
      return configs;

    } catch (const std::exception &e) {
      std::ostringstream err;
      err << "[discover_resources]: " << e.what();
      VIAM_SDK_LOG(error) << err.str();
      throw std::runtime_error(err.str());
    }
  }

  viam::sdk::ProtoStruct
  do_command(const viam::sdk::ProtoStruct &command) override {
    VIAM_SDK_LOG(error) << "do_command not implemented";
    return viam::sdk::ProtoStruct{};
  }
};

} // namespace discovery
} // namespace realsense
