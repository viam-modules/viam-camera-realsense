#pragma once

#include "sensors.hpp"
#include "time.hpp"
#include "utils.hpp"

#include <iostream>
#include <memory>
#include <string>

#include <viam/sdk/log/logging.hpp>

#include <boost/thread/synchronized_value.hpp>
#include <librealsense2/rs.hpp>

namespace realsense {
namespace device {

// Helper macro to use resource logging with explicit logger
#define VIAM_DEVICE_LOG(logger, level) VIAM_SDK_LOG_IMPL(logger, level)

/********************** UTILITIES ************************/

template <typename SensorT>
void enableGlobalTimestamp(SensorT &sensor, viam::sdk::LogSource &logger) {
  if (sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) {
    try {
      auto sensor_type = sensors::get_sensor_type(sensor, logger);
      if (sensor_type == sensors::SensorType::unknown) {
        throw std::runtime_error("Unknown sensor type");
      }
      sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0);
      VIAM_DEVICE_LOG(logger, info)
          << "[enableGlobalTimestamp] Enabled Global Timestamp for sensor: "
          << sensors::sensor_type_to_string(sensor_type, logger);
    } catch (const std::exception &e) {
      VIAM_DEVICE_LOG(logger, error)
          << "[enableGlobalTimestamp] Failed to enable Global Timestamp: "
          << e.what();
    }
  }
}

template <typename SensorT>
void disableAutoExposurePriority(SensorT &sensor,
                                 viam::sdk::LogSource &logger) {
  try {
    auto sensor_type = sensors::get_sensor_type(sensor, logger);
    if (sensor_type == sensors::SensorType::unknown) {
      throw std::runtime_error("Unknown sensor type");
    }
    // Only disable auto-exposure priority for color sensors
    if (sensor_type != sensors::SensorType::color) {
      return;
    }
    // CRITICAL: Disable auto-exposure priority to maintain frame sync.
    // When enabled, RGB sensor drops frames to adjust exposure, breaking
    // temporal alignment with depth (causes 5-20ms timestamp drift).
    // Tradeoff: slightly worse exposure in changing light conditions,
    // but tight temporal sync (mostly <5ms) between color and depth.
    if (sensor.supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY)) {
      try {
        // Disable auto-exposure priority to ensure constant FPS
        sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0.0);
        VIAM_DEVICE_LOG(logger, info)
            << "[disableAutoExposurePriority] Disabled Auto-Exposure Priority "
               "(constant FPS) for color sensor";
      } catch (const std::exception &e) {
        VIAM_DEVICE_LOG(logger, warn) << "[disableAutoExposurePriority] Failed "
                                         "to disable Auto-Exposure Priority: "
                                      << e.what();
      }
    }
  } catch (const std::exception &e) {
    VIAM_DEVICE_LOG(logger, error)
        << "[disableAutoExposurePriority] Failed to get sensor type: "
        << e.what();
  }
}

template <typename DeviceT>
std::optional<std::string> getCameraModel(std::shared_ptr<DeviceT> dev) {
  if (not dev->supports(RS2_CAMERA_INFO_NAME)) {
    return std::nullopt;
  }
  std::string camera_info = dev->get_info(RS2_CAMERA_INFO_NAME);
  // Model is the 3rd word in this string
  std::istringstream iss(camera_info);
  std::string word;
  int word_count = 0;
  while (iss >> word) {
    word_count++;
    if (word_count == 3) {
      return word;
    }
  }
  return std::nullopt;
}

template <typename DeviceT>
void printDeviceInfo(DeviceT const &dev, viam::sdk::LogSource &logger) {
  try {
    std::stringstream info;
    if (dev.supports(RS2_CAMERA_INFO_NAME)) {
      info << "DeviceInfo:\n"
           << "  Name:                           "
           << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
      info << "  Serial Number:                  "
           << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_PRODUCT_LINE)) {
      info << "  Product Line:                   "
           << dev.get_info(RS2_CAMERA_INFO_PRODUCT_LINE) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_PRODUCT_ID)) {
      info << "  Product ID:                      "
           << dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)) {
      info << "  USB Type Descriptor:            "
           << dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_FIRMWARE_VERSION)) {
      info << "  Firmware Version:               "
           << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION)) {
      info << "  Recommended Firmware Version:   "
           << dev.get_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION)
           << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID)) {
      info << "  Firmware Update ID:   "
           << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_PHYSICAL_PORT)) {
      info << "  Physical Port:   "
           << dev.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_DEBUG_OP_CODE)) {
      info << "  Debug OP Code:   "
           << dev.get_info(RS2_CAMERA_INFO_DEBUG_OP_CODE) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_ADVANCED_MODE)) {
      info << "  Advanced Mode:   "
           << dev.get_info(RS2_CAMERA_INFO_ADVANCED_MODE) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_CAMERA_LOCKED)) {
      info << "  Camera Locked:   "
           << dev.get_info(RS2_CAMERA_INFO_CAMERA_LOCKED) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER)) {
      info << "  ASIC Serial Number:             "
           << dev.get_info(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_DFU_DEVICE_PATH)) {
      info << "  DFU Device Path:            "
           << dev.get_info(RS2_CAMERA_INFO_DFU_DEVICE_PATH) << std::endl;
    }
    if (dev.supports(RS2_CAMERA_INFO_IP_ADDRESS)) {
      info << "  IP Address:            "
           << dev.get_info(RS2_CAMERA_INFO_IP_ADDRESS) << std::endl;
    }
    VIAM_DEVICE_LOG(logger, info) << info.str();

    // Warn about USB 2.x connections — depth streaming requires USB 3.x
    if (dev.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)) {
      std::string usb_type = dev.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
      if (!usb_type.empty() && usb_type[0] != '3') {
        VIAM_DEVICE_LOG(logger, warn)
            << "[printDeviceInfo] Device is connected via USB " << usb_type
            << ". USB 3.x is recommended for depth streaming. "
               "Depth frames may be unavailable or unreliable at USB 2.x "
               "speeds.";
      }
    }

    // Warn about outdated firmware
    if (dev.supports(RS2_CAMERA_INFO_FIRMWARE_VERSION) &&
        dev.supports(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION)) {
      std::string fw = dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
      std::string recommended_fw =
          dev.get_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION);
      if (fw != recommended_fw) {
        VIAM_DEVICE_LOG(logger, warn)
            << "[printDeviceInfo] Device firmware (" << fw
            << ") differs from recommended version (" << recommended_fw
            << "). Consider updating firmware to avoid stability issues.";
      }
    }

    // Note if camera is locked (advanced mode presets cannot be changed)
    if (dev.supports(RS2_CAMERA_INFO_CAMERA_LOCKED)) {
      std::string locked = dev.get_info(RS2_CAMERA_INFO_CAMERA_LOCKED);
      if (locked == "YES") {
        VIAM_DEVICE_LOG(logger, info)
            << "[printDeviceInfo] Camera is locked. Advanced mode JSON "
               "presets (depth table tuning, etc.) cannot be modified. "
               "This does not affect normal operation.";
      }
    }
  } catch (const std::exception &e) {
    VIAM_DEVICE_LOG(logger, error)
        << "[printDeviceInfo] Failed to retrieve device info with error: "
        << e.what()
        << ". Device may be in an invalid state or have firmware compatibility "
           "issues.";
  }
}

/********************** CALLBACKS ************************/

template <typename FrameT, typename FrameSetT, typename ViamConfigT>
void frameCallback(
    FrameT const &frame, std::uint64_t const maxFrameAgeMs,
    std::shared_ptr<boost::synchronized_value<FrameSetT>> &frame_set_,
    ViamConfigT const &viamConfig) {
  // With callbacks, all synchronized stream will arrive in a single
  // frameset
  int expected_frame_count = viamConfig.sensors.size();
  auto initial_frameset = frame.template as<FrameSetT>();
  if (not initial_frameset or initial_frameset.size() != expected_frame_count) {
    std::cerr << "[frame_callback] got count other than "
              << expected_frame_count << ": " << initial_frameset.size()
              << std::endl;
    return;
  }

  // Align the frameset if align-to-color is enabled (which it is by default in
  // our config)
  // auto frameset = align ? align->process(initial_frameset) :
  // initial_frameset;
  auto frameset = initial_frameset;

  double nowMs = time::getNowMs();
  auto color_frame = frameset.get_color_frame();
  if (not color_frame and
      utils::contains(sensors::SensorType::color, viamConfig.sensors)) {
    std::cerr << "[frame_callback] no color frame" << std::endl;
    return;
  }
  if (color_frame and
      not utils::contains(sensors::SensorType::color, viamConfig.sensors)) {
    std::cerr << "[frame_callback] received color frame when not expected"
              << std::endl;
    return;
  }

  if (color_frame) {
    double colorAge = nowMs - color_frame.get_timestamp();
    if (colorAge > maxFrameAgeMs) {
      std::cerr << "[frame_callback] received color frame is too stale, age: "
                << colorAge << "ms" << std::endl;
    }
  }

  auto depth_frame = frameset.get_depth_frame();
  if (not depth_frame and
      utils::contains(sensors::SensorType::depth, viamConfig.sensors)) {
    std::cerr << "[frame_callback] no depth frame" << std::endl;
    return;
  }
  if (depth_frame and
      not utils::contains(sensors::SensorType::depth, viamConfig.sensors)) {
    std::cerr << "[frame_callback] received depth frame when not expected"
              << std::endl;
    return;
  }
  if (depth_frame) {
    double depthAge = nowMs - depth_frame.get_timestamp();
    if (depthAge > maxFrameAgeMs) {
      std::cerr << "[frame_callback] received depth frame is too stale, age: "
                << depthAge << "ms" << std::endl;
    }
  }

  frame_set_ = std::make_shared<boost::synchronized_value<FrameSetT>>(frameset);
}

/************************ STREAM PROFILES ************************/

template <typename VideoStreamProfileT>
bool checkIfMatchingColorDepthProfiles(const VideoStreamProfileT &color,
                                       const VideoStreamProfileT &depth,
                                       viam::sdk::LogSource &logger) noexcept {
  if (std::tuple(color.width(), color.height(), color.fps()) ==
      std::tuple(depth.width(), depth.height(), depth.fps())) {
    VIAM_DEVICE_LOG(logger, info)
        << "using width: " << color.width() << " height: " << color.height()
        << " fps: " << color.fps();
    return true;
  }
  return false;
}

template <typename SensorT> class SensorTypeTraits;

template <> class SensorTypeTraits<rs2::color_sensor> {
public:
  static constexpr rs2_stream stream_type = RS2_STREAM_COLOR;
  static constexpr rs2_format format_type = RS2_FORMAT_RGB8;
};

template <> class SensorTypeTraits<rs2::depth_sensor> {
public:
  static constexpr rs2_stream stream_type = RS2_STREAM_DEPTH;
  static constexpr rs2_format format_type = RS2_FORMAT_Z16;
};

template <typename DeviceT, typename ConfigT, typename SensorT,
          typename VideoStreamProfileT, typename ViamConfigT>
std::shared_ptr<ConfigT>
createSingleSensorConfig(std::shared_ptr<DeviceT> dev,
                         ViamConfigT const &viamConfig,
                         viam::sdk::LogSource &logger) {

  VIAM_DEVICE_LOG(logger, info)
      << "[createSingleSensorConfig] Creating config for single "
         "sensor: "
      << SensorTypeTraits<SensorT>::stream_type;
  auto cfg = std::make_shared<ConfigT>();

  VIAM_DEVICE_LOG(logger, info)
      << "[createSingleSensorConfig] Querying sensors";
  // Query all sensors for the device
  auto sensors = dev->query_sensors();

  typename decltype(sensors)::value_type sensor;
  for (auto &s : sensors) {
    if (s.template is<SensorT>()) {
      sensor = s;
      enableGlobalTimestamp(sensor, logger);
    }
  }

  VIAM_DEVICE_LOG(logger, info)
      << "[createSingleSensorConfig] Got sensor, getting stream profiles";
  // Get stream profiles
  auto profiles = sensor.get_stream_profiles();

  VIAM_DEVICE_LOG(logger, info)
      << "[createSingleSensorConfig] Got " << profiles.size()
      << " stream profiles, looking for matches";
  // Find matching profiles
  for (auto &cp : profiles) {
    auto csp = cp.template as<VideoStreamProfileT>();
    if (csp.format() != SensorTypeTraits<SensorT>::format_type) {
      continue;
    }

    if ((not viamConfig.width) or
        (viamConfig.width == csp.width()) and
            (not viamConfig.height or (viamConfig.height == csp.height()))) {
      VIAM_DEVICE_LOG(logger, info)
          << "[createSingleSensorConfig] Found matching "
             "stream profile, enabling";
      cfg->enable_stream(SensorTypeTraits<SensorT>::stream_type,
                         csp.stream_index(), csp.width(), csp.height(),
                         csp.format(), csp.fps());
      VIAM_DEVICE_LOG(logger, info)
          << "[createSingleSensorConfig] enabled stream";
      return cfg;
    }
  }
  return nullptr;
}
// create a config for software depth-to-color alignment
template <typename DeviceT, typename ConfigT, typename ColorSensorT,
          typename DepthSensorT, typename VideoStreamProfileT,
          typename ViamConfigT>
std::shared_ptr<ConfigT> createSwD2CAlignConfig(std::shared_ptr<DeviceT> dev,
                                                ViamConfigT const &viamConfig,
                                                viam::sdk::LogSource &logger) {
  auto cfg = std::make_shared<ConfigT>();

  // Query all sensors for the device
  auto sensors = dev->query_sensors();

  typename decltype(sensors)::value_type color_sensor;
  typename decltype(sensors)::value_type depth_sensor;
  for (auto &s : sensors) {
    enableGlobalTimestamp(s, logger);
    if (s.template is<ColorSensorT>()) {
      color_sensor = s;
      disableAutoExposurePriority(color_sensor, logger);
    }
    if (s.template is<DepthSensorT>()) {
      depth_sensor = s;
    }
  }

  // Get stream profiles
  auto color_profiles = color_sensor.get_stream_profiles();
  auto depth_profiles = depth_sensor.get_stream_profiles();

  // Find matching profiles
  for (auto &cp : color_profiles) {
    auto csp = cp.template as<VideoStreamProfileT>();
    if (csp.format() != SensorTypeTraits<ColorSensorT>::format_type) {
      continue;
    }
    for (auto &dp : depth_profiles) {
      auto dsp = dp.template as<VideoStreamProfileT>();
      if (dsp.format() != SensorTypeTraits<DepthSensorT>::format_type) {
        continue;
      }
      if (checkIfMatchingColorDepthProfiles(csp, dsp, logger) and
          ((not viamConfig.width) or (viamConfig.width == csp.width())) and
          ((not viamConfig.height) or (viamConfig.height == csp.height()))) {
        VIAM_DEVICE_LOG(logger, info)
            << "[createSwD2CAlignConfig] Found matching color "
               "and depth stream profiles";
        cfg->enable_stream(SensorTypeTraits<ColorSensorT>::stream_type,
                           csp.stream_index(), csp.width(), csp.height(),
                           csp.format(), csp.fps());
        cfg->enable_stream(SensorTypeTraits<DepthSensorT>::stream_type,
                           dsp.stream_index(), dsp.width(), dsp.height(),
                           dsp.format(), dsp.fps());
        VIAM_DEVICE_LOG(logger, info)
            << "[createSwD2CAlignConfig] enabled color and depth streams";
        return cfg;
      }
    }
  }
  return nullptr;
}
template <typename DeviceT, typename ConfigT, typename ColorSensorT,
          typename DepthSensorT, typename VideoStreamProfileT,
          typename ViamConfigT>
std::shared_ptr<ConfigT> createConfig(std::shared_ptr<DeviceT> device,
                                      ViamConfigT const &viamConfig,
                                      viam::sdk::LogSource &logger) {
  std::shared_ptr<rs2::config> config = nullptr;

  if (utils::contains(sensors::SensorType::color, viamConfig.sensors) and
      utils::contains(sensors::SensorType::depth, viamConfig.sensors)) {
    VIAM_DEVICE_LOG(logger, info)
        << "[createConfig] Creating config with color and "
           "depth sensors";
    config =
        createSwD2CAlignConfig<DeviceT, ConfigT, ColorSensorT, DepthSensorT,
                               VideoStreamProfileT, ViamConfigT>(
            device, viamConfig, logger);

  } else if (viamConfig.sensors.size() == 1 &&
             utils::contains(sensors::SensorType::color, viamConfig.sensors)) {
    VIAM_DEVICE_LOG(logger, info)
        << "[createConfig] Creating config with color sensor";
    config = createSingleSensorConfig<DeviceT, ConfigT, ColorSensorT,
                                      VideoStreamProfileT, ViamConfigT>(
        device, viamConfig, logger);
  } else if (viamConfig.sensors.size() == 1 &&
             utils::contains(sensors::SensorType::depth, viamConfig.sensors)) {
    VIAM_DEVICE_LOG(logger, info)
        << "[createConfig] Creating config with depth sensor";
    config = createSingleSensorConfig<DeviceT, ConfigT, DepthSensorT,
                                      VideoStreamProfileT, ViamConfigT>(
        device, viamConfig, logger);
  } else {
    VIAM_DEVICE_LOG(logger, error)
        << "[createConfig] Unsupported sensor configuration requested, "
           "only "
           "'color', 'depth' or both are supported";
    throw std::runtime_error("Unsupported sensor configuration requested, "
                             "only 'color', 'depth' or both are supported");
  }
  // We are not currently supporting only depth sensor
  if (config == nullptr) {
    VIAM_DEVICE_LOG(logger, error)
        << "[createConfig] Current device configuration not "
           "supported";
    throw std::runtime_error(
        "Current device configuration not supported, check device logs");
  }

  return config;
}

/********************** DEVICE LIFECYCLE ************************/
template <typename ViamDeviceT>
bool destroyDevice(std::shared_ptr<boost::synchronized_value<ViamDeviceT>> &dev,
                   viam::sdk::LogSource &logger) noexcept {
  try {
    if (not dev) {
      VIAM_DEVICE_LOG(logger, error)
          << "[destroyDevice] trying to destroy an unexistent device";
      return false;
    }
    { // Begin scope for device lock
      auto device = dev->synchronize();
      VIAM_DEVICE_LOG(logger, info)
          << "[destroyDevice] destroying device " << device->serial_number;

      // Stop streaming if still running
      if (device->started) {
        VIAM_DEVICE_LOG(logger, info)
            << "[destroyDevice] stopping pipe " << device->serial_number;
        device->pipe->stop();
        device->started = false;
      }

      // Clear all resources
      VIAM_DEVICE_LOG(logger, info)
          << "[destroyDevice] clearing resources " << device->serial_number;
      device->pipe.reset();
      device->device.reset();
      device->config.reset();
      device->align.reset();
      device->point_cloud_filter.reset();

      VIAM_DEVICE_LOG(logger, info)
          << "[destroyDevice] device destroyed: " << device->serial_number;
    } // End scope for device lock
    dev = nullptr;
  } catch (const std::exception &e) {
    VIAM_DEVICE_LOG(logger, error)
        << "[destroyDevice] Exception caught while destroying device: "
        << e.what();
    dev = nullptr;
    return false;
  } catch (...) {
    VIAM_DEVICE_LOG(logger, error)
        << "[destroyDevice] Unknown exception caught while destroying device";
    dev = nullptr;
    return false;
  }
  return true;
}

template <typename ViamConfigT, typename ViamDeviceT, typename DeviceT,
          typename ConfigT, typename ColorSensorT, typename DepthSensorT,
          typename VideoStreamProfileT>
std::shared_ptr<boost::synchronized_value<ViamDeviceT>>
createDevice(std::string const &serial_number, std::shared_ptr<DeviceT> dev,
             std::unordered_set<std::string> const &supported_camera_models,
             ViamConfigT const &viamConfig, viam::sdk::LogSource &logger) {
  VIAM_DEVICE_LOG(logger, info)
      << "[createDevice] creating device serial number: " << serial_number;
  auto camera_model = getCameraModel(dev);
  if (not camera_model) {
    VIAM_DEVICE_LOG(logger, error)
        << "[createDevice] Failed to register camera serial number: "
        << serial_number << " since no camera model found";
    return nullptr;
  }
  VIAM_DEVICE_LOG(logger, info)
      << "[createDevice] Found camera model: " << *camera_model;
  if (supported_camera_models.count(*camera_model) == 0) {
    VIAM_DEVICE_LOG(logger, error)
        << "[createDevice] Failed to register camera serial number: "
        << serial_number
        << " since camera model is not D435 or D435i, camera model: "
        << *camera_model;
    return nullptr;
  } else {
    VIAM_DEVICE_LOG(logger, info)
        << "[createDevice] Camera model is supported: " << *camera_model;
  }

  std::shared_ptr<ConfigT> config =
      createConfig<DeviceT, ConfigT, ColorSensorT, DepthSensorT,
                   VideoStreamProfileT, ViamConfigT>(dev, viamConfig, logger);
  if (config == nullptr) {
    VIAM_DEVICE_LOG(logger, error)
        << "[createDevice] failed to create config for device: "
        << serial_number;
    return nullptr;
  }

  VIAM_DEVICE_LOG(logger, info)
      << "[createDevice] Config created for: " << serial_number;
  auto my_dev = boost::synchronized_value<ViamDeviceT>();
  my_dev->pipe = std::make_shared<std::decay_t<decltype(*my_dev->pipe)>>();
  my_dev->device = dev;
  my_dev->serial_number = serial_number;
  my_dev->point_cloud_filter = std::make_shared<PointCloudFilter>();
  my_dev->align = std::make_shared<std::decay_t<decltype(*my_dev->align)>>(
      RS2_STREAM_COLOR);
  my_dev->config = config;

  VIAM_DEVICE_LOG(logger, info) << "[createDevice] created " << serial_number;
  return std::make_shared<boost::synchronized_value<ViamDeviceT>>(my_dev);
}

/********************** STREAMING LIFECYCLE ************************/
template <typename ViamDeviceT, typename FrameSetT, typename ViamConfigT>
void startDevice(
    std::string const &serialNumber,
    std::shared_ptr<boost::synchronized_value<ViamDeviceT>> dev,
    std::shared_ptr<boost::synchronized_value<FrameSetT>> &frameSetStorage,
    std::uint64_t const maxFrameAgeMs, ViamConfigT const &viamConfig,
    viam::sdk::LogSource &logger) {
  VIAM_DEVICE_LOG(logger, info)
      << "[startDevice] starting device " << serialNumber;
  if (not dev) {
    std::ostringstream buffer;
    buffer << "[startDevice] unable to start device " << serialNumber
           << " since "
           << "viam device wrapper does not exist";
    throw std::runtime_error(buffer.str());
  }
  { // Begin scope for dev_ptr lock
    auto dev_ptr = dev->synchronize();
    if (dev_ptr->started) {
      std::ostringstream buffer;
      buffer << "[startDevice] unable to start already started device "
             << serialNumber;
      throw std::invalid_argument(buffer.str());
    }

    dev_ptr->config->enable_device(serialNumber);
    dev_ptr->pipe->start(*dev_ptr->config, [maxFrameAgeMs, &frameSetStorage,
                                            viamConfig](auto const &frame) {
      frameCallback(frame, maxFrameAgeMs, frameSetStorage, viamConfig);
    });

    dev_ptr->started = true;
  } // End scope for dev_ptr lock
  VIAM_DEVICE_LOG(logger, info)
      << "[startDevice]  device started " << serialNumber;
}

template <typename ViamConfigT, typename ViamDeviceT, typename DeviceT,
          typename ConfigT, typename ColorSensorT, typename DepthSensorT,
          typename VideoStreamProfileT>
void reconfigureDevice(
    std::shared_ptr<boost::synchronized_value<ViamDeviceT>> dev,
    ViamConfigT const &viamConfig, viam::sdk::LogSource &logger) {
  if (dev == nullptr) {
    VIAM_DEVICE_LOG(logger, error) << "[reconfigureDevice] device is null";
    throw std::runtime_error("device is null");
  }

  { // Begin scope for device_guard lock
    auto device_guard = dev->synchronize();
    auto new_config = createConfig<DeviceT, ConfigT, ColorSensorT, DepthSensorT,
                                   VideoStreamProfileT, ViamConfigT>(
        device_guard->device, viamConfig, logger);
    if (new_config == nullptr) {
      VIAM_DEVICE_LOG(logger, error)
          << "[reconfigureDevice] failed to create new config";
      throw std::runtime_error("failed to create new config");
    }

    if (device_guard->started) {
      VIAM_DEVICE_LOG(logger, error)
          << "[reconfigureDevice] cannot reconfigure a started device";
      throw std::runtime_error("cannot reconfigure a started device");
    }

    device_guard->config = new_config;
    VIAM_DEVICE_LOG(logger, info) << "[reconfigureDevice] device reconfigured";
  } // End scope for device_guard lock
}

template <typename ViamDeviceT>
bool stopDevice(std::shared_ptr<boost::synchronized_value<ViamDeviceT>> &dev,
                viam::sdk::LogSource &logger) noexcept {
  try {
    if (not dev) {
      VIAM_DEVICE_LOG(logger, error)
          << "[stopDevice] trying to stop a device that does not exist";
      return false;
    }
    { // Begin scope for dev_ptr lock
      auto dev_ptr = dev->synchronize();
      if (not dev_ptr->started) {
        VIAM_DEVICE_LOG(logger, error)
            << "[stopDevice] unable to stop device that is not "
               "currently running "
            << dev_ptr->serial_number;
        return false;
      }

      dev_ptr->pipe->stop();
      dev_ptr->started = false;

    } // End scope for dev_ptr lock
    return true;
  } catch (const std::exception &e) {
    VIAM_DEVICE_LOG(logger, error)
        << "[stopDevice] Exception caught while stopping device: " << e.what();
  } catch (...) {
    VIAM_DEVICE_LOG(logger, error)
        << "[stopDevice] Unknown exception caught while stopping device";
  }
  return false;
}
} // namespace device
} // namespace realsense
