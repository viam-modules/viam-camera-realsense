#pragma once
#include "device.hpp"
#include "download_utils.hpp"
#include "encoding.hpp"
#include "extrinsics.hpp"
#include "firmware_update.hpp"
#include "sensors.hpp"
#include "time.hpp"
#include "utils.hpp"
#include "zip_utils.hpp"
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>

#include <librealsense2/rs.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <boost/thread/synchronized_value.hpp>
namespace realsense {
static const std::unordered_set<std::string> SUPPORTED_CAMERA_MODELS = {
    "D435", "D435I"};
static constexpr std::uint64_t MAX_FRAME_AGE_MS =
    1e3; // time until a frame is considered stale, in miliseconds (equal to 1
static constexpr size_t MAX_GRPC_MESSAGE_SIZE =
    33554432; // 32MB gRPC message size limit
static constexpr std::uint64_t MAX_FRAME_SET_TIME_DIFF_MS =
    2; // max time difference between frames in a frameset to be considered
       // simultaneous, in miliseconds (equal to 2 ms)
static constexpr std::uint64_t TIMESTAMP_WARNING_LOG_INTERVAL_MS =
    60000; // 1
           // minute

enum class DoCommand : uint8_t {
  UPDATE_FIRMWARE,
  UNKNOWN = std::numeric_limits<uint8_t>::max()
};

static const std::unordered_map<std::string, uint8_t> DoCommandMap{
    {{"update_firmware", static_cast<uint8_t>(DoCommand::UPDATE_FIRMWARE)},
     {"unknown", static_cast<uint8_t>(DoCommand::UNKNOWN)}}};

const std::string service_name = "viam_realsense";

const std::string kColorSourceName = "color";
const std::string kDepthSourceName = "depth";
const std::string kPcdMimeType = "pointcloud/pcd";

template <typename SynchronizedContextT> class Realsense;

template <typename SynchronizedContextT> class RealsenseContext {
public:
  RealsenseContext(std::shared_ptr<SynchronizedContextT> ctx)
      : rs_context_(ctx) {
    setupCallback();
  }

  auto query_devices() const {
    auto rs_context = rs_context_->synchronize();
    return rs_context->query_devices();
  }

  void addInstance(Realsense<SynchronizedContextT> *instance) {
    instances_->insert(instance);
  }

  void removeInstance(Realsense<SynchronizedContextT> *instance) {
    instances_->erase(instance);
  }

  // Clear the devices changed callback (e.g., during firmware update)
  void clearDevicesChangedCallback() {
    auto rs_context = rs_context_->synchronize();
    rs_context->set_devices_changed_callback([](rs2::event_information &) {});
  }

  // Set a custom devices changed callback (e.g., for firmware update event
  // detection)
  template <typename CallbackT>
  void setDevicesChangedCallback(CallbackT callback) {
    auto rs_context = rs_context_->synchronize();
    rs_context->set_devices_changed_callback(callback);
  }

  // Restore the default devices changed callback
  void restoreDevicesChangedCallback() { setupCallback(); }

private:
  std::shared_ptr<SynchronizedContextT> rs_context_;
  boost::synchronized_value<
      std::unordered_set<Realsense<SynchronizedContextT> *>>
      instances_{std::unordered_set<Realsense<SynchronizedContextT> *>{}};

  void setupCallback() {
    auto rs_context = rs_context_->synchronize();
    // Set the callback to notify all instances when devices change
    rs_context->set_devices_changed_callback(
        [this](rs2::event_information &info) { notifyAllInstances(info); });
  }

  void notifyAllInstances(rs2::event_information &info) {
    auto instances_guard = instances_.synchronize();
    std::vector<Realsense<SynchronizedContextT> *> failed_instances;

    for (Realsense<SynchronizedContextT> *instance : *instances_guard) {
      if (instance != nullptr) {
        try {
          instance->handleDeviceChange(info);
        } catch (const std::exception &e) {
          std::cerr << "[RealsenseContext] Error notifying instance: "
                    << e.what() << std::endl;
          // Consider whether to remove failed instances
          failed_instances.push_back(instance);
        }
      }
    }

    for (auto *failed : failed_instances) {
      instances_guard->erase(failed);
    }
  }
};

struct RsResourceConfig {
  std::string resource_name{};
  std::string serial_number{};
  std::vector<sensors::SensorType> sensors{};
  std::optional<int> width{};
  std::optional<int> height{};

  RsResourceConfig() = default;

  explicit RsResourceConfig(std::string const &serial_number,
                            std::string const &resource_name,
                            std::vector<sensors::SensorType> const &sensors,
                            std::optional<int> width = std::nullopt,
                            std::optional<int> height = std::nullopt)
      : serial_number(serial_number), resource_name(resource_name),
        sensors(sensors), width(width), height(height) {}
  sensors::SensorType getMainSensor() const {
    if (sensors.empty()) {
      throw std::invalid_argument("sensors list is empty");
    }
    return sensors[0];
  }
};

struct DeviceFunctions {
  std::function<bool(
      std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>> &,
      viam::sdk::LogSource &)>
      stopDevice;
  std::function<bool(
      std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>> &,
      viam::sdk::LogSource &)>
      destroyDevice;
  std::function<void(const rs2::device &, viam::sdk::LogSource &)>
      printDeviceInfo;
  std::function<
      std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>>(
          std::string const &, std::shared_ptr<rs2::device>,
          std::unordered_set<std::string> const &,
          realsense::RsResourceConfig const &, viam::sdk::LogSource &)>
      createDevice;
  std::function<void(
      std::string const &,
      std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>> &,
      std::shared_ptr<boost::synchronized_value<rs2::frameset>> &,
      std::uint64_t, realsense::RsResourceConfig const &,
      viam::sdk::LogSource &)>
      startDevice;
  std::function<void(
      std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>> &,
      realsense::RsResourceConfig const &, viam::sdk::LogSource &)>
      reconfigureDevice;
};

template <typename SynchronizedContextT>
class Realsense final : public viam::sdk::Camera,
                        public viam::sdk::Reconfigurable {
public:
  Realsense(viam::sdk::Dependencies deps, viam::sdk::ResourceConfig cfg,
            std::shared_ptr<RealsenseContext<SynchronizedContextT>> ctx,
            std::shared_ptr<
                boost::synchronized_value<std::unordered_set<std::string>>>
                assigned_serials)
      : Realsense(deps, cfg, ctx, createDefaultDeviceFunctions(),
                  assigned_serials) {}
  Realsense(viam::sdk::Dependencies deps, viam::sdk::ResourceConfig cfg,
            std::shared_ptr<RealsenseContext<SynchronizedContextT>> ctx,
            DeviceFunctions device_funcs,
            std::shared_ptr<
                boost::synchronized_value<std::unordered_set<std::string>>>
                assigned_serials)
      : Camera(cfg.name()), config_(configure(deps, cfg)), realsense_ctx_(ctx),
        device_funcs_(device_funcs), assigned_serials_(assigned_serials) {

    std::string requested_serial_number = config_->serial_number;
    VIAM_RESOURCE_LOG(info)
        << "[constructor] start for resource " << config_->resource_name
        << " with serial number " << requested_serial_number;

    // This will the initial set of connected devices (i.e. the devices that
    // were connected before the callback was set)
    auto device_list = realsense_ctx_->query_devices();
    VIAM_RESOURCE_LOG(info)
        << "[constructor] start for resource " << config_->resource_name
        << " number of devices found: " << device_list.size();
    if (not assign_and_initialize_device(device_list)) {
      if (not requested_serial_number.empty()) {
        VIAM_RESOURCE_LOG(error) << "[constructor] failed to start device "
                                 << requested_serial_number;
        throw std::runtime_error("failed to start device " +
                                 requested_serial_number);
      } else {
        VIAM_RESOURCE_LOG(error) << "[constructor] failed to start a device";
        throw std::runtime_error("failed to start a device");
      }
    }
    realsense_ctx_->addInstance(this);
    physical_camera_assigned_ = true;

    VIAM_RESOURCE_LOG(info)
        << "Realsense constructor end " << requested_serial_number;
  }
  ~Realsense() {
    if (device_) {
      { // Begin scope for device_guard lock
        auto device_guard = device_->synchronize();
        VIAM_RESOURCE_LOG(info) << "[destructor] Realsense destructor start "
                                << device_guard->serial_number;
        { // Begin scope for serials_guard lock
          auto serials_guard = assigned_serials_->synchronize();
          serials_guard->erase(device_guard->serial_number);
        } // End scope for serials_guard lock
      } // End scope for device_guard lock
      realsense_ctx_->removeInstance(this);
    }

    // Now call stopDevice and destroyDevice (these will lock internally)
    device_funcs_.stopDevice(device_, this->logger_);
    device_funcs_.destroyDevice(device_, this->logger_);
    VIAM_RESOURCE_LOG(info) << "[destructor] Realsense destructor end";
  }
  void reconfigure(const viam::sdk::Dependencies &deps,
                   const viam::sdk::ResourceConfig &cfg) override {
    VIAM_RESOURCE_LOG(info) << "[reconfigure] reconfigure start";
    if (not physical_camera_assigned_) {
      VIAM_RESOURCE_LOG(error)
          << "[reconfigure] cannot reconfigure a device that "
             "does not have a physical device assigned";
      throw std::runtime_error("cannot reconfigure a device that does not have "
                               "a physical device assigned");
    }
    if (not device_) {
      VIAM_RESOURCE_LOG(error) << "[reconfigure] device is null";
      throw std::runtime_error("device is null");
    }

    std::string prev_serial_number;
    { // Begin scope for device_guard lock
      auto device_guard = device_->synchronize();
      prev_serial_number = device_guard->serial_number;
    } // End scope for device_guard lock

    VIAM_RESOURCE_LOG(error)
        << "[reconfigure] stopping device " << prev_serial_number;
    if (not device_funcs_.stopDevice(device_, this->logger_)) {
      VIAM_RESOURCE_LOG(error)
          << "[reconfigure] failed to stop device " << prev_serial_number;
      throw std::runtime_error("failed to stop device " + prev_serial_number);
    }

    config_ = configure(deps, cfg);
    auto device_list = realsense_ctx_->query_devices();

    /*
    If the user explicitly set a serial number, and it is different from the
    serial number of our current physical device, we need to destroy the
    previous device and create a new one.

    In all other cases (same serial number, or no serial number set), we reuse
    the existing device.
    */
    std::string requested_serial_number = config_->serial_number;
    if (not requested_serial_number.empty() and
        prev_serial_number != requested_serial_number) {
      if (device_) {
        { // Begin scope for device_guard lock
          auto device_guard = device_->synchronize();
          { // Begin scope for serials_guard lock
            auto serials_guard = assigned_serials_->synchronize();
            serials_guard->erase(device_guard->serial_number);
          } // End scope for serials_guard lock
        } // End scope for device_guard lock
      }
      VIAM_RESOURCE_LOG(error)
          << "[reconfigure] destroying device " << prev_serial_number;
      if (not device_funcs_.destroyDevice(device_, this->logger_)) {
        VIAM_RESOURCE_LOG(error)
            << "[reconfigure] failed to destroy device " << prev_serial_number;
        throw std::runtime_error("failed to destroy device " +
                                 prev_serial_number);
      }
      physical_camera_assigned_ = false;

      if (not assign_and_initialize_device(device_list)) {
        VIAM_RESOURCE_LOG(error) << "[reconfigure] failed to start device "
                                 << config_->serial_number;
        throw std::runtime_error("failed to start device " +
                                 config_->serial_number);
      }
      physical_camera_assigned_ = true;
    } else {
      realsense::RsResourceConfig config_copy = config_.get();
      VIAM_RESOURCE_LOG(info)
          << "[reconfigure] same serial number, reusing device "
          << prev_serial_number;
      device_funcs_.reconfigureDevice(device_, config_copy, this->logger_);
      device_funcs_.startDevice(config_copy.serial_number, device_,
                                latest_frameset_, MAX_FRAME_AGE_MS, config_copy,
                                this->logger_);
    }

    VIAM_RESOURCE_LOG(info) << "[reconfigure] Realsense reconfigure end";
  }

  DoCommand get_do_command(const viam::sdk::ProtoStruct &command) const {
    for (auto const &proto_command : command) {
      if (DoCommandMap.count(proto_command.first)) {
        return static_cast<DoCommand>(DoCommandMap.at(proto_command.first));
      }
    }
    return DoCommand::UNKNOWN;
  }

  viam::sdk::ProtoStruct
  do_command(const viam::sdk::ProtoStruct &command) override {
    VIAM_RESOURCE_LOG(info)
        << "[do_command] Received do_command, waiting for lock...";

    // Only 1 command is allowed at a time
    if (command.size() > 1) {
      viam::sdk::ProtoStruct response;
      response["success"] = false;
      response["error"] =
          "Firmware update command must contain exactly one parameter";
      VIAM_RESOURCE_LOG(error)
          << "[do_command] Invalid firmware update command: contains "
          << command.size() << " parameters, expected 1";
      return response;
    }

    // Acquire lock to prevent concurrent do_command calls
    // This is especially important for long-running operations like firmware
    // updates If this blocks, another do_command is currently executing
    std::lock_guard<std::mutex> lock(do_command_mutex_);

    VIAM_RESOURCE_LOG(info) << "[do_command] Lock acquired, processing command";
    DoCommand do_command = get_do_command(command);

    try {
      // Check if command contains "firmware_update"
      if (do_command == DoCommand::UPDATE_FIRMWARE) {
        VIAM_RESOURCE_LOG(info) << "[do_command] Received update_firmware";
#ifdef __APPLE__
        // Firmware update is not supported on macOS yet
        viam::sdk::ProtoStruct response;
        response["success"] = false;
        response["error"] = "Firmware update is not supported on macOS";
        VIAM_RESOURCE_LOG(error)
            << "[do_command] Firmware update not supported on macOS";
        return response;
#else
        // Extract the firmware URL from the command
        // Command structure: {"update_firmware": "url"} or {"update_firmware":
        // ""} Note: command.size() is already validated to be exactly 1 at the
        // top of do_command
        auto it = command.begin();
        auto const &fw_param = it->second;

        if (!fw_param.is_a<std::string>()) {
          viam::sdk::ProtoStruct response;
          response["success"] = false;
          response["error"] = "Firmware update URL must be a string";
          VIAM_RESOURCE_LOG(error) << "[do_command] Invalid firmware update "
                                      "parameter type (expected string)";
          return response;
        }

        std::string firmware_url = *fw_param.get<std::string>();
        return handleFirmwareUpdate(firmware_url);
#endif
      }

      VIAM_RESOURCE_LOG(error) << "[do_command] Unknown command";
      viam::sdk::ProtoStruct response;
      response["error"] =
          "Unknown command. Supported commands: update_firmware";
      return response;

    } catch (const std::exception &e) {
      VIAM_RESOURCE_LOG(error) << "[do_command] Error: " << e.what();
      viam::sdk::ProtoStruct response;
      response["error"] = std::string("Command failed: ") + e.what();
      return response;
    }
  }

  viam::sdk::Camera::image_collection
  get_images(std::vector<std::string> filter_source_names,
             const viam::sdk::ProtoStruct &extra) override {
    try {
      if (is_recovery_mode_.get()) {
        std::string error_msg =
            "Camera is in recovery/DFU mode and cannot stream images. "
            "Please update the firmware using do_command with firmware_update "
            "parameter.";
        VIAM_RESOURCE_LOG(error) << "[get_images] " << error_msg;
        throw std::runtime_error(error_msg);
      }

      bool should_process_color = false;
      bool should_process_depth = false;

      if (filter_source_names.empty()) {
        should_process_color = true;
        should_process_depth = true;
      } else {
        for (const auto &name : filter_source_names) {
          if (name == kColorSourceName) {
            should_process_color = true;
          }
          if (name == kDepthSourceName) {
            should_process_depth = true;
          }
        }
      }

      if (not latest_frameset_) {
        VIAM_RESOURCE_LOG(error) << "[get_images] no frameset available";
        throw std::runtime_error("no frameset available");
      }
      VIAM_RESOURCE_LOG(debug) << "[get_images] start";
      std::string serial_number = config_->serial_number;
      auto fs = latest_frameset_->get();

      std::vector<sensors::SensorType> sensors = config_->sensors;

      viam::sdk::Camera::image_collection response;
      for (auto const &sensor : sensors) {
        if (sensor == sensors::SensorType::color) {
          if (!should_process_color) {
            continue;
          }
          auto color = fs.get_color_frame();
          response.images.emplace_back(
              encoding::encodeVideoFrameToResponse(color));
          std::uint64_t timestamp =
              static_cast<std::uint64_t>(std::llround(color.get_timestamp()));

          std::chrono::milliseconds latestTimestamp(timestamp);
          response.metadata.captured_at = viam::sdk::time_pt{
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  latestTimestamp)};
        } else if (sensor == sensors::SensorType::depth) {
          if (!should_process_depth) {
            continue;
          }
          auto depth = fs.get_depth_frame();
          response.images.emplace_back(
              encoding::encodeDepthFrameToResponse(depth));
          std::uint64_t timestamp =
              static_cast<std::uint64_t>(std::llround(depth.get_timestamp()));

          std::chrono::milliseconds latestTimestamp(timestamp);
          response.metadata.captured_at = viam::sdk::time_pt{
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  latestTimestamp)};
        } else {
          throw std::invalid_argument(
              "unknown sensor: " +
              sensors::sensor_type_to_string(sensor, this->logger_));
        }
      }

      // if both color and depth are requested, use the older timestamp of the
      // two
      if (utils::contains(sensors::SensorType::color, sensors) and
          utils::contains(sensors::SensorType::depth, sensors) and
          should_process_color and should_process_depth) {
        auto const color = fs.get_color_frame();
        auto const depth = fs.get_depth_frame();
        auto const timeDiffMs = static_cast<std::uint64_t>(std::llround(
            std::abs(color.get_timestamp() - depth.get_timestamp())));
        auto const colorTS =
            static_cast<std::uint64_t>(std::llround(color.get_timestamp()));
        auto const depthTS =
            static_cast<std::uint64_t>(std::llround(depth.get_timestamp()));
        // log if the timestamps differ more than MAX_FRAME_SET_TIME_DIFF_MS,
        // at most once every TIMESTAMP_WARNING_LOG_INTERVAL_MS at warning
        // level and always at debug level
        if (timeDiffMs > MAX_FRAME_SET_TIME_DIFF_MS) {
          std::uint64_t now_ms = time::getNowMs();
          bool should_warn = false;
          {
            auto guard = last_timestamp_warning_log_time_ms_.synchronize();
            if (now_ms - *guard > TIMESTAMP_WARNING_LOG_INTERVAL_MS) {
              *guard = now_ms;
              should_warn = true;
            }
          }

          if (should_warn) {
            VIAM_RESOURCE_LOG(warn)
                << "color and depth timestamps differ by " << timeDiffMs
                << "ms, using older timestamp. "
                   "(This warning throttled to once per "
                << TIMESTAMP_WARNING_LOG_INTERVAL_MS / 1000
                << "s; see debug for all"
                   " occurrences)";
          } else {
            VIAM_RESOURCE_LOG(debug)
                << "color and depth timestamps differ by " << timeDiffMs
                << "ms, using older timestamp. color: " << colorTS
                << "ms depth: " << depthTS << "ms";
          }
        }

        // use the older of the two timestamps
        std::uint64_t timestamp = std::min(depthTS, colorTS);

        std::chrono::milliseconds latestTimestamp(timestamp);
        response.metadata.captured_at = viam::sdk::time_pt{
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                latestTimestamp)};
      }

      VIAM_RESOURCE_LOG(debug) << "[get_images] end";
      return response;
    } catch (const std::exception &e) {
      VIAM_RESOURCE_LOG(error) << "[get_images] error: " << e.what();
      throw std::runtime_error("failed to create images: " +
                               std::string(e.what()));
    }
  }
  viam::sdk::Camera::point_cloud
  get_point_cloud(std::string mime_type,
                  const viam::sdk::ProtoStruct &extra) override {
    try {
      if (is_recovery_mode_.get()) {
        std::string error_msg =
            "Camera is in recovery/DFU mode and cannot stream point clouds. "
            "Please update the firmware using do_command with firmware_update "
            "parameter.";
        VIAM_RESOURCE_LOG(error) << "[get_point_cloud] " << error_msg;
        throw std::runtime_error(error_msg);
      }
      if (not latest_frameset_) {
        VIAM_RESOURCE_LOG(error) << "[get_point_cloud] no frameset available";
        throw std::runtime_error("no frameset available");
      }
      VIAM_RESOURCE_LOG(debug) << "[get_point_cloud] start";
      auto fs = latest_frameset_->get();

      double nowMs = time::getNowMs();

      rs2::video_frame color_frame = fs.get_color_frame();
      if (not color_frame) {
        throw std::invalid_argument("no color frame");
      }

      time::throwIfTooOld(nowMs, color_frame.get_timestamp(), MAX_FRAME_AGE_MS,
                          "no recent color frame: check USB connection");

      rs2::depth_frame depth_frame = fs.get_depth_frame();
      if (not depth_frame) {
        throw std::invalid_argument("no depth frame");
      }
      time::throwIfTooOld(nowMs, depth_frame.get_timestamp(), MAX_FRAME_AGE_MS,
                          "no recent depth frame: check USB connection");

      if (color_frame.get_data() == nullptr or
          color_frame.get_data_size() == 0) {
        throw std::runtime_error("[get_image] color data is null");
      }

      if (depth_frame.get_data() == nullptr or
          depth_frame.get_data_size() == 0) {
        throw std::runtime_error("[get_point_cloud] depth data is null");
      }

      if (not device_) {
        VIAM_RESOURCE_LOG(error) << "[get_point_cloud] no device available";
        throw std::runtime_error("no device available");
      }
      std::vector<std::uint8_t> data;
      { // Begin scope for my_dev lock
        auto my_dev = device_->synchronize();

        if (not my_dev->started) {
          throw std::runtime_error("device is not started");
        }

        data = encoding::encodeRGBPointsToPCD(
            my_dev->point_cloud_filter->process(fs), this->logger_);
      } // End scope for my_dev lock

      if (data.size() > MAX_GRPC_MESSAGE_SIZE) {
        VIAM_RESOURCE_LOG(error)
            << "[get_point_cloud] data size exceeds gRPC message size limit";
        throw std::runtime_error("point cloud size " +
                                 std::to_string(data.size()) +
                                 " exceeds gRPC message size limit of " +
                                 std::to_string(MAX_GRPC_MESSAGE_SIZE));
      }

      VIAM_RESOURCE_LOG(debug) << "[get_point_cloud] end";
      return viam::sdk::Camera::point_cloud{kPcdMimeType, data};
    } catch (const std::exception &e) {
      VIAM_RESOURCE_LOG(error) << "[get_point_cloud] error: " << e.what();
      throw std::runtime_error("failed to create pointcloud: " +
                               std::string(e.what()));
    }
  }
  viam::sdk::Camera::properties get_properties() override {
    try {
      VIAM_RESOURCE_LOG(debug) << "[get_properties] start";
      if (not device_) {
        VIAM_RESOURCE_LOG(error) << "[get_properties] no device available";
        throw std::runtime_error("no device available");
      }
      auto fillResp = [this](viam::sdk::Camera::properties &p,
                             rs2_intrinsics const &props,
                             const rs2::stream_profile &stream,
                             const rs2::stream_profile &ref_stream) {
        p.supports_pcd = true;
        p.intrinsic_parameters.width_px = props.width;
        p.intrinsic_parameters.height_px = props.height;
        p.intrinsic_parameters.focal_x_px = props.fx;
        p.intrinsic_parameters.focal_y_px = props.fy;
        p.intrinsic_parameters.center_x_px = props.ppx;
        p.intrinsic_parameters.center_y_px = props.ppy;

        // Calculate extrinsics from stream to reference stream
        p.extrinsic_parameters =
            realsense::extrinsics::get_extrinsics(stream, ref_stream);
        /*
       Disabling distortion parameters for now, when this is reenabled, we need
       to make sure that get_properties works well through the SDK. A way to do
       this is to create a python script and query camera.get_properties and
       make sure it doesn't throw an error. This is related to this ticker:
       https://viam.atlassian.net/browse/RSDK-12408

       Which errors when creatng a new distorter here:
       https://github.com/viamrobotics/rdk/blob/062f15b372240c332fa309760da8f607c5af6c9a/components/camera/client.go#L315

       There is another fundamental aspect to this, the distorer presumably
       distorts images using the distortion models supperted here:
       https://github.com/viamrobotics/rdk/blob/e97069d07515d7e5961ba5ac2ef660619a2d6dda/rimage/transform/distorter.go#L10
       Tghe consists of BrownConradyDistortionType and
       KannalaBrandtDistortionType. But realsense reports a InverseBrownConrady
       as its distortion model, which aparently does the inverse operation, take
       a distorted image and undistort it. We need to figure out how to handle
       this.

        */
        // p.distortion_parameters.model =
        // rs2_distortion_to_string(props.model); for (auto const &coeff :
        // props.coeffs)
        //   p.distortion_parameters.parameters.push_back(coeff);

        // std::stringstream coeffs_stream;
        // for (size_t i = 0; i < p.distortion_parameters.parameters.size();
        // ++i) {
        //   if (i > 0)
        //     coeffs_stream << ", ";
        //   coeffs_stream << p.distortion_parameters.parameters[i];
        // }

        VIAM_RESOURCE_LOG(debug)
            << "[get_properties] properties: ["
            << "width: " << p.intrinsic_parameters.width_px << ", "
            << "height: " << p.intrinsic_parameters.height_px << ", "
            << "focal_x: " << p.intrinsic_parameters.focal_x_px << ", "
            << "focal_y: " << p.intrinsic_parameters.focal_y_px << ", "
            << "center_x: " << p.intrinsic_parameters.center_x_px << ", "
            << "center_y: " << p.intrinsic_parameters.center_y_px << "]";
        // << "distortion_model: " << p.distortion_parameters.model << ", "
        // << "distortion_coeffs: [" << coeffs_stream.str() << "]" << "]";
      };
      rs2_intrinsics props;
      viam::sdk::Camera::properties response{};
      { // Begin scope for my_dev lock
        auto my_dev = device_->synchronize();
        std::string serial_number = my_dev->serial_number;
        if (not my_dev->started or not my_dev->pipe) {
          std::ostringstream buffer;
          buffer << service_name << ": device with serial number "
                 << serial_number << " is not longer started";
          throw std::invalid_argument(buffer.str());
        }

        auto profile = my_dev->pipe->get_active_profile();
        auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH)
                                .as<rs2::video_stream_profile>();
        auto color_stream = profile.get_stream(RS2_STREAM_COLOR)
                                .as<rs2::video_stream_profile>();

        if (config_->getMainSensor() == sensors::SensorType::color) {
          if (not color_stream) {
            throw std::runtime_error("color stream is not available");
          }
          auto props = color_stream.get_intrinsics();
          auto ref_stream = depth_stream ? depth_stream : color_stream;
          fillResp(response, props, color_stream, ref_stream);
        } else if (config_->getMainSensor() == sensors::SensorType::depth) {
          if (not depth_stream) {
            throw std::runtime_error("depth stream is not available");
          }
          auto props = depth_stream.get_intrinsics();
          auto ref_stream = color_stream ? color_stream : depth_stream;
          fillResp(response, props, depth_stream, ref_stream);
        }
      } // End scope for my_dev lock

      VIAM_RESOURCE_LOG(debug) << "[get_properties] end";
      return response;
    } catch (const std::exception &e) {
      VIAM_RESOURCE_LOG(error) << "[get_properties] error: " << e.what();
      throw std::runtime_error("failed to create properties: " +
                               std::string(e.what()));
    }
  }
  std::vector<viam::sdk::GeometryConfig>
  get_geometries(const viam::sdk::ProtoStruct &extra) override {
    // This is the geometry for the D435 and D435i, the only models that we
    // currently support. See
    // https://github.com/viam-modules/viam-camera-realsense/pull/75 for
    // explanation of values. NOTE: If support for additional RealSense camera
    // models is added, update method accordingly.
    return {viam::sdk::GeometryConfig(viam::sdk::pose{-17.5, 0, -12.5},
                                      viam::sdk::box({90, 25, 25}), "box")};
  }

  static std::vector<std::string> validate(viam::sdk::ResourceConfig cfg) {
    auto attrs = cfg.attributes();

    // If sensors is provided, validate it, if not, we'll enable both color and
    // depth by default
    if (attrs.count("sensors")) {
      VIAM_SDK_LOG(info) << "[validate] Validating sensors config";
      if (not attrs["sensors"].is_a<viam::sdk::ProtoList>()) {
        VIAM_SDK_LOG(error) << "[validate] sensors is not a list";
        throw std::invalid_argument("sensors must be a list");
      }
      auto sensors_proto =
          attrs["sensors"].get_unchecked<viam::sdk::ProtoList>();

      if (sensors_proto.size() == 0 or sensors_proto.size() > 2) {
        VIAM_SDK_LOG(error)
            << "[validate] sensors field must contain 1 or 2 elements";
        throw std::invalid_argument(
            "sensors field must contain 1 or 2 elements");
      }
      if (sensors_proto.size() == 1) {
        VIAM_SDK_LOG(info) << "[validate] Validating that sensors is a string";
        if (not sensors_proto[0].is_a<std::string>()) {
          VIAM_SDK_LOG(error) << "[validate] sensors is not a string";
          throw std::invalid_argument("sensors must be a string");
        }
        std::string sensors_param =
            sensors_proto[0].get_unchecked<std::string>();
        if (sensors_param != "color" and sensors_param != "depth") {
          VIAM_SDK_LOG(error)
              << "[validate] sensors must be either 'color' or 'depth'";
          throw std::invalid_argument(
              "sensors must be either 'color' or 'depth'");
        }
      }
      if (sensors_proto.size() == 2) {
        VIAM_SDK_LOG(info) << "[validate] Validating that sensors is a string";
        if (not sensors_proto[0].is_a<std::string>() or
            not sensors_proto[1].is_a<std::string>()) {
          VIAM_SDK_LOG(error) << "[validate] sensors is not a string";
          throw std::invalid_argument("sensors must be a string");
        }
        std::string sensors_param_1 =
            sensors_proto[0].get_unchecked<std::string>();
        std::string sensors_param_2 =
            sensors_proto[1].get_unchecked<std::string>();
        if ((sensors_param_1 != "color" and sensors_param_1 != "depth") or
            (sensors_param_2 != "color" and sensors_param_2 != "depth")) {
          VIAM_SDK_LOG(error)
              << "[validate] sensors must be either 'color' or 'depth'";
          throw std::invalid_argument(
              "sensors must be either 'color' or 'depth'");
        }
        if (sensors_param_1 == sensors_param_2) {
          VIAM_SDK_LOG(error) << "[validate] sensors cannot contain duplicates";
          throw std::invalid_argument("sensors cannot contain duplicates");
        }
      }
    } else {
      VIAM_SDK_LOG(info) << "[validate] sensors not specified, will default to "
                            "[\"color\", \"depth\"]";
    }

    if (attrs.count("serial_number")) {
      VIAM_SDK_LOG(info)
          << "[validate] Validating that serial_number is a string";
      if (not attrs["serial_number"].is_a<std::string>()) {
        std::string error_message = "serial_number is not a string";
        VIAM_SDK_LOG(error) << "[validate] " << error_message;
        throw std::invalid_argument(error_message);
      }

      std::string serial = attrs["serial_number"].get_unchecked<std::string>();
      if (serial.empty()) {
        VIAM_SDK_LOG(info) << "[validate] serial_number is empty, "
                              "first available device will be used";
      }
    } else {
      VIAM_SDK_LOG(info) << "[validate] serial_number not specified, "
                            "first available device will be used";
    }

    if (attrs.count("width_px")) {
      if (not attrs["width_px"].is_a<double>()) {
        throw std::invalid_argument("width_px must be a double");
      }
      int width = attrs["width_px"].get_unchecked<double>();
      if (width <= 0) {
        throw std::invalid_argument("width_px must be positive");
      }
    }

    if (attrs.count("height_px")) {
      if (not attrs["height_px"].is_a<double>()) {
        throw std::invalid_argument("height_px must be a double");
      }
      int height = attrs["height_px"].get_unchecked<double>();
      if (height <= 0) {
        throw std::invalid_argument("height_px must be positive");
      }
    }

    // If we reach here, the serial number is valid
    return {};
  }
  static inline viam::sdk::Model model{"viam", "camera", "realsense"};

  // Handles device changes for this instance
  void handleDeviceChange(rs2::event_information &info) {
    VIAM_RESOURCE_LOG(info) << "[handleDeviceChange] Processing for serial: "
                            << config_->serial_number;

    deviceChangedCallback(info);
  }

private:
  boost::synchronized_value<RsResourceConfig> config_;
  std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>> device_;
  std::shared_ptr<boost::synchronized_value<rs2::frameset>> latest_frameset_;
  std::shared_ptr<boost::synchronized_value<std::unordered_set<std::string>>>
      assigned_serials_;
  boost::synchronized_value<bool> physical_camera_assigned_;
  boost::synchronized_value<std::uint64_t> last_timestamp_warning_log_time_ms_;
  boost::synchronized_value<bool> is_recovery_mode_{false};
  std::shared_ptr<rs2::device> recovery_device_ptr_;

  // Mutex to serialize do_command calls and prevent concurrent execution
  // This is critical for long-running operations like firmware updates
  std::mutex do_command_mutex_;

  DeviceFunctions device_funcs_;
  std::shared_ptr<RealsenseContext<SynchronizedContextT>> realsense_ctx_;

  void deviceChangedCallback(rs2::event_information &info) {
    std::cout << "[deviceChangedCallback] Device connection status changed"
              << std::endl;

    try {
      std::string const required_serial_number = config_->serial_number;
      { // Begin scope for current_device lock
        // Check if device_ is null before accessing it
        // This can happen after firmware update completes and device_ is
        // cleared
        if (device_) {
          auto current_device = device_->synchronize();
          if (info.was_removed(*current_device->device)) {
            std::cerr << "[deviceChangedCallback] Device removed: "
                      << current_device->serial_number << std::endl;
            { // Begin scope for serials_guard lock
              auto serials_guard = assigned_serials_->synchronize();
              serials_guard->erase(current_device->serial_number);
            } // End scope for serials_guard lock
            device_ = nullptr;
            physical_camera_assigned_ = false;
          }
        }
      } // End scope for current_device lock

      // Handling added devices, if any
      auto added_devices = info.get_new_devices();
      if (not physical_camera_assigned_) {
        if (assign_and_initialize_device(added_devices)) {
          physical_camera_assigned_ = true;
          std::cout << "[deviceChangedCallback] Device assigned successfully";
        } else {
          std::cerr << "[deviceChangedCallback] No matching device found";
        }
      }
    } catch (std::exception &e) {
      std::cerr << "[deviceChangedCallback] Exception occurred: " << e.what()
                << std::endl;
    }
  }

  viam::sdk::ProtoStruct handleFirmwareUpdate(const std::string &firmware_url) {
    // Note: This function is called from do_command which already holds the
    // mutex lock
    VIAM_RESOURCE_LOG(info)
        << "[handleFirmwareUpdate] Starting firmware update";
    VIAM_RESOURCE_LOG(info)
        << "[handleFirmwareUpdate] Firmware URL parameter: "
        << (firmware_url.empty() ? "(auto-detect)" : firmware_url);

    viam::sdk::ProtoStruct response;

    // Declare these outside try block so they're accessible in catch block
    std::shared_ptr<rs2::device> rs_device;
    std::string device_serial_number;

    try {

      // Temporarily clear the device change callback to prevent interference
      // It will be automatically restored when this scope exits
      realsense_ctx_->clearDevicesChangedCallback();
      auto callback_restorer = std::shared_ptr<void>(nullptr, [this](void *) {
        realsense_ctx_->restoreDevicesChangedCallback();
        VIAM_RESOURCE_LOG(info)
            << "[handleFirmwareUpdate] Restored device change callback";
      });

      // Get the rs2::device and its serial number for tracking

      // Handle recovery mode vs normal mode devices differently
      if (is_recovery_mode_.get()) {
        VIAM_RESOURCE_LOG(info)
            << "[handleFirmwareUpdate] Device is in recovery mode, using "
            << "stored recovery device pointer";

        if (!recovery_device_ptr_) {
          response["success"] = false;
          response["error"] = std::string(
              "Firmware update failed: No recovery device available");
          VIAM_RESOURCE_LOG(error) << "[handleFirmwareUpdate] Firmware update "
                                      "failed: No recovery device available";
          return response;
        }

        rs_device = recovery_device_ptr_;
        // Get the firmware update ID as the serial number for recovery devices
        device_serial_number = config_->serial_number;

        VIAM_RESOURCE_LOG(info)
            << "[handleFirmwareUpdate] Recovery device firmware update ID: "
            << device_serial_number;
      } else {
        // Normal mode device
        if (!device_) {
          response["success"] = false;
          response["error"] =
              std::string("Firmware update failed: No device available");
          VIAM_RESOURCE_LOG(error) << "[handleFirmwareUpdate] Firmware update "
                                      "failed: No device available";
          return response;
        }

        // Stop the device before firmware update
        VIAM_RESOURCE_LOG(info)
            << "[handleFirmwareUpdate] Stopping device before update";
        if (!device_funcs_.stopDevice(device_, this->logger_)) {
          response["success"] = false;
          response["error"] = std::string("Firmware update failed: Failed to "
                                          "stop device before firmware update");
          return response;
        }

        {
          auto device_guard = device_->synchronize();
          rs_device = device_guard->device;
          device_serial_number = device_guard->serial_number;
        }

        if (!rs_device) {
          response["success"] = false;
          response["error"] =
              std::string("Firmware update failed: Device pointer is null");
          return response;
        }
      }

      VIAM_RESOURCE_LOG(info) << "[handleFirmwareUpdate] Updating device with "
                              << "serial number: " << device_serial_number;

      auto update_result = viam::realsense::firmware_update::updateFirmware(
          rs_device, device_serial_number, firmware_url, realsense_ctx_,
          logger_);

      // Check if firmware update succeeded
      if (update_result.first) {
        // Success - clear device assignment
        device_ = nullptr;
        recovery_device_ptr_ = nullptr;
        physical_camera_assigned_ = false;
        is_recovery_mode_ = false;

        // Remove the device's serial number from the assigned set
        // This allows the device to be reassigned when it reconnects after
        // firmware update
        {
          auto serials_guard = assigned_serials_->synchronize();
          serials_guard->erase(device_serial_number);
        }

        response["success"] = true;
        response["message"] = update_result.second.count("message")
                                  ? update_result.second.at("message")
                                  : "Firmware update completed successfully. "
                                    "Device will reconnect shortly.";
      } else {
        // Failure - clear device state so it can be reassigned when it
        // reconnects
        VIAM_RESOURCE_LOG(info)
            << "[handleFirmwareUpdate] Firmware update failed, clearing device "
               "state. Device will be reassigned when it reconnects (either in "
               "normal mode or recovery/DFU mode).";

        // Clear device state - device may exit DFU mode automatically (on some
        // errors) or stay in DFU/recovery mode (on others like power loss).
        // Either way, it will be reassigned when detected by
        // deviceChangedCallback
        device_ = nullptr;
        physical_camera_assigned_ = false;

        // Remove serial from assigned set so it can be reassigned
        {
          auto serials_guard = assigned_serials_->synchronize();
          serials_guard->erase(device_serial_number);
        }

        // Return error
        response["success"] = false;
        if (update_result.second.count("error")) {
          response["error"] = update_result.second.at("error");
          // Extract error string for logging
          auto error_ptr = response["error"].get<std::string>();
          if (error_ptr) {
            VIAM_RESOURCE_LOG(error) << "[handleFirmwareUpdate] " << *error_ptr;
          } else {
            VIAM_RESOURCE_LOG(error)
                << "[handleFirmwareUpdate] Firmware update failed (error "
                   "message unavailable)";
          }
        } else {
          response["error"] = "Firmware update failed";
          VIAM_RESOURCE_LOG(error)
              << "[handleFirmwareUpdate] Firmware update failed";
        }
      }

    } catch (const std::exception &e) {
      VIAM_RESOURCE_LOG(error) << "[handleFirmwareUpdate] Error: " << e.what();

      // Clear device state on exception - device will reconnect automatically
      if (!is_recovery_mode_.get() && device_) {
        VIAM_RESOURCE_LOG(info) << "[handleFirmwareUpdate] Exception occurred, "
                                   "clearing device state. "
                                   "Device will be reassigned when it exits "
                                   "DFU mode and reconnects.";

        device_ = nullptr;
        physical_camera_assigned_ = false;

        // Remove serial from assigned set so it can be reassigned
        {
          auto serials_guard = assigned_serials_->synchronize();
          serials_guard->erase(device_serial_number);
        }
      }

      response["success"] = false;
      response["error"] = std::string("Firmware update failed: ") + e.what();
    }

    // Callback will be automatically restored by callback_restorer destructor

    return response;
  }

  template <typename DeviceListT>
  bool assign_and_initialize_device(DeviceListT const &device_list) {
    if (physical_camera_assigned_) {
      std::cerr << "[assign_and_initialize_device] Camera is already assigned"
                << std::endl;
      return true;
    }
    std::string requested_serial_number = config_->serial_number;
    VIAM_RESOURCE_LOG(info) << "[assign_and_initialize_device] starting device "
                            << config_->serial_number;
    // This will the initial set of connected devices (i.e. the devices that
    // were connected before the callback was set)
    VIAM_RESOURCE_LOG(info)
        << "[assign_and_initialize_device] Number of connected devices: "
        << device_list.size() << "\n";

    int devCount = device_list.size();
    for (int i = 0; i < devCount; i++) {
      try {
        VIAM_RESOURCE_LOG(debug) << "[assign_and_initialize_device] Attempting "
                                    "to access device at index "
                                 << i;
        auto dev = device_list[i];
        VIAM_RESOURCE_LOG(debug) << "[assign_and_initialize_device] "
                                    "Successfully accessed device at index "
                                 << i;

        device_funcs_.printDeviceInfo(dev, this->logger_);

        auto dev_ptr = std::make_shared<std::decay_t<decltype(dev)>>(dev);

        // Check if device is in recovery/DFU mode
        bool is_recovery = dev.template is<rs2::update_device>();
        std::string connected_device_serial_number;

        if (is_recovery) {
          // Recovery mode devices only have firmware update ID
          if (dev.supports(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID)) {
            connected_device_serial_number =
                dev.get_info(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID);
            VIAM_RESOURCE_LOG(warn)
                << "[assign_and_initialize_device] Device at index " << i
                << " is in recovery/DFU mode with update ID: "
                << connected_device_serial_number;
          } else {
            VIAM_RESOURCE_LOG(error)
                << "[assign_and_initialize_device] Recovery device at index "
                << i << " does not support firmware update ID";
            continue;
          }
        } else {
          // Normal mode devices have serial number
          if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
            connected_device_serial_number =
                dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
          } else {
            VIAM_RESOURCE_LOG(error)
                << "[assign_and_initialize_device] Normal device at index " << i
                << " does not support serial number";
            continue;
          }
        }

        VIAM_RESOURCE_LOG(info)
            << "[assign_and_initialize_device] trying connecting to device: "
            << connected_device_serial_number;

        // Atomically check and insert serial number
        { // Begin scope for serials_guard lock
          auto serials_guard = assigned_serials_->synchronize();
          if ((requested_serial_number.empty() &&
               serials_guard->count(connected_device_serial_number) == 0) ||
              (requested_serial_number == connected_device_serial_number)) {
            VIAM_RESOURCE_LOG(info)
                << "[assign_and_initialize_device] grabbing device: "
                << connected_device_serial_number;

            serials_guard->insert(connected_device_serial_number);
          } else {
            VIAM_RESOURCE_LOG(info)
                << "[assign_and_initialize_device] Could not grab device: "
                << connected_device_serial_number;
            continue;
          }
        } // End scope for serials_guard lock

        // Handle recovery mode devices differently - they can't stream
        if (is_recovery) {
          VIAM_RESOURCE_LOG(warn)
              << "[assign_and_initialize_device] Device "
              << connected_device_serial_number
              << " is in recovery/DFU mode. Skipping streaming initialization. "
              << "Only firmware updates are supported for this device.";
          is_recovery_mode_ = true;
          recovery_device_ptr_ =
              dev_ptr; // Store the device pointer for firmware updates
          physical_camera_assigned_ = true;
          return true;
        }

        // Normal device initialization with streaming support
        VIAM_RESOURCE_LOG(info)
            << "[assign_and_initialize_device] calling createDevice for: "
            << connected_device_serial_number;
        realsense::RsResourceConfig config_copy = config_.get();
        device_ = device_funcs_.createDevice(connected_device_serial_number,
                                             dev_ptr, SUPPORTED_CAMERA_MODELS,
                                             config_copy, this->logger_);
        BOOST_ASSERT(device_ != nullptr);

        VIAM_RESOURCE_LOG(info)
            << "[assign_and_initialize_device] calling startDevice for: "
            << connected_device_serial_number;
        device_funcs_.startDevice(connected_device_serial_number, device_,
                                  latest_frameset_, MAX_FRAME_AGE_MS,
                                  config_copy, this->logger_);
        physical_camera_assigned_ = true;
        is_recovery_mode_ = false;
        is_recovery_mode_ = false;
        return true;
      } catch (const std::exception &e) {
        VIAM_RESOURCE_LOG(error) << "[assign_and_initialize_device] Failed to "
                                    "access/initialize device at index "
                                 << i << ": " << e.what();
        // Continue trying other devices
        continue;
      }
    }
    return false;
  }

  RsResourceConfig configure(viam::sdk::Dependencies dependencies,
                             viam::sdk::ResourceConfig configuration) {
    auto attrs = configuration.attributes();

    /*
     * Validation already checks that serial_number exists, it is a string
     * and it is not empty, so we can safely extract it without additional
     * checks.
     */
    std::string serial;
    if (attrs.count("serial_number")) {
      serial = attrs["serial_number"].get_unchecked<std::string>();
    }

    // Default to both color and depth sensors if not specified
    std::vector<sensors::SensorType> sensors;
    if (attrs.count("sensors")) {
      auto sensors_list =
          attrs["sensors"].get_unchecked<viam::sdk::ProtoList>();
      for (const auto &sensor : sensors_list) {
        auto sensor_type = sensors::string_to_sensor_type(
            sensor.get_unchecked<std::string>(), this->logger_);
        if (sensor_type == sensors::SensorType::unknown) {
          throw std::runtime_error("Invalid sensor type: " +
                                   sensor.get_unchecked<std::string>());
        }
        sensors.push_back(sensor_type);
      }
    } else {
      // Default: enable both color and depth sensors
      sensors.push_back(sensors::SensorType::color);
      sensors.push_back(sensors::SensorType::depth);
    }
    std::optional<int> width;
    if (attrs.count("width_px")) {
      width = static_cast<int>(attrs["width_px"].get_unchecked<double>());
    }
    std::optional<int> height;
    if (attrs.count("height_px")) {
      height = static_cast<int>(attrs["height_px"].get_unchecked<double>());
    }
    auto native_config = realsense::RsResourceConfig(
        serial, configuration.name(), sensors, width, height);

    return native_config;
  }
  static DeviceFunctions createDefaultDeviceFunctions() {
    return DeviceFunctions{
        .stopDevice =
            [](std::shared_ptr<
                   boost::synchronized_value<device::ViamRSDevice<>>> &device,
               viam::sdk::LogSource &logger) {
              return device::stopDevice(device, logger);
            },
        .destroyDevice =
            [](std::shared_ptr<
                   boost::synchronized_value<device::ViamRSDevice<>>> &device,
               viam::sdk::LogSource &logger) {
              return device::destroyDevice(device, logger);
            },
        .printDeviceInfo =
            [](const auto &dev, viam::sdk::LogSource &logger) {
              device::printDeviceInfo(dev, logger);
            },
        .createDevice =
            [](std::string const &serial, std::shared_ptr<rs2::device> dev_ptr,
               std::unordered_set<std::string> const &supported_models,
               realsense::RsResourceConfig const &config,
               viam::sdk::LogSource &logger) {
              return device::createDevice<
                  realsense::RsResourceConfig, device::ViamRSDevice<>,
                  rs2::device, rs2::config, rs2::color_sensor,
                  rs2::depth_sensor, rs2::video_stream_profile>(
                  serial, dev_ptr, supported_models, config, logger);
            },
        .startDevice =
            [](const std::string &serial,
               std::shared_ptr<
                   boost::synchronized_value<device::ViamRSDevice<>>> &device,
               std::shared_ptr<boost::synchronized_value<rs2::frameset>>
                   &latest_frameset,
               std::uint64_t maxFrameSetFrameMs,
               realsense::RsResourceConfig const &viamConfig,
               viam::sdk::LogSource &logger) {
              return device::startDevice(serial, device, latest_frameset,
                                         maxFrameSetFrameMs, viamConfig,
                                         logger);
            },
        .reconfigureDevice =
            [](std::shared_ptr<
                   boost::synchronized_value<device::ViamRSDevice<>>>
                   device,
               realsense::RsResourceConfig const &viamConfig,
               viam::sdk::LogSource &logger) {
              device::reconfigureDevice<
                  realsense::RsResourceConfig, device::ViamRSDevice<>,
                  rs2::device, rs2::config, rs2::color_sensor,
                  rs2::depth_sensor, rs2::video_stream_profile>(
                  device, viamConfig, logger);
            }};
  };
};
}; // namespace realsense
