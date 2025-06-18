#include "camera_realsense.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <viam/sdk/log/logging.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/rpc/server.hpp>

#include "encoding.hpp"
#include "version_helpers.hpp"

namespace viam {
namespace realsense {
// Build a stringstream and automatically convert to string, all in one line.
//
// Global lock to serialize access to module thread state. It protects:
// 1. RealSense context
// 2. all objects derived from the RealSense context e.g. rs2 device objects
// 3. the invariant that there is only one pipeline per device e.g.
// pipeline.start
// 4. the registered devices map
// Additionally, it implicitly ensures that DeviceProperties weak ptrs are valid
// in the registered_devices_ map by making sure that the DeviceProperties
// objects that are dereferenced are removed from the map when they are
// destroyed.
static std::mutex g_realsense_module_lock;
// Global context for managing all RealSense devices and pipelines in the
// module. It is shared across threads; we synchronize accesses when calling
// context methods concurrently.
static rs2::context rs2_ctx;
// Global registry for devices and their properties/pipelines
static std::map<std::string, std::weak_ptr<DeviceProperties>> registered_devices_;

// Global flag to signal that the module is shutting down to prevent callback
// invocation during cleanup
// NOTE: Nick S: This is an imcomplete solution. We don't confirm that we
// terminate all threads before shutting down. We should add that.
static std::atomic<bool> module_shutting_down{false};
static std::atomic<bool> module_level_debug{false};

// Helper function to deregister device from global map
static void deregister_device(const std::string &serial_number) {
    if (serial_number.empty()) return;

    if (registered_devices_.erase(serial_number)) {
        if (module_level_debug.load()) {
            VIAM_SDK_LOG(info) << "unregistered device " << serial_number << " from global map.";
        }
    } else {
        VIAM_SDK_LOG(warn) << "device " << serial_number
                           << " not found in global map for unregistering.";
    }
}

// initialize will use the ResourceConfigs to begin the realsense pipeline.
RealSenseProperties CameraRealSense::initialize(sdk::ResourceConfig cfg) {
    if (debug_enabled) {
        VIAM_SDK_LOG(info) << "initialize start";
    }
    // TODO: protect device_ from concurrent reconfigures
    if (device_ != nullptr) {
        VIAM_SDK_LOG(info) << "reinitializing, restarting pipeline";
        {
            // wait until frameLoop is stopped
            auto start = std::chrono::high_resolution_clock::now();
            device_->shouldRun.store(false);
            while (device_->isRunning.load()) {
                if (debug_enabled) {
                    VIAM_SDK_LOG(info) << "waiting for frameLoop to stop in initialize";
                }
                if (std::chrono::high_resolution_clock::now() - start > std::chrono::seconds(1)) {
                    throw std::runtime_error("timed out waiting for frameLoop to stop");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }
    VIAM_SDK_LOG(info) << "initializing the Intel RealSense Camera resource";
    // set variables from config
    uint width = 0;
    uint height = 0;
    auto attrs = cfg.attributes();

    if (attrs.count("width_px")) {
        if (const double *width_val = attrs["width_px"].get<double>()) {
            width = static_cast<uint>(*width_val);
        }
    }

    if (attrs.count("height_px")) {
        if (const double *height_val = attrs["height_px"].get<double>()) {
            height = static_cast<uint>(*height_val);
        }
    }

    if (width == 0 || height == 0) {
        if (debug_enabled) {
            VIAM_SDK_LOG(info) << "note: will pick any suitable width and height";
        }
    }
    std::string serial_number_from_config;
    if (attrs.count("serial_number")) {
        if (const std::string *serial_val = attrs["serial_number"].get<std::string>()) {
            serial_number_from_config = *serial_val;
        }
    }
    if (attrs.count("debug")) {
        if (const bool *debug_val = attrs["debug"].get<bool>()) {
            debug_enabled = *debug_val || module_level_debug.load();
        }
    }

    bool littleEndianDepth = false;
    if (attrs.count("little_endian_depth")) {
        if (const bool *endian_depth = attrs["little_endian_depth"].get<bool>()) {
            littleEndianDepth = *endian_depth;
        }
    }

    bool disableDepth = true;
    bool disableColor = true;
    std::vector<std::string> sensors;

    if (attrs.count("sensors")) {
        if (const sdk::ProtoList *sensor_list_ptr = attrs["sensors"].get<sdk::ProtoList>()) {
            for (const auto &element : *sensor_list_ptr) {
                if (const std::string *sensor_name = element.get<std::string>()) {
                    if (*sensor_name == "color") {
                        disableColor = false;
                        sensors.push_back("color");
                    } else if (*sensor_name == "depth") {
                        disableDepth = false;
                        sensors.push_back("depth");
                    }
                }
            }
        }
    }

    if (disableColor && disableDepth) {
        throw std::runtime_error("cannot disable both color and depth");
    }

    if (debug_enabled) {
        VIAM_SDK_LOG(info) << "disableDepth: " << disableDepth << " disableColor: " << disableColor
                           << " sensors size " << sensors.size();
    }

    // DeviceProperties context also holds a bool that can stop the thread if
    // device gets disconnected
    std::shared_ptr<DeviceProperties> newDevice = std::make_shared<DeviceProperties>(
        width, height, disableColor, width, height, disableDepth, this->latest_frames_);
    device_ = std::move(newDevice);

    // First start of Pipeline
    RealSenseProperties props;
    auto [new_pipeline, new_props] = startPipeline(device_, serial_number_from_config);
    {
        std::lock_guard<std::mutex> device_lock(device_->pipeline_mu);
        device_->pipeline = std::move(new_pipeline);
    }
    props = std::move(new_props);
    if (debug_enabled) {
        VIAM_SDK_LOG(info) << "startPipeline returned pipeline with pointer id: "
                           << &device_->pipeline;
    }

    registered_devices_[this->device_->active_serial_number] = device_;
    if (debug_enabled) {
        VIAM_SDK_LOG(info) << "registered device " << this->device_->active_serial_number
                           << " in global map.";
    }

    // First start of camera thread
    props.sensors = sensors;
    props.mainSensor = sensors.front();
    VIAM_SDK_LOG(info) << "main sensor will be " << sensors.front();
    props.littleEndianDepth = littleEndianDepth;
    if (props.mainSensor == "depth") {
        if (debug_enabled) {
            VIAM_SDK_LOG(info) << std::boolalpha
                               << "depth little endian encoded: " << littleEndianDepth;
        }
    }
    std::promise<void> ready;
    std::thread cameraThread(frameLoop, std::ref(ready), device_, props.depthScaleMm,
                             std::ref(this->latest_frames_), debug_enabled);
    VIAM_SDK_LOG(info) << "waiting for camera frame loop thread to be ready...";
    ready.get_future().wait();
    VIAM_SDK_LOG(info) << "camera frame loop ready!";
    cameraThread.detach();

    return props;
}

CameraRealSense::CameraRealSense(sdk::Dependencies deps, sdk::ResourceConfig cfg)
    : Camera(cfg.name()) {
    if (module_level_debug.load()) {
        VIAM_SDK_LOG(info) << "[constructor] start";
    }
    RealSenseProperties props;
    auto start = std::chrono::high_resolution_clock::now();
    try {
        std::lock_guard<std::mutex> lock(g_realsense_module_lock);
        this->props_ = initialize(cfg);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

        if (debug_enabled) {
            VIAM_SDK_LOG(info) << "constructor initialize() took " << duration.count() << "ms";
        }
    } catch (const std::exception &e) {
        throw std::runtime_error("failed to initialize realsense: " + std::string(e.what()));
    }
    if (module_level_debug.load()) {
        VIAM_SDK_LOG(info) << "[constructor] end";
    }
}

CameraRealSense::~CameraRealSense() {
    if (debug_enabled) {
        VIAM_SDK_LOG(info) << "[destructor] start. destroying realsense camera resource";
    }
    // stop and wait for the frameLoop thread to exit
    if (!this->device_) {
        if (debug_enabled) {
            VIAM_SDK_LOG(info) << "[destructor] no device, early return";
        }
        return;
    }

    if (debug_enabled) {
        VIAM_SDK_LOG(info) << "[destructor] shouldRun: " << device_->shouldRun.load()
                           << " isRunning: " << device_->isRunning.load();
    }
    {
        // wait until frameLoop is stopped
        auto start = std::chrono::high_resolution_clock::now();
        device_->shouldRun.store(false);
        while (device_->isRunning.load()) {
            if (debug_enabled) {
                VIAM_SDK_LOG(info) << "waiting for frameLoop to stop in destructor";
            }
            if (std::chrono::high_resolution_clock::now() - start > std::chrono::seconds(1)) {
                VIAM_SDK_LOG(error) << "waiting for frameLoop to stop in destructor";
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    std::lock_guard<std::mutex> lock(g_realsense_module_lock);
    deregister_device(this->device_->active_serial_number);
    if (debug_enabled) {
        VIAM_SDK_LOG(info) << "[destructor] end";
    }
}

void CameraRealSense::reconfigure(const sdk::Dependencies &deps, const sdk::ResourceConfig &cfg) {
    if (debug_enabled) {
        VIAM_SDK_LOG(info) << "[reconfigure] start";
    }
    try {
        // clean up old registry entry before reinitializing
        std::lock_guard<std::mutex> lock(g_realsense_module_lock);
        deregister_device(this->device_->active_serial_number);
        RealSenseProperties props;
        auto start = std::chrono::high_resolution_clock::now();
        // TODO: we should probably protect props_ as well from concurrent
        // reconfigures
        this->props_ = initialize(cfg);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        if (debug_enabled) {
            VIAM_SDK_LOG(info) << "reconfigure initialize() took " << duration.count() << "ms";
        }
    } catch (const std::exception &e) {
        VIAM_SDK_LOG(error) << "failed to reconfigure realsense: " << e.what();
        throw;
    }
    if (debug_enabled) {
        VIAM_SDK_LOG(info) << "[reconfigure] end";
    }
}

sdk::Camera::raw_image CameraRealSense::get_image(std::string mime_type,
                                                  const sdk::ProtoStruct &extra) {
    if (debug_enabled) {
        VIAM_SDK_LOG(info) << "[get_image] start";
    }
    try {
        std::chrono::time_point<std::chrono::high_resolution_clock> start;
        if (debug_enabled) {
            start = std::chrono::high_resolution_clock::now();
        }

        rs2::frame latestColorFrame;
        std::shared_ptr<std::vector<uint16_t>> latestDepthFrame;
        {
            std::lock_guard<std::mutex> lock(this->latest_frames_.mutex);
            latestColorFrame = this->latest_frames_.colorFrame;
            latestDepthFrame = this->latest_frames_.depthFrame;
        }
        std::unique_ptr<sdk::Camera::raw_image> response;
        if (this->props_.mainSensor.compare("color") == 0) {
            if (this->device_->disableColor) {
                throw std::invalid_argument("color disabled");
            }
            if (mime_type.compare("image/png") == 0 || mime_type.compare("image/png+lazy") == 0) {
                response =
                    encodeColorPNGToResponse((const void *)latestColorFrame.get_data(),
                                             this->props_.color.width, this->props_.color.height);
            } else if (mime_type.compare("image/vnd.viam.rgba") == 0) {
                response =
                    encodeColorRAWToResponse((const unsigned char *)latestColorFrame.get_data(),
                                             this->props_.color.width, this->props_.color.height);
            } else {
                response =
                    encodeJPEGToResponse((const unsigned char *)latestColorFrame.get_data(),
                                         this->props_.color.width, this->props_.color.height);
            }
        } else if (this->props_.mainSensor.compare("depth") == 0) {
            if (this->device_->disableDepth) {
                throw std::invalid_argument("depth disabled");
            }
            if (mime_type.compare("image/vnd.viam.dep") == 0) {
                response = encodeDepthRAWToResponse(
                    (const unsigned char *)latestDepthFrame->data(), this->props_.depth.width,
                    this->props_.depth.height, this->props_.littleEndianDepth);
            } else {
                response =
                    encodeDepthPNGToResponse((const unsigned char *)latestDepthFrame->data(),
                                             this->props_.depth.width, this->props_.depth.height);
            }
        }

        if (debug_enabled) {
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            VIAM_SDK_LOG(info) << "[get_image]  total:           " << duration.count() << "ms\n";
        }

        if (debug_enabled) {
            VIAM_SDK_LOG(info) << "[get_image] end";
        }
        return std::move(*response);
    } catch (const std::exception &e) {
        VIAM_SDK_LOG(error) << "[get_image] failed to get image: " << e.what();
        throw;
    }
}

sdk::Camera::properties CameraRealSense::get_properties() {
    if (debug_enabled) {
        VIAM_SDK_LOG(info) << "[get_properties] start";
    }
    try {
        auto fillResp = [](sdk::Camera::properties *p, CameraProperties props) {
            p->supports_pcd = true;
            p->intrinsic_parameters.width_px = props.width;
            p->intrinsic_parameters.height_px = props.height;
            p->intrinsic_parameters.focal_x_px = props.fx;
            p->intrinsic_parameters.focal_y_px = props.fy;
            p->intrinsic_parameters.center_x_px = props.ppx;
            p->intrinsic_parameters.center_y_px = props.ppy;
            p->distortion_parameters.model = props.distortionModel;
            for (int i = 0; i < std::size(props.distortionParameters); i++) {
                p->distortion_parameters.parameters.push_back(props.distortionParameters[i]);
            }
        };

        sdk::Camera::properties response{};
        if (this->props_.mainSensor.compare("color") == 0) {
            fillResp(&response, this->props_.color);
        } else if (this->props_.mainSensor.compare("depth") == 0) {
            fillResp(&response, this->props_.depth);
        }
        if (debug_enabled) {
            VIAM_SDK_LOG(info) << "[get_properties] end";
        }
        return response;
    } catch (const std::exception &e) {
        VIAM_SDK_LOG(error) << "[get_properties] failed to get properties: " << e.what();
        throw;
    }
}

sdk::Camera::image_collection CameraRealSense::get_images() {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (debug_enabled) {
        start = std::chrono::high_resolution_clock::now();
    }
    sdk::Camera::image_collection response;

    rs2::frame latestColorFrame;
    std::shared_ptr<std::vector<uint16_t>> latestDepthFrame;
    std::chrono::milliseconds latestTimestamp;
    {
        std::lock_guard<std::mutex> lock(this->latest_frames_.mutex);
        latestColorFrame = this->latest_frames_.colorFrame;
        latestDepthFrame = this->latest_frames_.depthFrame;
        latestTimestamp = this->latest_frames_.timestamp;
    }

    for (const auto &sensor : this->props_.sensors) {
        if (sensor == "color") {
            std::unique_ptr<sdk::Camera::raw_image> color_response;
            color_response =
                encodeJPEGToResponse((const unsigned char *)latestColorFrame.get_data(),
                                     this->props_.color.width, this->props_.color.height);
            response.images.emplace_back(std::move(*color_response));
        } else if (sensor == "depth") {
            std::unique_ptr<sdk::Camera::raw_image> depth_response;
            depth_response = encodeDepthRAWToResponse(
                (const unsigned char *)latestDepthFrame->data(), this->props_.depth.width,
                this->props_.depth.height, this->props_.littleEndianDepth);
            response.images.emplace_back(std::move(*depth_response));
        }
    }

    response.metadata.captured_at =
        sdk::time_pt{std::chrono::duration_cast<std::chrono::nanoseconds>(latestTimestamp)};

    if (debug_enabled) {
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        VIAM_SDK_LOG(info) << "[get_images]  total:           " << duration.count() << "ms\n";
    }

    return response;
}

sdk::ProtoStruct CameraRealSense::do_command(const sdk::ProtoStruct &command) {
    VIAM_SDK_LOG(error) << "do_command not implemented";
    return sdk::ProtoStruct{};
}

sdk::Camera::point_cloud CameraRealSense::get_point_cloud(std::string mime_type,
                                                          const sdk::ProtoStruct &extra) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (debug_enabled) {
        start = std::chrono::high_resolution_clock::now();
    }

    rs2::frame latestColorFrame;
    rs2::frame latestDepthFrame;
    rs2::pointcloud pc;
    rs2::points points;
    std::vector<unsigned char> pcdBytes;
    {
        std::lock_guard<std::mutex> lock(this->latest_frames_.mutex);
        latestColorFrame = this->latest_frames_.colorFrame;
        latestDepthFrame = this->latest_frames_.rsDepthFrame;
    }

    if (latestColorFrame) {
        pc.map_to(latestColorFrame);
    }
    if (!latestDepthFrame) {
        VIAM_SDK_LOG(error) << "cannot get point cloud as there is no depth frame";
        return sdk::Camera::point_cloud{};
    }
    points = pc.calculate(latestDepthFrame);
    pcdBytes = rsPointsToPCDBytes(points, latestColorFrame);

    if (debug_enabled) {
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        VIAM_SDK_LOG(info) << "[get_point_cloud]  total:           " << duration.count() << "ms\n";
    }
    return sdk::Camera::point_cloud{mime_type, pcdBytes};
}

std::vector<sdk::GeometryConfig> CameraRealSense::get_geometries(const sdk::ProtoStruct &extra) {
    VIAM_SDK_LOG(error) << "get_geometries not implemented";
    return std::vector<sdk::GeometryConfig>{};
}

// Loop functions
void frameLoop(std::promise<void> &ready, std::shared_ptr<DeviceProperties> deviceProps,
               float depthScaleMm, AtomicFrameSet &latest_frames, bool debug_enabled) {
    try {
        deviceProps->isRunning.store(true);
        if (debug_enabled) {
            VIAM_SDK_LOG(info) << "[frameLoop] starting with thread ID: "
                               << std::hash<std::thread::id>{}(std::this_thread::get_id())
                               << " for device: " << deviceProps->active_serial_number;
        }
        bool readyOnce = false;

        if (debug_enabled) {
            VIAM_SDK_LOG(info) << "[frameLoop] frame loop is starting for device: "
                               << deviceProps->active_serial_number;
        }
        while (true) {
            {
                if (!deviceProps->shouldRun.load()) {
                    try {
                        if (debug_enabled) {
                            VIAM_SDK_LOG(info)
                                << "pipeline stopping with pointer id: " << &deviceProps->pipeline;
                        }
                        std::lock_guard<std::mutex> lock(deviceProps->pipeline_mu);
                        deviceProps->pipeline.stop();
                    } catch (const std::exception &e) {
                        VIAM_SDK_LOG(warn)
                            << "[frameLoop] Exception stopping pipeline: " << e.what();
                    }
                    if (debug_enabled) {
                        VIAM_SDK_LOG(info) << "[frameLoop] pipeline stopped, exiting frame "
                                              "loop for device: "
                                           << deviceProps->active_serial_number;
                    }
                    break;
                }
            }
            auto failureWait = std::chrono::milliseconds(5);

            std::chrono::time_point<std::chrono::high_resolution_clock> start;
            if (debug_enabled) {
                start = std::chrono::high_resolution_clock::now();
            }

            rs2::frameset frames;
            const uint timeoutMillis = 2000;
            /*
                D435 1920x1080 RGB + Depth ~20ms on a Raspberry Pi 4 Model B
            */
            bool succ = false;
            {
                std::lock_guard<std::mutex> lock(deviceProps->pipeline_mu);
                if (debug_enabled) {
                    VIAM_SDK_LOG(info) << "pipeline waiting for frames with pointer id: "
                                       << &deviceProps->pipeline;
                }
                succ = deviceProps->pipeline.try_wait_for_frames(&frames, timeoutMillis);
            }
            if (!succ) {
                if (debug_enabled) {
                    VIAM_SDK_LOG(error) << "[frameLoop] could not get frames from realsense after "
                                        << timeoutMillis << "ms";
                }
                std::this_thread::sleep_for(failureWait);
                continue;
            }
            if (debug_enabled) {
                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
                VIAM_SDK_LOG(info) << "[frameLoop] wait for frames: " << duration.count() << "ms\n";
            }

            if (!deviceProps->disableColor && !deviceProps->disableDepth) {
                std::chrono::time_point<std::chrono::high_resolution_clock> start;
                if (debug_enabled) {
                    start = std::chrono::high_resolution_clock::now();
                }

                try {
                    frames = deviceProps->frame_alignment.process(frames);
                } catch (const std::exception &e) {
                    VIAM_SDK_LOG(error)
                        << "[frameLoop] exception while aligning images: " << e.what();
                    std::this_thread::sleep_for(failureWait);
                    continue;
                }

                if (debug_enabled) {
                    auto stop = std::chrono::high_resolution_clock::now();
                    auto duration =
                        std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
                    VIAM_SDK_LOG(info)
                        << "[frameLoop] frame alignment: " << duration.count() << "ms\n";
                }
            }
            // scale every pixel value to be depth in units of mm
            std::unique_ptr<std::vector<uint16_t>> depthFrameScaled;
            if (!deviceProps->disableDepth) {
                auto depthFrame = frames.get_depth_frame();
                auto depthWidth = depthFrame.get_width();
                auto depthHeight = depthFrame.get_height();
                const uint16_t *depthFrameData = (const uint16_t *)depthFrame.get_data();
                depthFrameScaled =
                    std::make_unique<std::vector<uint16_t>>(depthWidth * depthHeight);
                for (int y = 0; y < depthHeight; y++) {
                    for (int x = 0; x < depthWidth; x++) {
                        auto px = (y * depthWidth) + x;
                        uint16_t depthScaled = depthScaleMm * depthFrameData[px];
                        (*depthFrameScaled)[px] = depthScaled;
                    }
                }
            }
            {
                std::lock_guard<std::mutex> lock(latest_frames.mutex);
                latest_frames.colorFrame = frames.get_color_frame();
                latest_frames.depthFrame = std::move(depthFrameScaled);
                latest_frames.rsDepthFrame = frames.get_depth_frame();
                latest_frames.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::duration<double, std::milli>(frames.get_timestamp()));
            }

            if (debug_enabled) {
                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
                VIAM_SDK_LOG(info) << "[frameLoop] total:           " << duration.count() << "ms\n";
            }

            if (!readyOnce) {
                readyOnce = true;
                ready.set_value();
            }
        }

        if (debug_enabled) {
            VIAM_SDK_LOG(info) << "[frameLoop] deviceProps->isRunning.store(false): "
                               << deviceProps->active_serial_number;
        }
        deviceProps->isRunning.store(false);
    } catch (const std::exception &e) {
        VIAM_SDK_LOG(error) << "[frameLoop] exception in frame loop with id "
                            << std::hash<std::thread::id>{}(std::this_thread::get_id())
                            << " for device: " << deviceProps->active_serial_number << ": "
                            << e.what() << " pipeline pointer id: " << &deviceProps->pipeline;
        throw;
    }

    if (debug_enabled) {
        VIAM_SDK_LOG(info) << "[frameLoop] exiting with thread ID: "
                           << std::hash<std::thread::id>{}(std::this_thread::get_id())
                           << " for device: " << deviceProps->active_serial_number;
    }
};

// gives the pixel to mm conversion for the depth sensor
float getDepthScale(rs2::device dev) {
    // Go over the device's sensors
    for (rs2::sensor &sensor : dev.query_sensors()) {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>()) {
            return dpt.get_depth_scale() * 1000.0;  // rs2 gives pix2meters
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

std::string printDeviceInfo(rs2::device selected_device) {
    VIAM_SDK_LOG(info) << "starting pipeline with selected device:";
    std::string serial_from_rs2;
    serial_from_rs2 = selected_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    auto usb_type = std::string(selected_device.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR));
    auto firmware_version = selected_device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
    auto recommended_firmware_version =
        selected_device.get_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION);

    VIAM_SDK_LOG(info) << "name:      " << selected_device.get_info(RS2_CAMERA_INFO_NAME);
    VIAM_SDK_LOG(info) << "serial:    " << serial_from_rs2;

    VIAM_SDK_LOG(info) << "firmware:  " << firmware_version;
    VIAM_SDK_LOG(info) << "recommended firmware:  " << recommended_firmware_version;

    if (version(firmware_version) < version(recommended_firmware_version)) {
        VIAM_SDK_LOG(warn) << "firmware version is lower than recommended "
                           << "version. Please upgrade camera's firmware or you "
                           << "may experience camera stability issues.";
    };
    VIAM_SDK_LOG(info) << "port:      " << selected_device.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);

    switch (usb_type.at(0)) {
        case '3':
            VIAM_SDK_LOG(info) << "usb type:  " << usb_type;
            break;
        default:
            VIAM_SDK_LOG(warn) << "usb type:  " << usb_type
                               << " please change the port and cable used with the camera to usb "
                                  "3.x or you may experience freezing problems due "
                                  "to usb bandwidth limitations: Look up intel realsense ticket "
                                  "RSDSO-9074.";
            break;
    }
    return serial_from_rs2;
}

std::tuple<rs2::pipeline, RealSenseProperties> startPipeline(
    std::shared_ptr<DeviceProperties> device_props, std::string target_serial_number) {
    rs2::device_list devices;
    rs2::device selected_device;

    devices = rs2_ctx.query_devices();

    if (devices.size() == 0) {
        throw std::runtime_error("no devices connected; please connect an Intel RealSense device");
    }

    for (auto &&dev_info_rs2 : devices) {
        std::string current_serial_number;
        current_serial_number = dev_info_rs2.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        VIAM_SDK_LOG(debug) << "found available RealSense device with serial number: "
                            << current_serial_number;
        if (!target_serial_number.empty() && current_serial_number == target_serial_number) {
            selected_device = dev_info_rs2;
            // don't break because we want to log all available devices
        }
    }
    if (target_serial_number.empty()) {
        VIAM_SDK_LOG(debug) << "no serial number specified in config, using "
                               "first available device";
        selected_device = devices.front();
    }
    if (!selected_device) {
        throw std::runtime_error("no device found with specified serial number: " +
                                 target_serial_number);
    }

    auto serial_from_rs2 = printDeviceInfo(selected_device);

    float depthScaleMm = 0.0;
    bool disableDepth = device_props->disableDepth;
    if (!disableDepth) {
        depthScaleMm = getDepthScale(selected_device);
    }

    rs2::config cfg;
    cfg.enable_device(serial_from_rs2);
    device_props->active_serial_number = serial_from_rs2;

    bool disableColor = device_props->disableColor;
    if (!disableColor) {
        VIAM_SDK_LOG(info) << "color width and height from config: (" << device_props->colorWidth
                           << ", " << device_props->colorHeight << ")";
        cfg.enable_stream(RS2_STREAM_COLOR, device_props->colorWidth, device_props->colorHeight,
                          RS2_FORMAT_RGB8);
    }

    if (!disableDepth) {
        VIAM_SDK_LOG(info) << "depth width and height from config: (" << device_props->depthWidth
                           << ", " << device_props->depthHeight << ")";
        cfg.enable_stream(RS2_STREAM_DEPTH, device_props->depthWidth, device_props->depthHeight,
                          RS2_FORMAT_Z16);
    }

    rs2::pipeline pipeline(rs2_ctx);
    pipeline.start(cfg);

    auto fillProps = [](auto intrinsics, std::string distortionModel) -> CameraProperties {
        CameraProperties camProps;
        camProps.width = intrinsics.width;
        camProps.height = intrinsics.height;
        camProps.fx = intrinsics.fx;
        camProps.fy = intrinsics.fy;
        camProps.ppx = intrinsics.ppx;
        camProps.ppy = intrinsics.ppy;
        if (distortionModel != "") {
            camProps.distortionModel = std::move(distortionModel);
            int distortionSize = std::min((int)std::size(intrinsics.coeffs), 5);
            for (int i = 0; i < distortionSize; i++) {
                camProps.distortionParameters[i] = double(intrinsics.coeffs[i]);
            }
        }
        return camProps;
    };

    RealSenseProperties props;
    props.depthScaleMm = depthScaleMm;
    if (!disableColor) {
        VIAM_SDK_LOG(debug) << "getting pipeline active profile with pointer id(1): " << &pipeline;
        auto const stream = pipeline.get_active_profile()
                                .get_stream(RS2_STREAM_COLOR)
                                .as<rs2::video_stream_profile>();
        auto intrinsics = stream.get_intrinsics();
        props.color = fillProps(intrinsics, "brown_conrady");
    }
    if (!disableDepth) {
        VIAM_SDK_LOG(debug) << "getting pipeline active profile with pointer id(2): " << &pipeline;
        auto const stream = pipeline.get_active_profile()
                                .get_stream(RS2_STREAM_DEPTH)
                                .as<rs2::video_stream_profile>();
        auto intrinsics = stream.get_intrinsics();
        props.depth = fillProps(intrinsics, "");
        if (!disableColor) {
            props.depth.width = props.color.width;
            props.depth.height = props.color.height;
        }
    }

    VIAM_SDK_LOG(info) << "pipeline started with:";
    VIAM_SDK_LOG(info) << "color_enabled:  " << std::boolalpha << !disableColor;
    if (!disableColor) {
        VIAM_SDK_LOG(info) << "color_width:    " << props.color.width
                           << "    color_height:   " << props.color.height;
    }

    VIAM_SDK_LOG(info) << "depth_enabled:  " << !disableDepth;
    if (!disableDepth) {
        auto alignedText = "";
        if (!disableColor) {
            alignedText = " (aligned to color)";
        }
        VIAM_SDK_LOG(info) << "depth_width:    " << props.depth.width << alignedText
                           << " depth_height:   " << props.depth.height << alignedText;
    }

    return std::make_tuple(pipeline, props);
};

// This is the global callback registered with the RealSense context.
// It dispatches events to the appropriate CameraRealSense instance.
void global_device_changed_handler(rs2::event_information &info) {
    if (module_level_debug.load()) {
        VIAM_SDK_LOG(info) << "[device_changed] thread ID: "
                           << std::hash<std::thread::id>{}(std::this_thread::get_id());
    }
    if (module_shutting_down.load()) {
        VIAM_SDK_LOG(info) << "[device_changed] module shutting down, ignoring "
                              "device changed event.";
        return;
    }

    std::vector<std::shared_ptr<DeviceProperties>> devices_to_reconnect;
    {
        VIAM_SDK_LOG(info) << "[device_changed] global device changed event received.";
        std::lock_guard<std::mutex> lock(g_realsense_module_lock);

        // TODO: please also handle (indicate that the background frameloop
        // should terminate maybe set shouldrun to false?) unplugs not just
        // replugs (the below logic)
        for (auto &&dev_info : info.get_new_devices()) {
            // assures we're checking a device that was part of the "added"
            // event
            if (!info.was_added(dev_info)) {
                continue;
            }

            std::string serial_number;
            try {
                serial_number = dev_info.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            } catch (const rs2::error &e) {
                VIAM_SDK_LOG(error) << "[device_changed] failed to get serial "
                                       "number for new device: "
                                    << e.what();
                continue;
            }

            VIAM_SDK_LOG(info) << "[device_changed] device added event for S/N: " << serial_number;

            auto it = registered_devices_.find(serial_number);
            if (it == registered_devices_.end()) {
                VIAM_SDK_LOG(info)
                    << "[device_changed] no active Viam resource matches S/N: " << serial_number
                    << ". Ignoring event for this device.";
                continue;
            }

            auto device_props_sptr = it->second.lock();
            if (!device_props_sptr) {
                VIAM_SDK_LOG(error) << "[device_changed] device S/N: " << serial_number
                                    << " found in map but weak_ptr is expired.";
                registered_devices_.erase(it);
                continue;
            }

            VIAM_SDK_LOG(info) << "[device_changed] matching active device found for S/N: "
                               << serial_number << ". Scheduling reconnect.";
            devices_to_reconnect.push_back(device_props_sptr);
        }
    }  // g_realsense_module_lock releases

    for (auto &device_props : devices_to_reconnect) {
        try {
            on_device_reconnect(info, device_props);
        } catch (const std::exception &e) {
            VIAM_SDK_LOG(error) << "[device_changed] failed to reconnect device "
                                << device_props->active_serial_number << ": " << e.what();
        }
    }
}

void on_device_reconnect(rs2::event_information &info, std::shared_ptr<DeviceProperties> device) {
    if (device == nullptr) {
        VIAM_SDK_LOG(error) << "[on_device_reconnect] received null device properties.";
        return;
    }

    VIAM_SDK_LOG(info) << "[on_device_reconnect] Processing reconnect for device "
                          "with configured S/N: "
                       << (device->active_serial_number.empty() ? "<any>"
                                                                : device->active_serial_number);
    {
        // wait until frameLoop is stopped
        auto start = std::chrono::high_resolution_clock::now();
        device->shouldRun.store(false);
        while (device->isRunning.load()) {
            VIAM_SDK_LOG(info) << "waiting for frameLoop to stop in on_device_reconnect";
            if (std::chrono::high_resolution_clock::now() - start > std::chrono::seconds(1)) {
                VIAM_SDK_LOG(error) << "waiting for frameLoop to stop in on_device_reconnect";
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    RealSenseProperties props;
    try {
        rs2::pipeline new_pipeline;
        {
            std::lock_guard<std::mutex> lock(g_realsense_module_lock);
            std::tie(new_pipeline, props) = startPipeline(device, device->active_serial_number);
        }
        std::lock_guard<std::mutex> device_lock(device->pipeline_mu);
        VIAM_SDK_LOG(info) << "pipeline restarted";
        device->pipeline = std::move(new_pipeline);
    } catch (const std::exception &e) {
        VIAM_SDK_LOG(error) << "[on_device_reconnect] Failed to restart pipeline: " << e.what();
        VIAM_SDK_LOG(info) << "[on_device_reconnect] device->isRunning.store(false): "
                           << device->active_serial_number;
        device->isRunning.store(false);
        device->shouldRun.store(false);
        return;
    }

    // Start the camera std::thread
    std::promise<void> ready;
    device->shouldRun.store(true);
    VIAM_SDK_LOG(info) << "[on_device_reconnect] device->isRunning.store(true): "
                       << device->active_serial_number;
    device->isRunning.store(true);

    // NOTE: Nick S:
    // Passing module_level_debug.load() is wrong here but we can't do better as
    // we don't have access in this function to whether or not the resource that
    // is using this device has debug logging enabled or not.
    //
    // The long term solution is to never stop the
    // background thread until the distructor is called.
    // That thread should also detect that the camera has been disconnected
    // and change it's behavior appropriately
    //
    // we should also not be detaching the camea thread. We should be making it a
    // member of the camera object and joining the thread in the destructor.
    // Overall the threadding mnodel in this module is worng but we don't have
    // time to fix it a this time... hance the hack.
    std::thread cameraThread(frameLoop, std::ref(ready), device, props.depthScaleMm,
                             std::ref(device->atomic_frame_set), module_level_debug.load());
    VIAM_SDK_LOG(info) << "[on_device_reconnect] waiting for camera frame loop "
                          "thread to be ready...";
    ready.get_future().wait();
    VIAM_SDK_LOG(info) << "[on_device_reconnect] camera frame loop ready!";
    cameraThread.detach();
};

// validate will validate the ResourceConfig. If there is an error, it will
// throw an exception.
std::vector<std::string> validate(sdk::ResourceConfig cfg) {
    try {
        auto attrs = cfg.attributes();

        if (attrs.count("width_px")) {
            if (const double *width = attrs["width_px"].get<double>()) {
                if (static_cast<int>(*width) < 0) {
                    throw std::invalid_argument("width_px cannot be negative");
                }
            }
        }

        if (attrs.count("height_px")) {
            if (const double *height = attrs["height_px"].get<double>()) {
                if (static_cast<int>(*height) < 0) {
                    throw std::invalid_argument("height_px cannot be negative");
                }
            }
        }

        if (attrs.count("serial_number")) {
            if (!attrs["serial_number"].get<std::string>()) {
                throw std::invalid_argument("serial_number must be a string");
            }
        }

        if (attrs.count("sensors")) {
            if (const sdk::ProtoList *sensors_list = attrs["sensors"].get<sdk::ProtoList>()) {
                if (sensors_list->empty()) {
                    throw std::invalid_argument(
                        "sensors field cannot be empty, must "
                        "list color and/or depth sensor");
                }
                for (const auto &sensor_element : *sensors_list) {
                    if (!sensor_element.get<std::string>()) {
                        throw std::invalid_argument(
                            "elements in 'sensors' list must be "
                            "strings (e.g., \"color\", "
                            "\"depth\")");
                    }
                }
            } else {
                throw std::invalid_argument("'sensors' attribute must be a list of strings");
            }
        } else {
            throw std::invalid_argument(
                "could not find required 'sensors' attribute in the config");
        }
        return {};
    } catch (const std::exception &e) {
        VIAM_SDK_LOG(error) << "[validate] failed to validate config: " << e.what();
        throw;
    }
}

int serve(int argc, char **argv) {
    for (size_t i = 0; i < argc; i++) {
        if (std::string(argv[i]) == "--log-level=debug") {
            module_level_debug.store(true);
            rs2::log_to_console(RS2_LOG_SEVERITY_DEBUG);
        }
    }
    std::shared_ptr<sdk::ModelRegistration> mr = std::make_shared<sdk::ModelRegistration>(
        sdk::API::get<sdk::Camera>(), sdk::Model{kAPINamespace, kAPIType, kAPISubtype},
        [](sdk::Dependencies deps, sdk::ResourceConfig cfg) -> std::shared_ptr<sdk::Resource> {
            return std::make_unique<CameraRealSense>(deps, cfg);
        },
        validate);

    std::vector<std::shared_ptr<sdk::ModelRegistration>> mrs = {mr};
    auto module_service = std::make_shared<sdk::ModuleService>(argc, argv, mrs);

    {
        std::lock_guard<std::mutex> lock(g_realsense_module_lock);
        rs2_ctx.set_devices_changed_callback(global_device_changed_handler);
        if (module_level_debug.load()) {
            VIAM_SDK_LOG(info) << "global device changed callback registered.";
        }
    }

    module_service->serve();
    module_shutting_down.store(true);

    if (module_level_debug.load()) {
        VIAM_SDK_LOG(info) << "serve end. thread ID: "
                           << std::hash<std::thread::id>{}(std::this_thread::get_id());
    }
    return EXIT_SUCCESS;
}

}  // namespace realsense
}  // namespace viam
