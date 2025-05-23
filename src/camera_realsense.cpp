#include "camera_realsense.hpp"
#include "encoding.hpp"


#include <chrono>        
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector> 

#include <viam/sdk/log/logging.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/rpc/server.hpp>

namespace viam {
namespace realsense {

// align to the color camera's origin when color and depth enabled
const rs2::align FRAME_ALIGNMENT = RS2_STREAM_COLOR;

// initialize will use the ResourceConfigs to begin the realsense pipeline.
std::tuple<RealSenseProperties, bool, bool> CameraRealSense::initialize(sdk::ResourceConfig cfg) {
    if (device_ != nullptr) {
        VIAM_SDK_LOG(info) << "reinitializing, restarting pipeline";
        {
            // wait until frameLoop is stopped
            std::unique_lock<std::mutex> lock(device_->mutex);
            device_->shouldRun = false;
            device_->cv.wait(lock, [this] { return !(device_->isRunning); });
        }
    }
    VIAM_SDK_LOG(info) << "initializing the Intel RealSense Camera resource";
    // set variables from config
    uint width = 0;
    uint height = 0;
    auto attrs = cfg.attributes();

    if (attrs.count("width_px")) {
        if (const double* width_val = attrs["width_px"].get<double>()) {
            width = static_cast<uint>(*width_val);
        }
    }

    if (attrs.count("height_px")) {
        if (const double* height_val = attrs["height_px"].get<double>()) {
            height = static_cast<uint>(*height_val);
        }
    }

    if (width == 0 || height == 0) {
        VIAM_SDK_LOG(debug) << "note: will pick any suitable width and height";
    }
    std::string serial_number;
    if (attrs.count("serial_number")) {
        if (const std::string* serial_val = attrs["serial_number"].get<std::string>()) {
            serial_number = *serial_val;
        }
    }
    if (attrs.count("debug")) {
        if (const bool* debug_val = attrs["debug"].get<bool>()) {
            debug_enabled = *debug_val;
        }
    }

    bool littleEndianDepth = false;
    if (attrs.count("little_endian_depth")) {
        if (const bool* endian_depth = attrs["little_endian_depth"].get<bool>()) {
            littleEndianDepth = endian_depth;
        }
    }

    bool disableDepth = true;
    bool disableColor = true;
    std::vector<std::string> sensors;

    if (attrs.count("sensors")) {
        if (const sdk::ProtoList* sensor_list_ptr = attrs["sensors"].get<sdk::ProtoList>()) {
            for (const auto& element : *sensor_list_ptr) {
                if (const std::string* sensor_name = element.get<std::string>()) {
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

    VIAM_SDK_LOG(debug) << "disableDepth: " << disableDepth << " disableColor: " << disableColor
        << " sensors size " << sensors.size();

    // DeviceProperties context also holds a bool that can stop the thread if device gets
    // disconnected
    std::shared_ptr<DeviceProperties> newDevice = std::make_shared<DeviceProperties>(
        width, height, disableColor, width, height, disableDepth, this->latest_frames_);
    device_ = std::move(newDevice);
    device_->serial_number_to_use = serial_number;

    // First start of Pipeline
    rs2::pipeline pipe;
    RealSenseProperties props;
    std::tie(pipe, props) = startPipeline(disableDepth, width, height, disableColor, width, height, serial_number);
    // First start of camera thread
    props.sensors = sensors;
    props.mainSensor = sensors.front();
    VIAM_SDK_LOG(info) << "main sensor will be " << sensors.front();
    props.littleEndianDepth = littleEndianDepth;
    if (props.mainSensor == "depth") {
        VIAM_SDK_LOG(debug) << std::boolalpha << "depth little endian encoded: " << littleEndianDepth;
    }
    std::promise<void> ready;
    std::thread cameraThread(frameLoop, pipe, std::ref(ready), device_, props.depthScaleMm, std::ref(this->latest_frames_));
    VIAM_SDK_LOG(info) << "waiting for camera frame loop thread to be ready...";
    ready.get_future().wait();
    VIAM_SDK_LOG(info) << "camera frame loop ready!";
    cameraThread.detach();
    return std::make_tuple(props, disableColor, disableDepth);
}

CameraRealSense::CameraRealSense(sdk::Dependencies deps, sdk::ResourceConfig cfg)
    : Camera(cfg.name()) {
    RealSenseProperties props;
    bool disableColor;
    bool disableDepth;
    try {
        std::tie(props, disableColor, disableDepth) = initialize(cfg);
    } catch (const std::exception& e) {
        throw std::runtime_error("failed to initialize realsense: " + std::string(e.what()));
    }
    this->props_ = props;
    this->disableColor_ = disableColor;
    this->disableDepth_ = disableDepth;
}

CameraRealSense::~CameraRealSense() {
    // stop and wait for the frameLoop thread to exit
    if (!this->device_) return;
    // wait until frameLoop is stopped
    std::unique_lock<std::mutex> lock(this->device_->mutex);
    this->device_->shouldRun = false;
    this->device_->cv.wait(lock, [this] { return !(device_->isRunning); });
}

void CameraRealSense::reconfigure(const sdk::Dependencies& deps, const sdk::ResourceConfig& cfg) {
    RealSenseProperties props;
    bool disableColor;
    bool disableDepth;
    try {
        std::tie(props, disableColor, disableDepth) = initialize(cfg);
    } catch (const std::exception& e) {
        throw std::runtime_error("failed to reconfigure realsense: " + std::string(e.what()));
    }
    this->props_ = props;
    this->disableColor_ = disableColor;
    this->disableDepth_ = disableDepth;
}

sdk::Camera::raw_image CameraRealSense::get_image(std::string mime_type,
                                                  const sdk::ProtoStruct& extra) {
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
        if (this->disableColor_) {
            throw std::invalid_argument("color disabled");
        }
        if (mime_type.compare("image/png") == 0 || mime_type.compare("image/png+lazy") == 0) {
            response =
                encodeColorPNGToResponse((const void*)latestColorFrame.get_data(),
                                         this->props_.color.width, this->props_.color.height);
        } else if (mime_type.compare("image/vnd.viam.rgba") == 0) {
            response =
                encodeColorRAWToResponse((const unsigned char*)latestColorFrame.get_data(),
                                         this->props_.color.width, this->props_.color.height);
        } else {
            response = encodeJPEGToResponse((const unsigned char*)latestColorFrame.get_data(),
                                            this->props_.color.width, this->props_.color.height);
        }
    } else if (this->props_.mainSensor.compare("depth") == 0) {
        if (this->disableDepth_) {
            throw std::invalid_argument("depth disabled");
        }
        if (mime_type.compare("image/vnd.viam.dep") == 0) {
            response = encodeDepthRAWToResponse((const unsigned char*)latestDepthFrame->data(),
                                                this->props_.depth.width, this->props_.depth.height,
                                                this->props_.littleEndianDepth);
        } else {
            response =
                encodeDepthPNGToResponse((const unsigned char*)latestDepthFrame->data(),
                                         this->props_.depth.width, this->props_.depth.height);
        }
    }

    if (debug_enabled) {
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        VIAM_SDK_LOG(debug) << "[get_image]  total:           " << duration.count() << "ms\n";
    }

    return std::move(*response);
}

sdk::Camera::properties CameraRealSense::get_properties() {
    auto fillResp = [](sdk::Camera::properties* p, CameraProperties props) {
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
    } else if (props_.mainSensor.compare("depth") == 0) {
        fillResp(&response, this->props_.depth);
    }

    return response;
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

    for (const auto& sensor : this->props_.sensors) {
        if (sensor == "color") {
            std::unique_ptr<sdk::Camera::raw_image> color_response;
            color_response =
                encodeJPEGToResponse((const unsigned char*)latestColorFrame.get_data(),
                                     this->props_.color.width, this->props_.color.height);
            response.images.emplace_back(std::move(*color_response));
        } else if (sensor == "depth") {
            std::unique_ptr<sdk::Camera::raw_image> depth_response;
            depth_response = encodeDepthRAWToResponse(
                (const unsigned char*)latestDepthFrame->data(), this->props_.depth.width,
                this->props_.depth.height, this->props_.littleEndianDepth);
            response.images.emplace_back(std::move(*depth_response));
        }
    }

    response.metadata.captured_at = 
        sdk::time_pt{std::chrono::duration_cast<std::chrono::nanoseconds>(latestTimestamp)};
    
    if (debug_enabled) {
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        VIAM_SDK_LOG(debug) << "[get_images]  total:           " << duration.count() << "ms\n";
    }

    return response;
}

sdk::ProtoStruct CameraRealSense::do_command(const sdk::ProtoStruct& command) {
    VIAM_SDK_LOG(error) << "do_command not implemented";
    return sdk::ProtoStruct{};
}

sdk::Camera::point_cloud CameraRealSense::get_point_cloud(std::string mime_type,
                                                          const sdk::ProtoStruct& extra) {
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
        VIAM_SDK_LOG(debug) << "[get_point_cloud]  total:           " << duration.count() << "ms\n";
    }
    return sdk::Camera::point_cloud{mime_type, pcdBytes};
}

std::vector<sdk::GeometryConfig> CameraRealSense::get_geometries(const sdk::ProtoStruct& extra) {
    VIAM_SDK_LOG(error) << "get_geometries not implemented";
    return std::vector<sdk::GeometryConfig>{};
}

// Loop functions
void frameLoop(rs2::pipeline pipeline, std::promise<void>& ready,
               std::shared_ptr<DeviceProperties> deviceProps, float depthScaleMm,
               AtomicFrameSet& instance_latest_frames) {
    bool readyOnce = false;
    {
        std::lock_guard<std::mutex> lock(deviceProps->mutex);
        deviceProps->shouldRun = true;
        deviceProps->isRunning = true;
    }
    // start the callback function that will look for camera disconnects and reconnects.
    // on reconnects, it will close and restart the pipeline and thread.
    rs2::context ctx;
    ctx.set_devices_changed_callback(
        [&](rs2::event_information& info) { on_device_reconnect(info, pipeline, deviceProps); });
    VIAM_SDK_LOG(info) << "[frameLoop] frame loop is starting";
    while (true) {
        {
            std::lock_guard<std::mutex> lock(deviceProps->mutex);
            if (!deviceProps->shouldRun) {
                pipeline.stop();
                VIAM_SDK_LOG(info) << "[frameLoop] pipeline stopped, exiting frame loop";
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
        bool succ = pipeline.try_wait_for_frames(&frames, timeoutMillis);
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
            VIAM_SDK_LOG(debug) << "[frameLoop] wait for frames: " << duration.count() << "ms\n";
        }

        if (!deviceProps->disableColor && !deviceProps->disableDepth) {
            std::chrono::time_point<std::chrono::high_resolution_clock> start;
            if (debug_enabled) {
                start = std::chrono::high_resolution_clock::now();
            }

            try {
                frames = FRAME_ALIGNMENT.process(frames);
            } catch (const std::exception& e) {
                VIAM_SDK_LOG(error) << "[frameLoop] exception while aligning images: " << e.what();
                std::this_thread::sleep_for(failureWait);
                continue;
            }

            if (debug_enabled) {
                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
                VIAM_SDK_LOG(debug) << "[frameLoop] frame alignment: " << duration.count() << "ms\n";
            }
        }
        // scale every pixel value to be depth in units of mm
        std::unique_ptr<std::vector<uint16_t>> depthFrameScaled;
        if (!deviceProps->disableDepth) {
            auto depthFrame = frames.get_depth_frame();
            auto depthWidth = depthFrame.get_width();
            auto depthHeight = depthFrame.get_height();
            const uint16_t* depthFrameData = (const uint16_t*)depthFrame.get_data();
            depthFrameScaled = std::make_unique<std::vector<uint16_t>>(depthWidth * depthHeight);
            for (int y = 0; y < depthHeight; y++) {
                for (int x = 0; x < depthWidth; x++) {
                    auto px = (y * depthWidth) + x;
                    uint16_t depthScaled = depthScaleMm * depthFrameData[px];
                    (*depthFrameScaled)[px] = depthScaled;
                }
            }
        }
        {
            std::lock_guard<std::mutex> lock(instance_latest_frames.mutex);
            instance_latest_frames.colorFrame = frames.get_color_frame();
            instance_latest_frames.depthFrame = std::move(depthFrameScaled);
            instance_latest_frames.rsDepthFrame = frames.get_depth_frame();
            instance_latest_frames.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double, std::milli>(frames.get_timestamp()));
        }

        if (debug_enabled) {
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            VIAM_SDK_LOG(debug) << "[frameLoop] total:           " << duration.count() << "ms\n";
        }

        if (!readyOnce) {
            readyOnce = true;
            ready.set_value();
        }
    }
    {
        std::lock_guard<std::mutex> lock(deviceProps->mutex);
        deviceProps->isRunning = false;
    }
    deviceProps->cv.notify_all();
};

// gives the pixel to mm conversion for the depth sensor
float getDepthScale(rs2::device dev) {
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors()) {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>()) {
            return dpt.get_depth_scale() * 1000.0;  // rs2 gives pix2meters
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

std::tuple<rs2::pipeline, RealSenseProperties> startPipeline(bool disableDepth, int depthWidth,
                                                             int depthHeight, bool disableColor,
                                                             int colorWidth, int colorHeight,
                                                             const std::string& target_serial_number) {
    rs2::context ctx;
    auto devices = ctx.query_devices();
    if (devices.size() == 0) {
        throw std::runtime_error("no devices connected; please connect an Intel RealSense device");
    }

    rs2::device selected_device;
    for (auto&& dev : devices) {
        std::string current_serial_number = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        VIAM_SDK_LOG(info) << "found available RealSense device with serial number: " << current_serial_number;
        if (!target_serial_number.empty() && current_serial_number == target_serial_number) {
            selected_device = dev;
            // don't break because we want to log all available devices
        }
    }
    if (target_serial_number.empty()) {
        VIAM_SDK_LOG(info) << "no serial number specified in config, using first available device";
        selected_device = devices.front();
    }
    if (!selected_device) {
        throw std::runtime_error("no device found with specified serial number: " + target_serial_number);
    }

    auto serial_from_rs2 = selected_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    VIAM_SDK_LOG(info) << "starting pipeline with selected device:";
    VIAM_SDK_LOG(info) << "name:      " << selected_device.get_info(RS2_CAMERA_INFO_NAME);
    VIAM_SDK_LOG(info) << "serial:    " << serial_from_rs2;
    VIAM_SDK_LOG(info) << "firmware:  " << selected_device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
    VIAM_SDK_LOG(info) << "port:      " << selected_device.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT);
    VIAM_SDK_LOG(info) << "usb type:  " << selected_device.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);

    float depthScaleMm = 0.0;
    if (!disableDepth) {
        depthScaleMm = getDepthScale(selected_device);
    }

    rs2::config cfg;
    cfg.enable_device(serial_from_rs2);

    if (!disableColor) {
        VIAM_SDK_LOG(info) << "color width and height from config: (" << colorWidth << ", " << colorHeight
                  << ")";
        cfg.enable_stream(RS2_STREAM_COLOR, colorWidth, colorHeight, RS2_FORMAT_RGB8);
    }

    if (!disableDepth) {
        VIAM_SDK_LOG(info) << "depth width and height from config: (" << depthWidth << ", " << depthHeight
                  << ")";
        cfg.enable_stream(RS2_STREAM_DEPTH, depthWidth, depthHeight, RS2_FORMAT_Z16);
    }

    rs2::pipeline pipeline(ctx);
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
    props.serial_number = serial_from_rs2;
    if (!disableColor) {
        auto const stream = pipeline.get_active_profile()
                                .get_stream(RS2_STREAM_COLOR)
                                .as<rs2::video_stream_profile>();
        auto intrinsics = stream.get_intrinsics();
        props.color = fillProps(intrinsics, "brown_conrady");
    }
    if (!disableDepth) {
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
        VIAM_SDK_LOG(info) << "color_width:    " << props.color.width << "    color_height:   " << props.color.height;
    }

    VIAM_SDK_LOG(info) << "depth_enabled:  " << !disableDepth;
    if (!disableDepth) {
        auto alignedText = "";
        if (!disableColor) {
            alignedText = " (aligned to color)";
        }
        VIAM_SDK_LOG(info) << "depth_width:    " << props.depth.width << alignedText << " depth_height:   " << props.depth.height << alignedText;
    }

    return std::make_tuple(pipeline, props);
};

void on_device_reconnect(rs2::event_information& info, rs2::pipeline pipeline,
                         std::shared_ptr<DeviceProperties> device) {
    if (device == nullptr) {
        throw std::runtime_error(
            "no device info to reconnect to. RealSense device was never initialized.");
    }
    if (!device->shouldRun) {
        return;
    }

    bool device_found_to_reconnect = false;
    std::string target_serial_number = device->serial_number_to_use;

    for (auto&& new_dev : info.get_new_devices()) {
        if (info.was_added(new_dev)) { // assures we're checking a device that was part of the "added" event
            std::string new_dev_serial = new_dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            if (target_serial_number.empty()) {
                // No specific S/N configured, any new device is a candidate.
                VIAM_SDK_LOG(info) << "[on_device_reconnect] A new RealSense device (S/N: " << new_dev_serial
                          << ") was connected. Attempting to use it as no specific S/N was configured.";
                device_found_to_reconnect = true;
                break; 
            } else {
                // Specific S/N configured, check for a match.
                if (new_dev_serial == target_serial_number) {
                    VIAM_SDK_LOG(info) << "[on_device_reconnect] Configured device with S/N " << target_serial_number
                              << " reconnected.";
                    device_found_to_reconnect = true;
                    break;
                }
            }
        }
    }

    if (!device_found_to_reconnect) {
        return;
    }

    VIAM_SDK_LOG(info) << "[on_device_reconnect] Device was reconnected, restarting pipeline";
    {
        // wait until frameLoop is stopped
        std::unique_lock<std::mutex> lock(device->mutex);
        device->shouldRun = false;
        device->cv.wait(lock, [device] { return !(device->isRunning); });
    }
    
    RealSenseProperties props;
    try {
        std::tie(pipeline, props) =
            startPipeline(device->disableDepth, device->depthWidth, device->depthHeight,
                            device->disableColor, device->colorWidth, device->colorHeight,
                            target_serial_number);
    } catch (const std::exception& e) {
        VIAM_SDK_LOG(error) << "[on_device_reconnect] Failed to restart pipeline: " << e.what();
        std::lock_guard<std::mutex> lock(device->mutex);
        device->isRunning = false; 
        device->shouldRun = false; 
        return;
    }

    // Start the camera std::thread
    std::promise<void> ready;
    {
        std::lock_guard<std::mutex> lock(device->mutex);
        device->shouldRun = true;
    }
    std::thread cameraThread(frameLoop, pipeline, std::ref(ready), device, props.depthScaleMm, std::ref(device->atomic_frame_set));
    VIAM_SDK_LOG(info) << "waiting for camera frame loop thread to be ready...";
    ready.get_future().wait();
    VIAM_SDK_LOG(info) << "camera frame loop ready!";
    cameraThread.detach();
};

// validate will validate the ResourceConfig. If there is an error, it will throw an exception.
std::vector<std::string> validate(sdk::ResourceConfig cfg) {
    auto attrs = cfg.attributes();

    if (attrs.count("width_px")) {
        if (const double* width = attrs["width_px"].get<double>()) {
            if (static_cast<int>(*width) < 0) {
                throw std::invalid_argument("width_px cannot be negative");
            }
        }
    }

    if (attrs.count("height_px")) {
        if (const double* height = attrs["height_px"].get<double>()) {
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
        if (const sdk::ProtoList* sensors_list = attrs["sensors"].get<sdk::ProtoList>()) {
            if (sensors_list->empty()) {
                throw std::invalid_argument(
                    "sensors field cannot be empty, must list color and/or depth sensor");
            }
            for (const auto& sensor_element : *sensors_list) {
                if (!sensor_element.get<std::string>()) {
                    throw std::invalid_argument(
                        "elements in 'sensors' list must be strings (e.g., \"color\", "
                        "\"depth\")");
                }
            }
        } else {
            throw std::invalid_argument("'sensors' attribute must be a list of strings");
        }
    } else {
        throw std::invalid_argument("could not find required 'sensors' attribute in the config");
    }

    return {};
}

int serve(int argc, char** argv) {
    std::shared_ptr<sdk::ModelRegistration> mr = std::make_shared<sdk::ModelRegistration>(
        sdk::API::get<sdk::Camera>(), sdk::Model{kAPINamespace, kAPIType, kAPISubtype},
        [](sdk::Dependencies deps, sdk::ResourceConfig cfg) -> std::shared_ptr<sdk::Resource> {
            return std::make_unique<CameraRealSense>(deps, cfg);
        },
        validate);

    std::vector<std::shared_ptr<sdk::ModelRegistration>> mrs = {mr};
    auto module_service = std::make_shared<sdk::ModuleService>(argc, argv, mrs);
    module_service->serve();

    return EXIT_SUCCESS;
}

}  // namespace realsense
}  // namespace viam
