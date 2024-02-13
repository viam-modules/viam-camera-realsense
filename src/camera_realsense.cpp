#include "camera_realsense.hpp"

#include <arpa/inet.h>
#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <pthread.h>
#include <signal.h>
#include <turbojpeg.h>

#include <algorithm>
#include <condition_variable>
#include <future>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <tuple>
#include <vector>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/rpc/server.hpp>

#include "third_party/fpng.h"
#include "third_party/lodepng.h"

namespace {
bool debug_enabled = false;
const uint32_t rgbaMagicNumber =
    htonl(1380401729);  // the utf-8 binary encoding for "RGBA", big-endian
const size_t rgbaMagicByteCount =
    sizeof(uint32_t);  // number of bytes used to represent the rgba magic number
const size_t rgbaWidthByteCount =
    sizeof(uint32_t);  // number of bytes used to represent rgba image width
const size_t rgbaHeightByteCount =
    sizeof(uint32_t);  // number of bytes used to represent rgba image height

// COLOR responses
struct color_response {
    std::vector<uint8_t> color_bytes;
};

struct jpeg_image {
    std::unique_ptr<unsigned char, decltype(&tjFree)> data;
    long unsigned int size;
    // constructor
    jpeg_image(unsigned char* imageData, long unsigned int imageSize)
        : data(imageData, &tjFree), size(imageSize) {}
};

jpeg_image encodeJPEG(const unsigned char* data, const uint width, const uint height) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (debug_enabled) {
        start = std::chrono::high_resolution_clock::now();
    }

    unsigned char* encoded = nullptr;
    long unsigned int encodedSize = 0;
    tjhandle handle = tjInitCompress();
    if (handle == nullptr) {
        throw std::runtime_error("[GetImage] failed to init JPEG compressor");
    }
    int success;
    try {
        success = tjCompress2(handle, data, width, 0, height, TJPF_RGB, &encoded, &encodedSize,
                              TJSAMP_420, 75, TJFLAG_FASTDCT);
        tjDestroy(handle);
    } catch (const std::exception& e) {
        tjDestroy(handle);
        throw std::runtime_error("[GetImage] JPEG compressor failed to compress: " +
                                 std::string(e.what()));
    }
    if (success != 0) {
        throw std::runtime_error("[GetImage] JPEG compressor failed to compress image");
    }
    if (debug_enabled) {
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "[GetImage]  JPEG color encode:     " << duration.count() << "ms\n";
    }

    jpeg_image output(encoded, encodedSize);
    std::cout << "exiting encodeJPEG func" << std::endl;
    return output;
}

std::unique_ptr<viam::sdk::Camera::raw_image> encodeJPEGToResponse(const unsigned char* data,
                                                                   const uint width,
                                                                   const uint height) {
    jpeg_image encoded = encodeJPEG(data, width, height);
    auto response = std::make_unique<viam::sdk::Camera::raw_image>();
    response->source_name = "color";
    response->mime_type = "image/jpeg";
    response->bytes.assign(encoded.data.get(), encoded.data.get() + encoded.size);
    std::cout << "exiting encodeJPEGToResponse func" << std::endl;
    return response;
}
}  // namespace

namespace viam {
namespace realsense {

// Global AtomicFrameSet
AtomicFrameSet GLOBAL_LATEST_FRAMES;

// CAMERA module methods

// initialize will use the ResourceConfigs to begin the realsense pipeline.
RealSenseProperties CameraRealSense::initialize(sdk::ResourceConfig cfg) {
    if (device_ != nullptr) {
        std::cout << "reinitializing, restarting pipeline" << std::endl;
        {
            // wait until frameLoop is stopped
            std::unique_lock<std::mutex> lock(device_->mutex);
            device_->shouldRun = false;
            device_->cv.wait(lock, [this] { return !(device_->isRunning); });
        }
    }
    std::cout << "initializing the Intel RealSense Camera Module" << std::endl;
    // set variables from config
    uint width = 0;
    uint height = 0;
    auto attrs = cfg.attributes();
    if (attrs->count("width_px") == 1) {
        std::shared_ptr<sdk::ProtoType> width_proto = attrs->at("width_px");
        auto width_value = width_proto->proto_value();
        if (width_value.has_number_value()) {
            uint width_num = static_cast<uint>(width_value.number_value());
            width = width_num;
        }
    }
    if (attrs->count("height_px") == 1) {
        std::shared_ptr<sdk::ProtoType> height_proto = attrs->at("height_px");
        auto height_value = height_proto->proto_value();
        if (height_value.has_number_value()) {
            uint height_num = static_cast<uint>(height_value.number_value());
            height = height_num;
        }
    }
    if (width == 0 || height == 0) {
        std::cout << "note: will pick any suitable width and height" << std::endl;
    }
    if (attrs->count("debug") == 1) {
        std::shared_ptr<sdk::ProtoType> debug_proto = attrs->at("debug");
        auto debug_value = debug_proto->proto_value();
        if (debug_value.has_bool_value()) {
            bool debug_bool = static_cast<bool>(debug_value.bool_value());
            debug_enabled = debug_bool;
        }
    }
    std::vector<std::string> sensors;
    sensors.push_back("color");

    // DeviceProperties context also holds a bool that can stop the thread if device gets
    // disconnected
    std::shared_ptr<DeviceProperties> newDevice = std::make_shared<DeviceProperties>(
        width, height, width, height);
    device_ = std::move(newDevice);

    // First start of Pipeline
    rs2::pipeline pipe;
    RealSenseProperties props;
    std::tie(pipe, props) = startPipeline(width, height);
    // First start of camera thread
    props.sensors = sensors;
    props.mainSensor = sensors[0];
    std::promise<void> ready;
    std::thread cameraThread(frameLoop, pipe, ref(ready), device_);
    std::cout << "waiting for camera frame loop thread to be ready..." << std::endl;
    ready.get_future().wait();
    std::cout << "camera frame loop ready!" << std::endl;
    cameraThread.detach();
    std::cout << "exiting initialize func" << std::endl;
    return props;
}

CameraRealSense::CameraRealSense(sdk::Dependencies deps, sdk::ResourceConfig cfg)
    : Camera(cfg.name()) {
    RealSenseProperties props;
    try {
        props = initialize(cfg);
    } catch (const std::exception& e) {
        throw std::runtime_error("failed to initialize realsense: " + std::string(e.what()));
    }
    this->props_ = props;
    std::cout << "exiting CameraRealSense constructor func" << std::endl;
}

CameraRealSense::~CameraRealSense() {
    // stop and wait for the frameLoop thread to exit
    if (!this->device_) return;
    // wait until frameLoop is stopped
    std::unique_lock<std::mutex> lock(this->device_->mutex);
    this->device_->shouldRun = false;
    this->device_->cv.wait(lock, [this] { return !(device_->isRunning); });
    std::cout << "exiting CameraRealSense constructor func" << std::endl;
}

void CameraRealSense::reconfigure(sdk::Dependencies deps, sdk::ResourceConfig cfg) {
    RealSenseProperties props;
    try {
        props = initialize(cfg);
    } catch (const std::exception& e) {
        throw std::runtime_error("failed to reconfigure realsense: " + std::string(e.what()));
    }
    this->props_ = props;
    std::cout << "exiting reconfigure func" << std::endl;
}

sdk::Camera::raw_image CameraRealSense::get_image(std::string mime_type,
                                                  const sdk::AttributeMap& extra) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (debug_enabled) {
        start = std::chrono::high_resolution_clock::now();
    }

    rs2::frame latestColorFrame;
    {
        std::lock_guard<std::mutex> lock(GLOBAL_LATEST_FRAMES.mutex);
        latestColorFrame = GLOBAL_LATEST_FRAMES.colorFrame;
    }
    std::unique_ptr<sdk::Camera::raw_image> response;
    response = encodeJPEGToResponse((const unsigned char*)latestColorFrame.get_data(),
                                    this->props_.color.width, this->props_.color.height);

    if (debug_enabled) {
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "[get_image]  total:           " << duration.count() << "ms\n";
    }

    std::cout << "exiting get_image func" << std::endl;
    return std::move(*response);
}

sdk::Camera::properties CameraRealSense::get_properties() {
    auto fillResp = [](sdk::Camera::properties* p, CameraProperties props, bool supportsPCD) {
        p->supports_pcd = supportsPCD;
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
    // pcd enabling will be a config parameter, for now, just put false
    bool pcdEnabled = false;
    if (this->props_.mainSensor.compare("color") == 0) {
        fillResp(&response, this->props_.color, pcdEnabled);
    }

    std::cout << "exiting get_properties func" << std::endl;
    return response;
}

sdk::Camera::image_collection CameraRealSense::get_images() {
    std::cerr << "get_images not implemented in this version" << std::endl;
    return sdk::Camera::image_collection{};
}

sdk::AttributeMap CameraRealSense::do_command(sdk::AttributeMap command) {
    std::cerr << "do_command not implemented" << std::endl;
    return sdk::AttributeMap{};
}

sdk::Camera::point_cloud CameraRealSense::get_point_cloud(std::string mime_type,
                                                          const sdk::AttributeMap& extra) {
    std::cerr << "get_point_cloud not implemented" << std::endl;
    return sdk::Camera::point_cloud{};
}
std::vector<sdk::GeometryConfig> CameraRealSense::get_geometries(const sdk::AttributeMap& extra) {
    std::cerr << "get_geometries not implemented" << std::endl;
    return std::vector<sdk::GeometryConfig>{};
}

// Loop functions
void frameLoop(rs2::pipeline pipeline, std::promise<void>& ready,
               std::shared_ptr<DeviceProperties> deviceProps) {
    bool readyOnce = false;
    {
        std::lock_guard<std::mutex> lock(deviceProps->mutex);
        deviceProps->shouldRun = true;
        deviceProps->isRunning = true;
    }
    // start the callback function for device disconnection handling
    rs2::context ctx;
    ctx.set_devices_changed_callback(
        [&](rs2::event_information& info) { on_device_reconnect(info, pipeline, deviceProps); });
    std::cout << "[frameLoop] frame loop is starting" << std::endl;

    while (deviceProps->shouldRun) {
        rs2::frameset frames;
        const uint timeoutMillis = 2000; // Adjust as necessary
        if (pipeline.try_wait_for_frames(&frames, timeoutMillis)) {
            {
                std::lock_guard<std::mutex> lock(GLOBAL_LATEST_FRAMES.mutex);
                GLOBAL_LATEST_FRAMES.colorFrame = frames.get_color_frame();
                // Update timestamp or other relevant metadata if needed
            }
            if (!readyOnce) {
                readyOnce = true;
                ready.set_value();
            }
        } else {
            if (debug_enabled) {
                std::cerr << "[frameLoop] could not get frames from realsense after "
                          << timeoutMillis << "ms" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); // Minimal wait
        }
    }

    // Cleanup and notify
    pipeline.stop();
    std::lock_guard<std::mutex> lock(deviceProps->mutex);
    deviceProps->isRunning = false;
    deviceProps->cv.notify_all();
    std::cout << "exiting frameLoop func" << std::endl;
}

std::tuple<rs2::pipeline, RealSenseProperties> startPipeline(int colorWidth, int colorHeight) {
    rs2::context ctx;
    auto devices = ctx.query_devices();
    if (devices.size() == 0) {
        throw std::runtime_error("no device connected; please connect an Intel RealSense device");
    }
    rs2::device selected_device = devices.front();

    auto serial = selected_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    std::cout << "found device:\n";
    std::cout << "name:      " << selected_device.get_info(RS2_CAMERA_INFO_NAME) << "\n";
    std::cout << "serial:    " << serial << "\n";
    std::cout << "firmware:  " << selected_device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION)
              << "\n";
    std::cout << "port:      " << selected_device.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT) << "\n";
    std::cout << "usb type:  " << selected_device.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)
              << "\n";

    rs2::config cfg;
    cfg.enable_device(serial);

    std::cout << "color width and height from config: (" << colorWidth << ", " << colorHeight
                << ")\n";
    cfg.enable_stream(RS2_STREAM_COLOR, colorWidth, colorHeight, RS2_FORMAT_RGB8);

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
    auto const stream = pipeline.get_active_profile()
                            .get_stream(RS2_STREAM_COLOR)
                            .as<rs2::video_stream_profile>();
    auto intrinsics = stream.get_intrinsics();
    props.color = fillProps(intrinsics, "brown_conrady");

    std::cout << "pipeline started with:\n";
    std::cout << "color_width:    " << props.color.width << "\n";
    std::cout << "color_height:   " << props.color.height << "\n";

    std::cout << "exiting startPipeline func" << std::endl;
    return std::make_tuple(pipeline, props);
};

void on_device_reconnect(rs2::event_information& info, rs2::pipeline pipeline,
                         std::shared_ptr<DeviceProperties> device) {
    if (device == nullptr) {
        throw std::runtime_error(
            "no device info to reconnect to. RealSense device was never initialized.");
    }
    if (info.was_added(info.get_new_devices().front())) {
        std::cout << "Device was reconnected, restarting pipeline" << std::endl;
        {
            // wait until frameLoop is stopped
            std::unique_lock<std::mutex> lock(device->mutex);
            device->shouldRun = false;
            device->cv.wait(lock, [device] { return !(device->isRunning); });
        }
        // Find and start the first available device
        RealSenseProperties props;
        try {
            std::tie(pipeline, props) =
                startPipeline(device->colorWidth, device->colorHeight);
        } catch (const std::exception& e) {
            std::cout << "caught exception: \"" << e.what() << "\"" << std::endl;
            return;
        }
        // Start the camera std::thread
        std::promise<void> ready;
        std::thread cameraThread(frameLoop, pipeline, ref(ready), device);
        std::cout << "waiting for camera frame loop thread to be ready..." << std::endl;
        ready.get_future().wait();
        std::cout << "camera frame loop ready!" << std::endl;
        cameraThread.detach();
    } else {
        std::cout << "Device disconnected, stopping frame pipeline" << std::endl;
        {
            std::lock_guard<std::mutex> lock(device->mutex);
            device->shouldRun = false;
        }
    }
    std::cout << "exiting on_device_reconnect func" << std::endl;
};

// validate will validate the ResourceConfig. If there is an error, it will throw an exception.
std::vector<std::string> validate(sdk::ResourceConfig cfg) {
    auto attrs = cfg.attributes();
    if (attrs->count("width_px") == 1) {
        std::shared_ptr<sdk::ProtoType> width_proto = attrs->at("width_px");
        auto width_value = width_proto->proto_value();
        if (width_value.has_number_value()) {
            int width_num = static_cast<int>(width_value.number_value());
            if (width_num < 0) {
                throw std::invalid_argument("width_px cannot be negative");
            }
        }
    }
    if (attrs->count("height_px") == 1) {
        std::shared_ptr<sdk::ProtoType> height_proto = attrs->at("height_px");
        auto height_value = height_proto->proto_value();
        if (height_value.has_number_value()) {
            int height_num = static_cast<int>(height_value.number_value());
            if (height_num < 0) {
                throw std::invalid_argument("height_px cannot be negative");
            }
        }
    }
    std::cout << "exiting validate func" << std::endl;
    return {};
}

int serve(int argc, char** argv) {
    std::shared_ptr<sdk::ModelRegistration> mr = std::make_shared<sdk::ModelRegistration>(
        sdk::API::get<sdk::Camera>(),
        sdk::Model{kAPINamespace, kAPIType, kAPISubtype},
        [](sdk::Dependencies deps, sdk::ResourceConfig cfg) -> std::shared_ptr<sdk::Resource> {
            return std::make_unique<CameraRealSense>(deps, cfg);
        },
        validate
    );

    std::vector<std::shared_ptr<sdk::ModelRegistration>> mrs = {mr};
    auto module_service = std::make_shared<sdk::ModuleService>(argc, argv, mrs);
    module_service->serve();

    std::cout << "exiting serve func" << std::endl;
    return EXIT_SUCCESS;
}

}  // namespace realsense
}  // namespace viam
