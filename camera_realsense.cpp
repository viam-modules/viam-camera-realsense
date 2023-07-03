#include <arpa/inet.h>
#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <pthread.h>
#include <signal.h>
#include <turbojpeg.h>

#include <future>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <tuple>
#include <vector>
#include <viam/sdk/components/camera/camera.hpp>
#include <viam/sdk/components/camera/server.hpp>
#include <viam/sdk/components/component.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/rpc/server.hpp>

#include "third_party/fpng.h"
#include "third_party/lodepng.h"

#define RESOURCE_TYPE "CameraRealSense"
#define API_NAMESPACE "viam"
#define API_TYPE "camera"
#define API_SUBTYPE "realsense"
#define htonll(x) \
    ((1 == htonl(1)) ? (x) : ((uint64_t)htonl((x)&0xFFFFFFFF) << 32) | htonl((x) >> 32))

using namespace std;
using namespace viam::sdk;
namespace vsdk = ::viam::sdk;

struct DeviceProperties {
    const int colorWidth;
    const int colorHeight;
    const bool disableColor;
    const int depthWidth;
    const int depthHeight;
    const bool disableDepth;
    bool shouldRun;
    bool isRunning;
    std::mutex mutex;

    DeviceProperties(int colorWidth_, int colorHeight_, bool disableColor_, int depthWidth_,
                     int depthHeight_, bool disableDepth_)
        : colorWidth(colorWidth_),
          colorHeight(colorHeight_),
          disableColor(disableColor_),
          depthWidth(depthWidth_),
          depthHeight(depthHeight_),
          disableDepth(disableDepth_),
          shouldRun(true),
          isRunning(false) {}
};

struct CameraProperties {
    int width;
    int height;
    float fx;
    float fy;
    float ppx;
    float ppy;
    string distortionModel;
    double distortionParameters[5];
};

struct RealSenseProperties {
    CameraProperties color;
    CameraProperties depth;
    float depthScaleMm;
    std::string mainSensor;
    bool littleEndianDepth;
	bool enablePointClouds;
};

struct PipelineWithProperties {
    rs2::pipeline pipeline;
    RealSenseProperties properties;
};

struct AtomicFrameSet {
    std::mutex mutex;
    rs2::frame colorFrame;
    shared_ptr<vector<uint16_t>> depthFrame;
};

// Global AtomicFrameSet
AtomicFrameSet GLOBAL_LATEST_FRAMES;

bool DEBUG = false;
const uint32_t rgbaMagicNumber =
    htonl(1380401729);  // the utf-8 binary encoding for "RGBA", big-endian
const size_t rgbaMagicByteCount =
    sizeof(uint32_t);  // number of bytes used to represent the rgba magic number
const size_t rgbaWidthByteCount =
    sizeof(uint32_t);  // number of bytes used to represent rgba image width
const size_t rgbaHeightByteCount =
    sizeof(uint32_t);  // number of bytes used to represent rgba image height

const uint64_t depthMagicNumber =
    htonll(4919426490892632400);  // the utf-8 binary encoding for "DEPTHMAP", big-endian
const size_t depthMagicByteCount =
    sizeof(uint64_t);  // number of bytes used to represent the depth magic number
const size_t depthWidthByteCount =
    sizeof(uint64_t);  // number of bytes used to represent depth image width
const size_t depthHeightByteCount =
    sizeof(uint64_t);  // number of bytes used to represent depth image height

// helper function to turn data pointer in a data vector for raw_image
std::vector<unsigned char> convertToVector(const unsigned char* data, size_t size) {
    const unsigned char* begin = data;
    const unsigned char* end = data + size;
    std::vector<unsigned char> vec(begin, end);
    return vec;
};

// COLOR responses
tuple<vector<uint8_t>, bool> encodeColorPNG(const uint8_t* data, const int width,
                                            const int height) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (DEBUG) {
        start = chrono::high_resolution_clock::now();
    }

    vector<uint8_t> encoded;
    if (!fpng::fpng_encode_image_to_memory(data, width, height, 3, encoded)) {
        cerr << "[GetImage]  failed to encode color PNG" << endl;
        return {encoded, false};
    }

    if (DEBUG) {
        auto stop = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
        cout << "[GetImage]  PNG color encode:      " << duration.count() << "ms\n";
    }

    return {encoded, true};
};

std::unique_ptr<vsdk::Camera::raw_image> encodeColorPNGToResponse(const uint8_t* data,
                                                                  const int width,
                                                                  const int height) {
    const auto& [encoded, ok] = encodeColorPNG(data, width, height);
    if (!ok) {
        throw std::runtime_error("failed to encode color PNG");
    }
    std::unique_ptr<vsdk::Camera::raw_image> response(new vsdk::Camera::raw_image{});
    response->mime_type = "image/png";
    response->bytes = encoded;
    return response;
    // response->bytes = std::vector<unsigned char>(encoded.data().begin(), encoded.data().end());
};

tuple<unsigned char*, long unsigned int, bool> encodeJPEG(const unsigned char* data,
                                                          const int width, const int height) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (DEBUG) {
        start = chrono::high_resolution_clock::now();
    }

    unsigned char* encoded = nullptr;
    long unsigned int encodedSize = 0;
    tjhandle handle = tjInitCompress();
    if (handle == nullptr) {
        cerr << "[GetImage]  failed to init JPEG compressor" << endl;
        return {encoded, encodedSize, false};
    }
    tjCompress2(handle, data, width, 0, height, TJPF_RGB, &encoded, &encodedSize, TJSAMP_420, 75,
                TJFLAG_FASTDCT);
    tjDestroy(handle);

    if (DEBUG) {
        auto stop = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
        cout << "[GetImage]  JPEG color encode:     " << duration.count() << "ms\n";
    }

    return {encoded, encodedSize, true};
};

std::unique_ptr<vsdk::Camera::raw_image> encodeJPEGToResponse(const unsigned char* data,
                                                              const int width, const int height) {
    const auto& [encoded, encodedSize, ok] = encodeJPEG(data, width, height);
    if (!ok) {
        tjFree(encoded);
        throw std::runtime_error("failed to encode color JPEG");
    }
    std::unique_ptr<vsdk::Camera::raw_image> response(new vsdk::Camera::raw_image{});
    response->mime_type = "image/jpeg";
    response->bytes = convertToVector(encoded, encodedSize);
    return response;
};

tuple<unsigned char*, size_t, bool> encodeColorRAW(const unsigned char* data, const uint32_t width,
                                                   const uint32_t height) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (DEBUG) {
        start = chrono::high_resolution_clock::now();
    }
    // set size of raw file
    size_t pixelByteCount = 4 * width * height;
    uint32_t widthToEncode = htonl(width);    // make sure everything is big-endian
    uint32_t heightToEncode = htonl(height);  // make sure everything is big-endian
    size_t totalByteCount =
        rgbaMagicByteCount + rgbaWidthByteCount + rgbaHeightByteCount + pixelByteCount;
    // memcpy data into buffer
    unsigned char* rawBuf = new unsigned char[totalByteCount];
    int offset = 0;
    std::memcpy(rawBuf + offset, &rgbaMagicNumber, rgbaMagicByteCount);
    offset += rgbaMagicByteCount;
    std::memcpy(rawBuf + offset, &widthToEncode, rgbaWidthByteCount);
    offset += rgbaWidthByteCount;
    std::memcpy(rawBuf + offset, &heightToEncode, rgbaHeightByteCount);
    offset += rgbaHeightByteCount;
    int pixelOffset = 0;
    uint8_t alphaValue = 255;  // alpha  channel is always 255 for color images
    for (int i = 0; i < width * height; i++) {
        std::memcpy(rawBuf + offset, data + pixelOffset, 3);  // 3 bytes for RGB
        std::memcpy(rawBuf + offset + 3, &alphaValue, 1);     // 1 byte for A
        pixelOffset += 3;
        offset += 4;
    }
    if (DEBUG) {
        auto stop = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
        cout << "[GetImage]  RAW color encode:      " << duration.count() << "ms\n";
    }

    return {rawBuf, totalByteCount, true};
};

std::unique_ptr<vsdk::Camera::raw_image> encodeColorRAWToResponse(const unsigned char* data,
                                                                  const uint width,
                                                                  const uint height) {
    const auto& [encoded, encodedSize, ok] = encodeColorRAW(data, width, height);
    if (!ok) {
        std::free(encoded);
        throw std::runtime_error("failed to encode color RAW");
    }
    std::unique_ptr<vsdk::Camera::raw_image> response(new vsdk::Camera::raw_image{});
    response->mime_type = "image/vnd.viam.rgba";
    response->bytes = convertToVector(encoded, encodedSize);
    return response;
};

// DEPTH responses
tuple<unsigned char*, size_t, bool> encodeDepthPNG(const unsigned char* data, const uint width,
                                                   const uint height) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (DEBUG) {
        start = chrono::high_resolution_clock::now();
    }

    unsigned char* encoded = 0;
    size_t encoded_size = 0;
    // convert data to guarantee big-endian
    size_t pixelByteCount = 2 * width * height;
    unsigned char* rawBuf = new unsigned char[pixelByteCount];
    int offset = 0;
    int pixelOffset = 0;
    for (int i = 0; i < width * height; i++) {
	uint16_t pix;
	std::memcpy(&pix, data + pixelOffset, 2);
	uint16_t pixEncode = htons(pix); // PNG expects pixel values to be big-endian
	std::memcpy(rawBuf + offset, &pixEncode, 2); 
        pixelOffset += 2;
        offset += 2;
    }
    unsigned result =
        lodepng_encode_memory(&encoded, &encoded_size, rawBuf, width, height, LCT_GREY, 16);
    if (result != 0) {
        cerr << "[GetImage]  failed to encode depth PNG" << endl;
        return {encoded, encoded_size, false};
    }

    if (DEBUG) {
        auto stop = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
        cout << "[GetImage]  PNG depth encode:      " << duration.count() << "ms\n";
    }

    return {encoded, encoded_size, true};
};

std::unique_ptr<vsdk::Camera::raw_image> encodeDepthPNGToResponse(const unsigned char* data,
                                                                  const uint width,
                                                                  const uint height) {
    const auto& [encoded, encoded_size, ok] = encodeDepthPNG(data, width, height);
    if (!ok) {
        std::free(encoded);
        throw std::runtime_error("failed to encode depth PNG");
    }
    std::unique_ptr<vsdk::Camera::raw_image> response(new vsdk::Camera::raw_image{});
    response->mime_type = "image/png";
    response->bytes = convertToVector(encoded, encoded_size);
    return response;
};

tuple<unsigned char*, size_t, bool> encodeDepthRAW(const unsigned char* data, const uint64_t width,
                                                   const uint64_t height, const bool littleEndian) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (DEBUG) {
        start = chrono::high_resolution_clock::now();
    }
    // Depth header contains 8 bytes worth of magic number, followed by 8 bytes for width and
    // another 8 bytes for height each pixel has 2 bytes.
    size_t pixelByteCount = 2 * width * height;
    uint64_t widthToEncode = htonll(width);    // make sure everything is big-endian
    uint64_t heightToEncode = htonll(height);  // make sure everything is big-endian
    size_t totalByteCount =
        depthMagicByteCount + depthWidthByteCount + depthHeightByteCount + pixelByteCount;
    // memcpy data into buffer
    unsigned char* rawBuf = new unsigned char[totalByteCount];
    int offset = 0;
    std::memcpy(rawBuf + offset, &depthMagicNumber, depthMagicByteCount);
    offset += depthMagicByteCount;
    std::memcpy(rawBuf + offset, &widthToEncode, depthWidthByteCount);
    offset += depthWidthByteCount;
    std::memcpy(rawBuf + offset, &heightToEncode, depthHeightByteCount);
    offset += depthHeightByteCount;
	if (littleEndian) {
    	std::memcpy(rawBuf + offset, data, pixelByteCount);
	} else {
		int pixelOffset = 0;
		for (int i = 0; i < width * height; i++) {
            uint16_t pix;
            std::memcpy(&pix, data + pixelOffset, 2);
            uint16_t pixEncode = htons(pix); // make sure the pixel values are big-endian 
            std::memcpy(rawBuf + offset, &pixEncode, 2); 
			pixelOffset += 2;
			offset += 2;
		}
	}

    if (DEBUG) {
        auto stop = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
        cout << "[GetImage]  RAW depth encode:      " << duration.count() << "ms\n";
    }

    return {rawBuf, totalByteCount, true};
};

std::unique_ptr<vsdk::Camera::raw_image> encodeDepthRAWToResponse(const unsigned char* data,
                                                                  const uint width,
                                                                  const uint height, const bool littleEndian) {
    const auto& [encoded, encodedSize, ok] = encodeDepthRAW(data, width, height, littleEndian);
    if (!ok) {
        std::free(encoded);
        throw std::runtime_error("failed to encode depth RAW");
    }
    std::unique_ptr<vsdk::Camera::raw_image> response(new vsdk::Camera::raw_image{});
    response->mime_type = "image/vnd.viam.dep";
    response->bytes = convertToVector(encoded, encodedSize);
    return response;
};

// Loop functions

// align to the color camera's origin when color and depth enabled
const rs2::align FRAME_ALIGNMENT = RS2_STREAM_COLOR;

void frameLoop(rs2::pipeline pipeline, promise<void>& ready, std::shared_ptr<DeviceProperties> deviceProps,
               float depthScaleMm) {
    bool readyOnce = false;
    {
        std::lock_guard<std::mutex> lock(deviceProps->mutex);
        deviceProps->shouldRun = true;
        deviceProps->isRunning = true;
    }
    while (true) {
        {
            std::lock_guard<std::mutex> lock(deviceProps->mutex);
            if (!deviceProps->shouldRun) {
                pipeline.stop();
                cout << "[frameLoop] pipeline stopped exiting thread" << endl;
                deviceProps->isRunning = false;
                break;
            }
        }
        auto failureWait = 5ms;

        auto start = chrono::high_resolution_clock::now();

        rs2::frameset frames;
        const uint timeoutMillis = 2000;
        /*
            D435 1920x1080 RGB + Depth ~20ms on a Raspberry Pi 4 Model B
        */
        bool succ = pipeline.try_wait_for_frames(&frames, timeoutMillis);
        if (!succ) {
            if (DEBUG) {
                cerr << "[frameLoop] could not get frames from realsense after " << timeoutMillis
                     << "ms" << endl;
            }
            this_thread::sleep_for(failureWait);
            continue;
        }
        if (DEBUG) {
            auto stop = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
            cout << "[frameLoop] wait for frames: " << duration.count() << "ms\n";
        }

        if (!deviceProps->disableColor && !deviceProps->disableDepth) {
            auto start = chrono::high_resolution_clock::now();

            try {
                frames = FRAME_ALIGNMENT.process(frames);
            } catch (const exception& e) {
                cerr << "[frameLoop] exception while aligning images: " << e.what() << endl;
                this_thread::sleep_for(failureWait);
                continue;
            }

            if (DEBUG) {
                auto stop = chrono::high_resolution_clock::now();
                auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
                cout << "[frameLoop] frame alignment: " << duration.count() << "ms\n";
            }
        }
        // scale every pixel value to be depth in units of mm
        unique_ptr<vector<uint16_t>> depthFrameScaled;
        if (!deviceProps->disableDepth) {
            auto depthFrame = frames.get_depth_frame();
            auto depthWidth = depthFrame.get_width();
            auto depthHeight = depthFrame.get_height();
            const uint16_t* depthFrameData = (const uint16_t*)depthFrame.get_data();
            // NOTE(erd): this is fast enough in -O3 (1920x1080 -> ~15ms) but could probably be
            // better
            depthFrameScaled = make_unique<vector<uint16_t>>(depthWidth * depthHeight);
            for (int y = 0; y < depthHeight; y++) {
                for (int x = 0; x < depthWidth; x++) {
                    auto px = (y * depthWidth) + x;
                    uint16_t depthScaled = depthScaleMm * depthFrameData[px];
                    (*depthFrameScaled)[px] = depthScaled;
                }
            }
        }
        GLOBAL_LATEST_FRAMES.mutex.lock();
        GLOBAL_LATEST_FRAMES.colorFrame = frames.get_color_frame();
        GLOBAL_LATEST_FRAMES.depthFrame = std::move(depthFrameScaled);
        GLOBAL_LATEST_FRAMES.mutex.unlock();

        if (DEBUG) {
            auto stop = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
            cout << "[frameLoop] total:           " << duration.count() << "ms\n";
        }

        if (!readyOnce) {
            readyOnce = true;
            ready.set_value();
        }
    }
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

tuple<rs2::pipeline, RealSenseProperties> startPipeline(bool disableDepth, int depthWidth, int depthHeight, bool disableColor, int colorWidth, int colorHeight) {
    rs2::context ctx;
    auto devices = ctx.query_devices();
    if (devices.size() == 0) {
        throw runtime_error("no device connected; please connect an Intel RealSense device");
    }
    rs2::device selected_device = devices.front();

    auto serial = selected_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    cout << "found device:\n";
    cout << "name:      " << selected_device.get_info(RS2_CAMERA_INFO_NAME) << "\n";
    cout << "serial:    " << serial << "\n";
    cout << "firmware:  " << selected_device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION) << "\n";
    cout << "port:      " << selected_device.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT) << "\n";
    cout << "usb type:  " << selected_device.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) << "\n";

    float depthScaleMm = 0.0;
    if (!disableDepth) {
        depthScaleMm = getDepthScale(selected_device);
    }

    rs2::config cfg;
    cfg.enable_device(serial);

    if (!disableColor) {
        cout << "color width and height from config: (" << colorWidth << ", " << colorHeight << ")\n";
        cfg.enable_stream(RS2_STREAM_COLOR, colorWidth, colorHeight,
                          RS2_FORMAT_RGB8);
    }

    if (!disableDepth) {
        cout << "depth width and height from config(" << depthWidth << ", " << depthHeight << ")\n";
        cfg.enable_stream(RS2_STREAM_DEPTH, depthWidth, depthHeight,
                          RS2_FORMAT_Z16);
    }

    rs2::pipeline pipeline(ctx);
    pipeline.start(cfg);


    auto fillProps = [](auto intrinsics, string distortionModel) -> CameraProperties {
        CameraProperties camProps;
        camProps.width = intrinsics.width;
        camProps.height = intrinsics.height;
        camProps.fx = intrinsics.fx;
        camProps.fy = intrinsics.fy;
        camProps.ppx = intrinsics.ppx;
        camProps.ppy = intrinsics.ppy;
        camProps.distortionModel = distortionModel;
        for (int i = 0; i < 5; i++) {
            camProps.distortionParameters[i] = double(intrinsics.coeffs[i]);
        }
        return camProps;
    };

    RealSenseProperties props;
    props.depthScaleMm = depthScaleMm;
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
        props.depth = fillProps(intrinsics, "no_distortion");
        if (!disableColor) {
            props.depth.width = props.color.width;
            props.depth.height = props.color.height;
        }
    }

    cout << "pipeline started with:\n";
    cout << "color_enabled:  " << boolalpha << !disableColor << "\n";
    if (!disableColor) {
        cout << "color_width:    " << props.color.width << "\n";
        cout << "color_height:   " << props.color.height << "\n";
    }
    cout << "depth_enabled:  " << !disableDepth << endl;
    if (!disableDepth) {
        auto alignedText = "";
        if (!disableColor) {
            alignedText = " (aligned to color)";
        }
        cout << "depth_width:    " << props.depth.width << alignedText << "\n";
        cout << "depth_height:   " << props.depth.height << alignedText << endl;
    }

    return make_tuple(pipeline, props);
};

constexpr char service_name[] = "camera_realsense";

// CAMERA module
class CameraRealSense : public vsdk::Camera {
   private:
    std::shared_ptr<DeviceProperties> device_;
    RealSenseProperties props_;
    bool disableColor_;
    bool disableDepth_;

    // initialize will use the ResourceConfigs to begin the realsense pipeline.
    tuple<RealSenseProperties, bool, bool> initialize(vsdk::ResourceConfig cfg) {
        if (device_ != nullptr) {
            std::lock_guard<std::mutex> lock(device_->mutex);
            device_->shouldRun = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        cout << "initializing the Intel RealSense Camera Module" << endl;
        // set variables from config
        int width = 0;
        int height = 0;
        auto attrs = cfg.attributes();
        if (attrs->count("width_px") == 1) {
            std::shared_ptr<ProtoType> width_proto = attrs->at("width_px");
            auto width_value = width_proto->proto_value();
            if (width_value.has_number_value()) {
                int width_num = static_cast<int>(width_value.number_value());
                width = width_num;
            }
        }
        if (attrs->count("height_px") == 1) {
            std::shared_ptr<ProtoType> height_proto = attrs->at("height_px");
            auto height_value = height_proto->proto_value();
            if (height_value.has_number_value()) {
                int height_num = static_cast<int>(height_value.number_value());
                height = height_num;
            }
        }
        if (width == 0 || height == 0) {
            cout << "note: will pick any suitable width and height" << endl;
        }
        if (attrs->count("debug") == 1) {
            std::shared_ptr<ProtoType> debug_proto = attrs->at("debug");
            auto debug_value = debug_proto->proto_value();
            if (debug_value.has_bool_value()) {
                bool debug_bool = static_cast<bool>(debug_value.bool_value());
                DEBUG = debug_bool;
            }
        }
        bool littleEndianDepth = false;
        if (attrs->count("little_endian_depth") == 1) {
            std::shared_ptr<ProtoType> endian_proto = attrs->at("little_endian_depth");
            auto endian_value = endian_proto->proto_value();
            if (endian_value.has_bool_value()) {
                bool endian_bool = static_cast<bool>(endian_value.bool_value());
                littleEndianDepth = endian_bool;
            }
        }
        bool enablePointClouds = false;
        if (attrs->count("enable_point_clouds") == 1) {
            std::shared_ptr<ProtoType> pointclouds_proto = attrs->at("enable_point_clouds");
            auto pointclouds_value = pointclouds_proto->proto_value();
            if (pointclouds_value.has_bool_value()) {
                bool pointclouds_bool = static_cast<bool>(pointclouds_value.bool_value());
                enablePointClouds = pointclouds_bool;
            }
        }
        bool disableDepth = true;
        bool disableColor = true;
        std::vector<std::string> sensors;
        if (attrs->count("sensors") == 1) {
            std::shared_ptr<ProtoType> sensor_proto = attrs->at("sensors");
            auto sensor_value = sensor_proto->proto_value();
            if (sensor_value.has_list_value()) {
                auto sensor_list = sensor_value.list_value();
                for (const auto element : sensor_list.values()) {
                    if (element.has_string_value()) {
                        std::string sensor_name = static_cast<std::string>(element.string_value());
                        if (sensor_name == "color") {
                            disableColor = false;
                            sensors.push_back("color");
                        }
                        if (sensor_name == "depth") {
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

        try {
            // DeviceProperties context also holds a bool that can stop the thread if device gets
            // disconnected
            std::shared_ptr<DeviceProperties> newDevice = std::make_shared<DeviceProperties>(width, height, disableColor, width, height, disableDepth);
            device_ = std::move(newDevice);

            // First start of Pipeline
            rs2::pipeline pipe;
            RealSenseProperties props;
            tie(pipe, props) = startPipeline(disableDepth, width, height, disableColor, width, height);
            // First start of camera thread
            props.mainSensor = sensors[0];
            cout << "main sensor will be " << sensors[0] << endl;
            props.littleEndianDepth = littleEndianDepth;
            if (props.mainSensor == "depth") {
                std::string endianString = (littleEndianDepth) ? "true" : "false";
                cout << "depth little endian encoded: " << endianString << endl;
            }
            props.enablePointClouds = enablePointClouds;
            std::string pointcloudString = (enablePointClouds) ? "true" : "false";
            cout << "point clouds enabled: " << pointcloudString << endl;
            promise<void> ready;
            thread cameraThread(frameLoop, pipe, ref(ready), device_, props.depthScaleMm);
            cout << "waiting for camera frame loop thread to be ready..." << flush;
            ready.get_future().wait();
            cout << " ready!" << endl;
            // start the callback function that will look for camera disconnects and reconnects.
            // on reconnects, it will close and restart the pipeline and thread.
            rs2::context ctx;
            ctx.set_devices_changed_callback(
                [&](rs2::event_information& info) { on_device_reconnect(info, pipe); });
            cameraThread.detach();
            return make_tuple(props, disableColor, disableDepth);
        } catch (const exception& e) {
            throw std::runtime_error("failed to initialize realsense: " + std::string(e.what()));
        }
    }

    void on_device_reconnect(rs2::event_information& info, rs2::pipeline pipeline) {
        if (device_ == nullptr) {
            throw std::runtime_error("no device info to reconnect to. RealSense device was never initialized.");
        }
        if (info.was_added(info.get_new_devices().front())) {
            std::cout << "Device was reconnected, restarting pipeline" << std::endl;
            {
                std::lock_guard<std::mutex> lock(device_->mutex);
                device_->shouldRun = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            // Find and start the first available device
            RealSenseProperties props;
            try {
                tie(pipeline, props) = startPipeline(device_->disableDepth, device_->depthWidth, device_->depthHeight, device_->disableColor, device_->colorWidth, device_->colorHeight);
            } catch (const exception& e) {
                cout << "caught exception: \"" << e.what() << "\"" << endl;
                return;
            }
            // Start the camera thread
            promise<void> ready;
            thread cameraThread(frameLoop, pipeline, ref(ready), device_, props.depthScaleMm);
            cout << "waiting for camera frame loop thread to be ready..." << flush;
            ready.get_future().wait();
            cout << " ready!" << endl;
            cameraThread.detach();
        } else {
            std::cout << "Device disconnected, stopping frame pipeline" << std::endl;
            {
                std::lock_guard<std::mutex> lock(device_->mutex);
                device_->shouldRun = false;
            }
        }
    };


   public:
    explicit CameraRealSense(vsdk::Dependencies deps, vsdk::ResourceConfig cfg)
        : Camera(cfg.name()) {
        RealSenseProperties props;
        bool disableColor;
        bool disableDepth;
        tie(props, disableColor, disableDepth) = initialize(cfg);
        this->props_ = props;
        this->disableColor_ = disableColor;
        this->disableDepth_ = disableDepth;
    }

    void reconfigure(vsdk::Dependencies deps, vsdk::ResourceConfig cfg) override {
        RealSenseProperties props;
        bool disableColor;
        bool disableDepth;
        tie(props, disableColor, disableDepth) = initialize(cfg);
        this->props_ = props;
        this->disableColor_ = disableColor;
        this->disableDepth_ = disableDepth;
    }

    vsdk::Camera::raw_image get_image(std::string mime_type) override {
        auto start = chrono::high_resolution_clock::now();

        // FUTURE(erd): we could track the last frame encode so as to not duplicate work if we
        // are ahead of the frame loop.
        GLOBAL_LATEST_FRAMES.mutex.lock();
        auto latestColorFrame = GLOBAL_LATEST_FRAMES.colorFrame;
        auto latestDepthFrame = GLOBAL_LATEST_FRAMES.depthFrame;
        GLOBAL_LATEST_FRAMES.mutex.unlock();

        std::unique_ptr<vsdk::Camera::raw_image> response;
        if (this->props_.mainSensor.compare("color") == 0) {
            if (this->disableColor_) {
                throw std::invalid_argument("color disabled");
            }
            if (mime_type.compare("image/png") == 0 || mime_type.compare("image/png+lazy") == 0) {
                response =
                    encodeColorPNGToResponse((const uint8_t*)latestColorFrame.get_data(),
                                             this->props_.color.width, this->props_.color.height);
            } else if (mime_type.compare("image/vnd.viam.rgba") == 0) {
                response =
                    encodeColorRAWToResponse((const unsigned char*)latestColorFrame.get_data(),
                                             this->props_.color.width, this->props_.color.height);
            } else {
                response =
                    encodeJPEGToResponse((const unsigned char*)latestColorFrame.get_data(),
                                         this->props_.color.width, this->props_.color.height);
            }
        } else if (this->props_.mainSensor.compare("depth") == 0) {
            if (this->disableDepth_) {
                throw std::invalid_argument("depth disabled");
            }
            if (mime_type.compare("image/vnd.viam.dep") == 0) {
                response =
                    encodeDepthRAWToResponse((const unsigned char*)latestDepthFrame->data(),
                                             this->props_.depth.width, this->props_.depth.height, this->props_.littleEndianDepth);
            } else {
                response =
                    encodeDepthPNGToResponse((const unsigned char*)latestDepthFrame->data(),
                                             this->props_.depth.width, this->props_.depth.height);
            }
        }

        if (DEBUG) {
            auto stop = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
            cout << "[get_image]  total:           " << duration.count() << "ms\n";
        }

        return *response;
    }

    vsdk::Camera::properties get_properties() override {
        auto fillResp = [](vsdk::Camera::properties* p, CameraProperties props, bool supportsPCD) {
            p->supports_pcd = supportsPCD;
            p->intrinsic_parameters.width_px = props.width;
            p->intrinsic_parameters.height_px = props.height;
            p->intrinsic_parameters.focal_x_px = props.fx;
            p->intrinsic_parameters.focal_y_px = props.fy;
            p->intrinsic_parameters.center_x_px = props.ppx;
            p->intrinsic_parameters.center_y_px = props.ppy;
            p->distortion_parameters.model = props.distortionModel;
            for (int i = 0; i < 5; i++) {
                p->distortion_parameters.parameters.push_back(props.distortionParameters[i]);
            }
        };

        vsdk::Camera::properties response{};
        // pcd enabling will be a config parameter, for now, just put false
        bool pcdEnabled = false;
        if (this->props_.mainSensor.compare("color") == 0) {
            fillResp(&response, this->props_.color, pcdEnabled);
        } else if (props_.mainSensor.compare("depth") == 0) {
            fillResp(&response, this->props_.depth, pcdEnabled);
        }

        return response;
    }

    vsdk::AttributeMap do_command(vsdk::AttributeMap command) override {
        std::cerr << "do_command not implemented" << std::endl;
        return 0;
    }
    vsdk::Camera::point_cloud get_point_cloud(std::string mime_type) override {
        std::cerr << "get_point_cloud not implemented" << std::endl;
        return vsdk::Camera::point_cloud{};
    }
    std::vector<vsdk::GeometryConfig> get_geometries() override {
        std::cerr << "get_geometries not implemented" << std::endl;
        return std::vector<vsdk::GeometryConfig>{};
    }
};

// validate will validate the ResourceConfig. If there is an error, it will throw an exception.
std::vector<std::string> validate(vsdk::ResourceConfig cfg) { return {}; }

int serve(const std::string& socket_path) {
    sigset_t sigset;
    sigemptyset(&sigset);
    sigaddset(&sigset, SIGINT);
    sigaddset(&sigset, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &sigset, NULL);

    auto module_registration = std::make_shared<vsdk::ModelRegistration>(
        vsdk::ResourceType{RESOURCE_TYPE}, vsdk::Camera::static_api(),
        vsdk::Model{API_NAMESPACE, API_TYPE, API_SUBTYPE},
        [](vsdk::Dependencies deps, vsdk::ResourceConfig cfg) -> std::shared_ptr<vsdk::Resource> {
            return std::make_shared<CameraRealSense>(deps, cfg);
        },
        [](vsdk::ResourceConfig cfg) -> std::vector<std::string> { return validate(cfg); }
    );

    try {
        vsdk::Registry::register_model(module_registration);
        std::cout << "registered model " << API_NAMESPACE <<  ":" << API_TYPE << ":" << API_SUBTYPE << std::endl;
    } catch (const std::runtime_error& e) {
        std::cerr << "error registering model: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    auto module_service = std::make_shared<vsdk::ModuleService_>(socket_path);

    auto server = std::make_shared<vsdk::Server>();
    module_service->add_model_from_registry(server, module_registration->api(),
                                            module_registration->model());

    module_service->start(server);

    std::thread server_thread([&server, &sigset]() {
        server->start();
        int sig = 0;
        auto result = sigwait(&sigset, &sig);
        server->shutdown();
    });

    server->wait();
    server_thread.join();

    return EXIT_SUCCESS;
}

int main(int argc, char* argv[]) {
    const std::string usage = "usage: camera_realsense /path/to/unix/socket";

    if (argc < 2) {
        std::cout << "ERROR: insufficient arguments\n";
        std::cout << usage << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "About to servce on socket " << argv[1] << std::endl;

    return serve(argv[1]);
}
