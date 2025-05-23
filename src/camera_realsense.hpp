#pragma once
#include <algorithm>
#include <atomic>
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
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/components/component.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>
#include <viam/sdk/rpc/server.hpp>

namespace viam {
namespace realsense {

constexpr char kResourceType[] = "CameraRealSense";
constexpr char kAPINamespace[] = "viam";
constexpr char kAPIType[] = "camera";
constexpr char kAPISubtype[] = "realsense";

struct AtomicFrameSet {
    std::mutex mutex;
    rs2::frame colorFrame;
    std::shared_ptr<std::vector<uint16_t>> depthFrame;
    rs2::frame rsDepthFrame;
    std::chrono::milliseconds timestamp;
};

struct DeviceProperties {
    const uint colorWidth;
    const uint colorHeight;
    const bool disableColor;
    const uint depthWidth;
    const uint depthHeight;
    const bool disableDepth;
    std::string active_serial_number;
    AtomicFrameSet &atomic_frame_set;
    std::atomic<bool> shouldRun;
    std::atomic<bool> isRunning;
    std::mutex pipeline_mu;
    rs2::pipeline pipeline;
    rs2::align frame_alignment;

    // DeviceProperties constructor
    DeviceProperties(int colorWidth_, int colorHeight_, bool disableColor_, int depthWidth_,
                     int depthHeight_, bool disableDepth_, AtomicFrameSet &frames_ref)
        : colorWidth(colorWidth_),
          colorHeight(colorHeight_),
          disableColor(disableColor_),
          depthWidth(depthWidth_),
          depthHeight(depthHeight_),
          disableDepth(disableDepth_),
          atomic_frame_set(frames_ref),
          shouldRun(true),
          isRunning(false),
          frame_alignment(RS2_STREAM_COLOR) {}
};

struct CameraProperties {
    uint width;
    uint height;
    float fx;
    float fy;
    float ppx;
    float ppy;
    std::string distortionModel;
    double distortionParameters[5];
};

struct RealSenseProperties {
    CameraProperties color;
    CameraProperties depth;
    float depthScaleMm;
    std::string mainSensor;
    std::vector<std::string> sensors;
    bool littleEndianDepth;
};

struct PipelineWithProperties {
    rs2::pipeline pipeline;
    RealSenseProperties properties;
};

// The underlying realsense loop functions
float getDepthScale(rs2::device dev);
void frameLoop(std::promise<void> &ready, std::shared_ptr<DeviceProperties> deviceProps,
               float depthScaleMm, AtomicFrameSet &instance_latest_frames);
void on_device_reconnect(rs2::event_information &info, std::shared_ptr<DeviceProperties> device);
std::tuple<rs2::pipeline, RealSenseProperties> startPipeline(
    std::shared_ptr<DeviceProperties> device_props, std::string target_serial_number);

// Module functions
std::vector<std::string> validate(sdk::ResourceConfig cfg);
int serve(int argc, char **argv);

// Forward declaration
void global_device_changed_handler(rs2::event_information &info);

// The camera module class and its methods
class CameraRealSense : public sdk::Camera, public sdk::Reconfigurable {
   private:
    std::shared_ptr<DeviceProperties> device_;
    RealSenseProperties props_;
    AtomicFrameSet latest_frames_;

    RealSenseProperties initialize(sdk::ResourceConfig cfg);

   public:
    explicit CameraRealSense(sdk::Dependencies deps, sdk::ResourceConfig cfg);
    ~CameraRealSense();
    void reconfigure(const sdk::Dependencies &deps, const sdk::ResourceConfig &cfg) override;
    sdk::Camera::raw_image get_image(std::string mime_type, const sdk::ProtoStruct &extra) override;
    sdk::Camera::properties get_properties() override;
    sdk::Camera::image_collection get_images() override;
    sdk::ProtoStruct do_command(const sdk::ProtoStruct &command) override;
    sdk::Camera::point_cloud get_point_cloud(std::string mime_type,
                                             const sdk::ProtoStruct &extra) override;
    std::vector<sdk::GeometryConfig> get_geometries(const sdk::ProtoStruct &extra) override;
};

}  // namespace realsense
}  // namespace viam
