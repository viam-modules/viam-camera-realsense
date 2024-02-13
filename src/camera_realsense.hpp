#pragma once
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
#include <viam/sdk/components/camera/camera.hpp>
#include <viam/sdk/components/camera/server.hpp>
#include <viam/sdk/components/component.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/rpc/server.hpp>

namespace viam {
namespace realsense {

constexpr char kResourceType[] = "CameraRealSense";
constexpr char kAPINamespace[] = "viam";
constexpr char kAPIType[] = "camera";
constexpr char kAPISubtype[] = "realsense";

struct DeviceProperties {
    const uint colorWidth;
    const uint colorHeight;
    const uint depthWidth;
    const uint depthHeight;
    bool shouldRun;
    bool isRunning;
    std::condition_variable cv;
    std::mutex mutex;
    // DeviceProperties construtor
    DeviceProperties(int colorWidth_, int colorHeight_, int depthWidth_,
                     int depthHeight_)
        : colorWidth(colorWidth_),
          colorHeight(colorHeight_),
          depthWidth(depthWidth_),
          depthHeight(depthHeight_),
          shouldRun(true),
          isRunning(false) {}
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
    bool enablePointClouds;
};

struct PipelineWithProperties {
    rs2::pipeline pipeline;
    RealSenseProperties properties;
};

struct AtomicFrameSet {
    std::mutex mutex;
    rs2::frame colorFrame;
    std::shared_ptr<std::vector<uint16_t>> depthFrame;
    std::chrono::milliseconds timestamp;
};

// The underlying realsense loop functions
float getDepthScale(rs2::device dev);
void frameLoop(rs2::pipeline pipeline, std::promise<void>& ready,
               std::shared_ptr<DeviceProperties> devicePropss);
void on_device_reconnect(rs2::event_information& info, rs2::pipeline pipeline,
                         std::shared_ptr<DeviceProperties> device);
std::tuple<rs2::pipeline, RealSenseProperties> startPipeline(int colorWidth, int colorHeight);

// Module functions
std::vector<std::string> validate(sdk::ResourceConfig cfg);
int serve(int argc, char** argv);

// The camera module class and its methods
class CameraRealSense : public sdk::Camera {
   private:
    std::shared_ptr<DeviceProperties> device_;
    RealSenseProperties props_;
    RealSenseProperties initialize(sdk::ResourceConfig cfg);

   public:
    explicit CameraRealSense(sdk::Dependencies deps, sdk::ResourceConfig cfg);
    ~CameraRealSense();
    void reconfigure(sdk::Dependencies deps, sdk::ResourceConfig cfg) override;
    sdk::Camera::raw_image get_image(std::string mime_type,
                                     const sdk::AttributeMap& extra) override;
    sdk::Camera::properties get_properties() override;
    sdk::Camera::image_collection get_images() override;
    sdk::AttributeMap do_command(sdk::AttributeMap command) override;
    sdk::Camera::point_cloud get_point_cloud(std::string mime_type,
                                             const sdk::AttributeMap& extra) override;
    std::vector<sdk::GeometryConfig> get_geometries(const sdk::AttributeMap& extra) override;
};

}  // namespace realsense
}  // namespace viam
