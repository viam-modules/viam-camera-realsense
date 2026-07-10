#pragma once

#include "time.hpp"

#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_set>

#include <viam/sdk/log/logging.hpp>

#include <boost/thread/synchronized_value.hpp>
#include <librealsense2/rs.hpp>

namespace realsense {
namespace device {
class PointCloudFilter {
public:
  PointCloudFilter()
      : pointcloud_(std::make_shared<rs2::pointcloud>()),
        align_to_color_(std::make_shared<rs2::align>(RS2_STREAM_COLOR)) {}
  std::pair<rs2::points, rs2::video_frame> process(rs2::frameset frameset) {
    // This body only runs against a live RealSense: every statement operates on
    // librealsense frame/align/pointcloud objects that cannot be constructed or
    // driven without hardware, so it is unreachable in the (camera-less) CI test
    // suite and is excluded from coverage. Validated on-device via the alignment
    // probe instead.
    // LCOV_EXCL_START
    // Validate both streams are present first so we can surface a helpful
    // message (align_to_color_->process below would otherwise throw a generic
    // error when the color stream is missing).
    if (!frameset.get_depth_frame()) {
      throw std::runtime_error("No depth frame in frameset");
    }
    if (!frameset.get_color_frame()) {
      throw std::runtime_error(
          "No color frame in frameset — point clouds require both color and "
          "depth streams. Possible causes: \"color\" is not listed in the "
          "sensors config, or the camera is not receiving enough USB bandwidth "
          "(try a different cable or port).");
    }

    // Register depth to the color frame before deprojecting. calculate()
    // produces vertices in the coordinate frame of the depth map it is given;
    // on a RealSense the depth/left-IR sensor is physically offset (~15 mm)
    // from the color sensor. Because get_properties() reports COLOR intrinsics
    // and downstream consumers (e.g. the detections-to-segments vision service)
    // project cloud points through those intrinsics WITHOUT applying extrinsics
    // — assuming the cloud is already registered to the color frame — a cloud
    // left in the depth frame is shifted by the depth<->color baseline (~30 px
    // horizontally at 0.5 m), causing 2D bounding boxes to select the wrong 3D
    // points. Aligning to color first makes calculate() emit vertices in the
    // color frame and reduces the texture mapping to identity.
    auto aligned = align_to_color_->process(frameset);
    auto depth_frame = aligned.get_depth_frame();
    auto color_frame = aligned.get_color_frame();

    pointcloud_->map_to(color_frame);
    auto points = pointcloud_->calculate(depth_frame);
    return std::make_pair(points, color_frame);
    // LCOV_EXCL_STOP
  }

private:
  std::shared_ptr<rs2::pointcloud> pointcloud_;
  std::shared_ptr<rs2::align> align_to_color_;
};

template <typename DeviceT = rs2::device, typename PipeT = rs2::pipeline,
          typename AligntT = rs2::align, typename ConfigT = rs2::config>
struct ViamRSDevice {
  std::string serial_number{};
  std::shared_ptr<DeviceT> device{};
  bool started{false};
  std::shared_ptr<PipeT> pipe{};
  std::shared_ptr<PointCloudFilter> point_cloud_filter{};
  std::shared_ptr<AligntT> align{};
  std::shared_ptr<ConfigT> config{};
};
/********************** UTILITIES ************************/
template <typename DeviceT> void printDeviceInfo(DeviceT const &dev);

/********************** CALLBACKS ************************/
template <typename EventInformationT, typename ViamDeviceT, typename FrameSetT>
void deviceChangedCallback(
    EventInformationT &info,
    std::unordered_set<std::string> const &supported_camera_models,
    boost::synchronized_value<std::shared_ptr<ViamDeviceT>> &device,
    std::string const &required_serial_number,
    boost::synchronized_value<std::shared_ptr<FrameSetT>> &frame_set_storage,
    std::uint64_t maxFrameAgeMs);

template <typename FrameT, typename FrameSetT, typename ViamConfigT>
void frameCallback(
    FrameT const &frame, std::uint64_t const maxFrameAgeMs,
    boost::synchronized_value<std::shared_ptr<FrameSetT>> &frame_set_,
    ViamConfigT const &viamConfig);

/********************** DEVICE LIFECYCLE ************************/
template <typename ViamConfigT, typename ViamDeviceT = ViamRSDevice<>,
          typename DeviceT = rs2::device, typename ConfigT = rs2::config,
          typename ColorSensorT = rs2::color_sensor,
          typename DepthSensorT = rs2::depth_sensor,
          typename VideoStreamProfileT = rs2::video_stream_profile>
std::shared_ptr<boost::synchronized_value<ViamDeviceT>>
createDevice(std::string const &serial_number, std::shared_ptr<DeviceT> dev,
             std::unordered_set<std::string> const &supported_camera_models,
             ViamConfigT const &viamConfig);

template <typename ViamDeviceT>
bool destroyDevice(
    std::shared_ptr<boost::synchronized_value<ViamDeviceT>> &dev) noexcept;

/********************** STREAMING LIFECYCLE ************************/
template <typename ViamConfigT, typename ViamDeviceT = ViamRSDevice<>,
          typename DeviceT = rs2::device, typename ConfigT = rs2::config,
          typename ColorSensorT = rs2::color_sensor,
          typename DepthSensorT = rs2::depth_sensor,
          typename VideoStreamProfileT = rs2::video_stream_profile>
void reconfigureDevice(
    std::shared_ptr<boost::synchronized_value<ViamDeviceT>> dev,
    ViamConfigT const &viamConfig);

template <typename ViamDeviceT, typename FrameSetT, typename ViamConfigT>
void startDevice(
    std::string const &serialNumber,
    std::shared_ptr<boost::synchronized_value<ViamDeviceT>> dev,
    std::shared_ptr<boost::synchronized_value<FrameSetT>> &frame_set_storage,
    std::uint64_t const maxFrameAgeMs, ViamConfigT const &viamConfig);

template <typename ViamDeviceT>
bool stopDevice(
    std::shared_ptr<boost::synchronized_value<ViamDeviceT>> &dev) noexcept;

} // namespace device
} // namespace realsense

#include "device_impl.hpp"
