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
template <typename AlignT = rs2::align, typename PointCloudT = rs2::pointcloud,
          typename FrameSetT = rs2::frameset>
class PointCloudFilter {
public:
  // Production: constructs the real librealsense objects.
  PointCloudFilter()
      : pointcloud_(std::make_shared<PointCloudT>()),
        align_to_color_(std::make_shared<AlignT>(RS2_STREAM_COLOR)) {}

  // Test seam: inject mock/fake align + pointcloud.
  PointCloudFilter(std::shared_ptr<PointCloudT> pointcloud,
                   std::shared_ptr<AlignT> align)
      : pointcloud_(std::move(pointcloud)),
        align_to_color_(std::move(align)) {}

  // Return type is deduced so PointCloudFilter<> yields exactly
  // std::pair<rs2::points, rs2::video_frame> (the unchanged caller contract),
  // while a fake FrameSetT yields a pair of the fake frame types.
  auto process(FrameSetT frameset) {
    // Validate the color stream is present so we can surface a helpful message
    // (align_to_color_->process below would otherwise throw a generic error
    // when the color stream is missing). The sole production caller
    // (get_point_cloud) already validates the depth frame.
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
  }

private:
  std::shared_ptr<PointCloudT> pointcloud_;
  std::shared_ptr<AlignT> align_to_color_;
};

template <typename DeviceT = rs2::device, typename PipeT = rs2::pipeline,
          typename AligntT = rs2::align, typename ConfigT = rs2::config,
          typename PointCloudFilterT = PointCloudFilter<>>
struct ViamRSDevice {
  std::string serial_number{};
  std::shared_ptr<DeviceT> device{};
  bool started{false};
  std::shared_ptr<PipeT> pipe{};
  std::shared_ptr<PointCloudFilterT> point_cloud_filter{};
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
