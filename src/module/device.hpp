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
  PointCloudFilter() : pointcloud_(std::make_shared<rs2::pointcloud>()) {}
  std::pair<rs2::points, rs2::video_frame> process(rs2::frameset frameset) {
    auto depth_frame = frameset.get_depth_frame();
    if (!depth_frame) {
      throw std::runtime_error("No depth frame in frameset");
    }
    auto color_frame = frameset.get_color_frame();
    if (!color_frame) {
      throw std::runtime_error(
          "No color frame in frameset — point clouds require both color and "
          "depth streams. Possible causes: \"color\" is not listed in the "
          "sensors config, or the camera is not receiving enough USB bandwidth "
          "(try a different cable or port).");
    }
    pointcloud_->map_to(color_frame);
    auto points = pointcloud_->calculate(depth_frame);
    return std::make_pair(points, color_frame);
  }

private:
  std::shared_ptr<rs2::pointcloud> pointcloud_;
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
