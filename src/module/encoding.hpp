#pragma once
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/log/logging.hpp>

#include <librealsense2/rs.hpp>

#include <cstdint>
#include <vector>
namespace realsense {
namespace encoding {

// mm_per_unit converts raw depth values to millimeters (the unit of
// image/vnd.viam.dep). Most D400 cameras use 1 mm depth units, but the D405
// defaults to 0.1 mm units, so raw values must be rescaled.
viam::sdk::Camera::raw_image
encodeDepthRAWToResponse(const std::uint8_t *data, const uint width,
                         const uint height, const float mm_per_unit = 1.0f);

viam::sdk::Camera::raw_image encodeJPEGToResponse(const std::uint8_t *data,
                                                  const uint width,
                                                  const uint height);

viam::sdk::Camera::raw_image
encodeVideoFrameToResponse(rs2::video_frame const &frame);

viam::sdk::Camera::raw_image
encodeDepthFrameToResponse(rs2::depth_frame const &frame);

std::vector<std::uint8_t>
encodeRGBPointsToPCD(std::pair<rs2::points, rs2::video_frame> &&data,
                     viam::sdk::LogSource &logger);

} // namespace encoding
} // namespace realsense
