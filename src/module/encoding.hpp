#pragma once
#include <viam/sdk/components/camera.hpp>

#include <librealsense2/rs.hpp>

#include <cstdint>
#include <vector>
namespace realsense {
namespace encoding {

viam::sdk::Camera::raw_image encodeDepthRAWToResponse(const std::uint8_t *data,
                                                      const uint width,
                                                      const uint height);

viam::sdk::Camera::raw_image encodeJPEGToResponse(const std::uint8_t *data,
                                                  const uint width,
                                                  const uint height);

viam::sdk::Camera::raw_image
encodeVideoFrameToResponse(rs2::video_frame const &frame);

viam::sdk::Camera::raw_image
encodeDepthFrameToResponse(rs2::depth_frame const &frame);

std::vector<std::uint8_t>
encodeRGBPointsToPCD(std::pair<rs2::points, rs2::video_frame> &&data);

} // namespace encoding
} // namespace realsense
