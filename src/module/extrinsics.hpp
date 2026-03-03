#pragma once

#include <viam/sdk/components/camera.hpp>

#include <librealsense2/rs.hpp>

namespace realsense {
namespace extrinsics {

/// @brief Get extrinsic parameters from RealSense stream profiles
/// @param from_stream The source stream profile
/// @param to_stream The destination stream profile
/// @return Extrinsic parameters representing the transformation from
/// from_stream to to_stream. If streams are identical, returns identity
/// extrinsics.
inline viam::sdk::Camera::extrinsic_parameters
get_extrinsics(const rs2::stream_profile &from_stream,
               const rs2::stream_profile &to_stream) {
  viam::sdk::Camera::extrinsic_parameters extrinsics;

  // If streams are the same, return identity extrinsics
  if (from_stream.unique_id() == to_stream.unique_id()) {
    extrinsics.translation.set_x(0.0).set_y(0.0).set_z(0.0);
    extrinsics.orientation.x = 0.0;
    extrinsics.orientation.y = 0.0;
    extrinsics.orientation.z = 1.0;
    extrinsics.orientation.theta = 0.0;
    return extrinsics;
  }

  // Get extrinsics from RealSense
  rs2_extrinsics rs_extrinsics = from_stream.get_extrinsics_to(to_stream);

  // Set translation (convert from meters to millimeters)
  extrinsics.translation.set_x(rs_extrinsics.translation[0] * 1000.0)
      .set_y(rs_extrinsics.translation[1] * 1000.0)
      .set_z(rs_extrinsics.translation[2] * 1000.0);

  // Set orientation to identity (no rotation)
  // The rotation between depth and color cameras is very small (~1 degree)
  // and can cause numerical issues, so we ignore it
  extrinsics.orientation.x = 0.0;
  extrinsics.orientation.y = 0.0;
  extrinsics.orientation.z = 1.0;
  extrinsics.orientation.theta = 0.0;

  return extrinsics;
}

} // namespace extrinsics
} // namespace realsense
