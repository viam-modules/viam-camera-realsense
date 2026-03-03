#pragma once

#include <iostream>

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

  // DEBUG: Print raw rotation matrix data
  std::cout << "=== DEBUG: Raw RealSense Extrinsics ===" << std::endl;
  std::cout << "Translation (m): [" << rs_extrinsics.translation[0] << ", "
            << rs_extrinsics.translation[1] << ", "
            << rs_extrinsics.translation[2] << "]" << std::endl;
  std::cout << "Rotation array (9 elements): [";
  for (int i = 0; i < 9; i++) {
    std::cout << rs_extrinsics.rotation[i];
    if (i < 8)
      std::cout << ", ";
  }
  std::cout << "]" << std::endl;

  // Print as 3x3 matrix if interpreted as row-major
  std::cout << "As row-major matrix:" << std::endl;
  for (int row = 0; row < 3; row++) {
    std::cout << "  [";
    for (int col = 0; col < 3; col++) {
      std::cout << rs_extrinsics.rotation[row * 3 + col];
      if (col < 2)
        std::cout << ", ";
    }
    std::cout << "]" << std::endl;
  }

  // Print as 3x3 matrix if interpreted as column-major
  std::cout << "As column-major matrix:" << std::endl;
  for (int row = 0; row < 3; row++) {
    std::cout << "  [";
    for (int col = 0; col < 3; col++) {
      std::cout << rs_extrinsics.rotation[col * 3 + row];
      if (col < 2)
        std::cout << ", ";
    }
    std::cout << "]" << std::endl;
  }

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

  std::cout << "Translation (mm): [" << extrinsics.translation.x() << ", "
            << extrinsics.translation.y() << ", " << extrinsics.translation.z()
            << "]" << std::endl;
  std::cout << "Orientation: identity (rotation ignored)" << std::endl;
  std::cout << "=======================================" << std::endl;

  return extrinsics;
}

} // namespace extrinsics
} // namespace realsense
