#pragma once

#include <viam/sdk/common/linear_algebra.hpp>
#include <viam/sdk/spatialmath/orientation.hpp>
#include <viam/sdk/spatialmath/orientation_types.hpp>

#include <librealsense2/rs.hpp>

namespace realsense {
namespace extrinsics {

/// @brief Extrinsic parameters define the position of the camera
/// relative to a reference frame (another sensor).
struct ExtrinsicParameters {
  /// @brief The translation from the reference frame to the camera.
  viam::sdk::Vector3 translation;
  /// @brief The orientation from the reference frame to the camera.
  viam::sdk::Orientation orientation;
};

/// @brief Convert a 3x3 rotation matrix to a quaternion
/// @param rotation 3x3 rotation matrix stored in row-major order
/// @return quaternion representing the rotation
inline viam::sdk::quaternion
rotation_matrix_to_quaternion(const float rotation[9]) {
  // Rotation matrix is stored as:
  // [0 1 2]
  // [3 4 5]
  // [6 7 8]

  double trace = rotation[0] + rotation[4] + rotation[8];
  viam::sdk::quaternion q;

  if (trace > 0.0) {
    double s = std::sqrt(trace + 1.0) * 2.0; // s = 4 * qw
    q.w = 0.25 * s;
    q.x = (rotation[7] - rotation[5]) / s;
    q.y = (rotation[2] - rotation[6]) / s;
    q.z = (rotation[3] - rotation[1]) / s;
  } else if ((rotation[0] > rotation[4]) && (rotation[0] > rotation[8])) {
    double s = std::sqrt(1.0 + rotation[0] - rotation[4] - rotation[8]) *
               2.0; // s = 4 * qx
    q.w = (rotation[7] - rotation[5]) / s;
    q.x = 0.25 * s;
    q.y = (rotation[1] + rotation[3]) / s;
    q.z = (rotation[2] + rotation[6]) / s;
  } else if (rotation[4] > rotation[8]) {
    double s = std::sqrt(1.0 + rotation[4] - rotation[0] - rotation[8]) *
               2.0; // s = 4 * qy
    q.w = (rotation[2] - rotation[6]) / s;
    q.x = (rotation[1] + rotation[3]) / s;
    q.y = 0.25 * s;
    q.z = (rotation[5] + rotation[7]) / s;
  } else {
    double s = std::sqrt(1.0 + rotation[8] - rotation[0] - rotation[4]) *
               2.0; // s = 4 * qz
    q.w = (rotation[3] - rotation[1]) / s;
    q.x = (rotation[2] + rotation[6]) / s;
    q.y = (rotation[5] + rotation[7]) / s;
    q.z = 0.25 * s;
  }

  return q;
}

/// @brief Get extrinsic parameters from RealSense stream profiles
/// @param from_stream The source stream profile
/// @param to_stream The destination stream profile
/// @return Extrinsic parameters representing the transformation from
/// from_stream to to_stream
inline ExtrinsicParameters
get_extrinsics(const rs2::stream_profile &from_stream,
               const rs2::stream_profile &to_stream) {
  rs2_extrinsics rs_extrinsics = from_stream.get_extrinsics_to(to_stream);

  ExtrinsicParameters extrinsics;

  // Set translation
  extrinsics.translation.set_x(rs_extrinsics.translation[0])
      .set_y(rs_extrinsics.translation[1])
      .set_z(rs_extrinsics.translation[2]);

  // Convert rotation matrix to quaternion
  extrinsics.orientation =
      rotation_matrix_to_quaternion(rs_extrinsics.rotation);

  return extrinsics;
}

} // namespace extrinsics
} // namespace realsense
