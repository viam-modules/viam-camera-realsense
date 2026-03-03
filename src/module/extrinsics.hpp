#pragma once

#include <cmath>

#include <viam/sdk/common/linear_algebra.hpp>
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/spatialmath/orientation.hpp>
#include <viam/sdk/spatialmath/orientation_types.hpp>

#include <librealsense2/rs.hpp>

namespace realsense {
namespace extrinsics {

/// @brief Convert quaternion to axis-angle representation
/// (orientation_vector_degrees)
/// @param q The quaternion
/// @return orientation_vector_degrees (axis x,y,z and angle theta in degrees)
inline viam::sdk::orientation_vector_degrees
quaternion_to_axis_angle(const viam::sdk::quaternion &q) {
  viam::sdk::orientation_vector_degrees result;

  // Handle identity quaternion (no rotation)
  if (std::abs(q.w - 1.0) < 1e-6) {
    result.x = 0.0;
    result.y = 0.0;
    result.z = 1.0;
    result.theta = 0.0;
    return result;
  }

  // Normalize quaternion
  double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  double qw = q.w / norm;
  double qx = q.x / norm;
  double qy = q.y / norm;
  double qz = q.z / norm;

  // Convert to axis-angle
  double angle_rad = 2.0 * std::acos(qw);
  double sin_half_angle = std::sqrt(1.0 - qw * qw);

  if (sin_half_angle < 1e-6) {
    // Angle is close to 0, axis doesn't matter
    result.x = 0.0;
    result.y = 0.0;
    result.z = 1.0;
    result.theta = 0.0;
  } else {
    result.x = qx / sin_half_angle;
    result.y = qy / sin_half_angle;
    result.z = qz / sin_half_angle;
    result.theta = angle_rad * 180.0 / M_PI; // Convert to degrees
  }

  return result;
}

/// @brief Convert a 3x3 rotation matrix to a quaternion
/// @param rotation 3x3 rotation matrix stored in column-major order (as used by
/// RealSense SDK)
/// @return quaternion representing the rotation
inline viam::sdk::quaternion
rotation_matrix_to_quaternion(const float rotation[9]) {
  // Rotation matrix is stored in column-major order:
  // Matrix:         Array indices:
  // [R00 R01 R02]   [0  3  6]
  // [R10 R11 R12] = [1  4  7]
  // [R20 R21 R22]   [2  5  8]

  double trace = rotation[0] + rotation[4] + rotation[8];
  viam::sdk::quaternion q;

  if (trace > 0.0) {
    double s = std::sqrt(trace + 1.0) * 2.0; // s = 4 * qw
    q.w = 0.25 * s;
    q.x = (rotation[5] - rotation[7]) / s; // (R21 - R12)
    q.y = (rotation[6] - rotation[2]) / s; // (R02 - R20)
    q.z = (rotation[1] - rotation[3]) / s; // (R10 - R01)
  } else if ((rotation[0] > rotation[4]) && (rotation[0] > rotation[8])) {
    double s = std::sqrt(1.0 + rotation[0] - rotation[4] - rotation[8]) *
               2.0;                        // s = 4 * qx
    q.w = (rotation[5] - rotation[7]) / s; // (R21 - R12)
    q.x = 0.25 * s;
    q.y = (rotation[3] + rotation[1]) / s; // (R01 + R10)
    q.z = (rotation[6] + rotation[2]) / s; // (R02 + R20)
  } else if (rotation[4] > rotation[8]) {
    double s = std::sqrt(1.0 + rotation[4] - rotation[0] - rotation[8]) *
               2.0;                        // s = 4 * qy
    q.w = (rotation[6] - rotation[2]) / s; // (R02 - R20)
    q.x = (rotation[3] + rotation[1]) / s; // (R01 + R10)
    q.y = 0.25 * s;
    q.z = (rotation[7] + rotation[5]) / s; // (R12 + R21)
  } else {
    double s = std::sqrt(1.0 + rotation[8] - rotation[0] - rotation[4]) *
               2.0;                        // s = 4 * qz
    q.w = (rotation[1] - rotation[3]) / s; // (R10 - R01)
    q.x = (rotation[6] + rotation[2]) / s; // (R02 + R20)
    q.y = (rotation[7] + rotation[5]) / s; // (R12 + R21)
    q.z = 0.25 * s;
  }

  return q;
}

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

  // Convert rotation matrix to quaternion, then to axis-angle
  auto quat = rotation_matrix_to_quaternion(rs_extrinsics.rotation);
  extrinsics.orientation = quaternion_to_axis_angle(quat);

  return extrinsics;
}

} // namespace extrinsics
} // namespace realsense
