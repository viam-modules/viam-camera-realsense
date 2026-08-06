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
///
/// Templated on the profile types so tests can substitute a stand-in exposing
/// unique_id() and get_extrinsics_to(); a real rs2::stream_profile can only be
/// handed out by a live device.
template <typename FromStreamT, typename ToStreamT>
inline viam::sdk::Camera::extrinsic_parameters
get_extrinsics(const FromStreamT &from_stream, const ToStreamT &to_stream) {
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

/// @brief The extrinsic parameters get_properties() publishes.
/// @param reference_stream The stream whose intrinsics get_properties() reports
/// @return Identity extrinsics.
///
/// extrinsic_parameters describes the transform from the frame the point cloud
/// is emitted in to the frame the reported intrinsics describe. Those are the
/// same frame: get_point_cloud() aligns depth to color before deprojecting (see
/// device::PointCloudFilter), so the cloud lands in the color frame, which is
/// also the stream get_properties() takes its intrinsics from whenever color is
/// configured. When only one sensor is configured there is no second frame to
/// be in — and no point cloud at all, since deprojection needs both streams.
/// Either way the transform is @p reference_stream to itself: identity.
///
/// Do NOT publish the real depth<->color baseline here. Consumers subtract
/// extrinsic_parameters.translation before projecting a cloud point through the
/// reported intrinsics (rdk's camera.Properties.PointToPixel), so a non-zero
/// translation re-applies a baseline rs2::align has already removed, shifting
/// every projected point by fx * T / Z — about 22 px at 0.63 m on a D435.
template <typename StreamProfileT>
inline viam::sdk::Camera::extrinsic_parameters
get_reported_extrinsics(const StreamProfileT &reference_stream) {
  return get_extrinsics(reference_stream, reference_stream);
}

} // namespace extrinsics
} // namespace realsense
