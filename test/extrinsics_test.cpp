#include <gtest/gtest.h>

#include "extrinsics.hpp"

#include <cmath>

using namespace realsense;

TEST(ExtrinsicsTest, QuaternionToAxisAngle_Identity) {
  // Identity quaternion (no rotation)
  viam::sdk::quaternion q;
  q.w = 1.0;
  q.x = 0.0;
  q.y = 0.0;
  q.z = 0.0;

  auto result = realsense::extrinsics::quaternion_to_axis_angle(q);

  EXPECT_NEAR(result.theta, 0.0, 1e-6);
  // For identity rotation, implementation returns Z-axis by convention
  EXPECT_NEAR(result.x, 0.0, 1e-6);
  EXPECT_NEAR(result.y, 0.0, 1e-6);
  EXPECT_NEAR(result.z, 1.0, 1e-6);
}

TEST(ExtrinsicsTest, QuaternionToAxisAngle_90DegreesAroundZ) {
  // 90 degree rotation around Z axis
  viam::sdk::quaternion q;
  q.w = std::cos(M_PI / 4.0); // cos(45°) for 90° rotation
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(M_PI / 4.0); // sin(45°) for 90° rotation

  auto result = realsense::extrinsics::quaternion_to_axis_angle(q);

  EXPECT_NEAR(result.x, 0.0, 1e-6);
  EXPECT_NEAR(result.y, 0.0, 1e-6);
  EXPECT_NEAR(result.z, 1.0, 1e-6);
  EXPECT_NEAR(result.theta, 90.0, 1e-3);
}

TEST(ExtrinsicsTest, QuaternionToAxisAngle_180DegreesAroundX) {
  // 180 degree rotation around X axis
  viam::sdk::quaternion q;
  q.w = 0.0;
  q.x = 1.0;
  q.y = 0.0;
  q.z = 0.0;

  auto result = realsense::extrinsics::quaternion_to_axis_angle(q);

  EXPECT_NEAR(result.x, 1.0, 1e-6);
  EXPECT_NEAR(result.y, 0.0, 1e-6);
  EXPECT_NEAR(result.z, 0.0, 1e-6);
  EXPECT_NEAR(result.theta, 180.0, 1e-3);
}

TEST(ExtrinsicsTest, RotationMatrixToQuaternion_Identity) {
  // Identity rotation matrix
  float rotation[9] = {
      1.0f, 0.0f, 0.0f, // Row 1
      0.0f, 1.0f, 0.0f, // Row 2
      0.0f, 0.0f, 1.0f  // Row 3
  };

  auto q = realsense::extrinsics::rotation_matrix_to_quaternion(rotation);

  // Identity quaternion: w=1, x=y=z=0 (with some tolerance for numerical
  // errors)
  EXPECT_NEAR(q.w, 1.0, 1e-6);
  EXPECT_NEAR(q.x, 0.0, 1e-6);
  EXPECT_NEAR(q.y, 0.0, 1e-6);
  EXPECT_NEAR(q.z, 0.0, 1e-6);
}

TEST(ExtrinsicsTest, RotationMatrixToQuaternion_90DegreesAroundZ) {
  // 90 degree rotation around Z axis
  // [cos(90) -sin(90) 0]   [0 -1 0]
  // [sin(90)  cos(90) 0] = [1  0 0]
  // [0        0       1]   [0  0 1]
  float rotation[9] = {
      0.0f, -1.0f, 0.0f, // Row 1
      1.0f, 0.0f,  0.0f, // Row 2
      0.0f, 0.0f,  1.0f  // Row 3
  };

  auto q = realsense::extrinsics::rotation_matrix_to_quaternion(rotation);

  // Expected quaternion for 90° around Z: w=cos(45°), z=sin(45°), x=y=0
  EXPECT_NEAR(q.w, std::cos(M_PI / 4.0), 1e-6);
  EXPECT_NEAR(q.x, 0.0, 1e-6);
  EXPECT_NEAR(q.y, 0.0, 1e-6);
  EXPECT_NEAR(q.z, std::sin(M_PI / 4.0), 1e-6);
}

TEST(ExtrinsicsTest, RotationMatrixToQuaternion_180DegreesAroundX) {
  // 180 degree rotation around X axis
  // [1  0        0      ]   [1  0  0]
  // [0  cos(180) -sin(180)] = [0 -1  0]
  // [0  sin(180)  cos(180)]   [0  0 -1]
  float rotation[9] = {
      1.0f, 0.0f,  0.0f, // Row 1
      0.0f, -1.0f, 0.0f, // Row 2
      0.0f, 0.0f,  -1.0f // Row 3
  };

  auto q = realsense::extrinsics::rotation_matrix_to_quaternion(rotation);

  // Expected quaternion for 180° around X: w=0, x=1, y=z=0
  EXPECT_NEAR(q.w, 0.0, 1e-6);
  EXPECT_NEAR(q.x, 1.0, 1e-6);
  EXPECT_NEAR(q.y, 0.0, 1e-6);
  EXPECT_NEAR(q.z, 0.0, 1e-6);
}

TEST(ExtrinsicsTest, QuaternionNormalization) {
  // Test that quaternion_to_axis_angle handles unnormalized quaternions
  viam::sdk::quaternion q;
  q.w = 2.0; // Unnormalized
  q.x = 0.0;
  q.y = 0.0;
  q.z = 0.0;

  auto result = realsense::extrinsics::quaternion_to_axis_angle(q);

  // Should still produce identity (or very close to it)
  EXPECT_NEAR(result.theta, 0.0, 1e-3);
}

TEST(ExtrinsicsTest, RoundTripConversion_Identity) {
  // Test: rotation matrix -> quaternion -> axis-angle
  float identity[9] = {
      1.0f, 0.0f, 0.0f, // Row 1
      0.0f, 1.0f, 0.0f, // Row 2
      0.0f, 0.0f, 1.0f  // Row 3
  };

  auto q = realsense::extrinsics::rotation_matrix_to_quaternion(identity);
  auto axis_angle = realsense::extrinsics::quaternion_to_axis_angle(q);

  EXPECT_NEAR(axis_angle.theta, 0.0, 1e-3);
}

TEST(ExtrinsicsTest, RoundTripConversion_90DegreesZ) {
  // Test: rotation matrix -> quaternion -> axis-angle
  float rotation_z_90[9] = {
      0.0f, -1.0f, 0.0f, // Row 1
      1.0f, 0.0f,  0.0f, // Row 2
      0.0f, 0.0f,  1.0f  // Row 3
  };

  auto q = realsense::extrinsics::rotation_matrix_to_quaternion(rotation_z_90);
  auto axis_angle = realsense::extrinsics::quaternion_to_axis_angle(q);

  // Should be 90 degrees around Z axis
  EXPECT_NEAR(axis_angle.x, 0.0, 1e-3);
  EXPECT_NEAR(axis_angle.y, 0.0, 1e-3);
  EXPECT_NEAR(axis_angle.z, 1.0, 1e-3);
  EXPECT_NEAR(axis_angle.theta, 90.0, 1e-3);
}

// Note: get_extrinsics with actual rs2::stream_profile objects requires
// a real RealSense device, so we test the identity case handling through
// the mathematical functions above. The identity check in get_extrinsics
// is tested indirectly through integration tests with real hardware.
