#include <gtest/gtest.h>

#include "extrinsics.hpp"

using namespace realsense;

// A real rs2::stream_profile can only be handed out by a live device, so these
// tests drive extrinsics.hpp through the same duck-typed surface it uses:
// unique_id() and get_extrinsics_to(). This mirrors how device.hpp is tested.
class FakeStreamProfile {
public:
  FakeStreamProfile(int unique_id, rs2_extrinsics extrinsics_to_other)
      : unique_id_(unique_id), extrinsics_to_other_(extrinsics_to_other) {}

  int unique_id() const { return unique_id_; }

  rs2_extrinsics get_extrinsics_to(const FakeStreamProfile &) const {
    return extrinsics_to_other_;
  }

private:
  int unique_id_;
  rs2_extrinsics extrinsics_to_other_;
};

namespace {

// Measured color->depth extrinsics from a D435 (translation in meters, as
// librealsense reports it). Rotation is ignored by get_extrinsics.
constexpr rs2_extrinsics kD435ColorToDepth = {
    {1, 0, 0, 0, 1, 0, 0, 0, 1},          // rotation
    {-0.015044f, -0.000173f, -0.000428f}, // translation
};

void expectIdentityOrientation(
    const viam::sdk::Camera::extrinsic_parameters &extrinsics) {
  EXPECT_DOUBLE_EQ(extrinsics.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(extrinsics.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(extrinsics.orientation.z, 1.0);
  EXPECT_DOUBLE_EQ(extrinsics.orientation.theta, 0.0);
}

void expectZeroTranslation(
    const viam::sdk::Camera::extrinsic_parameters &extrinsics) {
  EXPECT_DOUBLE_EQ(extrinsics.translation.x(), 0.0);
  EXPECT_DOUBLE_EQ(extrinsics.translation.y(), 0.0);
  EXPECT_DOUBLE_EQ(extrinsics.translation.z(), 0.0);
}

} // namespace

// Guards the regression tests below against being vacuously true: a non-zero
// baseline between two *different* streams must still come through, in mm.
TEST(ExtrinsicsTest, ConvertsMetersToMillimetersBetweenDistinctStreams) {
  const FakeStreamProfile color{0, kD435ColorToDepth};
  const FakeStreamProfile depth{1, kD435ColorToDepth};

  const auto result = extrinsics::get_extrinsics(color, depth);

  EXPECT_NEAR(result.translation.x(), -15.044, 1e-3);
  EXPECT_NEAR(result.translation.y(), -0.173, 1e-3);
  EXPECT_NEAR(result.translation.z(), -0.428, 1e-3);
  expectIdentityOrientation(result);
}

TEST(ExtrinsicsTest, IsIdentityForTheSameStream) {
  const FakeStreamProfile color{0, kD435ColorToDepth};

  const auto result = extrinsics::get_extrinsics(color, color);

  expectZeroTranslation(result);
  expectIdentityOrientation(result);
}

// Regression test for the extrinsic double-correction shipped in 0.22.4.
//
// get_point_cloud() aligns depth to the color frame before deprojecting, so the
// cloud is already registered to the stream whose intrinsics get_properties()
// reports. Publishing the depth<->color baseline made consumers that subtract
// extrinsic_parameters.translation before projecting (rdk's
// camera.Properties.PointToPixel) re-apply a baseline rs2::align had already
// removed — a ~22 px X shift at 0.63 m on a D435.
//
// The reference stream is color when color is configured, depth otherwise; in
// every case the reported extrinsics must be identity.
TEST(ExtrinsicsTest, ReportedExtrinsicsAreIdentityForColorAndDepthConfigured) {
  // Reference stream is color; it has a real, non-zero baseline to depth.
  const FakeStreamProfile color{0, kD435ColorToDepth};

  const auto result = extrinsics::get_reported_extrinsics(color);

  expectZeroTranslation(result);
  expectIdentityOrientation(result);
}

TEST(ExtrinsicsTest, ReportedExtrinsicsAreIdentityForASingleConfiguredSensor) {
  // sensors: ["depth"] — depth is both the reference stream and the only frame
  // there is. Same for sensors: ["color"].
  const FakeStreamProfile depth{1, kD435ColorToDepth};

  const auto result = extrinsics::get_reported_extrinsics(depth);

  expectZeroTranslation(result);
  expectIdentityOrientation(result);
}
