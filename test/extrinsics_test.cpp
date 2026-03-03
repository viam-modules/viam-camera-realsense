#include <gtest/gtest.h>

#include "extrinsics.hpp"

using namespace realsense;

// Note: get_extrinsics with actual rs2::stream_profile objects requires
// a real RealSense device. The extrinsics function returns identity orientation
// and translation based on RealSense SDK data.
// Full integration tests with real hardware are performed separately.

// Placeholder test to ensure the test file compiles
TEST(ExtrinsicsTest, Placeholder) { EXPECT_TRUE(true); }
