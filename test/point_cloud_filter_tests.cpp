#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "device.hpp"

#include <memory>

using ::testing::_;
using ::testing::InSequence;
using ::testing::Return;

namespace realsense {
namespace device {
namespace test {

// Identity tag lets a test prove WHICH frame flowed through (the original
// input frame vs. the frame produced by align->process).
enum class FrameTag { kOriginal, kAligned };

// Minimal stand-ins for the librealsense frame types. Validity drives the
// `if (!frame)` checks; the tag proves color-registration.
struct FakeVideoFrame {
  bool valid = true;
  FrameTag tag = FrameTag::kOriginal;
  explicit operator bool() const { return valid; }
};

struct FakeDepthFrame {
  bool valid = true;
  FrameTag tag = FrameTag::kOriginal;
  explicit operator bool() const { return valid; }
};

struct FakeFrameSet {
  FakeVideoFrame color;
  FakeDepthFrame depth;
  FakeVideoFrame get_color_frame() const { return color; }
  FakeDepthFrame get_depth_frame() const { return depth; }
};

// Stand-in for rs2::points.
struct FakePoints {};

class MockAlign {
public:
  MockAlign() = default;
  explicit MockAlign(rs2_stream) {}
  MOCK_METHOD(FakeFrameSet, process, (FakeFrameSet));
};

class MockPointCloud {
public:
  MOCK_METHOD(void, map_to, (FakeVideoFrame));
  MOCK_METHOD(FakePoints, calculate, (FakeDepthFrame));
};

using TestFilter = PointCloudFilter<MockAlign, MockPointCloud, FakeFrameSet>;

// Helper: build a filter with injected mocks, returning the mocks so the test
// can set expectations.
struct Harness {
  std::shared_ptr<MockPointCloud> pc = std::make_shared<MockPointCloud>();
  std::shared_ptr<MockAlign> align = std::make_shared<MockAlign>();
  TestFilter filter{pc, align};
};

TEST(PointCloudFilterTest, ColorFrameMissing_ThrowsHelpfulMessage) {
  Harness h;
  FakeFrameSet fs;
  fs.color.valid = false; // no color frame
  fs.depth.valid = true;

  // align->process must never run when the guard trips.
  EXPECT_CALL(*h.align, process(_)).Times(0);
  EXPECT_CALL(*h.pc, map_to(_)).Times(0);
  EXPECT_CALL(*h.pc, calculate(_)).Times(0);

  try {
    h.filter.process(fs);
    FAIL() << "expected process() to throw when color frame is missing";
  } catch (const std::exception &e) {
    EXPECT_THAT(std::string(e.what()),
                ::testing::HasSubstr(
                    "point clouds require both color and depth streams"));
  }
}

} // namespace test
} // namespace device
} // namespace realsense
