#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "encoding.hpp"
#include <viam/sdk/components/camera.hpp>

#include <librealsense2/rs.hpp>
#include <turbojpeg.h>

#include <cstdint>
#include <memory>
#include <vector>

using namespace realsense::encoding;
using namespace viam::sdk;

class EncodingTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create test RGB data (3x3 pixel image, RGB format)
    test_width_ = 3;
    test_height_ = 3;
    test_rgb_data_ = {
        255, 0,   0,   // Red pixel
        0,   255, 0,   // Green pixel
        0,   0,   255, // Blue pixel
        255, 255, 0,   // Yellow pixel
        255, 0,   255, // Magenta pixel
        0,   255, 255, // Cyan pixel
        128, 128, 128, // Gray pixel
        0,   0,   0,   // Black pixel
        255, 255, 255  // White pixel
    };

    // Create test depth data (3x3 pixel depth image, 16-bit depth values)
    test_depth_data_raw_ = {
        1000, 2000, 3000, // Row 1: increasing depth
        1500, 2500, 3500, // Row 2: increasing depth
        2000, 3000, 4000  // Row 3: increasing depth
    };

    // Convert depth data to bytes (little-endian)
    test_depth_data_.clear();
    for (uint16_t depth : test_depth_data_raw_) {
      test_depth_data_.push_back(depth & 0xFF);        // Low byte
      test_depth_data_.push_back((depth >> 8) & 0xFF); // High byte
    }
  }

  uint32_t test_width_, test_height_;
  std::vector<uint8_t> test_rgb_data_;
  std::vector<uint8_t> test_depth_data_;
  std::vector<uint16_t> test_depth_data_raw_;
};

TEST_F(EncodingTest, EncodeDepthRAWToResponse_ValidData) {
  auto result = encodeDepthRAWToResponse(test_depth_data_.data(), test_width_,
                                         test_height_);

  std::vector<uint8_t> expected_bytes{
      0x44, 0x45, 0x50, 0x54, 0x48, 0x4d, 0x41, 0x50, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x03, 0x03, 0xe8, 0x07, 0xd0, 0x0b, 0xb8, 0x05, 0xdc, 0x09,
      0xc4, 0x0d, 0xac, 0x07, 0xd0, 0x0b, 0xb8, 0x0f, 0xa0};

  EXPECT_EQ(result.source_name, "depth");
  EXPECT_EQ(result.mime_type, "image/vnd.viam.dep");
  EXPECT_EQ(result.bytes, expected_bytes);
}

TEST_F(EncodingTest, EncodeJPEGToResponse_ValidData) {
  auto result =
      encodeJPEGToResponse(test_rgb_data_.data(), test_width_, test_height_);

  EXPECT_EQ(result.source_name, "color");
  EXPECT_EQ(result.mime_type, "image/jpeg");
  EXPECT_FALSE(result.bytes.empty());
  EXPECT_GT(result.bytes.size(), 0);

  // JPEG files should start with FF D8 (JPEG magic bytes)
  EXPECT_EQ(result.bytes[0], 0xFF);
  EXPECT_EQ(result.bytes[1], 0xD8);
}

TEST_F(EncodingTest, EncodeDepthRAWToResponse_NullData) {
  EXPECT_THROW(
      {
        auto result =
            encodeDepthRAWToResponse(nullptr, test_width_, test_height_);
      },
      std::runtime_error);
}

TEST_F(EncodingTest, EncodeDepthRAWToResponse_ZeroDimensions) {
  EXPECT_THROW(
      { encodeDepthRAWToResponse(test_depth_data_.data(), 0, test_height_); },
      std::runtime_error);

  EXPECT_THROW(
      { encodeDepthRAWToResponse(test_depth_data_.data(), test_width_, 0); },
      std::runtime_error);
}

TEST_F(EncodingTest, EncodeJPEGToResponse_NullData) {
  EXPECT_THROW(
      {
        auto result = encodeJPEGToResponse(nullptr, test_width_, test_height_);
      },
      std::runtime_error);
}

TEST_F(EncodingTest, EncodeJPEGToResponse_ZeroDimensions) {
  EXPECT_THROW(
      { encodeJPEGToResponse(test_rgb_data_.data(), 0, test_height_); },
      std::runtime_error);

  EXPECT_THROW(
      { encodeJPEGToResponse(test_rgb_data_.data(), test_width_, 0); },
      std::runtime_error);
}

TEST_F(EncodingTest, EncodeJPEGToResponse_SinglePixel) {
  std::vector<uint8_t> single_pixel = {255, 128, 64}; // Single RGB pixel

  auto result = encodeJPEGToResponse(single_pixel.data(), 1, 1);

  EXPECT_EQ(result.source_name, "color");
  EXPECT_EQ(result.mime_type, "image/jpeg");
  EXPECT_FALSE(result.bytes.empty());

  // Even a single pixel should create a valid JPEG
  EXPECT_EQ(result.bytes[0], 0xFF);
  EXPECT_EQ(result.bytes[1], 0xD8);
}

TEST_F(EncodingTest, EncodeDepthRAWToResponse_SinglePixel) {
  std::vector<uint8_t> single_depth = {0x00, 0x10}; // 4096 in little-endian

  auto result = encodeDepthRAWToResponse(single_depth.data(), 1, 1);

  EXPECT_EQ(result.source_name, "depth");
  EXPECT_EQ(result.mime_type, "image/vnd.viam.dep");
  EXPECT_FALSE(result.bytes.empty());
}

TEST_F(EncodingTest, EncodeJPEGToResponse_ConsistentResults) {
  auto result1 =
      encodeJPEGToResponse(test_rgb_data_.data(), test_width_, test_height_);

  auto result2 =
      encodeJPEGToResponse(test_rgb_data_.data(), test_width_, test_height_);

  // Results should be identical for same input
  EXPECT_EQ(result1.source_name, result2.source_name);
  EXPECT_EQ(result1.mime_type, result2.mime_type);
  EXPECT_EQ(result1.bytes.size(), result2.bytes.size());
  EXPECT_EQ(result1.bytes, result2.bytes);
}

TEST_F(EncodingTest, EncodeDepthRAWToResponse_ConsistentResults) {
  auto result1 = encodeDepthRAWToResponse(test_depth_data_.data(), test_width_,
                                          test_height_);

  auto result2 = encodeDepthRAWToResponse(test_depth_data_.data(), test_width_,
                                          test_height_);

  // Results should be identical for same input
  EXPECT_EQ(result1.source_name, result2.source_name);
  EXPECT_EQ(result1.mime_type, result2.mime_type);
  EXPECT_EQ(result1.bytes.size(), result2.bytes.size());
  EXPECT_EQ(result1.bytes, result2.bytes);
}

TEST_F(EncodingTest, EncodeDepthRAW_VariousDepthRanges) {
  // Test with minimum depth values (0, 1, 2 in little-endian 16-bit format)
  std::vector<uint8_t> min_depth_bytes{0, 0, 1, 0, 2, 0};
  std::vector<uint8_t> expected_min_depth_values{
      0x44, 0x45, 0x50, 0x54, 0x48, 0x4d, 0x41, 0x50, // "DEPTHMAP"
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, // Width: 3
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, // Height: 1
      0x00, 0x00, 0x00,                               // Depth pixel 1: 0
      0x01, 0x00,                                     // Depth pixel 2: 1
      0x02                                            // Depth pixel 3: 2
  };

  auto min_result = encodeDepthRAWToResponse(min_depth_bytes.data(), 3, 1);
  EXPECT_FALSE(min_result.bytes.empty());
  EXPECT_EQ(min_result.source_name, "depth");
  EXPECT_EQ(min_result.mime_type, "image/vnd.viam.dep");
  EXPECT_EQ(min_result.bytes, expected_min_depth_values);

  // Test with maximum depth values (65533, 65534, 65535 in little-endian 16-bit
  // format)
  std::vector<uint8_t> max_depth_bytes{
      0xFD, 0xFF, // 65533 = 0xFFFD
      0xFE, 0xFF, // 65534 = 0xFFFE
      0xFF, 0xFF  // 65535 = 0xFFFF
  };
  std::vector<uint8_t> expected_max_depth_values{
      0x44, 0x45, 0x50, 0x54, 0x48, 0x4d, 0x41, 0x50, // "DEPTHMAP"
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, // Width: 3
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, // Height: 1
      0xFF, 0xFD,                                     // Depth pixel 1: 65533
      0xFF, 0xFE,                                     // Depth pixel 2: 65534
      0xFF, 0xFF                                      // Depth pixel 3: 65535
  };

  auto max_result = encodeDepthRAWToResponse(max_depth_bytes.data(), 3, 1);
  EXPECT_FALSE(max_result.bytes.empty());
  EXPECT_EQ(max_result.source_name, "depth");
  EXPECT_EQ(max_result.mime_type, "image/vnd.viam.dep");
  EXPECT_EQ(max_result.bytes, expected_max_depth_values);
}

TEST_F(EncodingTest, EncodeDepthRAW_D405DepthUnitsRescaledToMillimeters) {
  // The D405 defaults to 0.1 mm depth units (100 µm), so raw values must be
  // rescaled to the millimeters that image/vnd.viam.dep encodes.
  std::vector<uint8_t> depth_bytes{
      0x64, 0x00, // 100 raw units   -> 10 mm
      0x39, 0x30, // 12345 raw units -> 1234.5 -> 1235 mm (rounded)
      0xFF, 0xFF  // 65535 raw units -> 6553.5 -> 6554 mm
  };
  std::vector<uint8_t> expected_values{
      0x44, 0x45, 0x50, 0x54, 0x48, 0x4d, 0x41, 0x50, // "DEPTHMAP"
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, // Width: 3
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, // Height: 1
      0x00, 0x0A,                                     // Depth pixel 1: 10
      0x04, 0xD3,                                     // Depth pixel 2: 1235
      0x19, 0x9A                                      // Depth pixel 3: 6554
  };

  auto result = encodeDepthRAWToResponse(depth_bytes.data(), 3, 1, 0.1f);
  EXPECT_EQ(result.source_name, "depth");
  EXPECT_EQ(result.mime_type, "image/vnd.viam.dep");
  EXPECT_EQ(result.bytes, expected_values);
}

TEST_F(EncodingTest, EncodeDepthRAW_ScaleSaturatesAtUint16Max) {
  // Values that overflow uint16 after rescaling saturate at 65535 rather
  // than wrapping.
  std::vector<uint8_t> depth_bytes{0x40, 0x9C}; // 40000 = 0x9C40
  std::vector<uint8_t> expected_values{
      0x44, 0x45, 0x50, 0x54, 0x48, 0x4d, 0x41, 0x50, // "DEPTHMAP"
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, // Width: 1
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, // Height: 1
      0xFF, 0xFF                                      // 80000 mm -> 65535
  };

  auto result = encodeDepthRAWToResponse(depth_bytes.data(), 1, 1, 2.0f);
  EXPECT_EQ(result.bytes, expected_values);
}

TEST_F(EncodingTest, EncodeDepthRAW_UnitScaleMatchesDefault) {
  // An explicit 1 mm/unit scale must produce the same bytes as the default.
  auto scaled = encodeDepthRAWToResponse(test_depth_data_.data(), test_width_,
                                         test_height_, 1.0f);
  auto unscaled = encodeDepthRAWToResponse(test_depth_data_.data(), test_width_,
                                           test_height_);
  EXPECT_EQ(scaled.bytes, unscaled.bytes);
}

TEST_F(EncodingTest, EncodeDepthRAW_InvalidScaleThrows) {
  EXPECT_THROW(
      {
        encodeDepthRAWToResponse(test_depth_data_.data(), test_width_,
                                 test_height_, 0.0f);
      },
      std::runtime_error);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
