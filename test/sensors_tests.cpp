#include "../src/module/sensors.hpp"
#include "log_capture.hpp"
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/resource/resource.hpp>

namespace realsense {
namespace sensors {

// MockResource for testing
class MockResource : public viam::sdk::Resource {
public:
  MockResource() : viam::sdk::Resource("test_resource") {}
  viam::sdk::API api() const override {
    return viam::sdk::API::get<viam::sdk::Camera>();
  }
  // Expose the logger for testing
  viam::sdk::LogSource& get_logger() { return logger_; }
};

TEST(SensorsTest, SensorTypeToString) {
  test_utils::LogCaptureFixture log_capture;
  MockResource mock_resource;

  // Valid cases should not log errors
  EXPECT_EQ(sensor_type_to_string(SensorType::depth, mock_resource.get_logger()), "depth");
  EXPECT_EQ(sensor_type_to_string(SensorType::color, mock_resource.get_logger()), "color");

  auto logs_before = log_capture.get_error_logs();
  EXPECT_EQ(logs_before.size(), 0);

  // Invalid case should log error
  log_capture.clear();
  EXPECT_EQ(sensor_type_to_string(SensorType::unknown, mock_resource.get_logger()),
            "unknown");

  auto error_logs = log_capture.get_error_logs();
  ASSERT_EQ(error_logs.size(), 1);
  EXPECT_THAT(error_logs[0].message,
              ::testing::HasSubstr("Invalid sensor type"));
}

TEST(SensorsTest, StringToSensorType) {
  test_utils::LogCaptureFixture log_capture;
  MockResource mock_resource;

  // Valid cases
  EXPECT_EQ(string_to_sensor_type("depth", mock_resource.get_logger()), SensorType::depth);
  EXPECT_EQ(string_to_sensor_type("color", mock_resource.get_logger()), SensorType::color);
  EXPECT_EQ(log_capture.get_error_logs().size(), 0);

  // Invalid case should log error
  log_capture.clear();
  EXPECT_EQ(string_to_sensor_type("foo", mock_resource.get_logger()), SensorType::unknown);

  auto error_logs = log_capture.get_error_logs();
  ASSERT_EQ(error_logs.size(), 1);
  EXPECT_THAT(error_logs[0].message,
              ::testing::HasSubstr("Invalid sensor type: foo"));
}

// Dummy classes to mock rs2::sensor-like interface for get_sensor_type template
struct MockDepthSensor {
  template <typename T> bool is() const {
    return std::is_same<T, rs2::depth_sensor>::value;
  }
};

struct MockColorSensor {
  template <typename T> bool is() const {
    return std::is_same<T, rs2::color_sensor>::value;
  }
};

struct MockUnknownSensor {
  template <typename T> bool is() const { return false; }
};

TEST(SensorsTest, GetSensorType) {
  test_utils::LogCaptureFixture log_capture;
  MockResource mock_resource;

  MockDepthSensor depth_sensor;
  EXPECT_EQ(get_sensor_type(depth_sensor, mock_resource.get_logger()), SensorType::depth);
  EXPECT_EQ(log_capture.get_error_logs().size(), 0);

  MockColorSensor color_sensor;
  log_capture.clear();
  EXPECT_EQ(get_sensor_type(color_sensor, mock_resource.get_logger()), SensorType::color);
  EXPECT_EQ(log_capture.get_error_logs().size(), 0);

  MockUnknownSensor unknown_sensor;
  log_capture.clear();
  EXPECT_EQ(get_sensor_type(unknown_sensor, mock_resource.get_logger()),
            SensorType::unknown);

  auto error_logs = log_capture.get_error_logs();
  ASSERT_EQ(error_logs.size(), 1);
  EXPECT_THAT(error_logs[0].message,
              ::testing::HasSubstr("Invalid sensor type"));
}

} // namespace sensors
} // namespace realsense
