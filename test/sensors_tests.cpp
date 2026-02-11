#include "../src/module/sensors.hpp"
#include "log_capture.hpp"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace realsense {
namespace sensors {

TEST(SensorsTest, SensorTypeToString) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  // Valid cases should not log errors
  EXPECT_EQ(sensor_type_to_string(SensorType::depth, logger), "depth");
  EXPECT_EQ(sensor_type_to_string(SensorType::color, logger), "color");

  auto logs_before = log_capture.get_error_logs();
  EXPECT_EQ(logs_before.size(), 0);

  // Invalid case should log error
  log_capture.clear();
  EXPECT_EQ(sensor_type_to_string(SensorType::unknown, logger), "unknown");

  auto error_logs = log_capture.get_error_logs();
  ASSERT_EQ(error_logs.size(), 1);
  EXPECT_THAT(error_logs[0].message,
              ::testing::HasSubstr("Invalid sensor type"));
}

TEST(SensorsTest, StringToSensorType) {
  test_utils::LogCaptureFixture log_capture;
  viam::sdk::LogSource logger;

  // Valid cases
  EXPECT_EQ(string_to_sensor_type("depth", logger), SensorType::depth);
  EXPECT_EQ(string_to_sensor_type("color", logger), SensorType::color);
  EXPECT_EQ(log_capture.get_error_logs().size(), 0);

  // Invalid case should log error
  log_capture.clear();
  EXPECT_EQ(string_to_sensor_type("foo", logger), SensorType::unknown);

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
  viam::sdk::LogSource logger;

  MockDepthSensor depth_sensor;
  EXPECT_EQ(get_sensor_type(depth_sensor, logger), SensorType::depth);
  EXPECT_EQ(log_capture.get_error_logs().size(), 0);

  MockColorSensor color_sensor;
  log_capture.clear();
  EXPECT_EQ(get_sensor_type(color_sensor, logger), SensorType::color);
  EXPECT_EQ(log_capture.get_error_logs().size(), 0);

  MockUnknownSensor unknown_sensor;
  log_capture.clear();
  EXPECT_EQ(get_sensor_type(unknown_sensor, logger), SensorType::unknown);

  auto error_logs = log_capture.get_error_logs();
  ASSERT_EQ(error_logs.size(), 1);
  EXPECT_THAT(error_logs[0].message,
              ::testing::HasSubstr("Invalid sensor type"));
}

} // namespace sensors
} // namespace realsense
