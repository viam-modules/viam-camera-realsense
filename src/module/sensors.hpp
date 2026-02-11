#include <boost/algorithm/string.hpp>
#include <librealsense2/rs.hpp>

#include <viam/sdk/log/logging.hpp>

#pragma once

namespace realsense {
namespace sensors {

enum class SensorType { depth, color, unknown };

std::string sensor_type_to_string(SensorType const sensor_type,
                                  viam::sdk::LogSource &logger) {
  switch (sensor_type) {
  case SensorType::depth:
    return "depth";
  case SensorType::color:
    return "color";
  default:
    VIAM_SDK_LOG_IMPL(logger, error)
        << "[sensor_type_to_string] Invalid sensor type";
    return "unknown";
  }
}

SensorType string_to_sensor_type(std::string const &sensor_type,
                                 viam::sdk::LogSource &logger) {
  auto const sensor_type_lower = boost::algorithm::to_lower_copy(sensor_type);
  if (sensor_type_lower == "depth") {
    return SensorType::depth;
  } else if (sensor_type_lower == "color") {
    return SensorType::color;
  }
  VIAM_SDK_LOG_IMPL(logger, error)
      << "[string_to_sensor_type] Invalid sensor type: " << sensor_type;
  return SensorType::unknown;
}

template <typename SensorT>
SensorType get_sensor_type(SensorT const &sensor,
                           viam::sdk::LogSource &logger) {
  if (sensor.template is<rs2::depth_sensor>()) {
    return SensorType::depth;
  } else if (sensor.template is<rs2::color_sensor>()) {
    return SensorType::color;
  }
  VIAM_SDK_LOG_IMPL(logger, error) << "[get_sensor_type] Invalid sensor type";
  return SensorType::unknown;
}
} // namespace sensors
} // namespace realsense
