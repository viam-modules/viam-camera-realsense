#include "firmware_update.hpp"
#include "download_utils.hpp"
#include "zip_utils.hpp"
#include <optional>
#include <stdexcept>
#include <thread>

namespace viam {
namespace realsense {
namespace firmware_update {

std::vector<uint8_t> readFirmwareFile(const std::string &file_path) {
  std::ifstream file(file_path, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open firmware file: " + file_path);
  }

  std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);

  std::vector<uint8_t> firmware_data(size);
  if (!file.read(reinterpret_cast<char *>(firmware_data.data()), size)) {
    throw std::runtime_error("Failed to read firmware file: " + file_path);
  }

  return firmware_data;
}

// Get firmware download URL for a given version
std::optional<std::string>
getFirmwareURLForVersion(const std::string &version) {
  // Mapping of known firmware versions to download URLs
  static const std::unordered_map<std::string, std::string> firmware_url_map = {
      {"5.17.0.10", "https://realsenseai.com/wp-content/uploads/2025/07/"
                    "d400_series_production_fw_5_17_0_10.zip"},
      // Add more firmware versions here as they become available
  };

  auto it = firmware_url_map.find(version);
  if (it != firmware_url_map.end()) {
    return it->second;
  }
  return std::nullopt;
}

} // namespace firmware_update
} // namespace realsense
} // namespace viam
