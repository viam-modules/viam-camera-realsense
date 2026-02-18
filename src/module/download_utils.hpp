#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <viam/sdk/log/logging.hpp>

namespace viam {
namespace realsense {
namespace download_utils {

/**
 * Download a file from a URL using libcurl.
 * @param url The URL to download from
 * @return The downloaded file data as a vector of bytes
 * @throws std::runtime_error if download fails
 */
std::vector<uint8_t> downloadFirmwareFromURL(const std::string &url,
                                             viam::sdk::LogSource &logger);

} // namespace download_utils
} // namespace realsense
} // namespace viam
