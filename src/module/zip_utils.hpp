#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <viam/sdk/log/logging.hpp>

namespace viam {
namespace realsense {
namespace zip_utils {

/**
 * Check if data contains a ZIP file by looking for ZIP magic bytes.
 * @param data The data to check
 * @return true if data appears to be a ZIP file, false otherwise
 */
bool isZipFile(const std::vector<uint8_t> &data);

/**
 * Extract the first .bin file from a ZIP archive.
 * Supports both stored (uncompressed) and DEFLATE-compressed files.
 * Uses central directory parsing to handle data descriptors correctly.
 *
 * @param zip_data The complete ZIP file data
 * @return The extracted .bin file contents
 * @throws std::runtime_error if ZIP is invalid or no .bin file found
 */
std::vector<uint8_t> extractBinFromZip(const std::vector<uint8_t> &zip_data,
                                       viam::sdk::LogSource &logger);

} // namespace zip_utils
} // namespace realsense
} // namespace viam
