#include "zip_utils.hpp"
#include "utils.hpp"
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <zip.h>
#include <zlib.h>

namespace viam {
namespace realsense {
namespace zip_utils {

bool isZipFile(const std::vector<uint8_t> &data) {
  // Check for ZIP file signature (PK\x03\x04)
  if (data.size() < 4)
    return false;
  return data[0] == 0x50 && data[1] == 0x4B && data[2] == 0x03 &&
         data[3] == 0x04;
}

/*
 * Alternative implementation using libzip library.
 * To use this version:
 * 1. Install libzip: apt-get install libzip-dev (or add libzip/1.10.1 to
 * conanfile.py)
 * 2. Uncomment the includes at the top of this file
 * 3. Add libzip to CMakeLists.txt: find_package(libzip) and link against it
 * 4. Uncomment this function
 * 5. Change realsense.hpp to call extractBinFromZip2 instead of
 * extractBinFromZip
 *
 * Benefits: Simpler code, battle-tested library, handles all ZIP edge cases
 * Drawback: Additional dependency
 */

// Use CleanupPtr from utils.hpp for RAII management
using utils::CleanupPtr;

std::vector<uint8_t> extractBinFromZip(const std::vector<uint8_t> &zip_data,
                                       viam::sdk::LogSource &logger) {
  std::vector<uint8_t> binData;
  zip_error_t ziperror;
  zip_error_init(&ziperror);

  zip_source_t *src =
      zip_source_buffer_create(zip_data.data(), zip_data.size(), 0, &ziperror);
  if (!src) {
    std::ostringstream buffer;
    buffer << "failed to create zip buffer: " << zip_error_strerror(&ziperror);
    VIAM_SDK_LOG_IMPL(logger, error) << buffer.str();
    throw std::runtime_error(buffer.str());
  }

  // Ensure src cleanup if zip_open fails
  CleanupPtr<zip_source_free> srcCleanup(src);

  // If this succeeds, zip takes ownership of src, so src will be freed when
  // zip_close is called.
  zip_t *zip = zip_open_from_source(src, 0, &ziperror);
  if (!zip) {
    std::ostringstream buffer;
    buffer << "failed to open zip from source: "
           << zip_error_strerror(&ziperror);
    VIAM_SDK_LOG_IMPL(logger, error) << buffer.str();
    throw std::runtime_error(buffer.str());
  }

  srcCleanup.release();
  CleanupPtr<zip_close> zipCleanup(zip);

  // Find the first .bin file in the ZIP
  zip_int64_t num_entries = zip_get_num_entries(zip, 0);
  const char *binFileName = nullptr;

  for (zip_int64_t i = 0; i < num_entries; i++) {
    const char *fileName = zip_get_name(zip, i, 0);
    if (!fileName) {
      continue;
    }

    std::string name(fileName);
    if (name.size() >= 4 && name.substr(name.size() - 4) == ".bin") {
      binFileName = fileName;
      break;
    }
  }

  if (!binFileName) {
    VIAM_SDK_LOG_IMPL(logger, error) << "No .bin file found in ZIP archive";
    throw std::runtime_error("No .bin file found in ZIP archive");
  }

  VIAM_SDK_LOG_IMPL(logger, info)
      << "[extractBinFromZip] Found .bin file: " << binFileName;

  CleanupPtr<zip_fclose> binFile(zip_fopen(zip, binFileName, 0));
  if (!binFile) {
    std::ostringstream buffer;
    buffer << "failed to open the firmware bin file: "
           << zip_error_strerror(zip_file_get_error(binFile.get()));
    VIAM_SDK_LOG_IMPL(logger, error) << buffer.str();
    throw std::runtime_error(buffer.str());
  }

  zip_stat_t stats;
  zip_stat_init(&stats);
  if (zip_stat(zip, binFileName, 0, &stats) != 0) {
    std::ostringstream buffer;
    buffer << "failed to stat file: "
           << zip_error_strerror(zip_file_get_error(binFile.get()));
    throw std::invalid_argument(buffer.str());
  }

  binData.resize(stats.size);
  zip_int64_t bytesRead = zip_fread(binFile.get(), binData.data(), stats.size);
  if (bytesRead == -1) {
    zip_error_t *err = zip_file_get_error(binFile.get());
    std::ostringstream buffer;
    buffer << "failed to read bin: " << zip_error_strerror(err);
    VIAM_SDK_LOG_IMPL(logger, error) << buffer.str();
    throw std::runtime_error(buffer.str());
  }

  if (bytesRead != stats.size) {
    std::ostringstream buffer;
    buffer << "failed to fully read binary file, file size: " << stats.size
           << "bytes read: " << bytesRead;
    VIAM_SDK_LOG_IMPL(logger, error) << buffer.str();
    throw std::runtime_error(buffer.str());
  }

  return binData;
}

} // namespace zip_utils
} // namespace realsense
} // namespace viam
