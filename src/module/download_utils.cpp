#include "download_utils.hpp"
#include "utils.hpp"
#include <curl/curl.h>
#include <stdexcept>

namespace viam {
namespace realsense {
namespace download_utils {

// Use CleanupPtr from utils.hpp for RAII management
using utils::CleanupPtr;

// Static callback function for curl to write downloaded data
static size_t curlWriteCallback(void *contents, size_t size, size_t nmemb,
                                void *userp) {
  size_t const total_size = size * nmemb;
  std::vector<uint8_t> *buffer = static_cast<std::vector<uint8_t> *>(userp);
  buffer->insert(buffer->end(), static_cast<uint8_t *>(contents),
                 static_cast<uint8_t *>(contents) + total_size);
  return total_size;
}

std::vector<uint8_t> downloadFirmwareFromURL(const std::string &url,
                                             viam::sdk::LogSource &logger) {
  std::vector<uint8_t> firmware_data;

  // Initialize curl with RAII wrapper for automatic cleanup
  CleanupPtr<curl_easy_cleanup> curl(curl_easy_init());
  if (!curl) {
    std::string error_msg = "Failed to initialize curl";
    VIAM_SDK_LOG_IMPL(logger, error) << error_msg;
    throw std::runtime_error(error_msg);
  }

  // Set curl options
  curl_easy_setopt(curl.get(), CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl.get(), CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(curl.get(), CURLOPT_WRITEFUNCTION, curlWriteCallback);
  curl_easy_setopt(curl.get(), CURLOPT_WRITEDATA, &firmware_data);
  curl_easy_setopt(curl.get(), CURLOPT_TIMEOUT, 300L); // 5 minute timeout

  // Perform the download
  CURLcode res = curl_easy_perform(curl.get());

  // Check for errors
  if (res != CURLE_OK) {
    std::string error_msg = "Failed to download firmware: ";
    error_msg += curl_easy_strerror(res);
    VIAM_SDK_LOG_IMPL(logger, error) << error_msg;
    throw std::runtime_error(error_msg);
    // curl handle automatically cleaned up by unique_ptr
  }

  // Check HTTP response code
  long http_code = 0;
  curl_easy_getinfo(curl.get(), CURLINFO_RESPONSE_CODE, &http_code);
  // No manual cleanup needed - unique_ptr handles it automatically

  if (http_code != 200) {
    std::string error_msg = "HTTP error " + std::to_string(http_code) +
                            " when downloading firmware";
    VIAM_SDK_LOG_IMPL(logger, error) << error_msg;
    throw std::runtime_error(error_msg);
  }

  if (firmware_data.empty()) {
    std::string error_msg = "Downloaded firmware file is empty";
    VIAM_SDK_LOG_IMPL(logger, error) << error_msg;
    throw std::runtime_error(error_msg);
  }
  VIAM_SDK_LOG_IMPL(logger, info)
      << "[downloadFirmwareFromURL] Firmware downloaded successfully";

  return firmware_data;
}

} // namespace download_utils
} // namespace realsense
} // namespace viam
