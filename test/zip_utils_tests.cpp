#include "../src/module/zip_utils.hpp"
#include <cstring>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <memory>
#include <viam/sdk/log/logging.hpp>
#include <zip.h>

using namespace viam::realsense::zip_utils;

// Helper to create a minimal valid ZIP file with a .bin file
std::vector<uint8_t>
createValidZipWithBin(const std::string &filename,
                      const std::vector<uint8_t> &content) {
  // Create a temporary zip file
  std::filesystem::path tempDir = std::filesystem::temp_directory_path();
  std::filesystem::path zipPath =
      tempDir / ("test_" + std::to_string(rand()) + ".zip");

  int error = 0;
  zip_t *zip = zip_open(zipPath.c_str(), ZIP_CREATE | ZIP_TRUNCATE, &error);
  if (!zip) {
    throw std::runtime_error("Failed to create zip file");
  }

  // Add file to zip
  zip_source_t *file_src =
      zip_source_buffer(zip, content.data(), content.size(), 0);
  if (!file_src) {
    zip_close(zip);
    throw std::runtime_error("Failed to create file source");
  }

  if (zip_file_add(zip, filename.c_str(), file_src, ZIP_FL_ENC_UTF_8) < 0) {
    zip_source_free(file_src);
    zip_close(zip);
    throw std::runtime_error("Failed to add file to zip");
  }

  if (zip_close(zip) < 0) {
    throw std::runtime_error("Failed to close zip");
  }

  // Read the zip file into a vector
  std::ifstream file(zipPath, std::ios::binary);
  if (!file) {
    std::filesystem::remove(zipPath);
    throw std::runtime_error("Failed to read zip file");
  }

  std::vector<uint8_t> zipData((std::istreambuf_iterator<char>(file)),
                               std::istreambuf_iterator<char>());
  file.close();

  // Clean up temporary file
  std::filesystem::remove(zipPath);

  return zipData;
}

// Test fixture
class ZipUtilsTest : public ::testing::Test {
protected:
  void SetUp() override {
    // LogSource has default constructor
  }

  viam::sdk::LogSource logger_;
};

// ============================================================================
// Tests for isZipFile()
// ============================================================================

TEST_F(ZipUtilsTest, IsZipFile_ValidZipSignature_ReturnsTrue) {
  std::vector<uint8_t> data = {0x50, 0x4B, 0x03, 0x04, 0x00, 0x00};
  EXPECT_TRUE(isZipFile(data));
}

TEST_F(ZipUtilsTest, IsZipFile_EmptyData_ReturnsFalse) {
  std::vector<uint8_t> data;
  EXPECT_FALSE(isZipFile(data));
}

TEST_F(ZipUtilsTest, IsZipFile_TooShort_ReturnsFalse) {
  std::vector<uint8_t> data = {0x50, 0x4B, 0x03};
  EXPECT_FALSE(isZipFile(data));
}

TEST_F(ZipUtilsTest, IsZipFile_InvalidSignature_ReturnsFalse) {
  std::vector<uint8_t> data = {0x00, 0x00, 0x00, 0x00};
  EXPECT_FALSE(isZipFile(data));
}

TEST_F(ZipUtilsTest, IsZipFile_PartialSignature_ReturnsFalse) {
  std::vector<uint8_t> data = {0x50, 0x4B, 0x00, 0x00};
  EXPECT_FALSE(isZipFile(data));
}

TEST_F(ZipUtilsTest, IsZipFile_RandomData_ReturnsFalse) {
  std::vector<uint8_t> data = {0xFF, 0xFE, 0xFD, 0xFC, 0xFB};
  EXPECT_FALSE(isZipFile(data));
}

// ============================================================================
// Tests for extractBinFromZip()
// ============================================================================

TEST_F(ZipUtilsTest, ExtractBinFromZip_ValidZipWithBinFile_ExtractsCorrectly) {
  std::vector<uint8_t> binContent = {0x01, 0x02, 0x03, 0x04, 0x05};
  std::vector<uint8_t> zipData =
      createValidZipWithBin("firmware.bin", binContent);

  auto result = extractBinFromZip(zipData, logger_);

  EXPECT_EQ(result, binContent);
}

TEST_F(ZipUtilsTest, ExtractBinFromZip_EmptyBinFile_ReturnsEmpty) {
  std::vector<uint8_t> binContent;
  std::vector<uint8_t> zipData = createValidZipWithBin("empty.bin", binContent);

  auto result = extractBinFromZip(zipData, logger_);

  EXPECT_TRUE(result.empty());
}

TEST_F(ZipUtilsTest, ExtractBinFromZip_LargeBinFile_ExtractsCorrectly) {
  // Create a 1MB file
  std::vector<uint8_t> binContent(1024 * 1024);
  for (size_t i = 0; i < binContent.size(); i++) {
    binContent[i] = static_cast<uint8_t>(i % 256);
  }

  std::vector<uint8_t> zipData = createValidZipWithBin("large.bin", binContent);

  auto result = extractBinFromZip(zipData, logger_);

  EXPECT_EQ(result.size(), binContent.size());
  EXPECT_EQ(result, binContent);
}

TEST_F(ZipUtilsTest, ExtractBinFromZip_BinInSubdirectory_ExtractsCorrectly) {
  std::vector<uint8_t> binContent = {0xAA, 0xBB, 0xCC};
  std::vector<uint8_t> zipData =
      createValidZipWithBin("subdir/firmware.bin", binContent);

  auto result = extractBinFromZip(zipData, logger_);

  EXPECT_EQ(result, binContent);
}

TEST_F(ZipUtilsTest, ExtractBinFromZip_InvalidZipData_ThrowsException) {
  std::vector<uint8_t> invalidData = {0x00, 0x01, 0x02, 0x03};

  EXPECT_THROW(
      { extractBinFromZip(invalidData, logger_); }, std::runtime_error);
}

TEST_F(ZipUtilsTest, ExtractBinFromZip_EmptyZipData_ThrowsException) {
  std::vector<uint8_t> emptyData;

  EXPECT_THROW({ extractBinFromZip(emptyData, logger_); }, std::runtime_error);
}

TEST_F(ZipUtilsTest, ExtractBinFromZip_ZipWithoutBinFile_ThrowsException) {
  // Create a zip with a non-.bin file
  std::vector<uint8_t> txtContent = {0x48, 0x65, 0x6C, 0x6C, 0x6F}; // "Hello"
  std::vector<uint8_t> zipData =
      createValidZipWithBin("readme.txt", txtContent);

  EXPECT_THROW({ extractBinFromZip(zipData, logger_); }, std::runtime_error);
}

TEST_F(ZipUtilsTest,
       ExtractBinFromZip_BinFileWithSpecialCharacters_ExtractsCorrectly) {
  std::vector<uint8_t> binContent = {0x00, 0xFF, 0x01, 0xFE, 0x7F, 0x80};
  std::vector<uint8_t> zipData =
      createValidZipWithBin("special-name_123.bin", binContent);

  auto result = extractBinFromZip(zipData, logger_);

  EXPECT_EQ(result, binContent);
}

TEST_F(ZipUtilsTest,
       ExtractBinFromZip_UppercaseBinExtension_ExtractsCorrectly) {
  std::vector<uint8_t> binContent = {0xDE, 0xAD, 0xBE, 0xEF};
  // Note: The current implementation checks for lowercase .bin only
  // This test documents current behavior - if you want case-insensitive
  // matching, update the implementation
  std::vector<uint8_t> zipData =
      createValidZipWithBin("FIRMWARE.BIN", binContent);

  // Current implementation is case-sensitive, so this should throw
  EXPECT_THROW({ extractBinFromZip(zipData, logger_); }, std::runtime_error);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST_F(ZipUtilsTest, Integration_CheckThenExtract_WorksCorrectly) {
  std::vector<uint8_t> binContent = {0x01, 0x02, 0x03, 0x04};
  std::vector<uint8_t> zipData =
      createValidZipWithBin("firmware.bin", binContent);

  // First check if it's a zip
  ASSERT_TRUE(isZipFile(zipData));

  // Then extract
  auto result = extractBinFromZip(zipData, logger_);

  EXPECT_EQ(result, binContent);
}

TEST_F(ZipUtilsTest, Integration_NonZipData_IsZipReturnsFalseAndExtractThrows) {
  std::vector<uint8_t> nonZipData = {0x01, 0x02, 0x03, 0x04};

  EXPECT_FALSE(isZipFile(nonZipData));

  EXPECT_THROW({ extractBinFromZip(nonZipData, logger_); }, std::runtime_error);
}

TEST_F(ZipUtilsTest, Integration_RealWorldFirmwareFile_ExtractsCorrectly) {
  // Simulate a real firmware file scenario
  std::vector<uint8_t> firmwareData = {
      0x48, 0x45, 0x41, 0x44, 0x45, 0x52, // "HEADER"
      0x01, 0x02, 0x03, 0x04, 0x05, 0x06, // version info
      0xFF, 0xFF, 0xFF, 0xFF              // data
  };

  std::vector<uint8_t> zipData =
      createValidZipWithBin("D4XX_FW_Image-5.17.0.9.bin", firmwareData);

  // Verify it's a zip
  ASSERT_TRUE(isZipFile(zipData));

  // Extract and verify
  auto result = extractBinFromZip(zipData, logger_);
  EXPECT_EQ(result, firmwareData);
}
