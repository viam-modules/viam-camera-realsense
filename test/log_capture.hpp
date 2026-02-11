#pragma once

#include <boost/log/attributes/value_extraction.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/basic_sink_backend.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/make_shared.hpp>
#include <mutex>
#include <string>
#include <vector>
#include <viam/sdk/log/logging.hpp>

namespace test_utils {

// Structure to store captured log records
struct CapturedLogRecord {
  viam::sdk::log_level level;
  std::string message;
};

// Custom Boost.Log sink backend to capture log messages
class LogCaptureSink : public boost::log::sinks::basic_sink_backend<
                           boost::log::sinks::synchronized_feeding> {
public:
  void consume(const boost::log::record_view &rec) {
    std::lock_guard<std::mutex> lock(mutex_);

    CapturedLogRecord record;

    // Extract severity level
    auto level_attr = rec.attribute_values()["Severity"];
    if (level_attr) {
      record.level = level_attr.extract<viam::sdk::log_level>().get();
    }

    // Extract formatted message
    auto message = rec[boost::log::expressions::smessage];
    if (message) {
      record.message = message.get();
    }

    records_.push_back(record);
  }

  std::vector<CapturedLogRecord> get_records() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return records_;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    records_.clear();
  }

private:
  mutable std::mutex mutex_;
  std::vector<CapturedLogRecord> records_;
};

// RAII fixture for log capture setup/teardown
// Usage:
//   test_utils::LogCaptureFixture log_capture;
//   viam::sdk::LogSource logger;
//   // ... perform operations that log ...
//   auto error_logs = log_capture.get_error_logs();
//   EXPECT_EQ(error_logs.size(), 1);
class LogCaptureFixture {
public:
  LogCaptureFixture() {
    // Create backend and sink
    auto backend = boost::make_shared<LogCaptureSink>();
    backend_ = backend;
    sink_ =
        boost::make_shared<boost::log::sinks::synchronous_sink<LogCaptureSink>>(
            backend);

    // Register the sink with Boost.Log core
    boost::log::core::get()->add_sink(sink_);
  }

  ~LogCaptureFixture() {
    // Unregister the sink
    boost::log::core::get()->remove_sink(sink_);
  }

  // Get all captured log records
  std::vector<CapturedLogRecord> get_records() const {
    return backend_->get_records();
  }

  // Get only error-level logs
  std::vector<CapturedLogRecord> get_error_logs() const {
    auto all_records = backend_->get_records();
    std::vector<CapturedLogRecord> error_logs;
    for (const auto &record : all_records) {
      if (record.level == viam::sdk::log_level::error) {
        error_logs.push_back(record);
      }
    }
    return error_logs;
  }

  // Get only warning-level logs
  std::vector<CapturedLogRecord> get_warning_logs() const {
    auto all_records = backend_->get_records();
    std::vector<CapturedLogRecord> warning_logs;
    for (const auto &record : all_records) {
      if (record.level == viam::sdk::log_level::warn) {
        warning_logs.push_back(record);
      }
    }
    return warning_logs;
  }

  // Get logs by specific level
  std::vector<CapturedLogRecord>
  get_logs_by_level(viam::sdk::log_level level) const {
    auto all_records = backend_->get_records();
    std::vector<CapturedLogRecord> filtered_logs;
    for (const auto &record : all_records) {
      if (record.level == level) {
        filtered_logs.push_back(record);
      }
    }
    return filtered_logs;
  }

  // Clear all captured logs
  void clear() { backend_->clear(); }

private:
  boost::shared_ptr<LogCaptureSink> backend_;
  boost::shared_ptr<boost::log::sinks::synchronous_sink<LogCaptureSink>> sink_;
};

} // namespace test_utils
