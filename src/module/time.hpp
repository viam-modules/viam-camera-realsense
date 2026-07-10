#pragma once
#include <chrono>  // For std::chrono
#include <cstdint> // For std::uint64_t
#include <sstream>
#include <string>
namespace realsense {
namespace time {
inline double getNowMs() {
  // Must use system_clock (Unix-epoch domain) to match librealsense frame
  // timestamps under RS2_OPTION_GLOBAL_TIME_ENABLED. NOTE: do NOT use
  // high_resolution_clock here — on libc++ (macOS) it aliases steady_clock,
  // whose epoch is ~process start, so frame-age math (now - frame_ts) would be
  // hugely negative and rate-limit window math would underflow.
  auto now = std::chrono::system_clock::now();
  auto now_ms =
      std::chrono::duration<double, std::milli>(now.time_since_epoch())
          .count(); // Unix time (ms)

  return now_ms;
}

inline double timeSincePrevMs(double nowMs, double prevTimeMs) {
  if (nowMs > prevTimeMs) {
    return nowMs - prevTimeMs;
  }
  return 0.0;
}

inline bool isTooOld(double const nowMs, double const prevTimeMs,
                     double const maxAgeMs) {
  return timeSincePrevMs(nowMs, prevTimeMs) > maxAgeMs;
}

template <typename SinkT>
SinkT &logIfTooOld(SinkT &sink, double const nowMs, double const prevTimeMs,
                   double const maxAgeMs, std::string const &error_msg) {
  if (isTooOld(nowMs, prevTimeMs, maxAgeMs)) {
    sink << error_msg << ", timestamp: " << prevTimeMs
         << "ms, time diff: " << timeSincePrevMs(nowMs, prevTimeMs) << "ms";
  }
  return sink;
}

inline void throwIfTooOld(double const nowMs, double const prevTimeMs,
                          double const maxAgeMs, std::string const &error_msg) {
  if (isTooOld(nowMs, prevTimeMs, maxAgeMs)) {
    std::ostringstream buffer;
    buffer << error_msg << ", timestamp: " << prevTimeMs
           << "ms, time diff: " << timeSincePrevMs(nowMs, prevTimeMs) << "ms";
    throw std::invalid_argument(buffer.str());
  }
}

} // namespace time
} // namespace realsense
