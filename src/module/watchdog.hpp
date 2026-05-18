#pragma once

#include "time.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include <viam/sdk/log/logging.hpp>

#include <boost/thread/synchronized_value.hpp>

namespace realsense {
namespace watchdog {

// Forward declare so the template signature reads cleanly.
template <typename FrameSetT> class StaleFrameWatchdog;

// Default tunables. Defined as constexpr in the class body below; this
// block names them in one place for the reader.
//
//   POLL_INTERVAL_MS              = 1000   one poll per second; off the hot path
//   STALE_THRESHOLD_MS            = 10000  10x MAX_FRAME_AGE_MS; well above
//                                          normal intra-frameset jitter
//   CONSECUTIVE_POLLS_REQUIRED    = 3      ~3s of sustained staleness before action
//   POST_RESTART_GRACE_MS         = 10000  rs2::pipeline needs a few seconds to
//                                          start producing frames after restart
//   MAX_RESTARTS_PER_HOUR         = 6      ~one restart every 10 minutes max;
//                                          beyond that escalate to operator

// StaleFrameWatchdog
//
// Background thread that periodically checks the cached frameset for
// staleness (via frame.get_timestamp() age vs host clock). When the
// most-recent frame has been stale for CONSECUTIVE_POLLS_REQUIRED polls,
// the watchdog invokes a caller-provided RestartFn that is expected to
// tear down and re-create the rs2::pipeline.
//
// Owned by Realsense. Construct after the pipeline is started, destroy
// before the pipeline is torn down. Use pause()/resume() around
// operator-initiated lifecycle transitions (reconfigure, firmware
// update, USB device-change handling) so the watchdog does not race
// with those operations.
//
// Concurrency contract:
//   - frame_set_ is read through its synchronized_value; safe wrt.
//     frameCallback writes.
//   - The RestartFn must acquire whatever mutex serializes pipeline
//     mutations (in our case do_command_mutex_). Watchdog does not
//     manage that mutex itself.
//   - All public methods are safe to call from any thread.
template <typename FrameSetT>
class StaleFrameWatchdog {
 public:
  // Returns true if the restart was attempted and (best-effort) succeeded;
  // false if the watchdog should not count it against the rate limit.
  using RestartFn = std::function<bool()>;
  // Returns true if the device is in recovery mode (e.g. firmware update);
  // the watchdog skips its check while this returns true.
  using RecoveryCheckFn = std::function<bool()>;

  StaleFrameWatchdog(
      std::shared_ptr<boost::synchronized_value<FrameSetT>> frame_set,
      RecoveryCheckFn recovery_check, RestartFn on_stale,
      viam::sdk::LogSource logger)
      : frame_set_(std::move(frame_set)),
        recovery_check_(std::move(recovery_check)),
        on_stale_(std::move(on_stale)),
        logger_(std::move(logger)) {
    thread_ = std::thread([this]() { loop(); });
  }

  ~StaleFrameWatchdog() {
    running_.store(false);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  StaleFrameWatchdog(const StaleFrameWatchdog&) = delete;
  StaleFrameWatchdog& operator=(const StaleFrameWatchdog&) = delete;
  StaleFrameWatchdog(StaleFrameWatchdog&&) = delete;
  StaleFrameWatchdog& operator=(StaleFrameWatchdog&&) = delete;

  // Pause/resume during operator-initiated pipeline transitions
  // (reconfigure, firmware update, USB device change). Idempotent.
  void pause() noexcept { paused_.store(true); }
  void resume() noexcept { paused_.store(false); }

  // Enable or disable the restart action. When disabled, the watchdog
  // still detects and logs sustained staleness but does not invoke
  // on_stale_. Useful for canary deployment.
  void set_restart_enabled(bool enabled) noexcept {
    restart_enabled_.store(enabled);
  }

 private:
  static constexpr std::uint64_t POLL_INTERVAL_MS = 1000;
  static constexpr std::uint64_t STALE_THRESHOLD_MS = 10'000;
  static constexpr int CONSECUTIVE_POLLS_REQUIRED = 3;
  static constexpr std::uint64_t POST_RESTART_GRACE_MS = 10'000;
  static constexpr int MAX_RESTARTS_PER_HOUR = 6;
  static constexpr std::uint64_t ONE_HOUR_MS = 60ULL * 60ULL * 1000ULL;

  void loop() {
    VIAM_SDK_LOG_IMPL(logger_, info) << "[watchdog] started";
    int stale_count = 0;

    while (running_.load()) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(POLL_INTERVAL_MS));
      if (!running_.load()) break;

      if (paused_.load()) {
        stale_count = 0;
        continue;
      }
      if (recovery_check_ && recovery_check_()) {
        stale_count = 0;
        continue;
      }
      if (!frame_set_) {
        // No frameset cache yet (e.g., device hasn't streamed). Wait.
        stale_count = 0;
        continue;
      }

      double age_ms = 0.0;
      if (!compute_max_age_ms(age_ms)) {
        // No frame available yet; treat as not-stale.
        stale_count = 0;
        continue;
      }

      if (age_ms <= static_cast<double>(STALE_THRESHOLD_MS)) {
        stale_count = 0;
        continue;
      }

      stale_count += 1;
      if (stale_count < CONSECUTIVE_POLLS_REQUIRED) {
        continue;
      }

      // Sustained staleness confirmed.
      VIAM_SDK_LOG_IMPL(logger_, warn)
          << "[watchdog] sustained stale frame: age=" << age_ms
          << "ms threshold=" << STALE_THRESHOLD_MS << "ms";

      if (!restart_enabled_.load()) {
        VIAM_SDK_LOG_IMPL(logger_, info)
            << "[watchdog] restart disabled; continuing to monitor";
        stale_count = 0;
        continue;
      }

      if (rate_limited()) {
        VIAM_SDK_LOG_IMPL(logger_, error)
            << "[watchdog] rate-limited at " << MAX_RESTARTS_PER_HOUR
            << " restarts/hour — operator intervention needed";
        // Keep counter at threshold so we log on every subsequent poll
        // (no flapping silence). Reset only after grace would mask the
        // operator-attention signal.
        stale_count = CONSECUTIVE_POLLS_REQUIRED;
        continue;
      }

      bool ok = false;
      try {
        ok = on_stale_ ? on_stale_() : false;
      } catch (const std::exception& e) {
        VIAM_SDK_LOG_IMPL(logger_, error)
            << "[watchdog] restart callback threw: " << e.what();
      } catch (...) {
        VIAM_SDK_LOG_IMPL(logger_, error)
            << "[watchdog] restart callback threw unknown exception";
      }

      if (ok) {
        record_restart();
        std::this_thread::sleep_for(
            std::chrono::milliseconds(POST_RESTART_GRACE_MS));
      }
      stale_count = 0;
    }

    VIAM_SDK_LOG_IMPL(logger_, info) << "[watchdog] stopped";
  }

  // Returns true and writes the max age (in ms) of color/depth into
  // out_age_ms; returns false if no frame is currently available.
  bool compute_max_age_ms(double& out_age_ms) const {
    auto fs = frame_set_->get();
    double now_ms = time::getNowMs();
    double color_age = 0.0;
    double depth_age = 0.0;
    bool any = false;

    auto color = fs.get_color_frame();
    if (color) {
      color_age = now_ms - color.get_timestamp();
      any = true;
    }
    auto depth = fs.get_depth_frame();
    if (depth) {
      depth_age = now_ms - depth.get_timestamp();
      any = true;
    }
    if (!any) return false;
    out_age_ms = std::max(color_age, depth_age);
    return true;
  }

  bool rate_limited() {
    std::lock_guard<std::mutex> guard(restart_history_mtx_);
    prune_restart_history_locked();
    return restart_history_ms_.size() >=
           static_cast<size_t>(MAX_RESTARTS_PER_HOUR);
  }

  void record_restart() {
    std::lock_guard<std::mutex> guard(restart_history_mtx_);
    prune_restart_history_locked();
    restart_history_ms_.push_back(
        static_cast<std::uint64_t>(time::getNowMs()));
  }

  void prune_restart_history_locked() {
    auto cutoff = static_cast<std::uint64_t>(time::getNowMs()) - ONE_HOUR_MS;
    while (!restart_history_ms_.empty() &&
           restart_history_ms_.front() < cutoff) {
      restart_history_ms_.pop_front();
    }
  }

  std::shared_ptr<boost::synchronized_value<FrameSetT>> frame_set_;
  RecoveryCheckFn recovery_check_;
  RestartFn on_stale_;
  viam::sdk::LogSource logger_;

  std::atomic<bool> running_{true};
  std::atomic<bool> paused_{false};
  std::atomic<bool> restart_enabled_{true};

  std::mutex restart_history_mtx_;
  std::deque<std::uint64_t> restart_history_ms_;

  std::thread thread_;
};

}  // namespace watchdog
}  // namespace realsense
