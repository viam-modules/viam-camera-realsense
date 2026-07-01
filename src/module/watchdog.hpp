#pragma once

#include "time.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include <viam/sdk/log/logging.hpp>

namespace realsense {
namespace watchdog {

// Forward declare so the template signature reads cleanly.
template <typename FrameSetT> class StaleFrameWatchdog;

// StaleFrameWatchdog
//
// Background thread that polls the cached frameset (via FramesetGetter) and
// invokes RestartFn to rebuild the rs2::pipeline once a frame stays stale for
// consecutive_polls_required polls. Rate-limited (see Tunables).
//
// Owned by Realsense: construct after the pipeline starts, destroy before it
// tears down. pause()/resume() (thread-safe) bracket operator-driven
// transitions (reconfigure, firmware update, device change). RestartFn owns
// whatever locking serializes pipeline mutations.
template <typename FrameSetT> class StaleFrameWatchdog {
public:
  // Returns true if the restart was attempted and (best-effort) succeeded;
  // false if the watchdog should not count it against the rate limit.
  using RestartFn = std::function<bool()>;
  // Returns true if the device is in recovery mode (e.g. firmware update);
  // the watchdog skips its check while this returns true.
  using RecoveryCheckFn = std::function<bool()>;
  // Returns the current frameset. Called each poll so it always sees the
  // freshest cached value, regardless of how Realsense stores it internally.
  using FramesetGetter = std::function<FrameSetT()>;

  // Defaults are production values; tests inject small intervals.
  struct Tunables {
    std::uint64_t poll_interval_ms = 1000;        // check cadence
    std::uint64_t stale_threshold_ms = 10'000;    // age before a frame is stale
    int consecutive_polls_required = 3;           // debounce
    std::uint64_t post_restart_grace_ms = 10'000; // pipeline warm-up
    int max_restarts_per_hour = 6;                // restart-storm cap
  };

  StaleFrameWatchdog(FramesetGetter get_fs, RecoveryCheckFn recovery_check,
                     RestartFn on_stale, viam::sdk::LogSource logger,
                     Tunables tunables = {})
      : get_fs_(std::move(get_fs)), recovery_check_(std::move(recovery_check)),
        on_stale_(std::move(on_stale)), logger_(std::move(logger)),
        tunables_(tunables) {
    thread_ = std::thread([this]() { loop(); });
  }

  ~StaleFrameWatchdog() {
    { // Set the flag under the cv mutex before notifying so a thread about to
      // wait can't miss the wakeup (lost-wakeup race).
      std::lock_guard<std::mutex> lk(cv_mutex_);
      running_.store(false);
    }
    cv_.notify_all();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  StaleFrameWatchdog(const StaleFrameWatchdog &) = delete;
  StaleFrameWatchdog &operator=(const StaleFrameWatchdog &) = delete;
  StaleFrameWatchdog(StaleFrameWatchdog &&) = delete;
  StaleFrameWatchdog &operator=(StaleFrameWatchdog &&) = delete;

  // Pause/resume during operator-initiated pipeline transitions
  // (reconfigure, firmware update, USB device change). Idempotent.
  void pause() noexcept { paused_.store(true); }
  void resume() noexcept { paused_.store(false); }

private:
  static constexpr std::uint64_t ONE_HOUR_MS = 60ULL * 60ULL * 1000ULL;
  // Throttle the "rate-limited / operator intervention" error so a persistent
  // wedge signals once a minute instead of every poll.
  static constexpr std::uint64_t RATE_LIMITED_LOG_INTERVAL_MS = 60'000;

  // Sleep for up to ms, but return immediately if running_ is cleared (e.g.
  // during ~StaleFrameWatchdog). Keeps shutdown/teardown from blocking on a
  // full poll interval or post-restart grace period.
  void interruptible_wait(std::uint64_t ms) {
    std::unique_lock<std::mutex> lk(cv_mutex_);
    cv_.wait_for(lk, std::chrono::milliseconds(ms),
                 [this] { return !running_.load(); });
  }

  void loop() {
    VIAM_SDK_LOG_IMPL(logger_, info) << "[watchdog] started";
    int stale_count = 0;

    while (running_.load()) {
      interruptible_wait(tunables_.poll_interval_ms);
      if (!running_.load())
        break;

      if (paused_.load()) {
        stale_count = 0;
        continue;
      }
      if (recovery_check_ && recovery_check_()) {
        stale_count = 0;
        continue;
      }

      double age_ms = 0.0;
      if (!compute_max_age_ms(age_ms)) {
        // No frame available yet; treat as not-stale.
        stale_count = 0;
        continue;
      }

      if (age_ms <= static_cast<double>(tunables_.stale_threshold_ms)) {
        stale_count = 0;
        continue;
      }

      stale_count += 1;
      if (stale_count < tunables_.consecutive_polls_required) {
        continue;
      }

      // Sustained staleness confirmed.
      VIAM_SDK_LOG_IMPL(logger_, warn)
          << "[watchdog] sustained stale frame: age=" << age_ms
          << "ms threshold=" << tunables_.stale_threshold_ms << "ms";

      if (rate_limited()) {
        auto now = static_cast<std::uint64_t>(time::getNowMs());
        if (now - last_rate_limited_log_ms_ > RATE_LIMITED_LOG_INTERVAL_MS) {
          last_rate_limited_log_ms_ = now;
          VIAM_SDK_LOG_IMPL(logger_, error)
              << "[watchdog] rate-limited at "
              << tunables_.max_restarts_per_hour
              << " restarts/hour — operator intervention needed";
        }
        stale_count = 0;
        continue;
      }

      bool ok = false;
      try {
        ok = on_stale_ ? on_stale_() : false;
      } catch (const std::exception &e) {
        VIAM_SDK_LOG_IMPL(logger_, error)
            << "[watchdog] restart callback threw: " << e.what();
      } catch (...) {
        VIAM_SDK_LOG_IMPL(logger_, error)
            << "[watchdog] restart callback threw unknown exception";
      }

      if (ok) {
        record_restart();
        interruptible_wait(tunables_.post_restart_grace_ms);
      }
      stale_count = 0;
    }

    VIAM_SDK_LOG_IMPL(logger_, info) << "[watchdog] stopped";
  }

  // Returns true and writes the max age (in ms) of color/depth into
  // out_age_ms; returns false if no frame is currently available.
  bool compute_max_age_ms(double &out_age_ms) const {
    auto fs = get_fs_();
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
    if (!any)
      return false;
    out_age_ms = std::max(color_age, depth_age);
    return true;
  }

  bool rate_limited() {
    std::lock_guard<std::mutex> guard(restart_history_mtx_);
    prune_restart_history_locked();
    return restart_history_ms_.size() >=
           static_cast<size_t>(tunables_.max_restarts_per_hour);
  }

  void record_restart() {
    std::lock_guard<std::mutex> guard(restart_history_mtx_);
    prune_restart_history_locked();
    restart_history_ms_.push_back(static_cast<std::uint64_t>(time::getNowMs()));
  }

  void prune_restart_history_locked() {
    // Additive comparison (now - front) avoids unsigned underflow if getNowMs()
    // is ever smaller than ONE_HOUR_MS; front() <= now so this stays >= 0.
    auto now = static_cast<std::uint64_t>(time::getNowMs());
    while (!restart_history_ms_.empty() &&
           now - restart_history_ms_.front() > ONE_HOUR_MS) {
      restart_history_ms_.pop_front();
    }
  }

  FramesetGetter get_fs_;
  RecoveryCheckFn recovery_check_;
  RestartFn on_stale_;
  viam::sdk::LogSource logger_;
  Tunables tunables_;

  std::atomic<bool> running_{true};
  std::atomic<bool> paused_{false};

  // Guards the interruptible wait so shutdown can wake the loop immediately.
  std::mutex cv_mutex_;
  std::condition_variable cv_;

  std::mutex restart_history_mtx_;
  std::deque<std::uint64_t> restart_history_ms_;

  // Loop-thread only; last time the rate-limited error was logged.
  std::uint64_t last_rate_limited_log_ms_{0};

  std::thread thread_;
};

} // namespace watchdog
} // namespace realsense
