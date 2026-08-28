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
// Background thread that watches for the two ways a camera stops delivering
// frames, each with its own remedy:
//
//   * the pipeline wedges while the device is still attached — invokes
//     RestartFn to rebuild the rs2::pipeline once a frame stays stale for
//     consecutive_polls_required polls. Rate-limited (see Tunables).
//   * the device drops off the bus entirely — DeviceAbsentFn reports it and
//     ReattachFn re-enumerates, retried every reattach_interval_ms until the
//     camera comes back. A restart cannot help here: there is nothing to
//     restart, and without this the resource stays alive-but-dead until the
//     host reboots.
//
// Owned by Realsense: construct after the pipeline starts, destroy before it
// tears down. pause()/resume() (thread-safe) let the owner suspend stale
// checking across a deliberate pipeline transition. RestartFn and ReattachFn
// own whatever locking serializes pipeline and device-lifecycle mutations.
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
  // Returns true when there is no device to restart, i.e. the camera was
  // unplugged (or dropped off USB) after the resource was constructed.
  using DeviceAbsentFn = std::function<bool()>;
  // Re-enumerates the bus and re-initializes the configured camera. Returns
  // true once a device is attached again.
  using ReattachFn = std::function<bool()>;

  // Defaults are production values; tests inject small intervals.
  struct Tunables {
    std::uint64_t poll_interval_ms = 1000;        // check cadence
    std::uint64_t stale_threshold_ms = 10'000;    // age before a frame is stale
    int consecutive_polls_required = 3;           // debounce
    std::uint64_t post_restart_grace_ms = 10'000; // pipeline warm-up
    int max_restarts_per_hour = 6;                // restart-storm cap
    std::uint64_t reattach_interval_ms = 5'000;   // bus re-enumeration cadence
  };

  // device_absent/on_absent are optional: omit both to get stale-frame
  // detection only.
  StaleFrameWatchdog(FramesetGetter get_fs, RecoveryCheckFn recovery_check,
                     RestartFn on_stale, viam::sdk::LogSource logger,
                     Tunables tunables = {}, DeviceAbsentFn device_absent = {},
                     ReattachFn on_absent = {})
      : get_fs_(std::move(get_fs)), recovery_check_(std::move(recovery_check)),
        on_stale_(std::move(on_stale)),
        device_absent_(std::move(device_absent)),
        on_absent_(std::move(on_absent)), logger_(std::move(logger)),
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

  // Suspend/resume stale checking around a deliberate pipeline transition
  // driven by the owner. Idempotent. (Firmware update and device change are
  // already covered by the recovery-mode skip and RestartFn's own locking, so
  // there is currently no in-tree caller; kept as owner-facing API.)
  void pause() noexcept { paused_.store(true); }
  void resume() noexcept { paused_.store(false); }

private:
  static constexpr std::uint64_t ONE_HOUR_MS = 60ULL * 60ULL * 1000ULL;
  // Throttle the "rate-limited / operator intervention" error so a persistent
  // wedge signals once a minute instead of every poll.
  static constexpr std::uint64_t RATE_LIMITED_LOG_INTERVAL_MS = 60'000;
  // Same idea for the "still no device" warning during a long outage.
  static constexpr std::uint64_t REATTACH_LOG_INTERVAL_MS = 60'000;

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
      // Guard the poll: get_fs_/frame accessors are rs2 calls that can throw,
      // and an escaped exception on this thread would std::terminate the whole
      // module. A bad poll logs and is treated as not-stale.
      try {
        poll_once(stale_count);
      } catch (const std::exception &e) {
        VIAM_SDK_LOG_IMPL(logger_, error)
            << "[watchdog] poll error: " << e.what();
        stale_count = 0;
      } catch (...) {
        VIAM_SDK_LOG_IMPL(logger_, error) << "[watchdog] poll error (unknown)";
        stale_count = 0;
      }
    }

    VIAM_SDK_LOG_IMPL(logger_, info) << "[watchdog] stopped";
  }

  // One poll iteration. Early-returns (like `continue`) on any not-stale or
  // handled condition. May throw from get_fs_/frame accessors; loop() guards.
  void poll_once(int &stale_count) {
    if (paused_.load()) {
      stale_count = 0;
      return;
    }
    if (recovery_check_ && recovery_check_()) {
      stale_count = 0;
      return;
    }

    // Checked before frame age for two reasons: a pipeline restart cannot fix
    // a missing device, and a device that dropped off the bus before it ever
    // streamed has no frame to age — the stale path would never fire.
    // Ordered after recovery_check_ so a device in DFU mode (which also has no
    // streaming device) is never dragged out of the firmware-update flow.
    if (device_absent_ && device_absent_()) {
      stale_count = 0;
      maybe_reattach();
      return;
    }

    double age_ms = 0.0;
    if (!compute_max_age_ms(age_ms)) {
      // No frame available yet; treat as not-stale.
      stale_count = 0;
      return;
    }

    if (age_ms <= static_cast<double>(tunables_.stale_threshold_ms)) {
      stale_count = 0;
      return;
    }

    stale_count += 1;
    if (stale_count < tunables_.consecutive_polls_required) {
      return;
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
            << "[watchdog] rate-limited at " << tunables_.max_restarts_per_hour
            << " restarts/hour — operator intervention needed";
      }
      stale_count = 0;
      return;
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

  // Try to re-acquire a device, at most once per reattach_interval_ms.
  //
  // Deliberately not subject to the restart rate limit: an unplugged camera
  // may reappear at any time, so this has to keep trying for as long as the
  // resource lives. The interval (not the poll interval) bounds the cost,
  // since each attempt is a full bus enumeration.
  void maybe_reattach() {
    if (!on_absent_) {
      return;
    }
    auto now = static_cast<std::uint64_t>(time::getNowMs());
    if (now - last_reattach_attempt_ms_ < tunables_.reattach_interval_ms) {
      return;
    }
    last_reattach_attempt_ms_ = now;

    bool ok = false;
    try {
      ok = on_absent_();
    } catch (const std::exception &e) {
      VIAM_SDK_LOG_IMPL(logger_, error)
          << "[watchdog] reattach callback threw: " << e.what();
    } catch (...) {
      VIAM_SDK_LOG_IMPL(logger_, error)
          << "[watchdog] reattach callback threw unknown exception";
    }

    if (ok) {
      VIAM_SDK_LOG_IMPL(logger_, info)
          << "[watchdog] device reattached after " << reattach_attempts_ + 1
          << " attempt(s)";
      reattach_attempts_ = 0;
      last_reattach_log_ms_ = 0;
      return;
    }

    // Signal the first failure immediately (so the cause of an outage is
    // visible at once), then at most once a minute — a camera that stays
    // unplugged must not fill the log at the reattach cadence.
    if (reattach_attempts_ == 0 ||
        now - last_reattach_log_ms_ > REATTACH_LOG_INTERVAL_MS) {
      last_reattach_log_ms_ = now;
      VIAM_SDK_LOG_IMPL(logger_, warn)
          << "[watchdog] no device to stream from; reattach attempt "
          << reattach_attempts_ + 1
          << " found no matching camera — check the USB connection";
    }
    reattach_attempts_ += 1;
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

  // rate_limited/record_restart/prune_restart_history are called only from the
  // loop thread (via poll_once), so restart_history_ms_ needs no lock — same as
  // last_rate_limited_log_ms_ below.
  bool rate_limited() {
    prune_restart_history();
    return restart_history_ms_.size() >=
           static_cast<size_t>(tunables_.max_restarts_per_hour);
  }

  void record_restart() {
    prune_restart_history();
    restart_history_ms_.push_back(static_cast<std::uint64_t>(time::getNowMs()));
  }

  void prune_restart_history() {
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
  DeviceAbsentFn device_absent_;
  ReattachFn on_absent_;
  viam::sdk::LogSource logger_;
  Tunables tunables_;

  std::atomic<bool> running_{true};
  std::atomic<bool> paused_{false};

  // Guards the interruptible wait so shutdown can wake the loop immediately.
  std::mutex cv_mutex_;
  std::condition_variable cv_;

  // Loop-thread only (touched solely from poll_once); no lock needed.
  std::deque<std::uint64_t> restart_history_ms_;

  // Loop-thread only; last time the rate-limited error was logged.
  std::uint64_t last_rate_limited_log_ms_{0};

  // Loop-thread only (maybe_reattach); no lock needed. last_reattach_attempt_
  // starts at 0 so the first absent poll attempts immediately.
  std::uint64_t last_reattach_attempt_ms_{0};
  std::uint64_t last_reattach_log_ms_{0};
  int reattach_attempts_{0};

  std::thread thread_;
};

} // namespace watchdog
} // namespace realsense
