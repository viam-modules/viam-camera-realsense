#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "time.hpp"
#include "watchdog.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include <viam/sdk/log/logging.hpp>

using realsense::watchdog::StaleFrameWatchdog;

namespace {

// Minimal stand-in for rs2::frame: convertible to bool (valid?) and exposes
// get_timestamp(). The watchdog only ever touches these two operations.
struct FakeFrame {
  bool valid = false;
  double ts = 0.0;
  explicit operator bool() const { return valid; }
  double get_timestamp() const { return ts; }
};

// Minimal stand-in for rs2::frameset.
struct FakeFrameSet {
  FakeFrame color;
  FakeFrame depth;
  FakeFrame get_color_frame() const { return color; }
  FakeFrame get_depth_frame() const { return depth; }
};

// What the get_fs() callback should synthesize on each poll. Timestamps are
// computed relative to getNowMs() at call time, so "fresh" stays fresh and
// "stale" stays stale regardless of how long the test runs.
enum class FrameMode {
  None,       // no frames available yet
  Fresh,      // color + depth both current
  Stale,      // color + depth both far in the past
  ColorStale, // depth current, color far in the past (partial stall)
};

// Shared, thread-safe harness driving one watchdog instance. The watchdog
// thread reads it through the injected callbacks; the test thread mutates it.
class Harness {
public:
  void set_mode(FrameMode m) { mode_.store(m); }
  void set_recovery(bool r) { recovery_.store(r); }
  void set_restart_result(bool ok) { restart_result_.store(ok); }
  int restart_calls() const { return restart_calls_.load(); }

  StaleFrameWatchdog<FakeFrameSet>::FramesetGetter fs_getter() {
    return [this]() { return synth(); };
  }
  StaleFrameWatchdog<FakeFrameSet>::RecoveryCheckFn recovery_check() {
    return [this]() { return recovery_.load(); };
  }
  StaleFrameWatchdog<FakeFrameSet>::RestartFn on_stale() {
    return [this]() {
      restart_calls_.fetch_add(1);
      return restart_result_.load();
    };
  }

private:
  FakeFrameSet synth() const {
    // 5x the test stale threshold (100ms) — comfortably "stale".
    const double stale_age = 500.0;
    const double now = realsense::time::getNowMs();
    FakeFrameSet fs;
    switch (mode_.load()) {
    case FrameMode::None:
      break; // both frames invalid
    case FrameMode::Fresh:
      fs.color = {true, now};
      fs.depth = {true, now};
      break;
    case FrameMode::Stale:
      fs.color = {true, now - stale_age};
      fs.depth = {true, now - stale_age};
      break;
    case FrameMode::ColorStale:
      fs.color = {true, now - stale_age};
      fs.depth = {true, now};
      break;
    }
    return fs;
  }

  std::atomic<FrameMode> mode_{FrameMode::None};
  std::atomic<bool> recovery_{false};
  std::atomic<bool> restart_result_{true};
  std::atomic<int> restart_calls_{0};
};

// Fast tunables so each test runs in well under a second.
StaleFrameWatchdog<FakeFrameSet>::Tunables fast_tunables() {
  StaleFrameWatchdog<FakeFrameSet>::Tunables t;
  t.poll_interval_ms = 20;
  t.stale_threshold_ms = 100;
  t.consecutive_polls_required = 3;
  t.post_restart_grace_ms = 40;
  t.max_restarts_per_hour = 3;
  return t;
}

viam::sdk::LogSource make_logger() { return viam::sdk::LogSource{}; }

// Spin-wait until pred() is true or timeout elapses. Returns pred()'s final
// value so callers can assert on it directly (avoids fixed-sleep flakiness).
template <typename Pred>
bool wait_until(Pred pred, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred())
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return pred();
}

constexpr std::chrono::milliseconds kTimeout{2000};

} // namespace

// Sustained staleness triggers a restart.
TEST(WatchdogTest, FiresAfterSustainedStaleness) {
  Harness h;
  h.set_mode(FrameMode::Stale);
  StaleFrameWatchdog<FakeFrameSet> wd(h.fs_getter(), h.recovery_check(),
                                      h.on_stale(), make_logger(),
                                      fast_tunables());
  EXPECT_TRUE(wait_until([&] { return h.restart_calls() >= 1; }, kTimeout));
}

// A partial stall (color frozen, depth live) is still caught: the watchdog
// uses the max age across streams.
TEST(WatchdogTest, FiresOnPartialStall) {
  Harness h;
  h.set_mode(FrameMode::ColorStale);
  StaleFrameWatchdog<FakeFrameSet> wd(h.fs_getter(), h.recovery_check(),
                                      h.on_stale(), make_logger(),
                                      fast_tunables());
  EXPECT_TRUE(wait_until([&] { return h.restart_calls() >= 1; }, kTimeout));
}

// Fresh frames never trigger a restart.
TEST(WatchdogTest, DoesNotFireWhenFresh) {
  Harness h;
  h.set_mode(FrameMode::Fresh);
  StaleFrameWatchdog<FakeFrameSet> wd(h.fs_getter(), h.recovery_check(),
                                      h.on_stale(), make_logger(),
                                      fast_tunables());
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(h.restart_calls(), 0);
}

// No frame available yet is treated as not-stale.
TEST(WatchdogTest, DoesNotFireWhenNoFrame) {
  Harness h;
  h.set_mode(FrameMode::None);
  StaleFrameWatchdog<FakeFrameSet> wd(h.fs_getter(), h.recovery_check(),
                                      h.on_stale(), make_logger(),
                                      fast_tunables());
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(h.restart_calls(), 0);
}

// While the device reports recovery mode, the watchdog skips its check even
// if frames are stale.
TEST(WatchdogTest, SkipsDuringRecoveryMode) {
  Harness h;
  h.set_mode(FrameMode::Stale);
  h.set_recovery(true);
  StaleFrameWatchdog<FakeFrameSet> wd(h.fs_getter(), h.recovery_check(),
                                      h.on_stale(), make_logger(),
                                      fast_tunables());
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(h.restart_calls(), 0);
}

// pause() suspends checking (and resets the stale counter); resume() re-arms.
TEST(WatchdogTest, PauseSuppressesResumeReArms) {
  Harness h;
  h.set_mode(FrameMode::Stale);
  StaleFrameWatchdog<FakeFrameSet> wd(h.fs_getter(), h.recovery_check(),
                                      h.on_stale(), make_logger(),
                                      fast_tunables());
  wd.pause();
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(h.restart_calls(), 0) << "paused watchdog must not restart";

  wd.resume();
  EXPECT_TRUE(wait_until([&] { return h.restart_calls() >= 1; }, kTimeout))
      << "resumed watchdog must detect the still-stale stream";
}

// With restart disabled, sustained staleness is detected/logged but on_stale
// is never invoked.
TEST(WatchdogTest, RestartDisabledDetectsButDoesNotAct) {
  Harness h;
  h.set_mode(FrameMode::Stale);
  StaleFrameWatchdog<FakeFrameSet> wd(h.fs_getter(), h.recovery_check(),
                                      h.on_stale(), make_logger(),
                                      fast_tunables());
  wd.set_restart_enabled(false);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(h.restart_calls(), 0);
}

// Successful restarts are rate-limited to max_restarts_per_hour.
TEST(WatchdogTest, SuccessfulRestartsAreRateLimited) {
  Harness h;
  h.set_mode(FrameMode::Stale);
  h.set_restart_result(true); // each restart "succeeds" and counts
  auto t = fast_tunables();   // max_restarts_per_hour = 3
  StaleFrameWatchdog<FakeFrameSet> wd(h.fs_getter(), h.recovery_check(),
                                      h.on_stale(), make_logger(), t);
  // Give it long enough to blow past the cap if it weren't limited.
  std::this_thread::sleep_for(std::chrono::milliseconds(900));
  EXPECT_EQ(h.restart_calls(), t.max_restarts_per_hour);
}

// Failed restarts do NOT consume the rate-limit budget, so the watchdog keeps
// retrying past max_restarts_per_hour.
TEST(WatchdogTest, FailedRestartsAreNotRateLimited) {
  Harness h;
  h.set_mode(FrameMode::Stale);
  h.set_restart_result(false); // restart reports failure -> not recorded
  auto t = fast_tunables();    // max_restarts_per_hour = 3
  StaleFrameWatchdog<FakeFrameSet> wd(h.fs_getter(), h.recovery_check(),
                                      h.on_stale(), make_logger(), t);
  EXPECT_TRUE(wait_until(
      [&] { return h.restart_calls() > t.max_restarts_per_hour; }, kTimeout));
}

// Frames that recover after a stall stop the restart loop.
TEST(WatchdogTest, StopsRestartingAfterRecovery) {
  Harness h;
  h.set_mode(FrameMode::Stale);
  StaleFrameWatchdog<FakeFrameSet> wd(h.fs_getter(), h.recovery_check(),
                                      h.on_stale(), make_logger(),
                                      fast_tunables());
  ASSERT_TRUE(wait_until([&] { return h.restart_calls() >= 1; }, kTimeout));

  h.set_mode(FrameMode::Fresh);
  // Let any in-flight grace period elapse, then snapshot and confirm no
  // further restarts occur.
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  const int after_recovery = h.restart_calls();
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(h.restart_calls(), after_recovery);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
