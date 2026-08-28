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
  void set_throw(bool t) { throw_.store(t); }
  int restart_calls() const { return restart_calls_.load(); }

  void set_device_absent(bool a) { device_absent_.store(a); }
  void set_reattach_result(bool ok) { reattach_result_.store(ok); }
  void set_reattach_throw(bool t) { reattach_throw_.store(t); }
  int reattach_calls() const { return reattach_calls_.load(); }

  StaleFrameWatchdog<FakeFrameSet>::FramesetGetter fs_getter() {
    return [this]() -> FakeFrameSet {
      if (throw_.load())
        throw std::runtime_error("get_fs boom");
      return synth();
    };
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
  StaleFrameWatchdog<FakeFrameSet>::DeviceAbsentFn device_absent() {
    return [this]() { return device_absent_.load(); };
  }
  // Mirrors production: a successful reattach makes the device present again.
  StaleFrameWatchdog<FakeFrameSet>::ReattachFn on_absent() {
    return [this]() {
      reattach_calls_.fetch_add(1);
      if (reattach_throw_.load())
        throw std::runtime_error("reattach boom");
      const bool ok = reattach_result_.load();
      if (ok)
        device_absent_.store(false);
      return ok;
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
  std::atomic<bool> throw_{false};
  std::atomic<int> restart_calls_{0};
  std::atomic<bool> device_absent_{false};
  std::atomic<bool> reattach_result_{true};
  std::atomic<bool> reattach_throw_{false};
  std::atomic<int> reattach_calls_{0};
};

// Fast tunables so each test runs in well under a second.
StaleFrameWatchdog<FakeFrameSet>::Tunables fast_tunables() {
  StaleFrameWatchdog<FakeFrameSet>::Tunables t;
  t.poll_interval_ms = 20;
  t.stale_threshold_ms = 100;
  t.consecutive_polls_required = 3;
  t.post_restart_grace_ms = 40;
  t.max_restarts_per_hour = 3;
  t.reattach_interval_ms = 20;
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

// Successful restarts are rate-limited to max_restarts_per_hour.
TEST(WatchdogTest, SuccessfulRestartsAreRateLimited) {
  Harness h;
  h.set_mode(FrameMode::Stale);
  h.set_restart_result(true); // each restart "succeeds" and counts
  auto t = fast_tunables();   // max_restarts_per_hour = 3
  StaleFrameWatchdog<FakeFrameSet> wd(h.fs_getter(), h.recovery_check(),
                                      h.on_stale(), make_logger(), t);
  // Wait until it reaches the cap (timing-robust on slow CI runners), then
  // confirm it never exceeds it — the rate-limit invariant under test. Use a
  // generous timeout since reaching the cap takes several detect+grace cycles.
  ASSERT_TRUE(
      wait_until([&] { return h.restart_calls() >= t.max_restarts_per_hour; },
                 std::chrono::milliseconds(10000)));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
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

// Destruction must not block for a full poll interval — the interruptible wait
// should wake immediately when running_ is cleared.
TEST(WatchdogTest, DestructorPreemptsLongSleep) {
  Harness h;
  h.set_mode(FrameMode::Fresh);
  auto t = fast_tunables();
  t.poll_interval_ms = 5000; // long; dtor must not wait this out
  const auto start = std::chrono::steady_clock::now();
  {
    StaleFrameWatchdog<FakeFrameSet> wd(h.fs_getter(), h.recovery_check(),
                                        h.on_stale(), make_logger(), t);
    // Let the loop enter its wait before we tear down.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  } // ~StaleFrameWatchdog here: should return promptly, not after 5s.
  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now() - start)
                              .count();
  EXPECT_LT(elapsed_ms, 1000)
      << "destructor blocked on the poll interval (" << elapsed_ms << "ms)";
}

// An exception from get_fs_ must not kill the watchdog thread (an escaped
// exception would std::terminate the whole module). The thread should keep
// polling and resume detecting once the getter stops throwing.
TEST(WatchdogTest, SurvivesGetFsException) {
  Harness h;
  h.set_throw(true); // every poll's get_fs_ throws
  StaleFrameWatchdog<FakeFrameSet> wd(h.fs_getter(), h.recovery_check(),
                                      h.on_stale(), make_logger(),
                                      fast_tunables());
  // Poll through several throwing iterations — must not crash or exit.
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  EXPECT_EQ(h.restart_calls(), 0);

  // Recover: getter now returns stale frames; a live thread must still detect.
  h.set_throw(false);
  h.set_mode(FrameMode::Stale);
  EXPECT_TRUE(wait_until([&] { return h.restart_calls() >= 1; }, kTimeout))
      << "watchdog thread should survive get_fs exceptions and resume "
         "detecting";
}

// --- device-absent / reattach ------------------------------------------------

// The regression this path exists for: the camera drops off the bus after
// construction, so there is no device to restart. The watchdog must
// re-enumerate instead of giving up until the host reboots.
TEST(WatchdogTest, ReattachesWhenDeviceAbsent) {
  Harness h;
  h.set_mode(FrameMode::Stale);
  h.set_device_absent(true);
  StaleFrameWatchdog<FakeFrameSet> wd(
      h.fs_getter(), h.recovery_check(), h.on_stale(), make_logger(),
      fast_tunables(), h.device_absent(), h.on_absent());
  EXPECT_TRUE(wait_until([&] { return h.reattach_calls() >= 1; }, kTimeout));
}

// A restart cannot fix a missing device, so it must not be attempted — the
// reattach path owns this case exclusively.
TEST(WatchdogTest, DoesNotRestartWhileDeviceAbsent) {
  Harness h;
  h.set_mode(FrameMode::Stale);
  h.set_device_absent(true);
  h.set_reattach_result(false); // stays absent, so the state persists
  StaleFrameWatchdog<FakeFrameSet> wd(
      h.fs_getter(), h.recovery_check(), h.on_stale(), make_logger(),
      fast_tunables(), h.device_absent(), h.on_absent());
  ASSERT_TRUE(wait_until([&] { return h.reattach_calls() >= 3; }, kTimeout));
  EXPECT_EQ(h.restart_calls(), 0);
}

// A device that dropped off before ever streaming has no frame to age, so the
// stale path would never fire. Absence must be detected on its own.
TEST(WatchdogTest, ReattachesWithNoFramesEverDelivered) {
  Harness h;
  h.set_mode(FrameMode::None);
  h.set_device_absent(true);
  StaleFrameWatchdog<FakeFrameSet> wd(
      h.fs_getter(), h.recovery_check(), h.on_stale(), make_logger(),
      fast_tunables(), h.device_absent(), h.on_absent());
  EXPECT_TRUE(wait_until([&] { return h.reattach_calls() >= 1; }, kTimeout));
}

// Once reattached, the watchdog stops re-enumerating and goes back to
// ordinary stale-frame duty.
TEST(WatchdogTest, StopsReattachingOnceDevicePresent) {
  Harness h;
  h.set_mode(FrameMode::Fresh);
  h.set_device_absent(true);
  StaleFrameWatchdog<FakeFrameSet> wd(
      h.fs_getter(), h.recovery_check(), h.on_stale(), make_logger(),
      fast_tunables(), h.device_absent(), h.on_absent());
  ASSERT_TRUE(wait_until([&] { return h.reattach_calls() >= 1; }, kTimeout));

  const int after_success = h.reattach_calls();
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(h.reattach_calls(), after_success);
}

// A device in DFU mode also leaves device_ null; reattaching mid firmware
// update would fight the update. Recovery mode wins.
TEST(WatchdogTest, SkipsReattachDuringRecoveryMode) {
  Harness h;
  h.set_mode(FrameMode::Stale);
  h.set_device_absent(true);
  h.set_recovery(true);
  StaleFrameWatchdog<FakeFrameSet> wd(
      h.fs_getter(), h.recovery_check(), h.on_stale(), make_logger(),
      fast_tunables(), h.device_absent(), h.on_absent());
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(h.reattach_calls(), 0);
  EXPECT_EQ(h.restart_calls(), 0);
}

// Reattach runs on its own interval, not on every poll — each attempt is a
// full bus enumeration.
TEST(WatchdogTest, ReattachIsThrottledToItsInterval) {
  Harness h;
  h.set_mode(FrameMode::Stale);
  h.set_device_absent(true);
  h.set_reattach_result(false);
  auto t = fast_tunables();
  t.poll_interval_ms = 10;
  t.reattach_interval_ms = 5000; // one attempt, then quiet
  StaleFrameWatchdog<FakeFrameSet> wd(h.fs_getter(), h.recovery_check(),
                                      h.on_stale(), make_logger(), t,
                                      h.device_absent(), h.on_absent());
  ASSERT_TRUE(wait_until([&] { return h.reattach_calls() >= 1; }, kTimeout));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(h.reattach_calls(), 1);
}

// An exception out of the reattach callback must not kill the thread — it runs
// rs2 enumeration, which throws on transient libusb errors.
TEST(WatchdogTest, SurvivesReattachException) {
  Harness h;
  h.set_mode(FrameMode::Stale);
  h.set_device_absent(true);
  h.set_reattach_throw(true);
  StaleFrameWatchdog<FakeFrameSet> wd(
      h.fs_getter(), h.recovery_check(), h.on_stale(), make_logger(),
      fast_tunables(), h.device_absent(), h.on_absent());
  ASSERT_TRUE(wait_until([&] { return h.reattach_calls() >= 2; }, kTimeout));

  // Thread is alive: stop throwing, and it must reattach for real and then
  // resume ordinary stale-frame duty on the now-present device.
  h.set_reattach_throw(false);
  EXPECT_TRUE(wait_until([&] { return h.restart_calls() >= 1; }, kTimeout));
}

// Omitting both callbacks keeps the pre-existing stale-frame-only behavior.
TEST(WatchdogTest, WorksWithoutReattachCallbacks) {
  Harness h;
  h.set_mode(FrameMode::Stale);
  StaleFrameWatchdog<FakeFrameSet> wd(h.fs_getter(), h.recovery_check(),
                                      h.on_stale(), make_logger(),
                                      fast_tunables());
  EXPECT_TRUE(wait_until([&] { return h.restart_calls() >= 1; }, kTimeout));
  EXPECT_EQ(h.reattach_calls(), 0);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
