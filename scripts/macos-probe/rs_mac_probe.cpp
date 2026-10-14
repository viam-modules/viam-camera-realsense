// rs_mac_probe: reproduce and time the librealsense init path the module uses,
// step by step, many times, without viam-server. APP-16649.
//
// Each cycle mirrors device_impl.hpp's pre-start sequence:
//   query_devices -> query_sensors -> [set_option(AE priority) on color]
//   -> color.get_stream_profiles -> depth.get_stream_profiles
//   -> pipe.start(config) -> [set_option post-start] -> wait for frames -> stop
// and records the duration and any exception of each step. On macOS every
// sensor power-down makes libusb re-enumerate the device, so the durations
// are the tell: a set_option that should take a few ms taking hundreds of ms
// or seconds means a re-enumeration happened.
//
// Must run as root on macOS (libusb needs to capture the device).
#include <librealsense2/rs.hpp>
#include <libusb.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

namespace {

using clock_t_ = std::chrono::steady_clock;

struct Args {
  int cycles = 10;
  int width = 0;   // 0 = leave resolution to librealsense defaults
  int height = 0;
  int fps = 0;
  bool pre_option = true;   // module's current behavior
  bool post_option = false; // proposed behavior
  bool depth_options = false; // also set a depth XU option pre-start
  int frames = 5;
  int frame_timeout_ms = 5000;
  int sleep_ms = 500;
  bool fresh_context = false;
  bool stream = true;
  bool libusb_debug = false;
  bool rs_debug = false;
  bool rs_log_only = false;
  bool hold_capture = false;
  std::string serial;
};

std::optional<std::string> value_of(int argc, char **argv, const char *flag) {
  for (int i = 1; i < argc; ++i) {
    std::string a(argv[i]);
    std::string f(flag);
    if (a == f && i + 1 < argc) return std::string(argv[i + 1]);
    if (a.rfind(f + "=", 0) == 0) return a.substr(f.size() + 1);
  }
  return std::nullopt;
}
bool has(int argc, char **argv, const char *flag) {
  for (int i = 1; i < argc; ++i)
    if (std::string(argv[i]) == flag) return true;
  return false;
}

Args parse(int argc, char **argv) {
  Args a;
  if (auto v = value_of(argc, argv, "--cycles")) a.cycles = std::stoi(*v);
  if (auto v = value_of(argc, argv, "--width")) a.width = std::stoi(*v);
  if (auto v = value_of(argc, argv, "--height")) a.height = std::stoi(*v);
  if (auto v = value_of(argc, argv, "--fps")) a.fps = std::stoi(*v);
  if (auto v = value_of(argc, argv, "--frames")) a.frames = std::stoi(*v);
  if (auto v = value_of(argc, argv, "--frame-timeout-ms")) a.frame_timeout_ms = std::stoi(*v);
  if (auto v = value_of(argc, argv, "--sleep-ms")) a.sleep_ms = std::stoi(*v);
  if (auto v = value_of(argc, argv, "--serial")) a.serial = *v;
  if (auto v = value_of(argc, argv, "--pre-option")) a.pre_option = (*v == "1");
  if (auto v = value_of(argc, argv, "--post-option")) a.post_option = (*v == "1");
  if (auto v = value_of(argc, argv, "--depth-options")) a.depth_options = (*v == "1");
  a.fresh_context = has(argc, argv, "--fresh-context");
  a.stream = !has(argc, argv, "--no-stream");
  a.libusb_debug = has(argc, argv, "--libusb-debug");
  a.rs_debug = has(argc, argv, "--rs-debug");
  a.rs_log_only = has(argc, argv, "--rs-log-only");
  a.hold_capture = has(argc, argv, "--hold-capture");
  return a;
}

std::string now_str() {
  auto t = std::chrono::system_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch()) % 1000;
  std::time_t tt = std::chrono::system_clock::to_time_t(t);
  char buf[32];
  std::strftime(buf, sizeof buf, "%H:%M:%S", std::localtime(&tt));
  std::ostringstream o;
  o << buf << "." << std::setw(3) << std::setfill('0') << ms.count();
  return o.str();
}

struct StepStat {
  int ok = 0, fail = 0;
  std::vector<double> ms;
  std::map<std::string, int> errors;
};

std::map<std::string, StepStat> stats;
std::vector<std::string> step_order;

// Time `fn`; record success/failure under `name`; rethrow so the cycle aborts
// the way the module's init would.
template <typename F> void step(const std::string &name, F &&fn) {
  if (std::find(step_order.begin(), step_order.end(), name) == step_order.end())
    step_order.push_back(name);
  auto &s = stats[name];
  auto t0 = clock_t_::now();
  try {
    fn();
    double ms = std::chrono::duration<double, std::milli>(clock_t_::now() - t0).count();
    s.ok++;
    s.ms.push_back(ms);
    std::cout << "  " << now_str() << "  " << std::left << std::setw(28) << name << std::right
              << std::setw(9) << std::fixed << std::setprecision(1) << ms << " ms\n";
  } catch (const rs2::error &e) {
    double ms = std::chrono::duration<double, std::milli>(clock_t_::now() - t0).count();
    s.fail++;
    std::string msg = std::string(e.get_failed_function()) + ": " + e.what();
    s.errors[msg]++;
    std::cout << "  " << now_str() << "  " << std::left << std::setw(28) << name << std::right
              << std::setw(9) << std::fixed << std::setprecision(1) << ms << " ms  FAIL: " << msg << "\n";
    throw;
  } catch (const std::exception &e) {
    double ms = std::chrono::duration<double, std::milli>(clock_t_::now() - t0).count();
    s.fail++;
    s.errors[e.what()]++;
    std::cout << "  " << now_str() << "  " << std::left << std::setw(28) << name << std::right
              << std::setw(9) << std::fixed << std::setprecision(1) << ms << " ms  FAIL: " << e.what() << "\n";
    throw;
  }
}

double pct(std::vector<double> v, double p) {
  if (v.empty()) return 0;
  std::sort(v.begin(), v.end());
  size_t i = static_cast<size_t>(p * (v.size() - 1));
  return v[i];
}

void summary(int cycles, int cycle_ok) {
  std::cout << "\n==== summary: " << cycle_ok << "/" << cycles << " cycles fully succeeded ====\n";
  std::cout << std::left << std::setw(28) << "step" << std::right << std::setw(5) << "ok" << std::setw(6)
            << "fail" << std::setw(10) << "min ms" << std::setw(10) << "p50 ms" << std::setw(10) << "max ms\n";
  for (auto const &name : step_order) {
    auto &s = stats[name];
    std::cout << std::left << std::setw(28) << name << std::right << std::setw(5) << s.ok << std::setw(6) << s.fail
              << std::setw(10) << std::fixed << std::setprecision(1) << pct(s.ms, 0.0) << std::setw(10)
              << pct(s.ms, 0.5) << std::setw(10) << pct(s.ms, 1.0) << "\n";
    for (auto const &[msg, n] : s.errors)
      std::cout << "      x" << n << "  " << msg << "\n";
  }
}

// Phase 2 hypothesis test (APP-16649). librealsense is linked with the same
// static libusb this probe uses, and libusb's darwin backend keeps the capture
// count per device in process-global state. Capture the RealSense once here
// and inflate that count, so every later release inside librealsense
// decrements it without reaching zero and therefore never re-attaches the
// kernel driver, i.e. never re-enumerates the camera. The handle is leaked on
// purpose: the capture must outlive every librealsense handle.
libusb_device_handle *hold_capture(int extra) {
  libusb_context *ctx = nullptr;
  int rc = libusb_init(&ctx);
  if (rc != 0) {
    std::cout << "hold-capture: libusb_init failed: " << libusb_error_name(rc) << "\n";
    return nullptr;
  }
  libusb_device **list = nullptr;
  ssize_t n = libusb_get_device_list(ctx, &list);
  libusb_device_handle *held = nullptr;
  for (ssize_t i = 0; i < n && !held; ++i) {
    libusb_device_descriptor d{};
    if (libusb_get_device_descriptor(list[i], &d) != 0 || d.idVendor != 0x8086) continue;
    libusb_device_handle *h = nullptr;
    rc = libusb_open(list[i], &h);
    if (rc != 0) {
      std::cout << "hold-capture: libusb_open failed: " << libusb_error_name(rc) << "\n";
      continue;
    }
    int active = libusb_kernel_driver_active(h, 0);
    int det = libusb_detach_kernel_driver(h, 0); // captures the whole device
    std::cout << "hold-capture: idProduct=0x" << std::hex << d.idProduct << std::dec
              << " kernel_driver_active(0)=" << active << " detach=" << libusb_error_name(det) << "\n";
    if (det == 0) {
      for (int k = 0; k < extra; ++k) libusb_detach_kernel_driver(h, 0);
      std::cout << "hold-capture: capture count inflated by " << extra
                << "; kernel_driver_active(0) now " << libusb_kernel_driver_active(h, 0) << "\n";
      held = h;
    } else {
      libusb_close(h);
    }
  }
  libusb_free_device_list(list, 1);
  return held;
}

std::string ctx_json() {
  // Same as production: default context (USB backend on), no DDS.
  return "{\"dds\": false}";
}

} // namespace

int main(int argc, char **argv) try {
  std::cout << std::unitbuf; // keep probe lines ordered against libusb stderr
  Args a = parse(argc, argv);
  std::cout << "librealsense " << RS2_API_VERSION_STR << ", euid=" << geteuid() << "\n";

  if (a.rs_debug || a.rs_log_only) {
    // RSDK-13059: our conan librealsense is built without easylogging;
    // this call is expected to throw on that build.
    try {
      rs2::log_to_console(RS2_LOG_SEVERITY_DEBUG);
      std::cout << "rs2::log_to_console(DEBUG): OK\n";
    } catch (const std::exception &e) {
      std::cout << "rs2::log_to_console(DEBUG) threw: " << e.what() << "\n";
      if (a.rs_log_only) return 2;
    }
    if (a.rs_log_only) return 0;
  }
  if (a.libusb_debug) setenv("LIBUSB_DEBUG", "4", 1);
  if (a.hold_capture && !hold_capture(1000)) {
    std::cout << "hold-capture: no RealSense captured, continuing without it\n";
  }

  std::cout << "cycles=" << a.cycles << " res=" << a.width << "x" << a.height << "@" << a.fps
            << " pre_option=" << a.pre_option << " post_option=" << a.post_option
            << " depth_options=" << a.depth_options << " stream=" << a.stream
            << " fresh_context=" << a.fresh_context << " sleep_ms=" << a.sleep_ms
            << " hold_capture=" << a.hold_capture << "\n";

  std::shared_ptr<rs2::context> ctx;
  auto make_ctx = [&]() {
    ctx = std::make_shared<rs2::context>(ctx_json());
    // Log what the polling watcher reports. Re-enumerations show up here as
    // removed/added pairs if the 2 s poll lands inside the window.
    ctx->set_devices_changed_callback([](rs2::event_information &info) {
      int added = 0;
      try { added = static_cast<int>(info.get_new_devices().size()); } catch (...) {}
      std::cout << "  " << now_str() << "  ** devices_changed callback: " << added << " added\n";
    });
  };
  make_ctx();

  int cycle_ok = 0;
  for (int c = 1; c <= a.cycles; ++c) {
    std::cout << "\n-- cycle " << c << "/" << a.cycles << " --\n";
    if (a.fresh_context && c > 1) make_ctx();
    try {
      rs2::device dev;
      rs2::device_list list;
      step("query_devices", [&] {
        // Just the list. No device object is built yet, so no USB power-up.
        list = ctx->query_devices();
        std::cout << "    devices: " << list.size() << "\n";
        if (list.size() == 0) throw std::runtime_error("no devices");
      });
      step("create_device (list[i])", [&] {
        // rs2_create_device builds the ds_device: hardware-monitor reads over
        // the depth sensor's UVC XU, i.e. a D0 -> D3 power cycle. This is what
        // failed as non-root with "failed to set power state".
        for (uint32_t i = 0; i < list.size(); ++i) {
          rs2::device d = list[i];
          if (a.serial.empty() || (d.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) &&
                                   a.serial == d.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER))) {
            dev = d;
            break;
          }
        }
        if (!dev) throw std::runtime_error("requested serial not found");
        if (c == 1) {
          auto info = [&](rs2_camera_info i) { return dev.supports(i) ? dev.get_info(i) : "n/a"; };
          std::cout << "    " << info(RS2_CAMERA_INFO_NAME) << " sn=" << info(RS2_CAMERA_INFO_SERIAL_NUMBER)
                    << " fw=" << info(RS2_CAMERA_INFO_FIRMWARE_VERSION)
                    << " usb=" << info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) << "\n";
        }
      });

      rs2::sensor color, depth;
      step("query_sensors", [&] {
        for (auto &&s : dev.query_sensors()) {
          if (s.is<rs2::color_sensor>()) color = s;
          if (s.is<rs2::depth_sensor>()) depth = s;
        }
        if (!color || !depth) throw std::runtime_error("missing color or depth sensor");
      });

      // Software option, handled inside librealsense: no USB traffic expected.
      step("set_option(GLOBAL_TIME) x2", [&] {
        color.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f);
        depth.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f);
      });

      if (a.pre_option) {
        // XU option: powers the color sensor D0 -> transfer -> D3.
        step("pre-start set_option(AE_PRIO)", [&] {
          if (color.supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY))
            color.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0.0f);
        });
      }
      if (a.depth_options) {
        step("pre-start set_option(LASER)", [&] {
          if (depth.supports(RS2_OPTION_LASER_POWER))
            depth.set_option(RS2_OPTION_LASER_POWER, depth.get_option_range(RS2_OPTION_LASER_POWER).def);
        });
      }

      std::vector<rs2::stream_profile> cps, dps;
      step("color.get_stream_profiles", [&] { cps = color.get_stream_profiles(); });
      step("depth.get_stream_profiles", [&] { dps = depth.get_stream_profiles(); });

      if (!a.stream) {
        cycle_ok++;
        std::this_thread::sleep_for(std::chrono::milliseconds(a.sleep_ms));
        continue;
      }

      rs2::config cfg;
      cfg.enable_device(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
      cfg.enable_stream(RS2_STREAM_COLOR, -1, a.width, a.height, RS2_FORMAT_RGB8, a.fps);
      cfg.enable_stream(RS2_STREAM_DEPTH, -1, a.width, a.height, RS2_FORMAT_Z16, a.fps);

      auto pipe = std::make_shared<rs2::pipeline>(*ctx);
      rs2::pipeline_profile prof;
      step("pipe.start", [&] { prof = pipe->start(cfg); });
      if (c == 1) {
        for (auto &&sp : prof.get_streams()) {
          auto v = sp.as<rs2::video_stream_profile>();
          std::cout << "    stream " << sp.stream_name() << " " << v.width() << "x" << v.height() << "@" << v.fps()
                    << "\n";
        }
      }

      if (a.post_option) {
        step("post-start set_option(AE_PRIO)", [&] {
          for (auto &&s : prof.get_device().query_sensors()) {
            if (s.is<rs2::color_sensor>() && s.supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY))
              s.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0.0f);
          }
        });
      }

      step("first frameset", [&] { (void)pipe->wait_for_frames(a.frame_timeout_ms); });
      if (a.frames > 1) {
        step("next framesets", [&] {
          for (int i = 1; i < a.frames; ++i) (void)pipe->wait_for_frames(a.frame_timeout_ms);
        });
      }
      step("pipe.stop", [&] { pipe->stop(); });
      cycle_ok++;
    } catch (const std::exception &) {
      std::cout << "  cycle " << c << " ABORTED\n";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(a.sleep_ms));
  }
  summary(a.cycles, cycle_ok);
  return cycle_ok == a.cycles ? 0 : 1;
} catch (const std::exception &e) {
  std::cerr << "fatal: " << e.what() << "\n";
  return 3;
}
