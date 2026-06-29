#include "discovery.hpp"
#include "realsense.hpp"

#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/module/service.hpp>

#include <iostream>
#include <memory>

#include <boost/thread/synchronized_value.hpp>
#include <librealsense2/rs.hpp>

namespace vsdk = ::viam::sdk;

std::vector<std::shared_ptr<vsdk::ModelRegistration>>
create_all_model_registrations(
    std::shared_ptr<
        realsense::RealsenseContext<boost::synchronized_value<rs2::context>>>
        realsense_ctx,
    std::shared_ptr<boost::synchronized_value<std::unordered_set<std::string>>>
        assigned_serials) {
  std::vector<std::shared_ptr<vsdk::ModelRegistration>> registrations;

  registrations.push_back(std::make_shared<vsdk::ModelRegistration>(
      vsdk::API::get<vsdk::Camera>(),
      realsense::Realsense<boost::synchronized_value<rs2::context>>::model,
      [realsense_ctx, assigned_serials](vsdk::Dependencies deps,
                                        vsdk::ResourceConfig config) {
        return std::make_unique<
            realsense::Realsense<boost::synchronized_value<rs2::context>>>(
            std::move(deps), std::move(config), realsense_ctx,
            assigned_serials);
      },
      realsense::Realsense<realsense::RealsenseContext<
          boost::synchronized_value<rs2::context>>>::validate));

  registrations.push_back(std::make_shared<vsdk::ModelRegistration>(
      vsdk::API::get<vsdk::Discovery>(),
      realsense::discovery::RealsenseDiscovery<
          boost::synchronized_value<rs2::context>>::model,
      [realsense_ctx](vsdk::Dependencies deps, vsdk::ResourceConfig config) {
        return std::make_unique<realsense::discovery::RealsenseDiscovery<
            realsense::RealsenseContext<
                boost::synchronized_value<rs2::context>>>>(
            std::move(deps), std::move(config), realsense_ctx);
      }));

  return registrations;
}

int serve(int argc, char **argv) try {
  // Every Viam C++ SDK program must have one and only one Instance object
  // which is created before any other C++ SDK objects and stays alive until
  // all Viam C++ SDK objects are destroyed.
  vsdk::Instance inst;

  VIAM_SDK_LOG(info) << "[serve] Starting Realsense module";

#if defined(__APPLE__)
  // Log user ID and fail if it is Apple and not root
  auto uid = getuid();
  if (uid != 0) {
    std::cerr << "[serve] Realsense module is not running as root (user ID = "
              << uid
              << "), initialize viam-server with sudo: sudo viam-server "
                 "-config <path_to_config>"
              << std::endl;
    return EXIT_FAILURE;
  }

  // Enabling zlibrealsense debug logs for Mac only for now, as we don't count
  // with libraries with BUILD_EASYLOGGINGPP enabled on Linux, which is required
  // to enable debug logs.
  // https://viam.atlassian.net/browse/RSDK-13059
  for (size_t i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--log-level=debug") {
      rs2::log_to_console(RS2_LOG_SEVERITY_DEBUG);
    }
  }
#endif

  auto ctx = std::make_shared<boost::synchronized_value<rs2::context>>();
  // Wrap the context in a RealsenseContext, which will manage the callback for
  // device changes and notify all Realsense instances.
  // It also provides a thread-safe way to query connected devices.
  auto rs_ctx = std::make_shared<
      realsense::RealsenseContext<boost::synchronized_value<rs2::context>>>(
      ctx);

  // This keeps track of serial numbers that have already been assigned to
  // Realsense instances, to avoid assigning the same physical camera to
  // multiple instances.
  auto assigned_serials = std::make_shared<
      boost::synchronized_value<std::unordered_set<std::string>>>();

  auto module_service = std::make_shared<vsdk::ModuleService>(
      argc, argv, create_all_model_registrations(rs_ctx, assigned_serials));
  module_service->serve();

  return EXIT_SUCCESS;
} catch (const std::exception &ex) {
  std::cerr << "ERROR: A std::exception was thrown from `serve`: " << ex.what()
            << std::endl;
  return EXIT_FAILURE;
} catch (...) {
  std::cerr << "ERROR: An unknown exception was thrown from `serve`"
            << std::endl;
  return EXIT_FAILURE;
}

int main(int argc, char *argv[]) {
  std::cout << "Realsense C++ SDK version: " << RS2_API_VERSION_STR << "\n";
  const std::string usage = "usage: realsense /path/to/unix/socket";
  if (argc < 2) {
    std::cout << "ERROR: insufficient arguments\n";
    std::cout << usage << "\n";
    return EXIT_FAILURE;
  }

  return serve(argc, argv);
};
