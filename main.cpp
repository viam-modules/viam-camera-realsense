#include "src/camera_realsense.hpp"

#include <viam/sdk/common/instance.hpp>

int main(int argc, char** argv) {
    const std::string usage = "usage: camera_realsense /path/to/unix/socket";

    if (argc < 2) {
        std::cout << "ERROR: insufficient arguments\n";
        std::cout << usage << "\n";
        return EXIT_FAILURE;
    }

    viam::sdk::Instance instance;
    return viam::realsense::serve(argc, argv);
}
