#include "src/camera_realsense.hpp"

int main(int argc, char** argv) {
    const std::string usage = "usage: camera_realsense /path/to/unix/socket";

    if (argc < 2) {
        std::cout << "ERROR: insufficient arguments\n";
        std::cout << usage << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "About to serve on socket " << argv[1] << std::endl;

    return viam::realsense::serve(argc, argv);
}
