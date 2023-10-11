include(FetchContent)
include(ExternalProject)
cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(viam-camera-realsense
    DESCRIPTION "Viam Module for the Intel RealSense Camera"
    HOMEPAGE_URL https://github.com/viamrobotics/viam-camera-realsense
    LANGUAGES CXX
)
set(CMAKE_CXX_STANDARD 17)

# The image encoders
set(THIRD_PARTY_SOURCES src/third_party/fpng.cpp src/third_party/lodepng.cpp)

# construct the executable
add_executable(viam-camera-realsense src/camera_realsense.cpp ${THIRD_PARTY_SOURCES})

target_link_libraries(viam-camera-realsense PRIVATE
    grpc++
    protobuf
    turbojpeg
)

# The viam C++ SDK  - point to where the library files are located
find_package(viam-cpp-sdk REQUIRED)
target_link_libraries(viam-camera-realsense PRIVATE
    viam-cpp-sdk::viamsdk
)

# The realsense library 
find_package(realsense2 REQUIRED)
target_link_libraries(viam-camera-realsense PRIVATE
    realsense2::realsense2
)
# Add all the flags
target_compile_features(viam-camera-realsense PRIVATE 
    cxx_std_17
)
target_compile_options(viam-camera-realsense PRIVATE 
    -O4 
    -pthread 
    -Wredundant-move 
    -Wpessimizing-move 
    -Wl,-ldl
)

