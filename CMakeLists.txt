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

# The viam C++ SDK
FetchContent_Declare(
  viam-cpp-sdk
  GIT_REPOSITORY https://github.com/viamrobotics/viam-cpp-sdk.git
  GIT_TAG main
  #SOURCE_DIR ${CMAKE_SOURCE_DIR}/../viam-cpp-sdk
  CMAKE_ARGS -DVIAMCPPSDK_USE_DYNAMIC_PROTOS=ON
  FIND_PACKAGE_ARGS
)
FetchContent_MakeAvailable(viam-cpp-sdk)
find_package(viam-cpp-sdk CONFIG REQUIRED COMPONENTS Lib)

# The realsense library 
set(RS_LOCATION ${CMAKE_BINARY_DIR}/realsense_source)
if (NOT EXISTS ${RS_LOCATION})
    file(MAKE_DIRECTORY ${RS_LOCATION})
endif()
set(RS_CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=${RS_LOCATION} -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false")
ExternalProject_Add(
    librealsense
    GIT_REPOSITORY https://github.com/IntelRealSense/librealsense.git
    GIT_TAG v2.54.2 
    CMAKE_ARGS ${RS_CMAKE_ARGS}
)
include_directories(${RS_LOCATION}/include)
link_directories(${RS_LOCATION}/lib)

add_executable(viam-camera-realsense src/camera_realsense.cpp ${THIRD_PARTY_SOURCES})
add_dependencies(viam-camera-realsense librealsense)
target_compile_features(viam-camera-realsense PRIVATE cxx_std_17)
target_compile_options(viam-camera-realsense PRIVATE -O4 -pthread -Wredundant-move -Wpessimizing-move -Wl,-ldl)

target_link_libraries(viam-camera-realsense PRIVATE
    librealsense2.so # from linking RS_LOCATION/lib
    viam-cpp-sdk::viamsdk
    grpc++
    protobuf
    turbojpeg
)
