cmake_minimum_required(VERSION 3.25 FATAL_ERROR)
set(CMAKE_PROJECT_VERSION 0.0.1)


project(viam-camera-realsense
    DESCRIPTION "Viam Module for the Intel RealSense Camera"
    HOMEPAGE_URL https://github.com/viam-modules/viam-camera-realsense
    LANGUAGES CXX
)

include(cmake/simd_flags.cmake)

option(VIAM_REALSENSE_ENABLE_TESTS "Enable unit tests" OFF)

if (APPLE)
    list(PREPEND CMAKE_INSTALL_RPATH "@loader_path/../lib")
else()
    list(PREPEND CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(realsense2 REQUIRED)
find_package(viam-cpp-sdk REQUIRED)
find_package(libjpeg-turbo CONFIG REQUIRED)
find_package(CURL REQUIRED)
find_package(PkgConfig REQUIRED)

# Use pkg-config for libzip when available (more reliable with system packages)
# Fall back to find_package for Conan-provided libzip
pkg_check_modules(LIBZIP IMPORTED_TARGET libzip)
if(NOT LIBZIP_FOUND)
    find_package(libzip REQUIRED)
endif()

add_library(viamrealsense
    src/module/encoding.cpp
    src/module/zip_utils.cpp
    src/module/download_utils.cpp
    src/module/firmware_update.cpp
)
target_include_directories(viamrealsense
    PUBLIC src/module

)

target_compile_features(viamrealsense PUBLIC
    cxx_std_17
)
target_link_libraries(viamrealsense
    PUBLIC
        viam-cpp-sdk::viamsdk
        realsense2::realsense2
        libjpeg-turbo::turbojpeg-static
        CURL::libcurl
        $<IF:$<TARGET_EXISTS:PkgConfig::LIBZIP>,PkgConfig::LIBZIP,libzip::zip>
    PRIVATE
        ${CMAKE_DL_LIBS}
)
add_executable(viam-camera-realsense src/module/main.cpp)
target_link_libraries(viam-camera-realsense PRIVATE viamrealsense)

install(TARGETS 
    viamrealsense 
    viam-camera-realsense 
    DESTINATION .)
install( FILES meta.json DESTINATION .)


if (VIAM_REALSENSE_ENABLE_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
