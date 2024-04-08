cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(viam-camera-realsense
    DESCRIPTION "Viam Module for the Intel RealSense Camera"
    HOMEPAGE_URL https://github.com/viamrobotics/viam-camera-realsense
    LANGUAGES CXX
)
set(CMAKE_CXX_STANDARD 17)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(realsense2 REQUIRED)
find_package(viam-cpp-sdk REQUIRED)

# The image encoders
set(THIRD_PARTY_SOURCES src/third_party/fpng.cpp src/third_party/lodepng.cpp)

# construct the library
add_library(viamrealsense src/camera_realsense.cpp ${THIRD_PARTY_SOURCES})

target_link_libraries(viamrealsense 
    PUBLIC
        viam-cpp-sdk::viamsdk
        realsense2::realsense2
    PRIVATE
	turbojpeg
	${CMAKE_DL_LIBS}
)

# Add all the flags
target_compile_features(viamrealsense PUBLIC
    cxx_std_17
)

# Adjust compiler flags for SSE4 compatibility
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    target_compile_options(viamrealsense PRIVATE 
        -O2 
        -Wredundant-move 
        -Wpessimizing-move 
        -msse4
    )
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    target_compile_options(viamrealsense PRIVATE 
        -O2 
        -Wredundant-move 
        -Wpessimizing-move 
        -march=native
    )
endif()

# construct the module executable
add_executable(viam-camera-realsense main.cpp)

target_link_libraries(viam-camera-realsense PRIVATE viamrealsense)

# Adjust compiler flags for SSE4 compatibility
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    target_compile_options(viam-camera-realsense PRIVATE 
        -O2 
        -Wredundant-move 
        -Wpessimizing-move 
        -msse4
    )
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    target_compile_options(viam-camera-realsense PRIVATE 
        -O2 
        -Wredundant-move 
        -Wpessimizing-move 
        -march=native
    )
endif()

enable_testing()
add_subdirectory(test)
