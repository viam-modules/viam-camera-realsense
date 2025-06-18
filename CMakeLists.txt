cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(viam-camera-realsense
    DESCRIPTION "Viam Module for the Intel RealSense Camera"
    HOMEPAGE_URL https://github.com/viam-modules/viam-camera-realsense
    LANGUAGES CXX
)
set(CMAKE_CXX_STANDARD 17)

# Define an option for enabling build with AddressSanitizer
option(ENABLE_SANITIZER "Enable AddressSanitizer" OFF)
option(CMAKE_BUILD_TYPE RelWithDebInfo)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(realsense2 REQUIRED)
find_package(viam-cpp-sdk REQUIRED)

set(HELPER_SOURCES src/encoding.hpp src/version_helpers.hpp)
set(THIRD_PARTY_SOURCES src/third_party/fpng.cpp src/third_party/lodepng.cpp) 
# construct the library
add_library(viamrealsense src/camera_realsense.cpp ${THIRD_PARTY_SOURCES} ${HELPER_SOURCES}) 
target_link_libraries(viamrealsense PUBLIC viam-cpp-sdk::viamsdk realsense2::realsense2 PRIVATE turbojpeg ${CMAKE_DL_LIBS})

# Conditionally apply SIMD flags based on the processor architecture for fpng acceleration
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(viamrealsense PRIVATE -msse4.1 -maes -mpclmul)
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
        target_compile_options(viamrealsense PRIVATE -xsse4.1 -maes -mpclmul)
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        target_compile_options(viamrealsense PRIVATE /arch:SSE2)
    endif()
endif()

add_executable(viam-camera-realsense main.cpp)
target_link_libraries(viam-camera-realsense PRIVATE viamrealsense)

if(ENABLE_SANITIZER)
    add_compile_options(
        -fsanitize=address
        -fsanitize-address-use-after-scope
        -fno-omit-frame-pointer
    )
    add_link_options(
        -fsanitize=address
        -fno-omit-frame-pointer
    )
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -static-libasan")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libasan -fno-omit-frame-pointer")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-libasan")

    target_compile_options(viamrealsense PRIVATE
        -fsanitize=address
        -fsanitize-address-use-after-scope
        -fno-omit-frame-pointer
    )
    target_link_options(viamrealsense PRIVATE
        -fsanitize=address
        -fno-omit-frame-pointer
    )
    target_compile_options(viam-camera-realsense PRIVATE
        -fsanitize=address
        -fsanitize-address-use-after-scope
        -fno-omit-frame-pointer
    )
    target_link_options(viam-camera-realsense PRIVATE
        -fsanitize=address
        -fno-omit-frame-pointer
    )
endif()

# Add shared flags
target_compile_features(viamrealsense PUBLIC
    cxx_std_17
)
target_compile_options(viamrealsense PRIVATE 
    -O2 
    -Wredundant-move 
    -Wpessimizing-move 
)

# Apply the same architecture-specific flags to the executable for fpng acceleration
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(viam-camera-realsense PRIVATE -msse4.1 -maes -mpclmul)
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
        target_compile_options(viam-camera-realsense PRIVATE -xsse4.1 -maes -mpclmul)
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        target_compile_options(viam-camera-realsense PRIVATE /arch:SSE2)
    endif()
endif()

enable_testing()
add_subdirectory(test)
