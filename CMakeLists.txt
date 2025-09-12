cmake_minimum_required(VERSION 3.25 FATAL_ERROR)

project(viam-camera-realsense
    DESCRIPTION "Viam Module for the Intel RealSense Camera"
    HOMEPAGE_URL https://github.com/viam-modules/viam-camera-realsense
    LANGUAGES CXX
)

include(cmake/simd_flags.cmake)

option(VIAM_REALSENSE_ENABLE_SANITIZER "Enable AddressSanitizer" OFF)
option(VIAM_REALSENSE_ENABLE_TESTS "Enable unit tests" ON)


# Here we set the rpath to accomodate one and only one supported deployment
# case, a directory containing
# ├── bin
# │   └── orbbec-module
# ├── lib
#     └── <runtime dependencies>
if (APPLE)
    list(PREPEND CMAKE_INSTALL_RPATH "@loader_path/../lib")
else()
    list(PREPEND CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
endif()


set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(realsense2 REQUIRED)

find_package(libjpeg-turbo REQUIRED)

find_package(viam-cpp-sdk REQUIRED)

add_library(viamrealsense src/module/encoding.cpp)

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
    PRIVATE
        libjpeg-turbo::turbojpeg-static
        ${CMAKE_DL_LIBS}
)

add_executable(viam-camera-realsense src/module/main.cpp)
target_link_libraries(viam-camera-realsense PRIVATE viamrealsense)

if(VIAM_REALSENSE_ENABLE_SANITIZER)
    message(STATUS "Building with AddressSanitizer enabled")
    target_compile_options(viamrealsense PUBLIC
        -fsanitize=address
        -fsanitize-address-use-after-scope
        -fno-omit-frame-pointer
        -static-libasan
    )

    target_link_options(viamrealsense PUBLIC
        -fsanitize=address
        -fno-omit-frame-pointer
        -static-libasan
    )
endif()

if (NOT MSVC)
    target_compile_options(viamrealsense
        PRIVATE
            -Wredundant-move
            -Wpessimizing-move
    )
endif()

if (LINUX)
    set(_pre_exclude_regex
	    ".*ld-linux.*\\.so.*"
	    ".*libc\\.so.*"
	    ".*libdl\\.so.*"
	    ".*libgcc.*\\.so.*"
	    ".*libm\\.so.*"
	    ".*libpthread\\.so.*"
    )
endif()

if (APPLE)
    # For reasons unclear to me the install command below doesn't work without this
    set(_framework_dest_arg "FRAMEWORK" "DESTINATION" "lib")
endif()

install(
    TARGETS viam-camera-realsense
    #[[
    RUNTIME_DEPENDENCIES
		PRE_EXCLUDE_REGEXES ${_pre_exclude_regex}
    RUNTIME
    LIBRARY
    ${_framework_dest_arg}
    ]]
)

if (VIAM_REALSENSE_ENABLE_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
