include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

# Helper library target for linking to the individual test binaries
add_library(viamrealsense_test_common INTERFACE)

target_link_libraries(viamrealsense_test_common
    INTERFACE
        viamrealsense
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        viam-cpp-sdk::viamsdk
)

function(viamrealsense_add_gtest SOURCE_FILE_NAME)
    get_filename_component(TEST_EXECUTABLE_NAME ${SOURCE_FILE_NAME} NAME_WE)

    add_executable(${TEST_EXECUTABLE_NAME} ${SOURCE_FILE_NAME})
    target_link_libraries(${TEST_EXECUTABLE_NAME} PRIVATE viamrealsense_test_common)

    # DISCOVERY_MODE PRE_TEST is required to avoid running the test at build-time
    # which fails because shared libraries aren't in the path yet.
    gtest_discover_tests(${TEST_EXECUTABLE_NAME} DISCOVERY_TIMEOUT 60 DISCOVERY_MODE PRE_TEST)
endfunction()

viamrealsense_add_gtest(device_tests.cpp)
viamrealsense_add_gtest(encoding_tests.cpp)
viamrealsense_add_gtest(realsense_tests.cpp)
viamrealsense_add_gtest(time_tests.cpp)
viamrealsense_add_gtest(discovery_tests.cpp)
viamrealsense_add_gtest(sensors_tests.cpp)
viamrealsense_add_gtest(utils_tests.cpp)
viamrealsense_add_gtest(zip_utils_tests.cpp)
