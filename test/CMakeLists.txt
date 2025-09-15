include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  # If we add libgmock-dev to the docker files it will be possible to use find_package here
  # instead of always downloading and building from source
  # FIND_PACKAGE_ARGS NAMES GTest
)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

# Helper library target for linking to the individual test binaries
add_library(viamrealsense_test_common INTERFACE)

target_link_libraries(viamrealsense_test_common
    INTERFACE
        viamrealsense
        GTest::gtest
        GTest::gtest_main # provides main()
        GTest::gmock
        viam-cpp-sdk::viamsdk
)

function(viamrealsense_add_gtest SOURCE_FILE_NAME)
    get_filename_component(TEST_EXECUTABLE_NAME ${SOURCE_FILE_NAME} NAME_WE)

    add_executable(${TEST_EXECUTABLE_NAME} ${SOURCE_FILE_NAME})
    target_link_libraries(${TEST_EXECUTABLE_NAME} viamrealsense_test_common)

    gtest_discover_tests(${TEST_EXECUTABLE_NAME})
endfunction()

viamrealsense_add_gtest(device_tests.cpp)
viamrealsense_add_gtest(encoding_tests.cpp)
viamrealsense_add_gtest(realsense_tests.cpp)
viamrealsense_add_gtest(time_tests.cpp)
viamrealsense_add_gtest(discovery_tests.cpp)
