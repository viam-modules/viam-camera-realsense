# CMakeLists.txt for tests

find_package(GTest REQUIRED)

add_executable(realsense_tests camera_realsense_test.cpp)

include_directories(${CMAKE_SOURCE_DIR}/src)
target_link_libraries(realsense_tests 
    viamrealsense 
    GTest::GTest 
    viam-cpp-sdk::viamapi
)

add_test(NAME CameraTests COMMAND realsense_tests)
