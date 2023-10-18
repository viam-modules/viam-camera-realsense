# CMakeLists.txt for tests

find_package(GTest REQUIRED)
find_package(viam-cpp-sdk REQUIRED)

add_executable(realsense_tests camera_realsense_test.cpp)

include_directories(${CMAKE_SOURCE_DIR}/src)
target_link_libraries(realsense_tests GTest::GTest GTest::Main viam-cpp-sdk::viamsdk)

add_test(NAME CameraTests COMMAND realsense_tests)
