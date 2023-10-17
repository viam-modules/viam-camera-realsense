#include "camera_realsense.hpp"
#include <gtest/gtest.h>
#include <viam/sdk/components/camera/camera.hpp>
#include <viam/sdk/components/component.hpp>

namespace vsdk = ::viam::sdk;

TEST(ValidateTest, AddTest) {
    vsdk::ResourceConfig testConf = vsdk::ResourceConfig("camera");
    EXPECT_EQ(testConf.type(), "camera");
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
