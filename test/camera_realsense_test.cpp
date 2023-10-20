#include "camera_realsense.hpp"
#include <gtest/gtest.h>
#include <viam/sdk/components/camera/camera.hpp>
#include <viam/sdk/components/component.hpp>
#include <viam/sdk/components/camera/server.hpp>

namespace vsdk = ::viam::sdk;

TEST(ResourceTest, Type) {
    vsdk::ResourceConfig testConf = vsdk::ResourceConfig("camera");
    EXPECT_EQ(testConf.type(), "camera");
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
