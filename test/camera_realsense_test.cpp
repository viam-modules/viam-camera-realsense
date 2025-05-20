#include "camera_realsense.hpp"

#include <google/protobuf/struct.pb.h>
#include <gtest/gtest.h>
#include <viam/api/app/v1/robot.pb.h>

#include <viam/sdk/config/resource.hpp>

namespace vsdk = ::viam::sdk;
namespace vrs = ::viam::realsense;

// Test that a valid config throws no errors
TEST(ResourceConfigTest, ValidConfig) {
    ::viam::app::v1::ComponentConfig testConfig;
    testConfig.set_name("my_camera");
    testConfig.set_model("viam:camera:realsense");
    testConfig.set_api("rdk:component:camera");
    // add the sensors attribute
    ::google::protobuf::Value colorSensor;
    colorSensor.set_string_value("color");
    ::google::protobuf::Value depthSensor;
    depthSensor.set_string_value("depth");
    ::google::protobuf::ListValue sensorList;
    sensorList.add_values()->CopyFrom(colorSensor);
    sensorList.add_values()->CopyFrom(depthSensor);
    ::google::protobuf::Value sensorListValue;
    sensorListValue.mutable_list_value()->CopyFrom(sensorList);
    // create the attributes field
    (*testConfig.mutable_attributes()->mutable_fields())["sensors"] = sensorListValue;
    std::vector<std::string> expected;  // the empty string vector is expected
    EXPECT_EQ(vrs::validate(vsdk::from_proto(testConfig)), expected);
}

// Test that the validate function returns an expected error when the required "sensors" are not
// there.
TEST(ResourceConfigTest, InvalidSensorsConfig) {
    ::viam::app::v1::ComponentConfig testConfig;
    testConfig.set_name("my_camera");
    testConfig.set_model("viam:camera:realsense");
    testConfig.set_api("rdk:component:camera");
    // test that the sensors attribute is missing
    try {
        std::vector<std::string> result = vrs::validate(vsdk::from_proto(testConfig));
        FAIL() << "Expected std::invalid_argument to catch sensors attribute missing";
    } catch (const std::invalid_argument& e) {
        std::string errorMessage = e.what();
        EXPECT_EQ(errorMessage, "could not find required 'sensors' attribute in the config");
    } catch (...) {
        FAIL() << "Expected std::invalid_argument to catch sensors attribute missing";
    }
    // test that the sensors attribute is there, but empty
    ::google::protobuf::ListValue sensorList;
    ::google::protobuf::Value sensorListValue;
    sensorListValue.mutable_list_value()->CopyFrom(sensorList);
    (*testConfig.mutable_attributes()->mutable_fields())["sensors"] = sensorListValue;
    try {
        std::vector<std::string> result = vrs::validate(vsdk::from_proto(testConfig));
        FAIL() << "Expected std::invalid_argument to catch empty sensors list";
    } catch (const std::invalid_argument& e) {
        std::string errorMessage = e.what();
        EXPECT_EQ(errorMessage,
                  "sensors field cannot be empty, must list color and/or depth sensor");
    } catch (...) {
        FAIL() << "Expected std::invalid_argument to catch empty sensors list";
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
