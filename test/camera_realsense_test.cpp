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

    // Add a valid serial_number
    ::google::protobuf::Value serialNumberValue;
    serialNumberValue.set_string_value("1234567890");
    (*testConfig.mutable_attributes()->mutable_fields())["serial_number"] = serialNumberValue;

    std::vector<std::string> expected_deps;
    EXPECT_EQ(vrs::validate(vsdk::from_proto(testConfig)), expected_deps);
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
        std::vector<std::string> result =
            vrs::validate(vsdk::from_proto(testConfig));
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
        std::vector<std::string> result =
            vrs::validate(vsdk::from_proto(testConfig));
        FAIL() << "Expected std::invalid_argument to catch empty sensors list";
    } catch (const std::invalid_argument& e) {
        std::string errorMessage = e.what();
        EXPECT_EQ(errorMessage,
                  "sensors field cannot be empty, must list color and/or depth sensor");
    } catch (...) {
        FAIL() << "Expected std::invalid_argument to catch empty sensors list";
    }
}

// Test cases for serial_number validation
TEST(ResourceConfigTest, InvalidSerialNumberConfig) {
    ::viam::app::v1::ComponentConfig testConfig;
    testConfig.set_name("my_camera_serial_test");
    testConfig.set_model("viam:camera:realsense");
    testConfig.set_api("rdk:component:camera");

    // Construct a valid config with a color sensor
    ::google::protobuf::Value colorSensor;
    colorSensor.set_string_value("color");
    ::google::protobuf::ListValue sensorList;
    sensorList.add_values()->CopyFrom(colorSensor);
    ::google::protobuf::Value sensorListValue;
    sensorListValue.mutable_list_value()->CopyFrom(sensorList);
    (*testConfig.mutable_attributes()->mutable_fields())["sensors"] = sensorListValue;

    // Test that an empty serial_number string is valid
    ::google::protobuf::Value emptySerialNumberValue;
    emptySerialNumberValue.set_string_value("");
    (*testConfig.mutable_attributes()->mutable_fields())["serial_number"] = emptySerialNumberValue;
    std::vector<std::string> expected_deps_empty_serial; // empty vector expected for successful validation
    EXPECT_EQ(vrs::validate(vsdk::from_proto(testConfig)), expected_deps_empty_serial);

    // Test that a non-string serial_number throws an error
    ::google::protobuf::Value nonStringSerialNumberValue;
    nonStringSerialNumberValue.set_number_value(12345);
    (*testConfig.mutable_attributes()->mutable_fields())["serial_number"] = nonStringSerialNumberValue;
    try {
        vrs::validate(vsdk::from_proto(testConfig));
        FAIL() << "Expected std::invalid_argument for non-string serial_number";
    } catch (const std::invalid_argument& e) {
        EXPECT_EQ(std::string(e.what()), "serial_number must be a string");
    } catch (...) {
        FAIL() << "Expected std::invalid_argument for non-string serial_number, got something else.";
    }

    // Test that a valid serial_number passes
    ::google::protobuf::Value validSerialNumberValue;
    validSerialNumberValue.set_string_value("1234567890");
    (*testConfig.mutable_attributes()->mutable_fields())["serial_number"] = validSerialNumberValue;
    std::vector<std::string> expected_deps_valid_serial; // empty vector expected for successful validation
    EXPECT_EQ(vrs::validate(vsdk::from_proto(testConfig)), expected_deps_valid_serial);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
