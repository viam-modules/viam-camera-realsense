#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "device.hpp"
#include "realsense.hpp"
#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/components/camera.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/log/logging.hpp>

#include <librealsense2/rs.hpp>

// Mock logger for testing
class MockLogger : public viam::sdk::LogSource {
public:
  MOCK_METHOD(void, log,
              (viam::sdk::log_level level, const std::string &message));
};

namespace realsense {
namespace device {
std::ostream &operator<<(std::ostream &os, const ViamRSDevice<> &device) {
  os << "ViamRSDevice{serial: " << device.serial_number
     << ", started: " << device.started << "}";
  return os;
}
} // namespace device
} // namespace realsense

using namespace realsense;
using namespace viam::sdk;

class MockRsDevice : public rs2::device {
public:
  MockRsDevice(const std::string &serial) : serial_(serial) {}

  const char *get_info(rs2_camera_info info) const {
    if (info == RS2_CAMERA_INFO_SERIAL_NUMBER) {
      return serial_.c_str();
    }
    return "mock_info";
  }

  std::string get_serial() const { return serial_; }

private:
  std::string serial_;
};

class MockDeviceFunctions {
public:
  MOCK_METHOD(
      bool, stopDevice,
      (std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>> &),
      ());
  MOCK_METHOD(
      bool, destroyDevice,
      (std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>> &),
      ());
  MOCK_METHOD(void, printDeviceInfo, (const rs2::device &), ());
  MOCK_METHOD(std::shared_ptr<device::ViamRSDevice<>>, createDevice,
              (const std::string &, std::shared_ptr<rs2::device>,
               const std::unordered_set<std::string> &,
               const realsense::RsResourceConfig &),
              ());
  MOCK_METHOD(
      void, startDevice,
      (const std::string &,
       std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>> &,
       std::shared_ptr<boost::synchronized_value<rs2::frameset>> &,
       std::uint64_t, const realsense::RsResourceConfig &),
      ());
  MOCK_METHOD(
      void, reconfigureDevice,
      (std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>> &,
       realsense::RsResourceConfig const &),
      ());
};

DeviceFunctions
createMockDeviceFunctionsWithOrder(std::shared_ptr<MockDeviceFunctions> mock) {
  return DeviceFunctions{
      .stopDevice =
          [mock](
              std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>>
                  &device,
              viam::sdk::LogSource &) -> bool {
        return mock->stopDevice(device);
      },
      .destroyDevice =
          [mock](
              std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>>
                  &device,
              viam::sdk::LogSource &) -> bool {
        return mock->destroyDevice(device);
      },
      .printDeviceInfo =
          [mock](const auto &dev, viam::sdk::LogSource &) {
            mock->printDeviceInfo(dev);
          },
      .createDevice =
          [mock](const std::string &serial,
                 std::shared_ptr<rs2::device> dev_ptr,
                 const std::unordered_set<std::string> &supported_models,
                 const realsense::RsResourceConfig &config,
                 viam::sdk::LogSource &) // Add logger parameter
      -> std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>> {
        auto raw_device = mock->createDevice(serial, dev_ptr, supported_models,
                                             config); // Pass config
        return std::make_shared<
            boost::synchronized_value<device::ViamRSDevice<>>>(*raw_device);
      },
      .startDevice =
          [mock](
              const std::string &serial,
              std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>>
                  &device,
              std::shared_ptr<boost::synchronized_value<rs2::frameset>>
                  &latest_frameset,
              std::uint64_t maxFrameAgeMs,
              const realsense::RsResourceConfig &viamConfig,
              viam::sdk::LogSource &) {
            mock->startDevice(serial, device, latest_frameset, maxFrameAgeMs,
                              viamConfig);
          },
      .reconfigureDevice =
          [mock](
              std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>>
                  device,
              realsense::RsResourceConfig const &viamConfig,
              viam::sdk::LogSource &) {
            mock->reconfigureDevice(device, viamConfig);
          }};
}

DeviceFunctions createFullyMockedDeviceFunctions() {
  return DeviceFunctions{
      .stopDevice =
          [](std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>>
                 &device,
             viam::sdk::LogSource &) -> bool {
        std::cout << "Mock: stopDevice called" << std::endl;
        if (device) {
          auto locked_device = device->synchronize();
          locked_device->started = false;
        }
        return true;
      },
      .destroyDevice =
          [](std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>>
                 &device,
             viam::sdk::LogSource &) -> bool {
        std::cout << "Mock: destroyDevice called" << std::endl;
        device = nullptr;
        return true;
      },
      .printDeviceInfo =
          [](const auto &dev, viam::sdk::LogSource &) {
            std::cout << "Mock: printDeviceInfo called" << std::endl;
            if constexpr (std::is_same_v<std::decay_t<decltype(dev)>,
                                         MockRsDevice>) {
              std::cout << "Mock device serial: " << dev.get_serial()
                        << std::endl;
            }
          },
      .createDevice =
          [](const std::string &serial, std::shared_ptr<rs2::device> dev_ptr,
             const std::unordered_set<std::string> &supported_models,
             const realsense::RsResourceConfig &config, viam::sdk::LogSource &)
          -> std::shared_ptr<
              boost::synchronized_value<device::ViamRSDevice<>>> {
        std::cout << "Mock: createDevice called for " << serial << std::endl;

        auto mock_device = std::make_shared<device::ViamRSDevice<>>();
        mock_device->serial_number = serial;
        mock_device->device = nullptr;
        mock_device->started = false;

        return std::make_shared<
            boost::synchronized_value<device::ViamRSDevice<>>>(*mock_device);
      },
      .startDevice =
          [](const std::string &serial,
             std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>>
                 &device,
             std::shared_ptr<boost::synchronized_value<rs2::frameset>>
                 &latest_frameset,
             std::uint64_t maxFrameAgeMs,
             const realsense::RsResourceConfig &viamConfig,
             viam::sdk::LogSource &) {
            std::cout << "Mock: startDevice called for " << serial << std::endl;
            if (device) {
              auto locked_device = device->synchronize();
              locked_device->started = true;
            }
          },
      .reconfigureDevice =
          [](std::shared_ptr<boost::synchronized_value<device::ViamRSDevice<>>>
                 device,
             realsense::RsResourceConfig const &viamConfig,
             viam::sdk::LogSource &) {
            std::cout << "Mock: reconfigureDevice called" << std::endl;
            // No-op for mock
          }};
}

class SimpleMockContext {
public:
  std::vector<MockRsDevice> mock_devices_;
  std::function<void(rs2::event_information &)> callback_;

  auto query_devices() const { return mock_devices_; }

  void set_devices_changed_callback(
      std::function<void(rs2::event_information &)> cb) {
    callback_ = cb;
  }

  void add_device(const std::string &serial) {
    auto it = std::find_if(mock_devices_.begin(), mock_devices_.end(),
                           [&serial](const MockRsDevice &dev) {
                             return dev.get_serial() == serial;
                           });
    if (it == mock_devices_.end()) {
      mock_devices_.emplace_back(serial);
    }
  }

  void remove_device(const std::string &serial) {
    mock_devices_.erase(std::remove_if(mock_devices_.begin(),
                                       mock_devices_.end(),
                                       [&serial](const MockRsDevice &dev) {
                                         return dev.get_serial() == serial;
                                       }),
                        mock_devices_.end());
  }

  void replace_device(const std::string &old_serial,
                      const std::string &new_serial) {
    remove_device(old_serial);
    add_device(new_serial);
  }

  void clear_devices() { mock_devices_.clear(); }
};

class RealsenseTestEnvironment : public ::testing::Environment {
public:
  void SetUp() override { instance_ = std::make_unique<Instance>(); }

  void TearDown() override { instance_.reset(); }

private:
  std::unique_ptr<Instance> instance_;
};

class RealsenseTest : public ::testing::Test {
protected:
  void SetUp() override {
    test_serial_ = "test_device_123456";
    test_name_ = "test_realsense_camera";

    auto attributes = ProtoStruct{};
    attributes["serial_number"] = test_serial_;
    ProtoList sensors = {"color", "depth"};
    attributes["sensors"] = sensors;

    test_config_ = std::make_unique<ResourceConfig>(
        "rdk:component:camera",               // api
        "",                                   // namespace
        test_name_,                           // name
        attributes,                           // attributes
        "",                                   // implicitDependsOn
        Model("viam", "camera", "realsense"), // model
        LinkConfig{},                         // associatedResourceConfigs
        log_level::info                       // logLevel
    );

    test_deps_ = Dependencies{};

    mock_context_ =
        std::make_shared<boost::synchronized_value<SimpleMockContext>>();
    {
      auto locked_context = mock_context_->synchronize();
      locked_context->add_device(test_serial_);
    }

    mock_realsense_context_ = std::make_shared<
        RealsenseContext<boost::synchronized_value<SimpleMockContext>>>(
        mock_context_);
    assigned_serials_ = std::make_shared<
        boost::synchronized_value<std::unordered_set<std::string>>>();
  }

  void TearDown() override {
    // Cleanup happens automatically
  }

  std::string test_serial_;
  std::string test_name_;
  std::unique_ptr<ResourceConfig> test_config_;
  Dependencies test_deps_;
  std::shared_ptr<boost::synchronized_value<SimpleMockContext>> mock_context_;
  std::shared_ptr<
      RealsenseContext<boost::synchronized_value<SimpleMockContext>>>
      mock_realsense_context_;
  std::shared_ptr<boost::synchronized_value<std::unordered_set<std::string>>>
      assigned_serials_; // Add this
};

TEST_F(RealsenseTest, ValidateWithValidConfig) {
  auto attributes = ProtoStruct{};
  attributes["serial_number"] = test_serial_;
  ProtoList sensors = {"color", "depth"};
  attributes["sensors"] = sensors;

  ResourceConfig valid_config(
      "rdk:component:camera", "", test_name_, attributes, "",
      Model("viam", "camera", "realsense"), LinkConfig{}, log_level::info);

  EXPECT_NO_THROW({
    auto result = Realsense<SimpleMockContext>::validate(valid_config);
    EXPECT_TRUE(result.empty()); // No validation errors
  });
}

TEST_F(RealsenseTest, ValidateWithInvalidConfig_NoSerialNumber) {
  auto empty_attributes = ProtoStruct{};
  ResourceConfig invalid_config(
      "rdk:component:camera", "", test_name_, empty_attributes, "",
      Model("viam", "camera", "realsense"), LinkConfig{}, log_level::info);

  EXPECT_THROW(
      { Realsense<SimpleMockContext>::validate(invalid_config); },
      std::invalid_argument);
}

TEST_F(RealsenseTest, ValidateWithEmptySerialNumber) {
  auto empty_serial_attributes = ProtoStruct{};
  empty_serial_attributes["serial_number"] = "";
  ProtoList sensors = {"color", "depth"};
  empty_serial_attributes["sensors"] = sensors;
  ResourceConfig valid_config(
      "rdk:component:camera", "", test_name_, empty_serial_attributes, "",
      Model("viam", "camera", "realsense"), LinkConfig{}, log_level::info);

  EXPECT_NO_THROW({
    auto result = Realsense<SimpleMockContext>::validate(valid_config);
    EXPECT_TRUE(result.empty());
  });
}

TEST_F(RealsenseTest, DoCommandReturnsEmptyStruct) {
  Realsense<boost::synchronized_value<SimpleMockContext>> camera(
      test_deps_, *test_config_, mock_realsense_context_,
      createFullyMockedDeviceFunctions(), assigned_serials_);

  ProtoStruct command{};
  auto result = camera.do_command(command);

  // do_command should return an empty ProtoStruct since it's not implemented
  EXPECT_TRUE(result.empty());
}

TEST_F(RealsenseTest, GetGeometriesReturnsExpectedGeometry) {
  Realsense<boost::synchronized_value<SimpleMockContext>> camera(
      test_deps_, *test_config_, mock_realsense_context_,
      createFullyMockedDeviceFunctions(), assigned_serials_);

  ProtoStruct extra{};
  auto geometries = camera.get_geometries(extra);

  EXPECT_EQ(geometries.size(), 1);
  EXPECT_EQ(geometries,
            std::vector<viam::sdk::GeometryConfig>{viam::sdk::GeometryConfig(
                viam::sdk::pose{-17.5, 0, -12.5}, viam::sdk::box({90, 25, 25}),
                "box")});
}

TEST(RealsenseStaticTest, ModelExists) {
  auto &model = Realsense<rs2::context>::model;

  // Test that we can access the model without errors
  EXPECT_NO_THROW({
    auto model_copy = model; // Test copy constructor
  });
  EXPECT_EQ(model.to_string(), "viam:camera:realsense");
}

TEST(RsResourceConfigTest, ConstructorSetsCorrectValues) {
  std::string serial = "test_serial_123";
  std::string name = "test_camera";
  std::vector<std::string> sensors = {"color", "depth"};
  std::optional<int> width = 640;
  std::optional<int> height = 480;

  RsResourceConfig config(serial, name, sensors, width, height);

  EXPECT_EQ(config.serial_number, serial);
  EXPECT_EQ(config.resource_name, name);
  EXPECT_EQ(config.sensors, sensors);
  EXPECT_EQ(config.width, width);
  EXPECT_EQ(config.height, height);
}

TEST(RealsenseTemplateTest, CanInstantiateWithRealContext) {
  // This is just a compilation test - we don't actually create the object
  using RealRealsense = Realsense<rs2::context>;

  // If this compiles, the template instantiation works
  static_assert(
      std::is_same_v<decltype(RealRealsense::model), viam::sdk::Model>);
}

TEST_F(RealsenseTest, ReconfigureWithSameSerialNumber_StrictOrdering) {
  using ::testing::_;
  using ::testing::InSequence;
  using ::testing::Return;

  auto mock_device_funcs = std::make_shared<MockDeviceFunctions>();

  // Create mock devices that the mocks will return
  auto mock_device_1 = std::make_shared<device::ViamRSDevice<>>();
  mock_device_1->serial_number = "test_device_123456";
  mock_device_1->started = false;
  mock_device_1->device = nullptr;

  auto mock_device_2 = std::make_shared<device::ViamRSDevice<>>();
  mock_device_2->serial_number = "test_device_123456";
  mock_device_2->started = false;
  mock_device_2->device = nullptr;

  {
    InSequence seq;

    // Constructor sequence
    EXPECT_CALL(*mock_device_funcs, printDeviceInfo(_)).Times(1);
    EXPECT_CALL(*mock_device_funcs, createDevice("test_device_123456", _, _, _))
        .Times(1)
        .WillOnce(Return(mock_device_1)); // Return a valid mock device
    EXPECT_CALL(*mock_device_funcs,
                startDevice("test_device_123456", _, _, _, _))
        .Times(1);

    // Reconfigure sequence
    EXPECT_CALL(*mock_device_funcs, stopDevice(_))
        .Times(1)
        .WillOnce(Return(true));
    EXPECT_CALL(*mock_device_funcs, reconfigureDevice(_, _))
        .Times(1)
        .WillOnce(Return());
    EXPECT_CALL(*mock_device_funcs,
                startDevice("test_device_123456", _, _, _, _))
        .Times(1);

    EXPECT_CALL(*mock_device_funcs, stopDevice(_))
        .Times(1)
        .WillOnce(Return(true));
    EXPECT_CALL(*mock_device_funcs, destroyDevice(_))
        .Times(1)
        .WillOnce(Return(true));
  }

  auto isolated_context =
      std::make_shared<boost::synchronized_value<SimpleMockContext>>();
  {
    auto locked_context = isolated_context->synchronize();
    locked_context->add_device("test_device_123456");
  }
  auto isolated_realsense_context = std::make_shared<
      RealsenseContext<boost::synchronized_value<SimpleMockContext>>>(
      isolated_context);

  auto assigned_serials = std::make_shared<
      boost::synchronized_value<std::unordered_set<std::string>>>();

  Realsense<boost::synchronized_value<SimpleMockContext>> camera(
      test_deps_, *test_config_, isolated_realsense_context,
      createMockDeviceFunctionsWithOrder(mock_device_funcs), assigned_serials);

  EXPECT_NO_THROW({ camera.reconfigure(test_deps_, *test_config_); });
}

TEST_F(RealsenseTest, ReconfigureWithNewSerialNumber_StrictOrdering) {
  using ::testing::_;
  using ::testing::InSequence;
  using ::testing::Return;

  auto mock_device_funcs = std::make_shared<MockDeviceFunctions>();

  // Create mock devices that the mocks will return
  auto mock_device_1 = std::make_shared<device::ViamRSDevice<>>();
  mock_device_1->serial_number = "test_device_123456";
  mock_device_1->started = false;
  mock_device_1->device = nullptr;

  auto mock_device_2 = std::make_shared<device::ViamRSDevice<>>();
  mock_device_2->serial_number = "new_device_789";
  mock_device_2->started = false;
  mock_device_2->device = nullptr;

  {
    InSequence seq;

    // Constructor sequence
    EXPECT_CALL(*mock_device_funcs, printDeviceInfo(_)).Times(1);
    EXPECT_CALL(*mock_device_funcs, createDevice("test_device_123456", _, _, _))
        .Times(1)
        .WillOnce(Return(mock_device_1)); // Return a valid mock device
    EXPECT_CALL(*mock_device_funcs,
                startDevice("test_device_123456", _, _, _, _))
        .Times(1);

    // Reconfigure sequence
    EXPECT_CALL(*mock_device_funcs, stopDevice(_))
        .Times(1)
        .WillOnce(Return(true));
    EXPECT_CALL(*mock_device_funcs, destroyDevice(_))
        .Times(1)
        .WillOnce(Return(true));
    EXPECT_CALL(*mock_device_funcs, printDeviceInfo(_)).Times(1);
    EXPECT_CALL(*mock_device_funcs, createDevice("new_device_789", _, _, _))
        .Times(1)
        .WillOnce(Return(mock_device_2)); // Return a valid mock device
    EXPECT_CALL(*mock_device_funcs, startDevice("new_device_789", _, _, _, _))
        .Times(1);

    EXPECT_CALL(*mock_device_funcs, stopDevice(_))
        .Times(1)
        .WillOnce(Return(true));
    EXPECT_CALL(*mock_device_funcs, destroyDevice(_))
        .Times(1)
        .WillOnce(Return(true));
  }

  auto isolated_context =
      std::make_shared<boost::synchronized_value<SimpleMockContext>>();
  {
    auto locked_context = isolated_context->synchronize();
    locked_context->add_device("test_device_123456");
  }
  auto isolated_realsense_context = std::make_shared<
      RealsenseContext<boost::synchronized_value<SimpleMockContext>>>(
      isolated_context);

  auto assigned_serials = std::make_shared<
      boost::synchronized_value<std::unordered_set<std::string>>>();

  Realsense<boost::synchronized_value<SimpleMockContext>> camera(
      test_deps_, *test_config_, isolated_realsense_context,
      createMockDeviceFunctionsWithOrder(mock_device_funcs), assigned_serials);

  {
    auto locked_context = isolated_context->synchronize();
    locked_context->replace_device("test_device_123456", "new_device_789");
  }

  auto new_attributes = ProtoStruct{};
  new_attributes["serial_number"] = "new_device_789";
  ProtoList sensors = {"color", "depth"};
  new_attributes["sensors"] = sensors;

  ResourceConfig new_config(
      "rdk:component:camera", "", "new_camera", new_attributes, "",
      Model("viam", "camera", "realsense"), LinkConfig{}, log_level::info);

  EXPECT_NO_THROW({ camera.reconfigure(test_deps_, new_config); });
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::AddGlobalTestEnvironment(new RealsenseTestEnvironment);
  return RUN_ALL_TESTS();
}
