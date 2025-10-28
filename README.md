# [`realsense` module](https://app.viam.com/module/viam/realsense)

This [module](https://docs.viam.com/registry/#modular-resources) implements the [`rdk:component:camera` API](https://docs.viam.com/components/camera/) in a `viam:camera:realsense` model.
Configure this model on your machine to stream image and depth data from the [Intel RealSense](https://github.com/IntelRealSense/librealsense) family of cameras to Viam.

Navigate to the [**CONFIGURE** tab](https://docs.viam.com/build/configure/) of your [machine](https://docs.viam.com/fleet/machines/) in [the Viam app](https://app.viam.com/).
[Add `camera / realsense` to your machine](https://docs.viam.com/build/configure/#components).

> [!NOTE]
> For more information, see [Configure a Machine](https://docs.viam.com/manage/configuration/).

## Configure your `realsense` camera

### Configure with discovery service

1. On the [**CONFIGURE** tab](https://docs.viam.com/build/configure/), add the ** discovery / realsense:discovery ** service.
1. Save your configuration.
1. Click on the **TEST** panel for your discovery service.
   You should now see possible configurations.
1. Click **Add component** next to the configuration for your camera.

### Configure manually

Copy and paste the following attributes into your camera's JSON configuration:

```json
{
  "sensors": ["color", "depth"],
  "width_px": 640,
  "height_px": 480,
  "little_endian_depth": false,
  "serial_number": ""
}
```

Edit the attributes as applicable.

### Attributes

The following attributes are available for `viam:camera:realsense` cameras:

| Name | Type | Inclusion | Description |
| ---- | ---- | --------- | ----------- |
| `sensors` | list | **Required** | The RealSense data streams you want your robot to sense from. A list that contain the strings `color` and/or `depth`. The sensor that comes first in the list is designated the "main sensor", and is the image that gets returned by `get_image` calls and appears in the **CONTROL** tab on the [Viam app](https://app.viam.com). If you would like a list of images from all listed sensors simultaneously, use [`GetImages`](https://docs.viam.com/components/camera/#getimages).  |
| `width_px` | int | Optional | The width of the output images in pixels. If the RealSense cannot produce the requested resolution, the component will fail to be built. |
| `height_px` | int | Optional | The height of the output images in pixels. If the RealSense cannot produce the requested resolution, the component will fail to be built. |
| `little_endian_depth` | bool | Optional | A bool that specifies whether raw depth data should be encoded in a little-endian byte order. By default it is `false`, and encodes the raw depth data in a big-endian byte order. |
| `serial_number` | string | Optional | The serial number of the specific RealSense camera to use. To find your camera's serial number, the serial number of each plugged-in and available RealSense camera will be logged on module startup. You can also find device information using the [RealSense SDK directly](https://github.com/IntelRealSense/librealsense/blob/master/tools/enumerate-devices/readme.md). If this field is omitted or is an empty string, the module will use the first RealSense camera it detects. |

## Example configuration:

```
{
  "components": [
    {
      "name": "myRealSense",
      "attributes": {
        "sensors": ["color","depth"],
        "width_px": 640,
        "height_px": 480,
        "little_endian_depth": false,
        "serial_number": "YOUR_CAMERA_SERIAL_NUMBER"
      },
      "namespace": "rdk",
      "type": "camera",
      "model": "viam:camera:realsense"
    }
  ]
}
```

## Test the module

Before testing the module, make sure that your machine is connected to the Viam app, displaying as **Live** in the part status dropdown in the top right corner of the machine's page.

### Camera API

Once the `realsense` model is configured on your machine, you can access the depth and color data the camera streams through the [Viam camera API](https://docs.viam.com/components/camera/#api).
The following methods of the Viam camera API are supported:

- [`GetPointCloud`](https://docs.viam.com/components/camera/#getpointcloud): returns depth data and can return color data depending on the provided image
- [`GetImage`](https://docs.viam.com/components/camera/#getimage): returns color data
- [`GetImages`](https://docs.viam.com/components/camera/#getimages): returns both depth and color data, possible to filter by image source
- [`GetProperties`](https://docs.viam.com/components/camera/#getproperties): returns intrinsic properties of a camera

### CONTROL tab of app.viam.com

You can view the data your camera streams live on the **CONTROL** tab of the Viam app.
For more information, see [Control Machines](https://docs.viam.com/fleet/control/).

### Locally install the module

If you are using a Linux machine, and do not want to use the Viam registry, you can [download the module code from the registry](https://app.viam.com/module/viam/realsense) and use it directly on your machine.
Note that as of version 0.16.0-rc3, the module is no longer distributed as an AppImage.

Follow the instructions to [add a local module](https://docs.viam.com/operate/modules/support-hardware/#test-your-module-locally) just as you would for testing.
Provide an **Executable path** of `/usr/local/bin/viam-camera-realsense` when adding the module.

Or, if you aren't using the Viam app to manage your machine's configuration, modify your machine's JSON file as follows to add the `realsense` module to your machine:

```
  "modules": [
    {
      "type": "local",
      "name": "intel",
      "executable_path": "/usr/local/bin/viam-camera-realsense"
    }
  ],
```

## Known supported hardware

Support for specific hardware is known for the following devices. The table is not complete and subject to change.

| Devices               | D435 | D435i | D455 |
|-----------------------|------|-------|------|
| RPi 4B Bookworm       |  X   |       |      |
| RPi 4B Bullseye       |  X   |       |      |
| Orin Nano JetPack 5.1 |  X   |   X   |  X   |
| UP 4000               |  X   |       |      |

## Linux distribution recommendation

This module depends on the [`librealsense` SDK](https://github.com/IntelRealSense/librealsense/releases). As of the time of writing, Ubuntu is the only Linux Distro `librealsense` officially supports. The module works on our hardware setups using Bullseye on RPI4, and some setups on Bookworm. However, we recommend adhering to the requirements of the SDK dependency and to use Ubuntu when possible to avoid instability and unexpected behavior.

## Troubleshooting

If you get an error like "failed to set power state", or "Permission denied", you may need to install the udev rules for when the USB plugs in.

```
wget https://raw.githubusercontent.com/IntelRealSense/librealsense/7a7c2bcfbc03d45154ad63fa76b221b2bb9d228f/config/99-realsense-libusb.rules
sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

You can also look at the official RealSense troubleshooting guide [here](https://github.com/IntelRealSense/librealsense/wiki/Troubleshooting-Q%26A#q-i-ran-the-udev-rules-script-but-linux-still-get-permission-denied).

The module takes advantage of faster USB ports. Use the (blue) USB 3.0 port on the Raspberry Pi for faster streaming and access to more resolution options.

## Building the module

You can also build the module yourself from source using Conan. See the [SDK Docs](https://github.com/viamrobotics/viam-cpp-sdk/blob/main/BUILDING.md#building-with-conan)
for instructions on initial Conan setup and configuration. Then you can do

```
conan remote add viamconan https://viam.jfrog.io/artifactory/api/conan/viamconan
conan install .  -b missing -o:a "viam-cpp-sdk/*:shared=False"
cmake . --preset=conan-release
```

This will create a CMake build tree which can be built with CMake or directly by running `make` for example. You can pass arguments
to the `cmake` call to configure the build, for example `-G Ninja` to use `ninja` as the build system, or `-DVIAM_REALSENSE_ENABLE_SANITIZER=1` to
build the module with ASAN/LSAN enabled to test for memory leaks.

> [!NOTE]
> Running `conan install` above may generate an error related to `libudev`. You can fix this by installing `libudev` with your system
package manager, or follow the instructions in the error message to tell `conan` how to resolve this dependency.

If you would like to try to gather all of the dependencies yourself, you will need:

- [librealsense](https://github.com/IntelRealSense/librealsense)
  - `git checkout` and install from source.
  - be sure to use cmake flags `cmake .. -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false -DCMAKE_BUILD_TYPE=Release`
- [libjpegturbo](https://github.com/libjpeg-turbo/libjpeg-turbo)
- [libprotobuf](https://github.com/protocolbuffers/protobuf)
- [Viam C++ SDK](https://github.com/viamrobotics/viam-cpp-sdk/)
  - specifically `libviamsdk`, `libviamapi`, and `libviam_rust_utils`

then use CMake to generate a build.

## Using within a Frame System

It is important to consider that the coordinate system of the camera might not match the coordinate system of the component it is mounted on. Let us consider the scenario where the camera is mounted on a base, such that the camera faces the forward direction of movement of the base. Let the base's forward direction be the +Y axis. Following the right hand rule, the +X axis points right, and so the +Z axis of the base points up. For the intel realsense camera the +Z axis points out of the camera lens, the +X axis points to the right, and the +Y axis points down. To properly configure these components in the frame system we say that the camera's parent is the base. We say that the orientation of the camera in Viam's Orientation Vector Degrees is OX:0, OY:1, OZ:0, Theta:-90. It is important to note that the base itself must also be in the frame system, the base would have parent as world with the default Viam Orientation Vector Degrees values, i.e. OX:0, OY:0, OZ:1, Theta:0.

For the Z offset between the exterior of the camera and the point where depth = 0, find the depth start point (ground zero reference) in the [manufacturer data sheet](https://dev.intelrealsense.com/docs/intel-realsense-d400-series-product-family-datasheet).

Below is an image of the intel realsense's coordinate system.

![intel realsense internal coordinate system](https://www.intelrealsense.com/wp-content/uploads/2019/02/LRS_CS_axis_base.png)
