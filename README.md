# Intel RealSense Modular Component

This is a [Viam module](https://docs.viam.com/manage/configuration/#modules) for the [Intel® RealSense™](https://github.com/IntelRealSense/librealsense) family of cameras. Registered at https://app.viam.com/module/viam/realsense.

## Build and Run

To use this module, follow these instructions to [add a module from the Viam Registry](https://docs.viam.com/modular-resources/configure/#add-a-module-from-the-viam-registry) and select the `viam:camera:realsense` model from the [`realsense` module](https://app.viam.com/module/viam/realsense).

## Configure your Realsense Camera

> [!NOTE]  
> Before configuring your camera, you must [create a robot](https://docs.viam.com/manage/fleet/robots/#add-a-new-robot).

Navigate to the **Config** tab of your robot’s page in [the Viam app](https://app.viam.com/).
Click on the **Components** subtab and click **Create component**. Select the `camera` type, then select the `realsense` model.
Enter a name for your camera and click **Create**.

On the new component panel, copy and paste the following attribute template into your camera’s **Attributes** box:

```json
{
  "sensors": ["color","depth"],
  "width_px": 640,
  "height_px": 480,
  "little_endian_depth": false
}
```

Edit the attributes as applicable.

> [!NOTE]  
> For more information, see [Configure a Robot](https://docs.viam.com/manage/configuration/).

## Attributes

The following attributes are available for `viam:camera:realsense` cameras:

| Name | Type | Inclusion | Description |
| ---- | ---- | --------- | ----------- |
| `sensors` | list | **Required** | The RealSense data streams you want your robot to sense from. A list that contain the strings `color` and/or `depth`. The sensor that comes first in the list is designated the "main sensor", and is the image that gets returned by `get_image` calls and appears in the **Control** tab on the [Viam app](https://app.viam.com). If you would like a list of images from all listed sensors simultaneously, use `get_images`.  |
| `width_px` | int | Optional | The width of the output images in pixels. If the RealSense cannot produce the requested resolution, the component will fail to be built. |
| `height_px` | int | Optional | The height of the output images in pixels. If the RealSense cannot produce the requested resolution, the component will fail to be built. |
| `little_endian_depth` | bool | Optional | A bool that specifies whether raw depth data should be encoded in a little-endian byte order. By default it is `false`, and encodes the raw depth data in a big-endian byte order. |


### Example configuration:

```
{
  "components": [
    {
      "name": "myRealSense",
      "attributes": {
        "sensors": ["color","depth"],
        "width_px": 640,
        "height_px": 480,
        "little_endian_depth": false
      },
      "namespace": "rdk",
      "type": "camera",
      "model": "viam:camera:realsense"
    }
  ]
}
```

### Locally installing the module

If you do not want to use the Viam Registry, for Linux distros, download the module AppImage from our servers to your robot.

```
sudo curl -o /usr/local/bin/viam-camera-realsense http://packages.viam.com/apps/camera-servers/viam-camera-realsense-latest-aarch64.AppImage
sudo chmod a+rx /usr/local/bin/viam-camera-realsense
```

If you need the AppImage associated with a specific tag, replace `latest` in the URL with the tag version, i.e. `0.0.X`.

Then modify your robot's JSON file as follows:

```
  "modules": [
    {
      "type": "local",
      "name": "intel",
      "executable_path": "/usr/local/bin/viam-camera-realsense"
    }
  ],
```

## Integration Tests

### Running the tests
The repo comes with a suite of integration tests that allow one to test if the module works with an actual realsense device on the machine of interest. These integration tests are compiled into a binary and can be downloaded here:

```
curl -o realsense-integration-tests http://packages.viam.com/apps/camera-servers/realsense-integration-tests-latest-aarch64.AppImage
sudo chmod a+x realsense-integration-tests
./realsense-integration-tests -module /path/to/the/module
```

The binary takes one argument, which is the location to the module you would like to test out. Make sure only one RealSense device is connected to the machine you will be running the tests on.

### Compiling the integration tests

If you would like to compile the integration tests yourself, you will need to compile the binary on the same machine you expect to run it on.

- Copy the repo to your local robot: `git clone https://github.com/viamrobotics/viam-camera-realsense.git`
- run `make realsense-integration-tests`
- run the tests with `./realsense-integration-tests -module /path/to/realsense/module`

## Known Supported Hardware

Support for specific hardware is known for the following devices. The table is not complete and subject to change. In order to test out the module for your specific set up, it is recommended you run the integration tests provided.

| Devices               | D435 | D435i | D455 |
|-----------------------|------|-------|------|
| RPi 4B Bullseye       |  X   |       |      |
| Orin Nano JetPack 5.1 |  X   |   X   |  X   |


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

## Building The Module

You can also build it yourself using Docker and [Viam canon](https://github.com/viamrobotics/canon). 
Use the commands

```
docker pull ghcr.io/viamrobotics/viam-camera-realsense:arm64
git clone https://github.com/viamrobotics/viam-camera-realsense/
cd viam-camera-realsense/
canon -arch arm64 make appimage
```

This will use the Docker container to compile a binary for the `aarch64` architecture. The AppImage will be put in the `packaging/appimages/deploy` directory.

If you would like to try to gather all of the dependencies yourself and not use Docker, you will need:

- [librealsense](https://github.com/IntelRealSense/librealsense)
  - `git checkout` and install from source. 
  - be sure to use cmake flags `cmake .. -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false -DCMAKE_BUILD_TYPE=Release`
- [libjpegturbo](https://github.com/libjpeg-turbo/libjpeg-turbo)
- [libprotobuf](https://github.com/protocolbuffers/protobuf)
- [Viam C++ SDK](https://github.com/viamrobotics/viam-cpp-sdk/)
  - specifically `libviamsdk`, `libviamapi`, and `libviam_rust_utils`

then do `make viam-camera-realsense` to compile the binary, and `make appimage` to create the AppImage.

