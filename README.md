# Intel RealSense Modular Component

This is a [Viam module](https://docs.viam.com/manage/configuration/#modules) for the [Intel® RealSense™](https://github.com/IntelRealSense/librealsense) family of cameras.

## Getting Started

For Linux Distros, the simplest way of getting the camera server is by downloading the AppImage from

```
sudo curl -o /usr/local/bin/viam-camera-realsense http://packages.viam.com/apps/camera-servers/viam-camera-realsense-latest-aarch64.AppImage
sudo chmod a+rx /usr/local/bin/viam-camera-realsense
```

If you need the AppImage associated with a specific tag, replace `latest` in the URL with the tag version, i.e. `v0.0.1`.

### Troubleshooting

If you get an error like "failed to set power state", or "Permission denied", you may need to install the udev rules for when the USB plugs in. 

```
wget https://raw.githubusercontent.com/IntelRealSense/librealsense/7a7c2bcfbc03d45154ad63fa76b221b2bb9d228f/config/99-realsense-libusb.rules
sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/ 
sudo udevadm control --reload-rules 
sudo udevadm trigger
```

You can also look at the official RealSense troubleshooting guide [here](https://github.com/IntelRealSense/librealsense/wiki/Troubleshooting-Q%26A#q-i-ran-the-udev-rules-script-but-linux-still-get-permission-denied).

The intelrealgrpcserver takes advantage of faster USB ports. Use the (blue) USB 3.0 port on the Raspberry Pi for faster streaming and access to more resolution options.

## Attributes and Sample Config

The attributes for the module are as follows:
- `sensors` (required): a list that contain the strings `color` and/or `depth`. The sensor that comes first in the list is designated the "main sensor" and will be the image that gets returned by `get_image` calls and what will appear in the Control tab on app.viam.
- `width_px`, `height_px`: the width and height of the output images. If the RealSense cannot produce the requested resolution, the component will fail to be built.
- `little_endian_depth`: a bool that specifices whether raw depth data should be encoded in a little-endian byte order. By default it is `false`, and encodes the raw depth data in a big-endian byte order.
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
      },
      "namespace": "rdk",
      "type": "camera",
      "model": "viam:camera:realsense"
    }
  ],
  "modules": [
    {
      "executable_path": "/home/user/viam-camera-realsense",
      "name": "intel"
    }
  ],
}
```

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

