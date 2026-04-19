#!/bin/bash
set -euo pipefail

if [ $(whoami) != root ]; then
    echo "Please run this script with sudo"
    exit 1
fi

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

if [ "$(uname -s)" != "Darwin" ]; then
    cp "$CURR_DIR/99-realsense-libusb.rules" /etc/udev/rules.d/99-realsense-libusb.rules
    echo "udev rules installed at /etc/udev/rules.d/99-realsense-libusb.rules"
    udevadm control --reload-rules && udevadm trigger
    echo "udev rules reloaded successfully"
fi
