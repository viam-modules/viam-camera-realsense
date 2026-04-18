#!/bin/bash
set -euo pipefail

if [ $(whoami) != root ]; then
    echo "Please run this script with sudo"
    exit 1
fi

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

if [ "$(uname -s)" != "Darwin" ]; then
    # Install standard USB udev rules
    cp "$CURR_DIR/99-realsense-libusb.rules" /etc/udev/rules.d/99-realsense-libusb.rules
    echo "udev rules installed at /etc/udev/rules.d/99-realsense-libusb.rules"

    # Detect Tegra (Jetson) platform via device tree
    is_tegra=false
    if [ -f /proc/device-tree/compatible ] && grep -qi "tegra" /proc/device-tree/compatible 2>/dev/null; then
        is_tegra=true
    fi

    if [ "$is_tegra" = true ]; then
        echo "Tegra platform detected, installing MIPI/DFU rules"
        cp "$CURR_DIR/99-realsense-d4xx-mipi-dfu.rules" /etc/udev/rules.d/99-realsense-d4xx-mipi-dfu.rules
        cp "$CURR_DIR/rs-enum.sh" /usr/local/bin/rs-enum.sh
        cp "$CURR_DIR/rs_ipu6_d457_bind.sh" /usr/local/bin/rs_ipu6_d457_bind.sh
        chmod +x /usr/local/bin/rs-enum.sh /usr/local/bin/rs_ipu6_d457_bind.sh
        echo "MIPI/DFU rules installed for Tegra platform"
    fi

    udevadm control --reload-rules && udevadm trigger
    echo "udev rules reloaded successfully"
fi
