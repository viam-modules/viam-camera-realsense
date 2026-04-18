#!/usr/bin/env bash

set -euo pipefail

OS=$(uname)

if [[ "$OS" == 'Linux' ]]; then
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd -P)"
    sudo "$SCRIPT_DIR/install_udev_rules.sh"
fi
