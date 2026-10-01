#!/usr/bin/env bash
# Export vendored conan recipes into the local cache. Run before conan install/create.
set -euo pipefail
cd "$(dirname "$0")/.."

conan export etc/conan/recipes/librealsense --version 2.57.7
