#!/usr/bin/env bash
#
# Build and package the module as module.tar.gz. Expects `conan` on PATH
# with a default profile and the viamconan remote configured.

set -euxo pipefail

cd "$(dirname "$0")/.."

# Linux relies on the cpp-sdk-conan images' baked default profile.
if [ "$(uname -s)" = "Darwin" ]; then
    PROFILE=./etc/conan/macos.profile
else
    PROFILE=default
fi

conan create . \
    -o "&:with_tests=False" \
    -pr:a "${PROFILE}" \
    -c tools.system.package_manager:mode=install \
    --build=missing

conan install --requires=viam-camera-realsense/0.0.1 \
    -pr:a "${PROFILE}" \
    --lockfile-partial \
    --deployer-package "&" \
    --envs-generation false
