#!/usr/bin/env bash
#
# Build and package the module as module.tar.gz. Expects `conan` on PATH
# with a default profile and the viamconan remote configured.

set -euxo pipefail

cd "$(dirname "$0")/.."

# Linux relies on the environment's default profile (the cpp-sdk-conan
# images bake one; the cloud builder detects one). The detected profile says
# cppstd=gnu17, so pin 17 or package_ids miss the published binaries.
if [ "$(uname -s)" = "Darwin" ]; then
    PROFILE=./etc/conan/macos.profile
else
    PROFILE=default
fi

conan create . \
    -o "&:with_tests=False" \
    -pr:a "${PROFILE}" \
    -s:a compiler.cppstd=17 \
    -c tools.system.package_manager:mode=install \
    --build=missing

conan install --requires=viam-camera-realsense/0.0.1 \
    -pr:a "${PROFILE}" \
    -s:a compiler.cppstd=17 \
    --lockfile-partial \
    --deployer-package "&" \
    --envs-generation false
