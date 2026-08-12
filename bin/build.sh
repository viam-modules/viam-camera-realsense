#!/usr/bin/env bash
#
# Build and package the module as module.tar.gz. Expects `conan` on PATH
# with a default profile and the viamconan remote configured.

set -euxo pipefail

cd "$(dirname "$0")/.."

PROFILE=./etc/conan/module.profile

conan create . \
    -o "&:with_tests=False" \
    -pr:a "${PROFILE}" \
    --build=missing

conan install --requires=viam-camera-realsense/0.0.1 \
    -pr:a "${PROFILE}" \
    --lockfile-partial \
    --deployer-package "&" \
    --envs-generation false
