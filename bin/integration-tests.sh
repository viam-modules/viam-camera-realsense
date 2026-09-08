#!/usr/bin/env bash
#
# Build and run the software-device integration tests: the ctest label
# "integration" (test/integration_tests.cpp). No camera needed.
#
# Expects the conan-generated CMake preset from a prior
#   conan install . -o "&:with_tests=True" --build=missing [-pr:a <profile>]
#
# Usage:
#   ./bin/integration-tests.sh                 # all integration tests
#   ./bin/integration-tests.sh -R PointCloud   # extra args go to ctest
#   BUILD_DIR=build-asan/build/Release ./bin/integration-tests.sh
#
# Environment:
#   PRESET     CMake configure preset (default conan-release)
#   BUILD_DIR  build tree the preset configured (default build/Release)
#   JOBS       parallel build jobs (default: all cores)

set -euo pipefail

cd "$(dirname "$0")/.."

PRESET="${PRESET:-conan-release}"
BUILD_DIR="${BUILD_DIR:-build/Release}"
JOBS="${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)}"

if [ ! -f "${BUILD_DIR}/CMakeCache.txt" ]; then
    cmake --preset "${PRESET}"
fi

# gtest discovery runs at ctest time for every test binary, so build them all
# rather than just integration_tests.
cmake --build "${BUILD_DIR}" -j "${JOBS}"

# Cases are independent processes; -j overlaps the ~2 s librealsense context
# teardown each one pays on macOS (see the header of test/integration_tests.cpp).
ctest --test-dir "${BUILD_DIR}" -L integration -j "${JOBS}" --output-on-failure "$@"
