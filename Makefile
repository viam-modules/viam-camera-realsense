OUTPUT_NAME = viam-camera-realsense
BIN := build-conan/build/RelWithDebInfo/viam-camera-realsense

# OS Detection
UNAME_S := $(shell uname -s)
UNAME_M := $(shell uname -m)

# Set architecture-specific paths based on OS
ifeq ($(UNAME_S),Linux)
    export DEFAULT_PKG_CONFIG_PATH := /usr/lib/$(UNAME_M)-linux-gnu/pkgconfig:/usr/share/pkgconfig
    NPROC := $(shell nproc)
else ifeq ($(UNAME_S),Darwin)
    # macOS Homebrew paths
    ifeq ($(UNAME_M),arm64)
        export DEFAULT_PKG_CONFIG_PATH := /opt/homebrew/lib/pkgconfig:/usr/local/lib/pkgconfig
    else
        export DEFAULT_PKG_CONFIG_PATH := /usr/local/lib/pkgconfig
    endif
    NPROC := $(shell sysctl -n hw.ncpu)
endif

# Export for sub-processes (like bin/build.sh), does not pollute the parent shell
# Only add the colon if PKG_CONFIG_PATH is already set
ifeq ($(PKG_CONFIG_PATH),)
    export PKG_CONFIG_PATH := $(DEFAULT_PKG_CONFIG_PATH)
else
    export PKG_CONFIG_PATH := $(PKG_CONFIG_PATH):$(DEFAULT_PKG_CONFIG_PATH)
endif
 
# Common Conan settings to ensure binary cache hits across all build flows
export CONAN_FLAGS := -s:a build_type=Release -s:a compiler.cppstd=17

.PHONY: build setup test clean lint conan-pkg conan-build-test conan-install-test build-native test-native test-coverage

default: module.tar.gz

conan-build-test:
	test -f ./venv/bin/activate && . ./venv/bin/activate; \
	conan build . \
	-o "&:with_tests=True" \
	--output-folder=build-conan \
	--build=none \
	$(CONAN_FLAGS)

conan-install-test:
	test -f ./venv/bin/activate && . ./venv/bin/activate; \
	conan install . \
	-o "&:with_tests=True" \
	--output-folder=build-conan \
	--build=missing \
	$(CONAN_FLAGS)

conan-build-test-coverage:
	test -f ./venv/bin/activate && . ./venv/bin/activate; \
	conan build . \
	-o "&:with_tests=True" \
	--output-folder=build-conan \
	--build=none \
	$(CONAN_FLAGS) \
	-c tools.cmake.cmaketoolchain:user_toolchain="['$(CURDIR)/cmake/coverage_toolchain.cmake']"

test: conan-install-test conan-build-test
	cd build-conan/build/Release && . ./generators/conanrun.sh && ctest --output-on-failure

test-coverage: conan-install-test
	@echo "Building with coverage enabled..."
	@command -v lcov >/dev/null 2>&1 || { echo >&2 "lcov is not installed. Install it with: sudo apt-get install lcov"; exit 1; }
	test -f ./venv/bin/activate && . ./venv/bin/activate; \
	conan build . \
	-o "&:with_tests=True" \
	--output-folder=build-conan \
	--build=missing \
	$(CONAN_FLAGS) \
	-c tools.build:cxxflags="['--coverage']" \
	-c tools.build:sharedlinkflags="['--coverage']" \
	-c tools.build:exelinkflags="['--coverage']"
	@echo "Running tests..."
	cd build-conan/build/Release && . ./generators/conanrun.sh && ctest --output-on-failure
	@echo "Generating coverage report..."
	lcov --capture --directory build-conan/build/Release --output-file build-conan/coverage.info
	lcov --remove build-conan/coverage.info '/usr/*' '*/test/*' '*/build-conan/build/Release/_deps/*' --output-file build-conan/coverage.info
	@echo "\n=== Coverage Summary ==="
	lcov --list build-conan/coverage.info

# Native build targets for CI environments with pre-installed dependencies
build-native:
	mkdir -p build-native && cd build-native && \
	cmake .. -DVIAM_REALSENSE_ENABLE_TESTS=ON -DCMAKE_BUILD_TYPE=Release \
	$(if $(wildcard $(CURDIR)/build-conan/build/Release/generators/conan_toolchain.cmake),-DCMAKE_TOOLCHAIN_FILE=$(CURDIR)/build-conan/build/Release/generators/conan_toolchain.cmake -DCMAKE_PREFIX_PATH=$(CURDIR)/build-conan/build/Release/generators) && \
	make -j$(NPROC)

test-native: build-native
	cd build-native && ctest --output-on-failure

clean:
	rm -rf build-conan build-native module.tar.gz venv

setup:
	bin/setup.sh

# Both the commands below need to source/activate the venv in the same line as the
# conan call because every line of a Makefile runs in a subshell
conan-pkg:
	test -f ./venv/bin/activate && . ./venv/bin/activate; \
	conan create . \
	-o "&:with_tests=False" \
	$(CONAN_FLAGS) \
	--build=missing

module.tar.gz: conan-pkg meta.json
	test -f ./venv/bin/activate && . ./venv/bin/activate; \
	conan install --requires=viam-camera-realsense/0.0.1 \
	$(CONAN_FLAGS) \
	--lockfile-partial \
	--deployer-package "&" \
	--envs-generation false

lint:
	./bin/lint.sh


# Docker
BUILD_CMD = docker buildx build --pull $(BUILD_PUSH) --force-rm --no-cache --build-arg MAIN_TAG=$(MAIN_TAG) --build-arg BASE_TAG=$(BUILD_TAG) --platform linux/$(BUILD_TAG) -f $(BUILD_FILE) -t '$(MAIN_TAG):$(BUILD_TAG)' .
BUILD_PUSH = --load
BUILD_FILE = ./etc/Dockerfile.debian.bookworm

docker: docker-build docker-upload

docker-build: docker-arm64

docker-arm64: MAIN_TAG = ghcr.io/viam-modules/viam-camera-realsense
docker-arm64: BUILD_TAG = arm64
docker-arm64:
	$(BUILD_CMD)

docker-upload:
	docker push 'ghcr.io/viam-modules/viam-camera-realsense:arm64'

# CI targets that automatically push, avoid for local test-first-then-push workflows
docker-arm64-ci: MAIN_TAG = ghcr.io/viam-modules/viam-camera-realsense
docker-arm64-ci: BUILD_TAG = arm64
docker-arm64-ci: BUILD_PUSH = --push
docker-arm64-ci:
	$(BUILD_CMD)

docker-amd64-ci: MAIN_TAG = ghcr.io/viam-modules/viam-camera-realsense
docker-amd64-ci: BUILD_TAG = amd64
docker-amd64-ci: BUILD_PUSH = --push
docker-amd64-ci:
	$(BUILD_CMD)
