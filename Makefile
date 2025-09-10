default: viam-camera-realsense

lint:
	./bin/run-clang-format.sh

build: lint
	mkdir -p build

build/build.ninja: build CMakeLists.txt
	cd build && cmake -G Ninja -DVIAM_REALSENSE_ENABLE_SANITIZER=$(SANITIZE) -DCMAKE_BUILD_TYPE=RelWithDebInfo ..

SANITIZE ?= OFF
viam-camera-realsense: src/* build/build.ninja
	cd build && ninja viam-camera-realsense -j 4
	cp build/viam-camera-realsense .

all: default

clean:
	rm -rf viam-camera-realsense build

clean-all: clean
	git clean -fxd

test: build/build.ninja
	cd build && ninja -j 4 test/all && ctest --output-on-failure

.PHONY: build lint

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

TAG_VERSION?=latest
# Define a function for building AppImages
define BUILD_APPIMAGE
    export TAG_NAME=$(TAG_VERSION); \
    cd packaging/appimages && \
    mkdir -p deploy && \
    rm -f deploy/$(1)* && \
    appimage-builder --recipe $(1)-$(2).yml
endef

# Targets for building AppImages
appimage-arm64: export OUTPUT_NAME = viam-camera-realsense
appimage-arm64: export ARCH = aarch64
appimage-arm64: viam-camera-realsense
	$(call BUILD_APPIMAGE,$(OUTPUT_NAME),$(ARCH))
	cp ./packaging/appimages/$(OUTPUT_NAME)-*-$(ARCH).AppImage ./packaging/appimages/deploy/

appimage-amd64: export OUTPUT_NAME = viam-camera-realsense
appimage-amd64: export ARCH = x86_64
appimage-amd64: viam-camera-realsense
	$(call BUILD_APPIMAGE,$(OUTPUT_NAME),$(ARCH))
	cp ./packaging/appimages/$(OUTPUT_NAME)-*-$(ARCH).AppImage ./packaging/appimages/deploy/
