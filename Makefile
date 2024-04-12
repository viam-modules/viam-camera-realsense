# format the source code
format: src/*.cpp src/*.hpp test/*.cpp
	ls src/*.cpp src/*.hpp test/*.cpp | xargs clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4, ColumnLimit: 100}"

viam-camera-realsense: src/*
	rm -rf build/ && \
	mkdir build && \
	cd build && \
	cmake -G Ninja .. && \
	ninja all -j 4 && \
	cp viam-camera-realsense ../

realsense-integration-tests: integration/tests/*
	cd integration && \
	go test -c -o realsense-integration-tests ./tests/ && \
	cp realsense-integration-tests ../

default: viam-camera-realsense

all: default

clean:
	rm -rf viam-camera-realsense

clean-all: clean
	git clean -fxd

# Docker
BUILD_CMD = docker buildx build --pull $(BUILD_PUSH) --force-rm --no-cache --build-arg MAIN_TAG=$(MAIN_TAG) --build-arg BASE_TAG=$(BUILD_TAG) --platform linux/$(BUILD_TAG) -f $(BUILD_FILE) -t '$(MAIN_TAG):$(BUILD_TAG)' .
BUILD_PUSH = --load
BUILD_FILE = ./etc/Dockerfile.debian.bookworm

docker: docker-build docker-upload

docker-build: docker-arm64

docker-arm64: MAIN_TAG = ghcr.io/viamrobotics/viam-camera-realsense
docker-arm64: BUILD_TAG = arm64
docker-arm64:
	$(BUILD_CMD)

docker-upload:
	docker push 'ghcr.io/viamrobotics/viam-camera-realsense:arm64'

# CI targets that automatically push, avoid for local test-first-then-push workflows
docker-arm64-ci: MAIN_TAG = ghcr.io/viamrobotics/viam-camera-realsense
docker-arm64-ci: BUILD_TAG = arm64
docker-arm64-ci: BUILD_PUSH = --push
docker-arm64-ci:
	$(BUILD_CMD)

docker-amd64-ci: MAIN_TAG = ghcr.io/viamrobotics/viam-camera-realsense
docker-amd64-ci: BUILD_TAG = amd64
docker-amd64-ci: BUILD_PUSH = --push
docker-amd64-ci:
	$(BUILD_CMD)

TAG_VERSION?=latest
# AppImages
define build_appimage
	cd packaging/appimages && \
	mkdir -p packaging/appimages/deploy && \
	rm -f packaging/appimages/deploy/$1* && \
	appimage-builder --recipe packaging/appimages/$2 && \
	cp packaging/appimages/$1* packaging/appimages/deploy/
endef

appimage-arm64: export TAG_NAME = ${TAG_VERSION}
appimage-arm64: viam-camera-realsense
	$(call build_appimage,viam-camera-realsense-aarch64.AppImage,viam-camera-realsense-aarch64.yml)

appimage-amd64: export TAG_NAME = ${TAG_VERSION}
appimage-amd64: viam-camera-realsense
	$(call build_appimage,viam-camera-realsense-x86_64.AppImage,viam-camera-realsense-x86_64.yml)

integration-appimage-arm64: export TAG_NAME = ${TAG_VERSION}
integration-appimage-arm64: realsense-integration-tests
	$(call build_appimage,realsense-integration-tests-aarch64.AppImage,realsense-integration-tests-aarch64.yml)

integration-appimage-amd64: export TAG_NAME = ${TAG_VERSION}
integration-appimage-amd64: realsense-integration-tests
	$(call build_appimage,realsense-integration-tests-x86_64.AppImage,realsense-integration-tests-x86_64.yml)

# Phony targets
.PHONY: format viam-camera-realsense realsense-integration-tests docker docker-build docker-arm64 docker-upload docker-arm64-ci docker-amd64-ci appimage-arm64 appimage-amd64 integration-appimage-arm64 integration-appimage-amd64