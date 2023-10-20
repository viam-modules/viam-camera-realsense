# format the source code
format: src/*.cpp src/*.hpp
	ls src/*.cpp src/*.hpp | xargs clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4, ColumnLimit: 100}"

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

TAG_VERSION?=latest
# build the AppImages
appimage: export TAG_NAME = ${TAG_VERSION}
appimage: viam-camera-realsense
	cd packaging/appimages && \
	mkdir -p deploy && \
	rm -f deploy/viam-camera-realsense* && \
	appimage-builder --recipe viam-camera-realsense-aarch64.yml
	cp ./packaging/appimages/viam-camera-realsense-*-aarch64.AppImage  ./packaging/appimages/deploy/

integration-appimage: export TAG_NAME = ${TAG_VERSION}
integration-appimage: realsense-integration-tests
	cd packaging/appimages && \
	mkdir -p deploy && \
	rm -f deploy/realsense-integration-tests* && \
	appimage-builder --recipe realsense-integration-tests-aarch64.yml
	cp ./packaging/appimages/realsense-integration-tests-*-aarch64.AppImage  ./packaging/appimages/deploy/


