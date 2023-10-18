# compile the binary
format: src/*.cpp src/*.hpp
	clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4, ColumnLimit: 100}" src/*.cpp && \
	clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4, ColumnLimit: 100}" src/*.hpp

viam-camera-realsense: 
	rm -rf build/ && \
	mkdir build && \
	cd build && \
	cmake .. && \
	make && \
	cp viam-camera-realsense ../

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
# build the AppImage 
appimage: export TAG_NAME = ${TAG_VERSION}
appimage: viam-camera-realsense
	cd packaging/appimages && \
	rm -rf deploy && \
	mkdir -p deploy && \
	appimage-builder --recipe viam-camera-realsense-aarch64.yml
	cp ./packaging/appimages/viam-camera-realsense-*-aarch64.AppImage  ./packaging/appimages/deploy/


