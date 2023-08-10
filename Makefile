# compile the binary
SDK_LOCATION = /usr/local
CPP_COMPILER = g++
THIRD_PARTY_SOURCES = third_party/fpng.cpp third_party/lodepng.cpp
SERVER_TARGETS = $(THIRD_PARTY_SOURCES) camera_realsense.cpp

GCC_FLAGS = -O4 -pthread -Wredundant-move -Wpessimizing-move -Wl,-ldl
ifeq ($(shell arch), x86_64)
   GCC_FLAGS += -mpclmul -msse2 -msse4.2
endif

SDK_INCLUDE = -I$(SDK_LOCATION)/include -I$(SDK_LOCATION)/include/viam/api -L$(SDK_LOCATION)/lib
SDK_FLAGS = -lviamsdk -lviam_rust_utils -lviamapi 

LIB_FLAGS = $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH) pkg-config --cflags grpc++ realsense2 --libs protobuf grpc++ libturbojpeg realsense2)

viam-camera-realsense: $(SERVER_TARGETS)
	$(CPP_COMPILER) -std=c++17 -o viam-camera-realsense camera_realsense.cpp $(THIRD_PARTY_SOURCES) $(SDK_INCLUDE) $(LIB_FLAGS) $(SDK_FLAGS) $(GCC_FLAGS)

default: viam-camera-realsense

format: *.cpp
	clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4, ColumnLimit: 100}" *.cpp

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


