CWD := .
BUILD_DIR := $(cwd)/build
INSTALL_DIR := $(BUILD_DIR)/AppDir

# Docker
TAG_VERSION := latest

# Module
# Creates appimage cmake build.
# Builds docker image with viam-cpp-sdk and camera-realsense installed.
.PHONY: build
build:
	docker build -t viam-camera-realsense:$(TAG_VERSION) \
		--memory=16g \
		--build-arg TAG=$(TAG_VERSION) \
		-f ./etc/Dockerfile.debian.bookworm ./

# Runs docker image with shell.
run-docker:
	docker run \
		--device /dev/fuse \
		--cap-add SYS_ADMIN \
		-it --name camera-realsense viam-camera-realsense:$(TAG_VERSION)

package:
	cd etc && \
	appimage-builder --recipe viam-camera-realsense-aarch64.yml


# Copies binary and AppImage from container to host.
bin-module:
	rm -rf bin | true && \
	mkdir -p bin && \
	docker rm viam-camera-realsense-bin | true && \
	docker run -d -it --name viam-camera-realsense-bin viam-camera-realsense:$(TAG_VERSION) && \
	docker cp viam-camera-realsense-bin:/root/opt/src/viam-camera-realsense/etc/viam-camera-realsense-latest-aarch64.AppImage ./bin && \
	docker stop viam-camera-realsense-bin && \
	docker rm viam-camera-realsense-bin

appimage: build bin-module

# SDK
.PHONY: build-sdk
build-sdk:
	cd viam-cpp-sdk && \
	mkdir -p build && \
	cd build && \
	cmake -DVIAMCPPSDK_USE_DYNAMIC_PROTOS=ON -DVIAMCPPSDK_OFFLINE_PROTO_GENERATION=ON .. -G Ninja && \
	ninja -j $(shell nproc) && \
	sudo ninja install -j $(shell nproc))

run-sdk:
	docker build -t viam-cpp-sdk -f ./viam-cpp-sdk/etc/docker/Dockerfile.debian.bookworm ./ && \
	docker run -it viam-cpp-sdk /bin/bash


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

camera-module: $(SERVER_TARGETS)
	$(CPP_COMPILER) -std=c++17 -o viam-camera-realsense camera_realsense.cpp $(THIRD_PARTY_SOURCES) $(SDK_INCLUDE) $(LIB_FLAGS) $(SDK_FLAGS) $(GCC_FLAGS)

BIN_OUTPUT_PATH = bin/$(shell uname -s)-$(shell uname -m)

TOOL_BIN = bin/tools/$(shell uname -s)-$(shell uname -m)

PATH_WITH_TOOLS="`pwd`/$(TOOL_BIN):${PATH}"

UNAME := $(shell uname)


default: camera-module

format: *.cpp
	clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4, ColumnLimit: 100}" *.cpp

all: default

clean:
	rm -rf viam-camera-realsense

clean-all: clean
	git clean -fxd

