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

default: camera-module

format: *.cpp
	clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4, ColumnLimit: 100}" *.cpp

all: default

clean:
	rm -rf viam-camera-realsense

clean-all: clean
	git clean -fxd

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
run-docker: build
	docker run \
		--device /dev/fuse \
		--cap-add SYS_ADMIN \
		-it viam-camera-realsense:$(TAG_VERSION)

package:
	cd packaging/appimages && \
	appimage-builder --recipe viam-camera-realsense-aarch64.yml

# Copies binary and AppImage from container to host.
copy-bin:
	rm -rf bin | true && \
	mkdir -p bin && \
	docker rm viam-camera-realsense-bin | true && \
	docker run -d -it --name viam-camera-realsense-bin viam-camera-realsense:$(TAG_VERSION) && \
	docker exec --workdir /root/opt/src/viam-camera-realsense viam-camera-realsense-bin make package
	docker cp viam-camera-realsense-bin:/root/opt/src/viam-camera-realsense/packaging/appimages/viam-camera-realsense-latest-aarch64.AppImage ./bin && \
	docker stop viam-camera-realsense-bin && \
	docker rm viam-camera-realsense-bin

appimage: build copy-bin


