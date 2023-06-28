CWD := .
BUILD_DIR := $(cwd)/build
INSTALL_DIR := $(BUILD_DIR)/AppDir

# Docker
L4T_VERSION := 35.3.1

# Module
# Builds/installs module.
.PHONY: build
build:
	rm -rf $(BUILD_DIR) | true && \
	mkdir -p build && \
	cd build && \
	cmake -DCMAKE_INSTALL_PREFIX=$(INSTALL_DIR) .. -G Ninja && \
	ninja -j $(shell nproc) && \
	sudo ninja install -j $(shell nproc)

# Creates appimage cmake build.
package:
	cd etc && \
	appimage-builder --recipe viam-realsense-module-aarch64.yml

# Builds docker image with viam-cpp-sdk and camera-realsense installed.
image:
	rm -rf build | true && \
	docker build -t viam-realsense:$(L4T_VERSION) \
		--memory=16g \
		--build-arg TAG=$(L4T_VERSION) \
		-f ./etc/Dockerfile.debian.bookworm ./

# Runs docker image with shell.
docker-module:
	docker run \
		--device /dev/fuse \
		--cap-add SYS_ADMIN \
		-it viam-realsense:$(L4T_VERSION)

# Copies binary and AppImage from container to host.
bin-module:
	rm -rf bin | true && \
	mkdir -p bin && \
	docker rm viam-realsense-bin | true && \
	docker run -d -it --name viam-realsense-bin viam-realsense:$(L4T_VERSION) && \
	docker cp viam-realsense-bin:/root/opt/src/viam-realsense/etc/viam-realsense-0.0.1-aarch64.AppImage ./bin && \
	docker cp viam-realsense-bin:/root/opt/src/viam-realsense/build/realsense-mr ./bin && \
	docker stop viam-realsense-bin

# SDK
.PHONY: build-sdk
build-sdk:
	cd viam-cpp-sdk && \
	mkdir -p build && \
	cd build && \
	cmake -DVIAMCPPSDK_USE_DYNAMIC_PROTOS=ON -DVIAMCPPSDK_OFFLINE_PROTO_GENERATION=ON .. -G Ninja && \
	ninja -j $(shell nproc) && \
	sudo ninja install -j $(shell nproc))

docker-sdk:
	docker build -t viam-cpp-sdk -f ./viam-cpp-sdk/etc/docker/Dockerfile.debian.bookworm ./ && \
	docker run -it viam-cpp-sdk /bin/bash

BIN_OUTPUT_PATH = bin/$(shell uname -s)-$(shell uname -m)

TOOL_BIN = bin/tools/$(shell uname -s)-$(shell uname -m)

PATH_WITH_TOOLS="`pwd`/$(TOOL_BIN):${PATH}"

UNAME := $(shell uname)

ifeq ($(UNAME), Darwin)
ifneq ($(shell which brew), )
   PKG_CONFIG_PATH_EXTRA=$(PKG_CONFIG_PATH):/usr/local/lib/pkgconfig:$(shell find $(shell which brew > /dev/null && brew --prefix) -name openssl.pc | head -n1 | xargs dirname)
endif
endif

LIB_FLAGS = $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH):$(PKG_CONFIG_PATH_EXTRA) pkg-config --cflags realsense2 --libs libturbojpeg realsense2 viam-cpp-sdk-libviamsdk)
GCC_FLAGS = -pthread -Wl,-ldl

ifeq ($(UNAME), Darwin)
	# There's a strange bug in the realsense pc file that adds this when it's not necessary at all
	# when brew is in use. This may break something in the future.
   LIB_FLAGS := $(LIB_FLAGS:-L/usr/local/lib/x86_64-linux-gnu=)
endif


ifeq ($(shell arch), x86_64)
   GCC_FLAGS += -mpclmul -msse2 -msse4.2
endif

THIRD_PARTY_SOURCES = third_party/fpng.cpp third_party/lodepng.cpp

default: intelrealgrpcserver-release-opt

format: *.cpp
	clang-format -i --style="{BasedOnStyle: Google, IndentWidth: 4, ColumnLimit: 100}" *.cpp

all: default

clean:
	rm -rf intelrealgrpcserver

clean-all: clean
	git clean -fxd

setup:
	sudo apt install -y libturbojpeg-dev protobuf-compiler-grpc libgrpc-dev libgrpc++-dev || brew install jpeg-turbo grpc openssl --quiet

$(TOOL_BIN)/buf:
	mkdir -p $(TOOL_BIN)
	GOBIN=`pwd`/$(TOOL_BIN) go install github.com/bufbuild/buf/cmd/buf@v1.13.1

$(TOOL_BIN)/protoc-gen-grpc-cpp:
	ln -sf `which grpc_cpp_plugin` $(TOOL_BIN)/protoc-gen-grpc-cpp

SERVER_TARGETS = $(THIRD_PARTY_SOURCES) camera_realsense.cpp
CPP_COMPILER = g++
CPP_FLAGS = -std=c++17 -o intelrealgrpcserver camera_realsense.cpp $(THIRD_PARTY_SOURCES) $(LIB_FLAGS) $(GCC_FLAGS)

intelrealgrpcserver: $(SERVER_TARGETS)
	$(CPP_COMPILER) $(CPP_FLAGS_EXTRA) $(CPP_FLAGS)

camera-module: $(SERVER_TARGETS)
	$(CPP_COMPILER) -std=c++17 -o camera_module camera_realsense.cpp third_party/fpng.cpp third_party/lodepng.cpp -I/usr/local/include -I/usr/local/include/viam/api -L/usr/local/lib -L/usr/lib/x86_64-linux-gnu -lturbojpeg -lrealsense2 -lviamsdk -lviam_rust_utils -lviamapi -lgrpc++ -lgrpc -lgpr -lprotobuf -pthread -Wl,-ldl

intelrealgrpcserver-debug: CPP_FLAGS_EXTRA = -pg
intelrealgrpcserver-debug: intelrealgrpcserver

intelrealgrpcserver-release: intelrealgrpcserver

intelrealgrpcserver-release-opt: CPP_FLAGS_EXTRA = -O3
intelrealgrpcserver-release-opt: intelrealgrpcserver

appimages: clean default
	cd packaging/appimages && appimage-builder --recipe intelrealgrpcserver-`uname -m`.yml
	mkdir -p packaging/appimages/deploy/
	mv packaging/appimages/*.AppImage* packaging/appimages/deploy/
	chmod 755 packaging/appimages/deploy/*.AppImage

appimages-deploy:
	gsutil -m -h "Cache-Control: no-cache" cp packaging/appimages/deploy/* gs://packages.viam.com/apps/camera-servers/
