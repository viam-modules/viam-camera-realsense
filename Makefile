BIN_OUTPUT_PATH = bin/$(shell uname -s)-$(shell uname -m)

TOOL_BIN = bin/tools/$(shell uname -s)-$(shell uname -m)

PATH_WITH_TOOLS="`pwd`/$(TOOL_BIN):${PATH}"

UNAME := $(shell uname)

ifeq ($(UNAME), Darwin)
ifneq ($(shell which brew), )
   PKG_CONFIG_PATH_EXTRA=$(PKG_CONFIG_PATH):/usr/local/lib/pkgconfig:$(shell find $(shell which brew > /dev/null && brew --prefix) -name openssl.pc | head -n1 | xargs dirname)
endif
endif

LIB_FLAGS = $(shell PKG_CONFIG_PATH=$(PKG_CONFIG_PATH):$(PKG_CONFIG_PATH_EXTRA) pkg-config --cflags grpc++ realsense2 --libs protobuf grpc++ libturbojpeg realsense2 viamsdk viamapi)
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
