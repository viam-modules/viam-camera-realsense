import os
import tarfile
import re
import json
from tempfile import TemporaryDirectory

from conan import ConanFile
from conan.api.output import ConanOutput
from conan.tools.build import check_min_cppstd
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy, load
from conan.internal.deploy import _flatten_directory

class ViamRealsense(ConanFile):
    name = "viam-camera-realsense"

    license = "Apache-2.0"
    url = "https://github.com/viam-modules/viam-camera-realsense"
    package_type = "application"
    settings = "os", "compiler", "build_type", "arch"
    options = {"with_tests": [True, False]}
    default_options = {
        "with_tests": False,
        "viam-cpp-sdk/*:shared": False,
        "libzip/*:shared": False
    }

    exports_sources = "CMakeLists.txt", "LICENSE", "src/*", "cmake/*", "meta.json", "test/*", "*.sh", "99-realsense-libusb.rules"

    version = "0.0.1"

    def set_version(self):
        content = load(self, "CMakeLists.txt")
        self.version = re.search("set\(CMAKE_PROJECT_VERSION (.+)\)", content).group(1).strip()

    def validate(self):
        check_min_cppstd(self, 17)

    def requirements(self):
        self.requires("viam-cpp-sdk/0.31.0")
        self.requires("librealsense/2.57.7")
        self.requires("libjpeg-turbo/[>=2.1.0 <3]")
        self.requires("libcurl/[>=8.0.0 <9]")
        self.requires("libzip/1.11.1")
        # Pin to prebuilt versions; the SDK's floating ranges drifted to
        # newer releases that aren't prebuilt, forcing slow source builds.
        self.requires("grpc/1.72.0", override=True)
        self.requires("protobuf/5.27.0", override=True)
        self.requires("abseil/20250127.0", override=True)
        self.requires("re2/20230301", override=True)
        self.requires("openssl/3.6.0", override=True)
        self.requires("zlib/1.3.1", override=True)
        self.requires("xtensor/[>=0.24.3 <0.27.0]", override=True)

    def layout(self):
        cmake_layout(self, src_folder=".")

    def generate(self):
        tc = CMakeToolchain(self)
        tc.variables["VIAM_REALSENSE_ENABLE_TESTS"] = self.options.with_tests
        tc.generate()

        CMakeDeps(self).generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def deploy(self):
        with TemporaryDirectory(dir=self.deploy_folder) as tmp_dir:
            self.output.debug(f"Creating temporary directory {tmp_dir}")

            self.output.info("Deploying necessary files to module.tar.gz")

            # Copy the main binary to bin/
            copy(self, "viam-camera-realsense", src=self.package_folder, dst=tmp_dir)

            # Copy meta.json to root
            copy(self, "meta.json", src=self.package_folder, dst=tmp_dir)

            # Copy udev rules and install scripts
            for pat in ["*.sh", "99-realsense-libusb.rules"]:
                copy(self, pat, src=self.package_folder, dst=tmp_dir)

            self.output.info("Creating module.tar.gz")
            with tarfile.open(os.path.join(self.deploy_folder, "module.tar.gz"), "w|gz") as tar:
                tar.add(tmp_dir, arcname=".", recursive=True)

                self.output.info("module.tar.gz contents:")
                for mem in tar.getmembers():
                    self.output.info(mem.name)
