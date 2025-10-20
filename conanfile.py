
import os
import tarfile
from tempfile import TemporaryDirectory

from conan import ConanFile
from conan.api.output import ConanOutput
from conan.tools.build import check_min_cppstd
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy
from conan.internal.deploy import _flatten_directory

class ViamRealsense(ConanFile):
    name = "viam-camera-realsense"

    license = "Apache-2.0"
    url = "https://github.com/viam-modules/viam-camera-realsense"
    package_type = "application"
    settings = "os", "compiler", "build_type", "arch"

    exports_sources = "CMakeLists.txt", "LICENSE", "src/*", "cmake/*", "meta.json"

    version = "0.0.1"

    def validate(self):
        check_min_cppstd(self, 17)

    def requirements(self):
        self.requires("viam-cpp-sdk/0.20.1")
        self.requires("librealsense/2.56.5")
        self.requires("libjpeg-turbo/[>=2.1.0 <3]")

    def layout(self):
        cmake_layout(self, src_folder=".")

    def generate(self):
        tc = CMakeToolchain(self)
        tc.cache_variables["VIAM_REALSENSE_ENABLE_TESTS"] = False
        tc.generate()

        CMakeDeps(self).generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        CMake(self).install()

        # Use CPack to build the module.tar.gz and manually copy it to the package folder
        CMake(self).build(target='package')
        copy(self, pattern="module.tar.gz", src=self.build_folder, dst=self.package_folder)

    def deploy(self):
        copy(self, pattern="module.tar.gz", src=self.package_folder, dst=self.deploy_folder)
