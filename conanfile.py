import re

from conan import ConanFile
from conan.tools.build import check_min_cppstd
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.files import copy, load

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
        self.requires("viam-cpp-sdk/0.40.0")
        self.requires("librealsense/2.57.7")
        self.requires("libjpeg-turbo/[>=2.1.0 <3]")
        self.requires("libcurl/[>=8.0.0 <9]")
        self.requires("libzip/1.11.1")

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

        # CPack assembles module.tar.gz from the CMake install rules
        cmake.build(target="package")
        copy(self, "module.tar.gz", src=self.build_folder, dst=self.package_folder)

    def deploy(self):
        copy(self, "module.tar.gz", src=self.package_folder, dst=self.deploy_folder)
