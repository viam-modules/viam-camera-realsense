
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

    exports_sources = "CMakeLists.txt", "LICENSE", "src/*", "cmake/*", "meta.json-no-appimage"

    version = "0.0.1"

    def validate(self):
        check_min_cppstd(self, 17)

    def requirements(self):
        self.requires("viam-cpp-sdk/0.19.0")
        self.requires("librealsense/2.56.5")
        self.requires("libjpeg-turbo/2.1.5")

    def layout(self):
        cmake_layout(self, src_folder=".")

    def generate(self):
        tc = CMakeToolchain(self)
        tc.cache_variables["VIAM_REALSENSE_ENABLE_TESTS"] = False
        tc.cache_variables["VIAM_REALSENSE_DISABLE_APPIMAGE"] = True
        tc.generate()

        CMakeDeps(self).generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        CMake(self).install()

    def deploy(self):
        with TemporaryDirectory(dir=self.deploy_folder) as tmp_dir:
            output = ConanOutput(scope="module_tgz")

            output.debug(f"Creating temporary directory {tmp_dir}")
            output.info("Copying installed files")

            copy(self, "*", src=os.path.join(self.package_folder, "bin"), dst=os.path.join(tmp_dir, "bin"))
            copy(self, "meta.json", src=self.package_folder, dst=tmp_dir)

            symlinks = self.conf.get("tools.deployer:symlinks", check_type=bool, default=True)

            for req, dep in self.dependencies.host.items():
                if not req.run:
                    continue
                if dep.package_folder is None:
                    continue
                cpp_info = dep.cpp_info.aggregated_components()

                for libdir in cpp_info.libdirs:
                    if not os.path.isdir(libdir):
                        continue

                    _flatten_directory(dep, libdir, os.path.join(tmp_dir, "lib"), symlinks, [".so*"])

            output.info("Creating module.tar.gz")
            with tarfile.open("module.tar.gz", "w|gz") as tar:
                tar.add(tmp_dir, ".")

                output.debug("module.tar.gz contents:")
                for mem in tar.getmembers():
                    output.debug(mem.name)

