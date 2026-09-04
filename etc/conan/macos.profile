# Toolchain-of-record for the darwin legs; pass with -pr:a so package_ids
# match across CI and local builds. Composes on the environment's default
# profile. Linux builds use the cpp-sdk-conan images' baked default instead.
include(default)

[settings]
build_type=Release
compiler.cppstd=17
# set to match SDK's version
os.version=14.0
arch=armv8
compiler=apple-clang
compiler.version=21
compiler.libcxx=libc++
