# Toolchain-of-record for the darwin legs; pass with -pr:a so package_ids
# match across CI and local builds. Composes on the environment's default
# profile. Linux builds use the cpp-sdk-conan images' baked default instead.
include(default)

[settings]
build_type=Release
compiler.cppstd=17
# os.version deliberately unset until the SDK's pinned publish workflow
# ships binaries with os.version=14.0 -- it participates in the package_id.
arch=armv8
compiler=apple-clang
compiler.version=21
compiler.libcxx=libc++
