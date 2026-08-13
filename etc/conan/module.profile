# Toolchain-of-record; pass with -pr:a so package_ids match across CI and
# the cloud builder. Composes on the environment's default profile.
include(default)

[settings]
build_type=Release
compiler.cppstd=17
{% if platform.system() == "Darwin" %}
os.version=14.0
arch=armv8
compiler=apple-clang
compiler.version=21
compiler.libcxx=libc++
{% endif %}

[conf]
tools.system.package_manager:mode=install
