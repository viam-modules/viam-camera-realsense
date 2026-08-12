# Toolchain-of-record; pass with -pr:a so package_ids match across CI and
# the cloud builder. Composes on the environment's default profile.
include(default)

[settings]
build_type=Release
compiler.cppstd=17

[conf]
tools.system.package_manager:mode=install
