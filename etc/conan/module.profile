# Toolchain-of-record; pass with -pr:a so package_ids match across CI and
# the cloud builder. Composes on the environment's default profile.
include(default)

[settings]
build_type=Release
compiler.cppstd=17
{% if platform.system() == "Darwin" %}
{# os.version deliberately unset until the SDK's pinned publish workflow
   ships binaries with os.version=14.0 — it participates in the package_id #}
arch=armv8
compiler=apple-clang
compiler.version=21
compiler.libcxx=libc++
{% endif %}

[conf]
tools.system.package_manager:mode=install
