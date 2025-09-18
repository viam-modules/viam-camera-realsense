if (LINUX)
   set(_pre_exclude_regex
        ".*ld-linux.*\\.so.*"
        ".*libc\\.so.*"
        ".*libdl\\.so.*"
        ".*libgcc.*\\.so.*"
        ".*libm\\.so.*"
        ".*libpthread\\.so.*"
        ".*libssl\\.so.*"
        ".*libstdc\\+\\+\\.so.*"
        ".*libz\\.so.*"
   )
endif()

if (APPLE)
    # For reasons unclear to me the install command below doesn't work without this
    set(_framework_dest_arg "FRAMEWORK" "DESTINATION" "lib")
endif()

install(
    TARGETS
        viam-camera-realsense
    RUNTIME_DEPENDENCIES
        PRE_EXCLUDE_REGEXES ${_pre_exclude_regex}
    RUNTIME
    LIBRARY
    ${_framework_dest_arg}
)

install(
    FILES
        meta.json-no-appimage
    RENAME
        meta.json
    DESTINATION .
)

set(CPACK_PACKAGE_NAME "viam-realsense")
set(CPACK_PACKAGE_FILE_NAME "module")
set(CPACK_GENERATOR "TGZ")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY 0)
include(CPack)
