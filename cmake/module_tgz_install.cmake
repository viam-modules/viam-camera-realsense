if (LINUX)
   set(_pre_exclude_regex
   	    ".*ld-linux.*\\.so.*"
   	    ".*libc\\.so.*"
   	    ".*libdl\\.so.*"
   	    ".*libgcc.*\\.so.*"
   	    ".*libm\\.so.*"
   	    ".*libpthread\\.so.*"
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
        PRE_EXCLUDE_REGEX ${_pre_exclude_regex}
    RUNTIME
    LIBRARY
    ${_framework_dest_arg}
)

install(
    FILES
        meta.json-no-appimage
    DESTINATION ${CMAKE_INSTALL_PREFIX}
    RENAME meta.json
)
