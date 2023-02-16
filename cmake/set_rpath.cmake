# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif("${isSystemDir}" STREQUAL "-1")



macro(set_target_rpath __Target)
    set (extra_args ${ARGN})

    set_target_properties(${__Target} PROPERTIES LINK_FLAGS "-Wl,-rpath,.,-disable-new-dtags")  # set RPATH ok ok
    set_property(
            TARGET ${__Target}
            PROPERTY BUILD_RPATH
            "${CMAKE_BINARY_DIR}/lib"
            "$ORIGIN/../lib"
            "$ORIGIN/.."
            ${extra_args}

    )
    set_property(
            TARGET ${__Target}
            PROPERTY INSTALL_RPATH
            "${CMAKE_INSTALL_PREFIX}/lib"
            "$ORIGIN/../lib"
            "$ORIGIN/.."
            ${extra_args}

    )
endmacro(set_target_rpath)

macro(set_target_runpath __Target)
    set (extra_args ${ARGN})

    set_target_properties(${__Target} PROPERTIES LINK_FLAGS "-Wl,-rpath,.,-enable-new-dtags")  # set RPATH ok ok
    set_property(
            TARGET ${__Target}
            PROPERTY BUILD_RPATH
            "${CMAKE_BINARY_DIR}/lib"
            "$ORIGIN/../lib"
            "$ORIGIN/.."
            ${extra_args}

    )
    set_property(
            TARGET ${__Target}
            PROPERTY INSTALL_RPATH
            "${CMAKE_INSTALL_PREFIX}/lib"
            "$ORIGIN/../lib"
            "$ORIGIN/.."
            ${extra_args}

    )
endmacro(set_target_runpath)