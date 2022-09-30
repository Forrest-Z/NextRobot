macro (mymacro required_arg1 required_arg2)
    # Cannot use ARGN directly with list() command,
    # so copy it to a variable first.
    set (extra_args ${ARGN})

    # Did we get any optional args?
    list(LENGTH extra_args extra_count)
    if (${extra_count} GREATER 0)
        list(GET extra_args 0 optional_arg)
        message ("Got an optional arg: ${optional_arg}")
    endif ()
endmacro (mymacro)


macro(find_opencv __OPENCV_ROOT __OPENCV_VERSION )

    set (extra_args ${ARGN})

#    message("__OPENCV_ROOT     : " ${__OPENCV_ROOT})
#    message("__OPENCV_VERSION : " ${__OPENCV_VERSION})
#    message("extra_args        : " ${extra_args})


    set(OpenCV_ROOT ${__OPENCV_ROOT})
    list(LENGTH extra_args extra_count)
    if (${extra_count} EQUAL 0)
        list(APPEND extra_args core)
        list(APPEND extra_args imgproc)
        list(APPEND extra_args imgcodecs)
        list(APPEND extra_args highgui)
        message("-- Add default COMPONENTS : " ${extra_args})

    endif ()

#    file(GLOB_RECURSE OpenCV_DIR  ${OpenCV_ROOT1}/lib/cmake/**/OpenCVConfig.cmake)
#    message(OpenCV_ROOT : ${OpenCV_ROOT1})
#    message(OpenCV_DIR : ${OpenCV_DIR})

    find_package( OpenCV ${__OPENCV_VERSION} REQUIRED COMPONENTS ${extra_args}
            #        NO_DEFAULT_PATH
            #        NO_CMAKE_PATH
            #        NO_CMAKE_ENVIRONMENT_PATH
            #        NO_SYSTEM_ENVIRONMENT_PATH
            #        NO_CMAKE_PACKAGE_REGISTRY
            #        NO_CMAKE_BUILDS_PATH
            #        NO_CMAKE_SYSTEM_PATH
            #        NO_CMAKE_SYSTEM_PACKAGE_REGISTRY

            )
    message("-- Found OpenCV_LIBS : " ${OpenCV_LIBS})
    message("-- Found OpenCV_INCLUDE_DIRS : " ${OpenCV_INCLUDE_DIRS})
endmacro(find_opencv)

macro(add_opencv_dep __Target)
    target_link_libraries( ${__Target} PUBLIC ${OpenCV_LIBS} )
    #target_link_directories(hello_opencv PUBLIC ${OPENCV_ROOT}/lib )
    target_include_directories(${__Target} PUBLIC  ${OpenCV_INCLUDE_DIRS} )
endmacro(add_opencv_dep)