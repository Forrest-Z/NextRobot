find_package(Threads REQUIRED)

# ---------------------------------------------------------------------------------------------------------
# Handle C++ standard version.
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # use -std=c++... instead of -std=gnu++...


# ---------------------------------------------------------------------------------------------------------

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

# ---------------------------------------------------------------------------------------------------------

# flags

# fix ld: unrecognized option '--push-state--no-as-needed'
# https://stackoverflow.com/questions/50024731/ld-unrecognized-option-push-state-no-as-needed
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=gold")



# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Debug' as none was specified.")
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
            "MinSizeRel" "RelWithDebInfo")
endif()


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Werror=return-type")

if (CMAKE_BUILD_TYPE MATCHES Release)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -Wextra -pedantic")
    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-generate -fprofile-use")


    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--whole-archive  -Wl,--no-whole-archive")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math -O3 -march=native -ftree-vectorize -fopt-info-vec-optimized ")

endif ()

if (CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -Wextra -pedantic")
    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-generate -fprofile-use")


    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--whole-archive  -Wl,--no-whole-archive")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math -O3 -march=native -ftree-vectorize -fopt-info-vec-optimized ")

endif ()
if (CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    #-Wall -Wextra -pedantic
    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -Ofast  -ffast-math -ftree-vectorize   -march=native -funsafe-loop-optimizations -mavx -mfma")
    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -Wall -Wextra   -Ofast   ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -Wall -Wextra   -march=native   -O3  -Ofast -funsafe-loop-optimizations ")

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -ftree-vectorize  -ffast-math -fopt-info-vec-optimized  -opt-report=5 ")

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -pthread -fprofile-arcs -mstackrealign  -march=native -ftest-coverage")


endif ()
if (CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -Wextra -pedantic -Wno-dev -Wno-unknown-pragmas -Wno-sign-compare -Woverloaded-virtual -Wwrite-strings -Wno-unused")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -g -O0 -o -ggdb  -ggdb3 -fprofile-arcs -mstackrealign  -march=native  ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread  -g -O0 ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -mstackrealign  -march=native  -fstack-protector")

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -fno-optimize-sibling-calls -fno-omit-frame-pointer -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free")

    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined   -fsanitize=address -fsanitize=leak -fsanitize=leak ")
    #set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address  -fsanitize=leak")
    #set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -fsanitize=address  -fsanitize=leak")
endif ()

function(set_asan target)
    # Treat all warnings as errors
    #    target_compile_options(${target} PRIVATE "-Werror")
    target_compile_options(${target} PUBLIC "-fsanitize=undefined")
    target_compile_options(${target} PUBLIC "-fno-builtin-malloc")
    target_compile_options(${target} PUBLIC "-fno-builtin-calloc")
    target_compile_options(${target} PUBLIC "-fno-builtin-realloc")
    target_compile_options(${target} PUBLIC "-fno-builtin-free")


#    target_compile_options(${target} PUBLIC "-fsanitize-address-use-after-scope")

    target_compile_options(${target} PUBLIC "-fsanitize=address")
    target_compile_options(${target} PUBLIC "-fno-optimize-sibling-calls")


    target_compile_options(${target} PUBLIC "-fsanitize=leak")
    target_compile_options(${target} PUBLIC "-fno-omit-frame-pointer")

    target_compile_options(${target} PUBLIC "-fstack-protector")
    target_compile_options(${target} PUBLIC  "-fuse-ld=gold")

    target_link_libraries(${target} PUBLIC "-fsanitize=address  -fsanitize=leak -fsanitize=undefined")
    #    target_compile_options(${target} PUBLIC "-fsanitize=memory")
    #    target_link_libraries(${target} PUBLIC "-fsanitize=memory")
    #    target_compile_options(${target} PUBLIC "-fsanitize=thread")
    #    target_link_libraries(${target} PUBLIC "-fsanitize=thread")

endfunction(set_asan)
# ---------------------------------------------------------------------------------------------------------


function(print_include_dir target)
    get_target_property(LIBA_INCLUDES ${target} INCLUDE_DIRECTORIES)
    message(${target} INCLUDES : ${LIBA_INCLUDES})
endfunction()

# ---------------------------------------------------------------------------------------------------------


function(set_omp target)
    target_include_directories(${target} PUBLIC ${OpenMP_CXX_INCLUDE_DIRS} )
    target_link_libraries(${target} PUBLIC
            ${OpenMP_CXX_LIBRARIES}
            )
    target_compile_options(${target} PUBLIC ${OpenMP_CXX_FLAGS})
endfunction()










# ---------------------------------------------------------------------------------------------------------



