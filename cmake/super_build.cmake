include(FetchContent)
include(ExternalProject)
# This function is used to force a build on a dependant project at cmake configuration phase.
#
function (build_external_project target prefix url) #FOLLOWING ARGUMENTS are the CMAKE_ARGS of ExternalProject_Add
    include(ProcessorCount)
    ProcessorCount(N)

    set(trigger_build_dir ${CMAKE_BINARY_DIR}/force_${target})

    #mktemp dir in build tree
    file(MAKE_DIRECTORY ${trigger_build_dir} ${trigger_build_dir}/build)

    #generate false dependency project
    set(CMAKE_LIST_CONTENT "
        cmake_minimum_required(VERSION 3.0)
        project(${target}_build)
        include(ExternalProject)

        ExternalProject_add(${target}
            PREFIX ${prefix}/${target}
            URL ${url}
            CMAKE_ARGS ${ARGN} -DCMAKE_INSTALL_PREFIX=${trigger_build_dir}/install
            #INSTALL_COMMAND \"\"
            INSTALL_COMMAND ${CMAKE_COMMAND} --install .
            )

        add_custom_target(trigger_${target})
        add_dependencies(trigger_${target} ${target})
    ")

    file(WRITE ${trigger_build_dir}/CMakeLists.txt "${CMAKE_LIST_CONTENT}")

    execute_process(COMMAND ${CMAKE_COMMAND} -G${CMAKE_GENERATOR} ..
            WORKING_DIRECTORY ${trigger_build_dir}/build
            )
    execute_process(COMMAND ${CMAKE_COMMAND} --build . -j${N}
            WORKING_DIRECTORY ${trigger_build_dir}/build
            )
    set(${target}_ROOT ${trigger_build_dir}/install PARENT_SCOPE)

    #    execute_process(COMMAND ${CMAKE_COMMAND} --install ./build --prefix ${trigger_build_dir}/install
    #            WORKING_DIRECTORY ${trigger_build_dir}
    #            )
endfunction()

function (build_external_project_make target prefix url) #FOLLOWING ARGUMENTS are the CMAKE_ARGS of ExternalProject_Add
    include(ProcessorCount)
    ProcessorCount(N)

    set(trigger_build_dir ${CMAKE_BINARY_DIR}/force_${target})

    #mktemp dir in build tree
    file(MAKE_DIRECTORY ${trigger_build_dir} ${trigger_build_dir}/build)

    #generate false dependency project
    set(CMAKE_LIST_CONTENT "
        cmake_minimum_required(VERSION 3.0)
        project(${target}_build)
        include(ExternalProject)
        #make -w generic
        ExternalProject_add(${target}
            PREFIX ${prefix}/${target}
            URL ${url}
    CONFIGURE_COMMAND "make clean"
    BUILD_COMMAND  make  linux MYLIBS="-lrt -ldl -lreadline"
    BUILD_IN_SOURCE TRUE
    INSTALL_COMMAND make INSTALL_TOP=${trigger_build_dir}/install install
            )

        add_custom_target(trigger_${target})
        add_dependencies(trigger_${target} ${target})
    ")

    file(WRITE ${trigger_build_dir}/CMakeLists.txt "${CMAKE_LIST_CONTENT}")


    execute_process(COMMAND ${CMAKE_COMMAND} -G${CMAKE_GENERATOR} ..
            WORKING_DIRECTORY ${trigger_build_dir}/build
            )
    execute_process(COMMAND ${CMAKE_COMMAND} --build . -j${N}
            WORKING_DIRECTORY ${trigger_build_dir}/build
            )

    set(${target}_ROOT ${trigger_build_dir}/install PARENT_SCOPE)

    #    execute_process(COMMAND ${CMAKE_COMMAND} --install ./build --prefix ${trigger_build_dir}/install
    #            WORKING_DIRECTORY ${trigger_build_dir}
    #            )
endfunction()


# install_deps must use with following command
#install(CODE [[ set(__install_target $<TARGET_FILE:fruit_greet>) ]]  )
macro(install_deps __target)



    if(  DEFINED catkin_package)
        install(CODE
                [[
        set(__install_bin_destination ${CATKIN_PACKAGE_BIN_DESTINATION})
        set(__install_lib_destination ${CATKIN_PACKAGE_LIB_DESTINATION})

        ]]
                )
        install(TARGETS ${__target}
                RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
                )
        message(install_deps install_bin_destination : ${CATKIN_PACKAGE_BIN_DESTINATION})
        message(install_deps install_lib_destination : ${CATKIN_PACKAGE_LIB_DESTINATION})
    else()
        install(CODE
                [[
        set(__install_bin_destination ${CMAKE_INSTALL_PREFIX}/bin)
        set(__install_lib_destination ${CMAKE_INSTALL_PREFIX}/lib)
        ]]
                )
        install(TARGETS ${__target} DESTINATION bin)

        message(install_deps install_bin_destination : ${CMAKE_INSTALL_PREFIX}/bin)
        message(install_deps install_lib_destination : ${CMAKE_INSTALL_PREFIX}/lib)

    endif()


    install(CODE [[

    if( NOT  DEFINED __install_target)
        message(FATAL "
        __install_target NOT DEFINED
        ")
    else()
        message(STATUS "__install_target DEFINED")
    endif()
  file(GET_RUNTIME_DEPENDENCIES

    EXECUTABLES ${__install_target}
    RESOLVED_DEPENDENCIES_VAR _r_deps
    PRE_EXCLUDE_REGEXES "usr.*"
    POST_EXCLUDE_REGEXES "^/(usr|lib).*"
    UNRESOLVED_DEPENDENCIES_VAR _u_deps
    DIRECTORIES ${MY_DEPENDENCY_PATHS}
  )
  message("\n\n    Found dependencies :")

  foreach(dep_filename ${_r_deps})
        message("    -- ${dep_filename}")
       file(INSTALL
              DESTINATION "${__install_lib_destination}"
              TYPE SHARED_LIBRARY
              FOLLOW_SYMLINK_CHAIN
              FILES "${dep_filename}"
            )
  endforeach()
  list(LENGTH _u_deps _u_length)
  if("${_u_length}" GREATER 0)
    message(WARNING "Unresolved dependencies detected!")
  endif()

  unset(__install_target)
]])
endmacro()

