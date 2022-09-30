configure_file(package.xml.in ${CMAKE_CURRENT_SOURCE_DIR}/package.xml)

if(  DEFINED catkin_package)
    #file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/package.xml  ${PACKAGE_XML_CONTENT})
    catkin_package(
            INCLUDE_DIRS include
            LIBRARIES ${PROJECT_NAME})

    set(shared_dirs "launch" "param" )
    foreach(dir ${shared_dirs})
        if(EXISTS "${dir}" AND IS_DIRECTORY "${dir}")
            message("EXISTS：${dir}")
            #Installing roslaunch Files or Other Resources
            install(DIRECTORY ${dir}/
                    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
                    PATTERN ".svn" EXCLUDE)
        endif()
    endforeach()



endif()


