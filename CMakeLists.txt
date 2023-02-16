cmake_minimum_required(VERSION 3.0.2)
cmake_policy(SET CMP0074 NEW)
project(NextRobot)



include(cmake/common.cmake)
include(cmake/set_rpath.cmake)
include(cmake/super_build.cmake)
include(cmake/find_opencv_helper.cmake)
set(CMAKE_CXX_STANDARD  14)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")




# 设置生成的安装包名字
set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
# 设置支持指定安装目录的控制为 ON
set(CPACK_SET_DESTDIR ON)
# 设置安装到的目录路径
set(CPACK_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
# 这是生成的安装的版本号信息
set(CPACK_PACKAGE_VERSION "1.0.0")
# 设置 group 名字
set(CPACK_RPM_PACKAGE_GROUP "vesoft")
# 设置 vendor 名字
set(CPACK_PACKAGE_VENDOR "vesoft")
# 设置 license 信息
set(CPACK_RPM_PACKAGE_LICENSE "Apache 2.0 + Common Clause 1.0")
set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Yan QiDong")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "This is a demo package from Yan QiDong.")
set(CPACK_DEBIAN_PACKAGE_VERSION ${PACKAGE_VERSION})
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  tf
  interactive_markers
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   nav_msgs#   sensor_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES topic_serialization
#  CATKIN_DEPENDS nav_msgs roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/topic_serialization.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/topic_serialization_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_topic_serialization.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


build_external_project(mosquitto lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mosquitto-2.0.15.tar.gz -DCMAKE_BUILD_TYPE=Release -DcJSON_DIR=${cJSON_DIR} -DDOCUMENTATION=OFF)
set(mosquitto_LIBRARY ${mosquitto_ROOT}/lib/libmosquitto.so ${mosquitto_ROOT}/lib/libmosquittopp.so)
set(mosquitto_INCLUDE_DIR ${mosquitto_ROOT}/include)

#build_external_project(tinyxml2 lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinyxml2-9.0.0.tar.gz  -DCMAKE_BUILD_TYPE=Release  )
#find_package(tinyxml2 REQUIRED)
# tinyxml2::tinyxml2

build_external_project(Ceres lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ceres-solver-2.1.0.tar.gz  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMINIGLOG=ON -DGFLAGS=OFF -DSUITESPARSE=OFF -DBUILD_EXAMPLES=OFF)
find_package(Ceres  REQUIRED)
message(Ceres_ROOT : ${Ceres_ROOT} ,CERES_LIBRARIES : ${CERES_LIBRARIES} )


build_external_project(PCL lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/pcl-pcl-1.12.1.tar.gz  -DCMAKE_BUILD_TYPE=Release  -DBUILD_examples=OFF -DBUILD_apps=OFF -DBUILD_global_tests=OFF -DBUILD_simulation=OFF)
find_package(PCL REQUIRED)


build_external_project(autodiff lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/autodiff-0.6.12.tar.gz  -DCMAKE_BUILD_TYPE=Release  -DAUTODIFF_BUILD_TESTS=OFF -DAUTODIFF_BUILD_PYTHON=OFF -DAUTODIFF_BUILD_EXAMPLES=OFF -DAUTODIFF_BUILD_DOCS=OFF)
find_package(autodiff REQUIRED)



#octomap-1.9.6.tar.gz
build_external_project(octomap lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/octomap-1.9.6.tar.gz  -DCMAKE_BUILD_TYPE=Release  -DBUILD_OCTOVIS_SUBPROJECT=OFF -DBUILD_DYNAMICETD3D_SUBPROJECT=OFF)
find_package(octomap REQUIRED)


#ifm3d
#ifm3d-0.93.0.zip
build_external_project(ifm3d lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ifm3d-0.93.0.zip -DBUILD_SHARED_LIBS=OFF  -DCMAKE_BUILD_TYPE=Release  -DPCL_ROOT=${PCL_ROOT} -DBUILD_TESTS=OFF)

set(glog_ROOT ${ifm3d_ROOT}/lib/cmake/glog )

find_package(glog REQUIRED)
set(CURL_ROOT ${ifm3d_ROOT}/lib/cmake/CURL )

find_package(CURL REQUIRED)


set(xmlrpc-c_ROOT ${ifm3d_ROOT}/lib/cmake/xmlrpc-c )
find_package(
        xmlrpc-c CONFIG
        REQUIRED COMPONENTS xmlrpc xmlrpc_client xmlrpc++ xmlrpc_client++
)


find_package(ifm3d REQUIRED COMPONENTS
        camera
        framegrabber
        stlimage
        )


add_executable(scan_republisher src/scan_republisher.cpp src/message/MqttClient.cpp)
target_link_libraries(scan_republisher PUBLIC  ${catkin_LIBRARIES} ${mosquitto_LIBRARY} ) 
target_include_directories(scan_republisher PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include  ${catkin_INCLUDE_DIRS} ${mosquitto_INCLUDE_DIR}   ${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann)


if (build_1)
add_executable(scan_filter_intensity src/scan_filter_intensity.cpp)
target_link_libraries(scan_filter_intensity PUBLIC  ${catkin_LIBRARIES}  ${CERES_LIBRARIES})
target_include_directories(scan_filter_intensity PUBLIC  ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include  ${catkin_INCLUDE_DIRS}   ${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann)

install(TARGETS scan_filter_intensity
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
endif()



find_package (Eigen3 REQUIRED NO_MODULE)

#add_library(pcl_norm_2d STATIC src/icp/PCA2D.cpp src/icp/Normal2dEstimation.cpp)
#target_include_directories( pcl_norm_2d PUBLIC BEFORE ${EIGEN3_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/include  ${CMAKE_SOURCE_DIR}/third_party/nlohmann  ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} )





add_executable(scan_slam src/scan_matching.cpp src/ros_tool/RvizInteractiveControlTool.cpp)
target_include_directories(scan_slam PUBLIC BEFORE   ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include  ${catkin_INCLUDE_DIRS}   ${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} )

target_link_libraries(scan_slam PUBLIC  ${catkin_LIBRARIES}  ${CERES_LIBRARIES}  ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${OCTOMAP_LIBRARIES} )
set(scan_slam_debug OFF)
if(scan_slam_debug)
    set_asan(scan_slam)
endif()


# https://github.com/openresty/luajit2
build_external_project_command(luajit lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/luajit2-2.1-20230119.tar.gz "mkdir -p ${CMAKE_BINARY_DIR}/force_luajit/install/bin/luajit-2.1.0-beta3" "make PREFIX=${CMAKE_BINARY_DIR}/force_luajit/install" "make install PREFIX=${CMAKE_BINARY_DIR}/force_luajit/install")
set(LUA_LIBRARIES ${luajit_ROOT}/lib/libluajit-5.1.a)
set(LUA_INCLUDE_DIR ${luajit_ROOT}/include/luajit-2.1)

add_library(liblua STATIC IMPORTED)
set_target_properties(liblua PROPERTIES  IMPORTED_LOCATION "${LUA_LIBRARIES}")
target_include_directories(liblua INTERFACE ${LUA_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/third_party/sol)
target_link_libraries(liblua INTERFACE  dl )

#build_external_project(Matplot++ lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/matplotplusplus-1.1.0.tar.gz  -DBUILD_EXAMPLES=OFF -DBUILD_SHARED_LIBS=ON -DBUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON)
#find_package(Matplot++ REQUIRED)



add_executable(scan_docking src/scan_docking.cpp
        src/pid.cpp src/perception/laserscan_filter.cpp
        src/perception/optim_circle_solver.cpp src/perception/pcl_circle_fitting.cpp
        src/perception/pointcloud_detection.cpp
        src/control/optim_path_planner.cpp
        src/control/control_utils.cpp

        )
target_include_directories(scan_docking PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include  ${CMAKE_CURRENT_SOURCE_DIR}/include/OptimLib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen-3.4.0 ${catkin_INCLUDE_DIRS}   ${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} )


target_link_libraries(scan_docking PUBLIC  liblua autodiff::autodiff  ${catkin_LIBRARIES}  ${CERES_LIBRARIES}  ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${OCTOMAP_LIBRARIES}
#        Matplot++::matplot
        )

set(scan_docking_debug OFF)
if(scan_docking_debug)
    set_asan(scan_docking)
endif()



# ${Ceres_ROOT}/lib
set(shared_dirs ${mosquitto_ROOT}/lib ${PCL_ROOT}/lib )
foreach(dir ${shared_dirs})
    if(EXISTS "${dir}" AND IS_DIRECTORY "${dir}")
        message("EXISTS：${dir}")
        #Installing roslaunch Files or Other Resources
        install(DIRECTORY ${dir}/
                DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
                PATTERN ".svn" EXCLUDE)
    endif()
endforeach()


install(TARGETS scan_republisher
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(TARGETS scan_slam
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(TARGETS scan_docking
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )




install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "launch/data" EXCLUDE
        )

set_target_rpath(scan_slam)
#set_target_runpath(scan_slam)
include(CPack)
