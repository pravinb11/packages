cmake_minimum_required(VERSION 3.5)
project(planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
# find_package(roslib REQUIRED)
# find_package(actionlib REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(move_base_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ufomap_manager REQUIRED)
find_package(control_planner_interface REQUIRED)
# find_package(traversability_analysis REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ufomap_ros REQUIRED)
find_package(ufomap REQUIRED)
find_package(ufomap_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(intf_pkg REQUIRED)
find_package(std_msgs REQUIRED)
# set(CMAKE_CXX_STANDARD 1 4)
set(CMAKE_CXX_STANDARD 17)

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

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/PreprocessMsgs.msg
        msg/ViewpointsWithFrontiers.msg
        msg/IdPointPair.msg
        msg/RoadMapMsg.msg
        msg/EdgePair.msg
    DEPENDENCIES
        geometry_msgs
        ufomap_manager
)

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

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        or-tools/include
)
# ====================================================================================================
add_library(topo_planner
        src/graph/kdtree.c
        src/graph/plan_graph.cpp
        src/perception/grid_map_2d.cpp
        src/perception/map_2d_manager.cpp
        src/perception/ufomap.cpp
        src/perception/lidar_model.cpp
        src/preprocess/viewpoint_manager.cpp
        src/preprocess/topo_graph.cpp
        src/topo_planner/topo_planner.cpp
        src/rapid_cover_planner/rapid_cover_planner.cpp
        )
target_include_directories(topo_planner PUBLIC  ${PCL_INCLUDE_DIRS}
                                                ${ufomap_manager_INCLUDE_DIRS}
                                                "${planner_SOURCE_DIR}/../ufomap/include"
                                                ${control_planner_interface_INCLUDE_DIRS}
                                                ${pcl_ros_INCLUDE_DIRS})
target_link_libraries(topo_planner ${PCL_LIBRARIES})
ament_target_dependencies(topo_planner  rclcpp
                                        pcl_conversions
                                        pcl_msgs
                                        tf2_msgs
                                        visualization_msgs
                                        tf2_geometry_msgs
                                        pcl_ros
                                        grid_map_ros
                                        grid_map_msgs
                                        intf_pkg
                                        std_msgs)
# ====================================================================================================

add_executable(topo_planner_node src/topo_planner/topo_planner_node.cpp)
target_include_directories(topo_planner_node PUBLIC ${control_planner_interface_INCLUDE_DIRS}
                                                    ${ufomap_manager_INCLUDE_DIRS}
                                                    "${planner_SOURCE_DIR}/../ufomap/include")
target_link_libraries(topo_planner_node topo_planner
                                         UFO::Map)
ament_target_dependencies(topo_planner_node
        # topo_planner
        "message_filters"
        "rclcpp"
        "pcl_ros"
        "std_msgs"
        "pcl_conversions"
        "visualization_msgs"
        "tf2_ros"
        "nav_msgs"
        "geometry_msgs"
        # "actionlib"
        ufomap_ros
        ufomap
        ufomap_manager
        ufomap_msgs
        control_planner_interface
)
ament_export_include_directories(include)
ament_export_libraries(topo_planner)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_topo_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

install(TARGETS topo_planner_node
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
									params
        DESTINATION share/${PROJECT_NAME})

# ament_export_dependencies(actionlib actionlib_msgs geometry_msgs message_generation move_base_msgs nav_msgs rclcpp sensor_msgs std_srvs tf2_ros ufomap_manager control_planner_interface visualization_tools traversability_analysis)
ament_export_dependencies(actionlib_msgs geometry_msgs message_generation nav_msgs rclcpp sensor_msgs std_srvs tf2_ros ufomap_manager control_planner_interface visualization_tools)
ament_package()

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
