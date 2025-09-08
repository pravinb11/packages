cmake_minimum_required(VERSION 3.5)

project(exploration_manager)

find_package(ament_cmake REQUIRED)
find_package(control_planner_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rcl REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(libstatistics_collector REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(intf_pkg REQUIRED)
find_package(autonovus_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(include
                        ${control_planner_interface_INCLUDE_DIRS}
                        ${rcl_INCLUDE_DIRS}
                        ${builtin_interfaces_INCLUDE_DIRS}
                        ${libstatistics_collector_INCLUDE_DIRS}
                        ${visualization_msgs_INCLUDE_DIRS}
                        ${rclcpp_action_INCLUDE_DIRS}
                        ${unique_identifier_msgs_INCLUDE_DIRS}
                        ${intf_pkg_INCLUDE_DIRS})

add_library(explorer src/explorer.cpp)
ament_target_dependencies(explorer rclcpp nav_msgs intf_pkg visualization_msgs autonovus_msgs rclcpp_action)

add_executable(explorer_node src/explorer_node.cpp)
ament_target_dependencies(explorer_node std_msgs
                                        control_planner_interface
                                        visualization_msgs
                                        builtin_interfaces
                                        libstatistics_collector
                                        rclcpp_action
                                        unique_identifier_msgs
                                        nav_msgs
                                        geometry_msgs
                                        intf_pkg
                                        autonovus_msgs)

target_link_libraries(explorer_node explorer) 
                                        # control_planner_interface::control_planner_interface_lib 
                                        # rcl)

install(TARGETS explorer_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
									params
        DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(explorer)
ament_export_dependencies(control_planner_interface geometry_msgs nav_msgs rclcpp std_msgs tf2_ros autonovus_msgs rclcpp_action)
ament_package()