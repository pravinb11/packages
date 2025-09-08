cmake_minimum_required(VERSION 3.8)
project(action_bridge_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)

ament_export_dependencies(rosidl_default_runtime)

# ==================================================

add_library(action_server SHARED
  src/fibonacci_action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
# rosidl_target_interfaces(action_server
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")
# OR
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(action_server "${cpp_typesupport_target}")
ament_target_dependencies(action_server
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_server PLUGIN "action_tutorials_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ==================================================

add_library(ac_lib SHARED src/fibonacci_action_client.cpp)
# rosidl_target_interfaces(ac_lib
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")
# OR
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ac_lib "${cpp_typesupport_target}")
ament_target_dependencies(ac_lib
  rclcpp
  rclcpp_action
  rclcpp_components)
rclcpp_components_register_node(ac_lib 
  PLUGIN "action_tutorials_cpp_ns::FibonacciActionClient" 
  EXECUTABLE ac_exe)
ament_export_targets(export_ac_lib)
install(TARGETS
  ac_lib
  EXPORT export_ac_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ==================================================

add_executable(s_and_ac_exe src/subscriber_and_action_client.cpp)
# rosidl_target_interfaces(s_and_ac_exe
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")
# OR
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(s_and_ac_exe "${cpp_typesupport_target}")
ament_target_dependencies(s_and_ac_exe
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs)
install(TARGETS s_and_ac_exe
  DESTINATION lib/${PROJECT_NAME})

# ==================================================

add_executable(action_bridge_exe src/action_bridge.cpp)
ament_target_dependencies(action_bridge_exe
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  nav_msgs
  nav2_msgs)
install(TARGETS action_bridge_exe
  DESTINATION lib/${PROJECT_NAME})

# ==================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()