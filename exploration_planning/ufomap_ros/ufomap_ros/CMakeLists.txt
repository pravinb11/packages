cmake_minimum_required(VERSION 3.8.2)
project(ufomap_ros)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(ufomap REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(
  include
  "${ufomap_ros_SOURCE_DIR}/../../ufomap/include"
)

# add_library(${PROJECT_NAME} STATIC
#   src/conversions.cpp
# )
add_library(${PROJECT_NAME} SHARED
            src/conversions.cpp)
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    # UFO::Map
    ufomap
    "geometry_msgs"
    "sensor_msgs"
)
ament_export_include_directories(include ${catkin_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
ament_export_dependencies(geometry_msgs rclcpp sensor_msgs)
ament_package()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
)