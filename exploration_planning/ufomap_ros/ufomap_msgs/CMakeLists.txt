cmake_minimum_required(VERSION 3.8.2)

project(ufomap_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(ufomap REQUIRED)

set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(${PROJECT_NAME}
                            msg/AABB.msg
                            msg/Point.msg
                            msg/Capsule.msg
                            msg/Triangle.msg
                            msg/BoundingVolume.msg
                            msg/Ray.msg
                            msg/Sphere.msg
                            msg/Cylinder.msg
                            msg/Plane.msg
                            msg/Cone.msg
                            msg/OBB.msg
                            msg/LineSegment.msg
                            msg/UFOMap.msg
                            msg/Frustum.msg
                            msg/UFOMapStamped.msg
                            msg/Ellipsoid.msg
                            msg/UFOMapMetaData.msg
                        DEPENDENCIES
                            std_msgs
)

include_directories(include
                    "${ufomap_msgs_SOURCE_DIR}/../../ufomap/include")

add_library(${PROJECT_NAME}_lib src/conversions.cpp)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_lib 
                            ufomap
                            std_msgs)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME}_lib)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(geometry_msgs std_msgs rosidl_default_runtime)

rosidl_target_interfaces(${PROJECT_NAME}_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")    # Debug 5

install(TARGETS ${PROJECT_NAME}_lib
        EXPORT export_${PROJECT_NAME}_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h")

ament_package()