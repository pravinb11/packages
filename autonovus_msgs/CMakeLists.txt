cmake_minimum_required(VERSION 3.10)
project(autonovus_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/MCSentryDebug.msg"
  "msg/TMMStatistics.msg"
  "msg/TMMCameraChannelInfo.msg"
  "msg/MHFloorStatus.msg"
  "msg/MOTQuaternion32.idl"
  "msg/MOTBoundingBox.idl"
  "msg/MOTCustomObject.idl"
  "msg/MOTCustomObjects.idl"
  "msg/MOTDetectedObject.idl"
  "msg/MOTDetectedObjectKinematics.idl"
  "msg/MOTDetectedObjects.idl"
  "msg/MOTEstimatedObject.idl"
  "msg/MOTEstimatedPointsKinematics.idl"
  "msg/MOTEstimatedPath.idl"
  "msg/MOTObjectClassification.idl"
  "msg/MOTPredictedObject.idl"
  "msg/MOTPredictedObjectKinematics.idl"
  "msg/MOTPredictedObjects.idl"
  "msg/MOTPredictedPath.idl"
  "msg/MOTShape.idl"
  "msg/MOTTrackedObject.idl"
  "msg/MOTTrackedObjectKinematics.idl"
  "msg/MOTTrackedObjects.idl"
  "msg/MOTTrajectory.idl"
  "msg/MOTObjectDetection.msg"
  "msg/MOTTrajectoryPoint.idl"
  "msg/MOTPath.idl"
  "msg/MOTPathPoint.idl"
  "msg/MOTPathWithLaneId.idl"
  "msg/MOTPathPointWithLaneId.idl"
  "msg/MOTPublishedTime.msg"
  "msg/MOTMultiArrayDimension.msg"
  "msg/MOTMultiArrayLayout.msg"
  "msg/MOTBoolStamped.msg"
  "msg/MOTFloat32Stamped.msg"
  "msg/MOTFloat32MultiArrayStamped.msg"
  "msg/MOTFloat64Stamped.msg"
  "msg/MOTFloat64MultiArrayStamped.msg"
  "msg/MOTInt32Stamped.msg"
  "msg/MOTInt32MultiArrayStamped.msg"
  "msg/MOTInt64Stamped.msg"
  "msg/MOTInt64MultiArrayStamped.msg"
  "msg/MOTStringStamped.msg"
  "msg/MOTDetectedObjectWithFeature.msg"
  "msg/MOTDetectedObjectsWithFeature.msg"
  "msg/MOTFeature.msg"
  "msg/PIStatus.msg"
  "msg/PITarget.msg"
  "msg/ONNavigateGoal.msg"
  "msg/BoundingBox.msg"
  "msg/ObjectDetection.msg"
  "msg/Statistics.msg"
  "msg/ChannelInfo.msg"
  "msg/StairParam.msg"
  "msg/BoundingBox3D.msg"
)

# Add service files
set(srv_files
  "srv/TMMCheckSafety.srv"
  "srv/TMMInitialize.srv"
  "srv/SavePCDMap.srv"
  "srv/MHSetPCDCostmap.srv"
  "srv/MHSetStairsCostmap.srv"
  "srv/StartPIMonitoring.srv"
  "srv/StopPIMonitoring.srv"
  "srv/StartExploration.srv"
  "srv/StopExploration.srv"
  "srv/StairDetail.srv"
  "srv/CheckSafety.srv"
  "srv/Initialize.srv"
  "srv/TransitionAreaDetail.srv"
  "srv/TNPreprocessingStart.srv"
  "srv/TNPreprocessingStop.srv"
)

set(action_files
  "action/RPMultiFloorNavigate.action"
  "action/LHPNavigateThroughPath.action"
  "action/LHPComputeLongHorizonPath.action"
  "action/RPMultiPoseNavigate.action"
  "action/ONNavigateWhenMapUnavailable.action"
  "action/StartExploration.action"
  "action/NMNavigateToGoal.action"
  "action/NMNavigateToGoalWithGPS.action"
  "action/NMAutoNavGoal.action"
  "action/EPTriggerExploration.action"
  "action/TNTransitionNavigator.action"
)

rosidl_generate_interfaces(${PROJECT_NAME} 
${msg_files} 
${srv_files}
${action_files}
DEPENDENCIES geometry_msgs std_msgs nav_msgs sensor_msgs unique_identifier_msgs diagnostic_msgs builtin_interfaces
)


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
