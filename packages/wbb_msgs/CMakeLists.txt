cmake_minimum_required(VERSION 3.8)
project(wbb_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} 
  msg/ImagePose.msg
  msg/ImagePoint.msg
  msg/ImageMarkerPos.msg
  msg/ImageMarkerPosArray.msg
  msg/SegmentationGrid.msg
  DEPENDENCIES
  builtin_interfaces
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()