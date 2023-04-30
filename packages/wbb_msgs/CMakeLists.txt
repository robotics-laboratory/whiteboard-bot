cmake_minimum_required(VERSION 3.8)
project(wbb_msgs)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} 
  msg/ImagePose.msg
  msg/ImagePoint.msg
  msg/ImageMarkerPos.msg
  msg/ImageMarkerPosArray.msg
  msg/Control.msg
  DEPENDENCIES
  builtin_interfaces
  geometry_msgs
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
