cmake_minimum_required(VERSION 3.8)
project(camera)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_executable(camera_calibration src/camera_calibration.cpp src/params.cpp)

add_executable(camera src/main.cpp src/camera_node.cpp src/camera_calibration_import.cpp)

target_include_directories(camera_calibration PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_include_directories(camera PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(camera_calibration OpenCV rclcpp yaml_cpp_vendor)

ament_target_dependencies(camera rclcpp sensor_msgs OpenCV cv_bridge yaml_cpp_vendor)

install(TARGETS camera_calibration camera
  DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

ament_package()
