cmake_minimum_required(VERSION 3.8)
project(camera)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

find_package(detection REQUIRED)
find_package(wbb_msgs REQUIRED)

add_executable(camera_calibration src/camera_calibration.cpp src/params.cpp)

add_executable(camera src/main.cpp src/camera_node.cpp src/params.cpp)

target_include_directories(camera_calibration PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_include_directories(camera PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(
    camera_calibration
    OpenCV
    rclcpp
    yaml_cpp_vendor)

ament_target_dependencies(
    camera
    cv_bridge
    foxglove_msgs
    OpenCV
    rclcpp
    sensor_msgs
    std_msgs
    yaml_cpp_vendor
    visualization_msgs
    wbb_msgs
    detection)

install(TARGETS camera_calibration camera
  DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

ament_package()
