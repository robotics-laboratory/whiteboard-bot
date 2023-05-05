cmake_minimum_required(VERSION 3.8)
project(segmentation)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(detection REQUIRED)
find_package(wbb_msgs REQUIRED)

add_executable(segmentation src/main.cpp src/segmentation_node.cpp)

target_include_directories(segmentation PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(
    segmentation
    cv_bridge
    foxglove_msgs
    OpenCV
    rclcpp
    std_msgs
    sensor_msgs
    detection
    wbb_msgs )

install(TARGETS segmentation
  DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

ament_package()
