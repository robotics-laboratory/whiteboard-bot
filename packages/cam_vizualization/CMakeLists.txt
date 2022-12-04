cmake_minimum_required(VERSION 3.8)
project(cam_vizualization)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(cam_vizualization src/main.cpp src/cam_vizualization.cpp)

target_include_directories(cam_vizualization PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(cam_vizualization rclcpp nav_msgs sensor_msgs OpenCV cv_bridge visualization_msgs tf2_msgs)

install(TARGETS cam_vizualization
  DESTINATION lib/${PROJECT_NAME})

ament_package()
