cmake_minimum_required(VERSION 3.8)
project(planner)

add_compile_options(-Wall -Wextra -Wpedantic)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)

find_package(detection REQUIRED)
find_package(wbb_msgs REQUIRED)

add_executable(planner src/main.cpp src/planner_node.cpp src/graph_helper.cpp)

target_include_directories(${PROJECT_NAME}  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(
    ${PROJECT_NAME} 
    Boost
    foxglove_msgs
    OpenCV
    rclcpp
    detection
    wbb_msgs
)

install(TARGETS ${PROJECT_NAME} 
  DESTINATION lib/${PROJECT_NAME})

  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

ament_package()
