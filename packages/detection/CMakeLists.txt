cmake_minimum_required(VERSION 3.8)
project(detection)

add_compile_options(-Wall -Wextra -Wpedantic)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(wbb_msgs REQUIRED)

add_library(
    ${PROJECT_NAME} SHARED
    src/detection.cpp
)

ament_target_dependencies(
    ${PROJECT_NAME}
    foxglove_msgs
    OpenCV
    rclcpp
    std_msgs
    visualization_msgs
    wbb_msgs )

target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    Boost
    foxglove_msgs
    OpenCV
    rclcpp
    std_msgs
    visualization_msgs
    wbb_msgs )

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

ament_package()