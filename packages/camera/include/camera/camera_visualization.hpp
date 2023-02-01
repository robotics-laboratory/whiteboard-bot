#pragma once

#include <camera/camera.hpp>

#include <chrono>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/header.hpp>
#include <stdio.h>
#include <opencv2/aruco.hpp>

class CameraNode : public rclcpp::Node {
  public:
    CameraNode(CalibrationParams calibration_params);

  private:
    void handleCameraOnTimer();
    void remapImage(cv::Mat& raw_image, cv::Mat& undistorted_image);
    void updateHomography(
        std::vector<int>& ids, std::vector<std::vector<cv::Point2f>>& corners,
        std::vector<std::pair<int, std::vector<cv::Point2f>>>& id_corners_marker);

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_publisher;
    cv::VideoCapture camera;
    cv::Size frame_size;
    cv::Ptr<cv::aruco::Dictionary> aruco_markers_dict =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Matx33f homography_matrix{cv::Matx33f::eye()};
    CalibrationParams calibration_params;
};
