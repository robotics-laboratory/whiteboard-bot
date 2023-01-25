#pragma once

#include <chrono>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <opencv2/aruco.hpp>

class CamVisualization : public rclcpp::Node {
  public:
    CamVisualization(
        cv::VideoCapture camera, cv::Matx33f camera_matrix,
        cv::Vec<float, 5> distance_coefficients);

  private:
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher;
    cv::VideoCapture camera;
    cv::Ptr<cv::aruco::Dictionary> aruco_markers_dict =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Matx33f homography_matrix{1, 0, 0, 0, 1, 0, 0, 0, 1};
    cv::Matx33f camera_matrix;
    cv::Vec<float, 5> distance_coefficients;
};

bool import_camera_calibration(cv::Matx33f& camera_matrix, cv::Vec<float, 5>& distance_coefficients);