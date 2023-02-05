#pragma once

#include <camera/params.h>

#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <opencv2/aruco.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

class CameraNode : public rclcpp::Node {
  public:
    CameraNode();

  private:
    void handleCameraOnTimer();
    cv::Mat getImage();
    cv::Mat removeDistorsion(const cv::Mat& raw_image);
    void tryUpdateHomography(const cv::Mat& undistorted_image);
    cv::Mat warp(const cv::Mat& undistorted_image);
    void publisRawImage(const cv::Mat& warped_image);
    void publishPreview(const cv::Mat& warped_image);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr preview_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

    cv::VideoCapture camera_;
    cv::Size frame_size_;
    cv::Ptr<cv::aruco::Dictionary> aruco_markers_dict_ =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Matx33f homography_matrix_{cv::Matx33f::eye()};
    IntrinsicCameraParameters calibration_params_;
};
