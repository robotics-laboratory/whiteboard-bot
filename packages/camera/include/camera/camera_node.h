#pragma once

#include <cv_bridge/cv_bridge.h>

#include <foxglove_msgs/msg/image_marker_array.hpp>
#include <memory>
#include <opencv2/aruco.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/image_marker.hpp>
#include <wbb_msgs/msg/image_marker_pos.hpp>
#include <wbb_msgs/msg/image_marker_pos_array.hpp>
#include <wbb_msgs/msg/image_pixel_scale.hpp>
#include <wbb_msgs/msg/image_pose.hpp>

#include "camera/params.h"
#include "detection/detection.h"

namespace wbb {

class CameraNode : public rclcpp::Node {
  public:
    CameraNode();

  private:
    void handleCameraOnTimer();
    cv::Mat getImage();
    cv::Mat removeDistortion(const cv::Mat& raw_image);
    DetectionResult detectMarkers(const cv::Mat& undistorted_image);
    void tryUpdateHomography(const DetectionResult& detection);
    cv::Mat warp(const cv::Mat& undistorted_image);
    std::optional<BotPose> getBotPose(const std::optional<Marker>& ego);
    std::vector<cv::Point2f> makeBotBox(const std::optional<Marker>& ego);
    DetectionResult transform(const DetectionResult& detection);

    void publishImage(const cv::Mat& warped_image, const rclcpp::Time& capturing_time);
    void publishImageCorners(const std::vector<Marker>& markers);
    void publishImageBorder(const std::vector<cv::Point2f>& border);
    void publishBotEgo(const std::optional<BotPose>& ego, const rclcpp::Time& capturing_time);
    void publishPixelScale(const std::vector<Marker>& markers);

    void publishPreview(const cv::Mat& warped_image, const rclcpp::Time& capturing_time);
    void publishPreviewCorners(
        const std::vector<Marker>& markers, const rclcpp::Time& capturing_time);
    void publishBotBox(const std::vector<cv::Point2f>& border, const rclcpp::Time& capturing_time);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr preview_publisher_ = nullptr;
    rclcpp::Publisher<wbb_msgs::msg::ImagePixelScale>::SharedPtr pixel_scale_publisher_ = nullptr;

    rclcpp::Publisher<wbb_msgs::msg::ImagePose>::SharedPtr robot_ego_publisher_ = nullptr;
    rclcpp::Publisher<wbb_msgs::msg::ImageMarkerPosArray>::SharedPtr image_corners_publisher_ =
        nullptr;
    rclcpp::Publisher<wbb_msgs::msg::ImageMarkerPos>::SharedPtr robot_border_publisher_ = nullptr;

    rclcpp::Publisher<foxglove_msgs::msg::ImageMarkerArray>::SharedPtr preview_corners_publisher_ =
        nullptr;
    rclcpp::Publisher<visualization_msgs::msg::ImageMarker>::SharedPtr preview_border_publisher_ =
        nullptr;

    cv::VideoCapture camera_;
    cv::Ptr<cv::aruco::Dictionary> aruco_markers_dict_ =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    cv::Size frame_size_{0, 0};
    cv::Matx33f homography_matrix_{cv::Matx33f::eye()};
    IntrinsicCameraParameters calibration_params_;
};

}  // namespace wbb