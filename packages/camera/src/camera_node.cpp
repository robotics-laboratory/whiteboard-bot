#include "camera/camera_node.h"
#include "camera/messages_helper.h"

#include <std_msgs/msg/header.hpp>

#include <chrono>
#include <cmath>
#include <stdio.h>

namespace wbb {

namespace {

const int kBOT_MARKER_ID = 4;
const float kBotMarkerSize = 0.1f;
const float kBaseLength = 0.2f;
const float kBaseWidth = 0.15f;

}  // namespace

CameraNode::CameraNode() : Node("CameraNode") {
    camera_.open(0);

    if (!camera_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Unable to open the camera");
        throw std::runtime_error("Unable to open the camera");
    }
    RCLCPP_INFO(this->get_logger(), "Camera opened successfully");

    const auto width = this->declare_parameter<int>("frame_width", 1280);
    const auto height = this->declare_parameter<int>("frame_height", 720);

    camera_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    camera_.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    frame_size_ = cv::Size(width, height);

    std::string path_to_calibration_file = this->declare_parameter<std::string>(
        "calibration_params_path", "/wbb/packages/camera/config/calibration.yaml");
    calibration_params_ = importCameraCalibration(path_to_calibration_file);

    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/board/image", 10);

    preview_publisher_ =
        this->create_publisher<sensor_msgs::msg::CompressedImage>("/board/preview", 10);

    robot_ego_publisher_ = this->create_publisher<wbb_interfaces::msg::ImagePose>("/robot/ego", 10);

    image_corners_publisher_ = this->create_publisher<wbb_interfaces::msg::ImageMarkerPoseArray>(
        "/board/image/corners", 10);

    image_border_publisher_ =
        this->create_publisher<wbb_interfaces::msg::ImageMarkerPose>("/board/image/ego", 10);

    preview_corners_publisher_ =
        this->create_publisher<foxglove_msgs::msg::ImageMarkerArray>("/board/preview/corners", 10);

    preview_border_publisher_ =
        this->create_publisher<visualization_msgs::msg::ImageMarker>("/board/preview/ego", 10);

    markers_ =
        std::vector<std::vector<cv::Point2f>>(5, std::vector<cv::Point2f>(4, cv::Point2f(0, 0)));

    const auto camera_period =
        std::chrono::milliseconds(this->declare_parameter<int>("camera_period", 100));
    timer_ =
        this->create_wall_timer(camera_period, std::bind(&CameraNode::handleCameraOnTimer, this));
}

cv::Mat CameraNode::getImage() {
    cv::Mat raw_image;
    camera_.read(raw_image);
    return raw_image;
}

cv::Mat CameraNode::removeDistortion(const cv::Mat& raw_image) {
    cv::Mat undistorted_image;
    cv::Mat map_x;
    cv::Mat map_y;

    cv::initUndistortRectifyMap(
        calibration_params_.camera_matrix,
        calibration_params_.distortion,
        cv::Matx33f::eye(),
        calibration_params_.camera_matrix,
        frame_size_,
        CV_32FC1,
        map_x,
        map_y);

    cv::remap(raw_image, undistorted_image, map_x, map_y, cv::INTER_LINEAR);
    RCLCPP_DEBUG(this->get_logger(), "Image remapped successfully");
    return undistorted_image;
}

void CameraNode::tryUpdateHomography(const cv::Mat& undistorted_image) {
    std::vector<int> ids;
    MarkersCoords corners;
    cv::aruco::detectMarkers(undistorted_image, aruco_markers_dict_, corners, ids);

    std::vector<std::pair<int, std::vector<cv::Point2f>>> id_corners_marker;

    for (size_t i = 0; i < ids.size(); ++i) {
        markers_[ids[i]] = corners[i];
        if (ids[i] == kBOT_MARKER_ID) {
            continue;
        }

        id_corners_marker.push_back({ids[i], corners[i]});
    }

    if (id_corners_marker.size() != 4) {
        RCLCPP_DEBUG(this->get_logger(), "Skip homography update!");
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Update homography");

    std::sort(id_corners_marker.begin(), id_corners_marker.end(), [](auto left, auto right) {
        return left.first < right.first;
    });

    std::vector<cv::Point2f> src_vertices;

    for (size_t i = 0; i < id_corners_marker.size(); ++i) {
        src_vertices.push_back(id_corners_marker[i].second[i]);
    }

    const std::vector<cv::Point2f> dst_vertices = {
        cv::Point(0, 0),
        cv::Point(frame_size_.width - 1, 0),
        cv::Point(frame_size_.width - 1, frame_size_.height - 1),
        cv::Point(0, frame_size_.height - 1)};

    homography_matrix_ = cv::findHomography(src_vertices, dst_vertices);
}

cv::Mat CameraNode::warp(const cv::Mat& undistorted_image) {
    cv::Mat warped_image;
    cv::warpPerspective(
        undistorted_image, warped_image, homography_matrix_, undistorted_image.size());
    return warped_image;
}

MarkersCoords CameraNode::transformMarkers() {
    MarkersCoords transformed_markers(5);

    for (size_t i = 0; i < 5; ++i) {
        cv::perspectiveTransform(markers_[i], transformed_markers[i], homography_matrix_);
    }

    const auto bot_center =
        (transformed_markers[kBOT_MARKER_ID][0] + transformed_markers[kBOT_MARKER_ID][2]) / 2;

    bot_pos_.x = bot_center.x;
    bot_pos_.y = bot_center.y;

    return transformed_markers;
}

void CameraNode::publishImage(const cv::Mat& warped_image) {
    sensor_msgs::msg::Image raw_msg;
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", warped_image).toImageMsg(raw_msg);
    image_publisher_->publish(raw_msg);
}

void CameraNode::publishImageBorder(const std::vector<cv::Point2f>& border) {
    const auto image_border_msg = getImageMarkerPoseMsg(border);
    image_border_publisher_->publish(image_border_msg);
}

void CameraNode::publishRobotEgo() {
    const auto ego_msg = getImagePoseMsg(bot_pos_);
    robot_ego_publisher_->publish(ego_msg);
}

void CameraNode::publishImageCorners(const MarkersCoords& markers) {
    const auto image_corners_msg = getImageMarkerPoseArrayMsg(markers);
    image_corners_publisher_->publish(image_corners_msg);
}

void CameraNode::publishPreview(const cv::Mat& warped_image) {
    sensor_msgs::msg::CompressedImage compressed_msg;
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", warped_image)
        .toCompressedImageMsg(compressed_msg);
    preview_publisher_->publish(compressed_msg);
}

void CameraNode::publishPreviewCorners(const MarkersCoords& markers) {
    const auto preview_corners_msg = getMarkerArrayMsg(markers);
    preview_corners_publisher_->publish(preview_corners_msg);
}

void CameraNode::publishRobotBorder(const std::vector<cv::Point2f>& border) {
    const auto robot_border_msg = getMarkerMsg(border, 1);
    preview_border_publisher_->publish(robot_border_msg);
}

std::vector<cv::Point2f> CameraNode::getBorder(const std::vector<cv::Point2f>& marker) {
    cv::Point2f norm_vec = marker[0] - marker[3];
    float marker_pixel_size = cv::norm(norm_vec);
    bot_pos_.theta = 180 * atan(norm_vec.y / norm_vec.x) / (CV_PI);

    float scale = marker_pixel_size / kBotMarkerSize;

    int base_pixel_lenght = ceil<int>(kBaseLength * scale);
    int base_pixel_width = ceil<int>(kBaseWidth * scale);

    cv::RotatedRect border_rect = cv::RotatedRect(
        cv::Point2f(bot_pos_.x, bot_pos_.y),
        cv::Size2f(base_pixel_lenght, base_pixel_width),
        bot_pos_.theta);

    cv::Point2f rect_points[4];
    border_rect.points(rect_points);

    std::vector<cv::Point2f> border(rect_points, rect_points + 4);

    return border;
}

void CameraNode::handleCameraOnTimer() {
    const cv::Mat raw_image = getImage();

    const cv::Mat undistorted_image = removeDistortion(raw_image);

    tryUpdateHomography(undistorted_image);

    const cv::Mat warped_image = warp(undistorted_image);
    const MarkersCoords transformed_markers = transformMarkers();
    const std::vector<cv::Point2f> border = getBorder(transformed_markers[kBOT_MARKER_ID]);

    publishImage(warped_image);
    publishImageCorners(transformed_markers);
    publishImageBorder(border);
    publishRobotEgo();

    publishPreview(warped_image);
    publishRobotBorder(border);
    publishPreviewCorners(transformed_markers);
}

}  // namespace wbb