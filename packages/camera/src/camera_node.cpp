#include <camera/camera_node.hpp>

#include <std_msgs/msg/header.hpp>

#include <chrono>
#include <stdio.h>

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

    preview_publisher_ =
        this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/compressed", 10);
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/raw", 10);

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
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(undistorted_image, aruco_markers_dict_, corners, ids);

    if (ids.size() != 4) {
        RCLCPP_DEBUG(this->get_logger(), "Skip homography update!");
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Update homography");

    std::vector<std::pair<int, std::vector<cv::Point2f>>> id_corners_marker;

    for (size_t i = 0; i < ids.size(); ++i) {
        id_corners_marker.push_back({ids[i], corners[i]});
    }

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

void CameraNode::publisRawImage(const cv::Mat& warped_image) {
    sensor_msgs::msg::Image raw_msg;
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", warped_image).toImageMsg(raw_msg);
    image_publisher_->publish(raw_msg);
}

void CameraNode::publishPreview(const cv::Mat& warped_image) {
    sensor_msgs::msg::CompressedImage compressed_msg;
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", warped_image)
        .toCompressedImageMsg(compressed_msg);
    preview_publisher_->publish(compressed_msg);
}

void CameraNode::handleCameraOnTimer() {
    const cv::Mat raw_image = getImage();

    const cv::Mat undistorted_image = removeDistortion(raw_image);

    tryUpdateHomography(undistorted_image);
    const cv::Mat warped_image = warp(undistorted_image);

    publisRawImage(warped_image);
    publishPreview(warped_image);
}
