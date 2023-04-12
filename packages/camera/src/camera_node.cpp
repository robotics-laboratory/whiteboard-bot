#include "camera/camera_node.h"
#include "camera/messages_helper.h"

#include <std_msgs/msg/header.hpp>

#include <chrono>
#include <cmath>
#include <optional>
#include <stdio.h>

namespace wbb {

namespace {

const int kBotMarkerId = 4;
const float kBotMarkerSize = 0.1f;
const float kBaseLength = 0.2f;
const float kBaseWidth = 0.15f;
const int kCornersMsgId = 0;
const int kBotMsgId = 1;
const int kBorderId = 6;

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

    robot_ego_publisher_ = this->create_publisher<wbb_msgs::msg::ImagePose>("/robot/ego", 10);

    image_corners_publisher_ =
        this->create_publisher<wbb_msgs::msg::ImageMarkerPosArray>("/board/image/corners", 10);

    robot_border_publisher_ =
        this->create_publisher<wbb_msgs::msg::ImageMarkerPos>("/board/image/ego", 10);

    preview_corners_publisher_ =
        this->create_publisher<foxglove_msgs::msg::ImageMarkerArray>("/board/preview/corners", 10);

    preview_border_publisher_ =
        this->create_publisher<visualization_msgs::msg::ImageMarker>("/board/preview/ego", 10);

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

DetectionResult CameraNode::detectMarkers(const cv::Mat& undistorted_image) {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(undistorted_image, aruco_markers_dict_, corners, ids);

    DetectionResult detection;
    Marker marker;

    for (size_t i = 0; i < ids.size(); ++i) {
        marker.id = ids[i];
        marker.corners = corners[i];

        if (ids[i] == kBotMarkerId) {
            detection.ego = marker;
        } else {
            detection.corners.push_back(marker);
        }
    }

    std::sort(detection.corners.begin(), detection.corners.end(), [](auto left, auto right) {
        return left.id < right.id;
    });

    return detection;
}

void CameraNode::tryUpdateHomography(const DetectionResult& detection) {
    if (detection.corners.size() != 4) {
        RCLCPP_DEBUG(this->get_logger(), "Skip homography update!");
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Update homography");

    std::vector<cv::Point2f> src_vertices;

    for (size_t i = 0; i < detection.corners.size(); ++i) {
        src_vertices.push_back(detection.corners[i].corners[i]);
    }

    const std::vector<cv::Point2f> dst_vertices = {
        cv::Point(0, 0),
        cv::Point(frame_size_.width - 1, 0),
        cv::Point(frame_size_.width - 1, frame_size_.height - 1),
        cv::Point(0, frame_size_.height - 1)};

    homography_matrix_ = cv::findHomography(src_vertices, dst_vertices);
}

cv::Mat CameraNode::warp(const cv::Mat& undistorted_image) {
    cv::Mat warped_image(undistorted_image.size(), CV_32FC1);
    cv::warpPerspective(
        undistorted_image, warped_image, homography_matrix_, undistorted_image.size());
    return warped_image;
}

std::optional<BotPose> CameraNode::getBotPose(const std::optional<Marker>& ego) {
    BotPose bot_pose;

    if (!ego) {
        return std::nullopt;
    }

    const cv::Point2f bot_center = (ego->corners[0] + ego->corners[2]) / 2;
    bot_pose.x = bot_center.x;
    bot_pose.y = bot_center.y;

    cv::Point2f norm_vec = ego->corners[0] - ego->corners[3];
    bot_pose.theta = atan2(norm_vec.y, norm_vec.x);

    return bot_pose;
}

DetectionResult CameraNode::transform(const DetectionResult& detection) {
    DetectionResult transformed_detection;

    for (size_t i = 0; i < detection.corners.size(); ++i) {
        std::vector<cv::Point2f> transformed_marker(4, cv::Point(0, 0));
        cv::perspectiveTransform(
            detection.corners[i].corners, transformed_marker, homography_matrix_);
        transformed_detection.corners.push_back(
            Marker(detection.corners[i].id, transformed_marker));
    }

    if (detection.ego) {
        std::vector<cv::Point2f> transformed_marker(4, cv::Point(0, 0));
        cv::perspectiveTransform(detection.ego->corners, transformed_marker, homography_matrix_);
        transformed_detection.ego = Marker(detection.ego->id, transformed_marker);
    }
    RCLCPP_DEBUG(this->get_logger(), "Transformed successfully");
    return transformed_detection;
}

void CameraNode::publishImage(const cv::Mat& warped_image, const rclcpp::Time& capturing_time) {
    sensor_msgs::msg::Image raw_msg;
    cv_bridge::CvImage(msg::getHeader(capturing_time), "bgr8", warped_image).toImageMsg(raw_msg);
    image_publisher_->publish(raw_msg);
}

void CameraNode::publishImageBorder(const std::vector<cv::Point2f>& border) {
    Marker border_marker;
    border_marker.corners = border;
    border_marker.id = kBorderId;
    const auto image_border_msg = msg::toImageMarkerPos(border_marker);
    robot_border_publisher_->publish(image_border_msg);
}

void CameraNode::publishRobotEgo(
    const std::optional<BotPose>& bot_pose, const rclcpp::Time& capturing_time) {
    if (!bot_pose) {
        return;
    }

    const auto ego_msg = msg::toImagePose(*bot_pose, capturing_time);
    robot_ego_publisher_->publish(ego_msg);
}

void CameraNode::publishImageCorners(const std::vector<Marker>& markers) {
    const auto image_corners_msg = msg::toImageMarkerPosArray(markers);
    image_corners_publisher_->publish(image_corners_msg);
}

void CameraNode::publishPreview(const cv::Mat& warped_image, const rclcpp::Time& capturing_time) {
    sensor_msgs::msg::CompressedImage compressed_msg;
    cv_bridge::CvImage(msg::getHeader(capturing_time), "bgr8", warped_image)
        .toCompressedImageMsg(compressed_msg);
    preview_publisher_->publish(compressed_msg);
}

void CameraNode::publishPreviewCorners(
    const std::vector<Marker>& markers, const rclcpp::Time& capturing_time) {

    std::vector<std::vector<cv::Point2f>> coords;
    for (auto marker : markers) {
        coords.push_back(marker.corners);
    }
    const auto preview_corners_msg = msg::makeLineStripArray(kCornersMsgId, coords, capturing_time);
    preview_corners_publisher_->publish(preview_corners_msg);
}

void CameraNode::publishRobotBorder(
    const std::vector<cv::Point2f>& border, const rclcpp::Time& capturing_time) {
    visualization_msgs::msg::ImageMarker robot_border_msg;
    if (border.size() == 0) {
        robot_border_msg = msg::makeLineStrip(kBotMsgId, border, capturing_time, false, false);
    } else {
        robot_border_msg = msg::makeLineStrip(kBotMsgId, border, capturing_time);
    }

    preview_border_publisher_->publish(robot_border_msg);
}

std::vector<cv::Point2f> CameraNode::getBorder(const std::optional<Marker>& ego) {
    if (!ego) {
        return {};
    }

    cv::Point2f norm_vec = ego->corners[0] - ego->corners[3];
    const float marker_pixel_size = cv::norm(norm_vec);
    float scale = marker_pixel_size / kBotMarkerSize;

    BotPose bot_pose = *(getBotPose(ego));

    int base_pixel_lenght = ceil<int>(kBaseLength * scale);
    int base_pixel_width = ceil<int>(kBaseWidth * scale);

    cv::RotatedRect border_rect = cv::RotatedRect(
        cv::Point2f(bot_pose.x, bot_pose.y),
        cv::Size2f(base_pixel_lenght, base_pixel_width),
        bot_pose.theta * 180 / CV_PI);

    cv::Point2f rect_points[4];
    border_rect.points(rect_points);

    std::vector<cv::Point2f> border(rect_points, rect_points + 4);

    return border;
}

void CameraNode::handleCameraOnTimer() {
    const cv::Mat raw_image = getImage();
    rclcpp::Clock clock;
    rclcpp::Time capturing_time = clock.now();

    const cv::Mat undistorted_image = removeDistortion(raw_image);
    const DetectionResult detection = detectMarkers(undistorted_image);

    tryUpdateHomography(detection);

    cv::Mat warped_image = warp(undistorted_image);

    DetectionResult transformed_detection = transform(detection);

    const std::optional<BotPose> bot_pose = getBotPose(transformed_detection.ego);
    const std::vector<cv::Point2f> border = getBorder(transformed_detection.ego);

    publishImage(warped_image, capturing_time);
    publishImageCorners(transformed_detection.corners);
    publishImageBorder(border);
    publishRobotEgo(bot_pose, capturing_time);
    
    publishPreview(warped_image, capturing_time);
    publishRobotBorder(border, capturing_time);
    publishPreviewCorners(transformed_detection.corners, capturing_time);
}

}  // namespace wbb
