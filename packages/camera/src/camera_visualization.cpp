#include <camera/camera_visualization.hpp>

using namespace std::chrono_literals;

CameraNode::CameraNode(CalibrationParams calibration_params)
    : Node("CameraNode"), calibration_params(calibration_params) {
    camera.open(0);

    if (!camera.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Unable to open the camera");
        exit(-1);
    }
    RCLCPP_INFO(this->get_logger(), "Camera opened successfully");

    cv::Mat frame;
    camera.read(frame);
    frame_size = cv::Size(frame.cols, frame.rows);

    compressed_publisher =
        this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/compressed", 10);
    raw_publisher = this->create_publisher<sensor_msgs::msg::Image>("camera/raw", 10);
    timer = this->create_wall_timer(100ms, std::bind(&CameraNode::handleCameraOnTimer, this));
}

void CameraNode::remapImage(cv::Mat& raw_image, cv::Mat& undistorted_image) {
    cv::Mat map_x;
    cv::Mat map_y;

    cv::initUndistortRectifyMap(
        calibration_params.camera_matrix,
        calibration_params.distance_coefficients,
        cv::Matx33f::eye(),
        calibration_params.camera_matrix,
        frame_size,
        CV_32FC1,
        map_x,
        map_y);

    cv::remap(raw_image, undistorted_image, map_x, map_y, cv::INTER_LINEAR);
    RCLCPP_DEBUG(this->get_logger(), "Image remapped successfully");
}

void CameraNode::updateHomography(
    std::vector<int>& ids, std::vector<std::vector<cv::Point2f>>& corners,
    std::vector<std::pair<int, std::vector<cv::Point2f>>>& id_corners_marker) {
    RCLCPP_DEBUG(this->get_logger(), "Starting recalculating of homography matrix");

    for (size_t i = 0; i < ids.size(); ++i) {
        id_corners_marker.push_back({ids[i], corners[i]});
    }

    std::sort(id_corners_marker.begin(), id_corners_marker.end(), [](auto left, auto right) {
        return left.first < right.first;
    });

    std::vector<cv::Point2f> src_vertices;

    for (size_t i = 0; i < ids.size(); ++i) {
        src_vertices.push_back(id_corners_marker[i].second[i]);
    }
    std::vector<cv::Point2f> dst_vertices = {
        cv::Point(0, 0),
        cv::Point(frame_size.width - 1, 0),
        cv::Point(frame_size.width - 1, frame_size.height - 1),
        cv::Point(0, frame_size.height - 1)};

    homography_matrix = cv::findHomography(src_vertices, dst_vertices);
    RCLCPP_DEBUG(this->get_logger(), "Homography matrix recalculated successfully");
}

void CameraNode::handleCameraOnTimer() {
    cv_bridge::CvImagePtr cv_ptr;

    cv::Mat raw_image;
    camera.read(raw_image);

    cv::Mat undistorted_image;

    remapImage(raw_image, undistorted_image);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::pair<int, std::vector<cv::Point2f>>> id_corners_marker;
    cv::aruco::detectMarkers(undistorted_image, aruco_markers_dict, corners, ids);

    if (ids.size() == 4) {
        updateHomography(ids, corners, id_corners_marker);
    }

    cv::Mat warped_image;
    cv::warpPerspective(
        undistorted_image, warped_image, homography_matrix, undistorted_image.size());

    sensor_msgs::msg::Image raw_msg;
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", warped_image).toImageMsg(raw_msg);
    raw_publisher->publish(raw_msg);
    RCLCPP_DEBUG(this->get_logger(), "Raw image published successfully");

    sensor_msgs::msg::CompressedImage compressed_msg;
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", warped_image)
        .toCompressedImageMsg(compressed_msg);
    compressed_publisher->publish(compressed_msg);
    RCLCPP_DEBUG(this->get_logger(), "Compressed image published successfully");
}
