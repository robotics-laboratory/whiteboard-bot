#include <camera/camera_visualization.hpp>

using namespace std::chrono_literals;

CamVisualization::CamVisualization(
    cv::VideoCapture camera, cv::Matx33f camera_matrix, cv::Vec<float, 5> distance_coefficients)
    : Node("CamVisualization")
    , camera(camera)
    , camera_matrix(camera_matrix)
    , distance_coefficients(distance_coefficients) {
    publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>("CamTopic", 10);
    timer = this->create_wall_timer(100ms, std::bind(&CamVisualization::timer_callback, this));
}

void CamVisualization::timer_callback() {
    cv_bridge::CvImagePtr cv_ptr;

    cv::Mat raw_image;
    camera.read(raw_image);

    cv::Size frame_size(raw_image.cols, raw_image.rows);
    cv::Mat map_x;
    cv::Mat map_y;

    cv::initUndistortRectifyMap(
        camera_matrix,
        distance_coefficients,
        cv::Matx33f::eye(),
        camera_matrix,
        frame_size,
        CV_32FC1,
        map_x,
        map_y);

    cv::Mat undistorted_image;
    cv::remap(raw_image, undistorted_image, map_x, map_y, cv::INTER_LINEAR);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::pair<int, std::vector<cv::Point2f>>> id_corners_marker;
    cv::aruco::detectMarkers(undistorted_image, aruco_markers_dict, corners, ids);

    if (ids.size() == 4) {
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

    cv::Mat warped_image;
    cv::warpPerspective(undistorted_image, warped_image, homography_matrix, undistorted_image.size());

    sensor_msgs::msg::CompressedImage msg;
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", warped_image).toCompressedImageMsg(msg);

    publisher->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "Frame published successfully");
}
