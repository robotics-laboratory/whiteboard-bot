#include <camera/camera_calibration_import.hpp>

CalibrationParams import_camera_calibration(std::string path) {
    YAML::Node config;
    try {
        config = YAML::LoadFile(path);
    } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to open calibration file");
        exit(1);
    }

    cv::Matx33f camera_matrix;
    cv::Vec<float, 5> distance_coefficients;
    if (config["camera_matrix"]) {
        std::vector<float> values = config["camera_matrix"].as<std::vector<float>>();
        for (size_t i = 0; i < 3; ++i) {
            for (size_t j = 0; j < 3; ++j) {
                camera_matrix(i, j) = values[i * 3 + j];
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Camera matrix imported successfully");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to import camera matrix");
        exit(1);
    }

    if (config["distance_coefficients"]) {
        std::vector<float> values = config["distance_coefficients"].as<std::vector<float>>();
        for (size_t i = 0; i < 5; ++i) {
            distance_coefficients[i] = values[i];
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Distance coefficients imported successfully");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to import camera matrix");
        exit(1);
    }

    return CalibrationParams(camera_matrix, distance_coefficients);
}