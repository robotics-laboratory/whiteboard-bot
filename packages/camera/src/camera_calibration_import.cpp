#include <camera/params.h>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

static auto& logger() {
  static auto logger = rclcpp::get_logger("CalibrationImport");
  return logger;
}

IntrinsicCameraParameters import_camera_calibration(const std::string& path_to_yaml) {

    YAML::Node config = YAML::LoadFile(path_to_yaml);
    RCLCPP_INFO(logger(), "Calibration file opened successfully");

    cv::Matx33f camera_matrix;
    cv::Vec<float, 5> distorsion;

    std::vector<float> camera_matrix_values = config["camera_matrix"].as<std::vector<float>>();
        for (size_t i = 0; i < 3; ++i) {
            for (size_t j = 0; j < 3; ++j) {
                camera_matrix(i, j) = camera_matrix_values[i * 3 + j];
            }
        }
    RCLCPP_INFO(logger(), "Camera matrix imported successfully");

    std::vector<float> distorsion_values = config["distorsion"].as<std::vector<float>>();
        for (size_t i = 0; i < 5; ++i) {
            distorsion[i] = distorsion_values[i];
        }
    RCLCPP_INFO(logger(), "Distorsion coefficients imported successfully");

    return IntrinsicCameraParameters(camera_matrix, distorsion);
}
