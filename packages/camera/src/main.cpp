#include <camera/camera_visualization.hpp>

bool import_camera_calibration(cv::Matx33f& camera_matrix, cv::Vec<float, 5>& distance_coefficients) {
    std::ifstream input_file("/wbb/calibration/camera_calibration_coefs");

    if (!input_file) {
        return false;
    }

    for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
            float value;
            input_file >> value;
            camera_matrix(i, j) = value;
        }
    }

    for (size_t i = 0; i < 5; ++i) {
        float value;
        input_file >> value;
        distance_coefficients[i] = value;
    }

    input_file.close();
    return true;
};

int main(int argc, char* argv[]) {
    cv::VideoCapture input;
    input.open(0);

    if (!input.isOpened()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to open the camera");
        return -1;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Camera opened successfully");

    cv::Matx33f camera_matrix;
    cv::Vec<float, 5> distance_coefficients;

    if (!import_camera_calibration(camera_matrix, distance_coefficients)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unable to open the calibration file");
        return -1;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Calibration file opened successfully");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_unique<CamVisualization>(input, camera_matrix, distance_coefficients));
    rclcpp::shutdown();
    return 0;
}
