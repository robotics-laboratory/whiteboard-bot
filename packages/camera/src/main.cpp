#include <camera/camera_visualization.hpp>
#include <camera/logger.hpp>

bool import_camera_calibration(cv::Matx33f& camera_matrix, cv::Vec<float, 5>& distance_coefficients) {
    LOG_INIT_COUT();
    std::ifstream input_file("/wbb/calibration/camera_calibration_coefs");

    if (!input_file) {
        log(LOG_ERR) << "Unable to open calibration file\n";
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
    LOG_INIT_COUT();

    // log.set_log_level(LOG_DEBUG);

    cv::VideoCapture input;
    input.open(0);

    if (!input.isOpened()) {
        log(LOG_ERR) << "Unable to open the camera\n";
        return -1;
    }
    log(LOG_INFO) << "Camera opened successfully\n";

    cv::Matx33f camera_matrix;
    cv::Vec<float, 5> distance_coefficients;

    if (!import_camera_calibration(camera_matrix, distance_coefficients)) {
        log(LOG_ERR) << "Unable to open the calibration file\n";
        return -1;
    }
    log(LOG_INFO) << "Calibration file opened successfully\n";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_unique<CamVisualization>(input, camera_matrix, distance_coefficients));
    rclcpp::shutdown();
    return 0;
}
