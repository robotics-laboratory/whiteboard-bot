#include <camera/params.h>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

void print_manual() {
    std::cout << "Usage: camera_calibration path_to_images path_to_calibration_file\n";
}

namespace {
auto& logger() {
    static auto logger = rclcpp::get_logger("CameraCalibration");
    return logger;
}
}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        RCLCPP_ERROR(logger(), "Bad arguments were given");
        print_manual();
        return -1;
    }

    const std::string path_to_images = argv[1];
    const std::string path_to_calibration_file = argv[2];

    std::vector<cv::String> names;
    cv::glob(path_to_images + "/*.png", names, false);

    const IntrinsicCameraParameters params = calibrate(names);
    RCLCPP_INFO(logger(), "Calibrated successfully");

    exportCameraCalibration(path_to_calibration_file, params);
    RCLCPP_INFO(logger(), "Matrices saved successfully");

    return 0;
}
