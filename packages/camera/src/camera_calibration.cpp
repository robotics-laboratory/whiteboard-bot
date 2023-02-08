#include <camera/params.h>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

void print_manual() {
    std::cout << "Usage: ./camera_calibration path_to_images path_to_calibration_file\n";
}

namespace {
auto& logger() {
    static auto logger = rclcpp::get_logger("CameraCalibration");
    return logger;
}
}  // namespace

void export_camera_calibration(
    const std::string& path_to_yaml, const IntrinsicCameraParameters& params) {
    std::ofstream output_file(path_to_yaml);

    if (!output_file) {
        RCLCPP_ERROR(logger(), "Unable to open output file");
        return;
    }

    YAML::Emitter output;

    output << YAML::BeginMap;

    output << YAML::Key << "camera_matrix";
    output << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            output << params.camera_matrix(i, j);
        }
    }

    output << YAML::EndSeq;

    output << YAML::Key << "distorsion";
    output << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < 5; ++i) {
        output << params.distorsion[i];
    }
    output << YAML::EndSeq;

    output << YAML::EndMap;

    output_file << output.c_str();
    output_file.close();
}

int main(int argc, char** argv) {
    if (argc != 3) {
        print_manual();
        RCLCPP_ERROR(logger(), "Bad arguments were given");
        return -1;
    }

    std::string path_to_images = argv[1];
    std::string path_to_calibration_file = argv[2];

    std::vector<cv::String> names;
    cv::glob(path_to_images + "/*.png", names, false);

    IntrinsicCameraParameters params = calirate(names);
    RCLCPP_INFO(logger(), "Calibrated successfully");

    export_camera_calibration(path_to_calibration_file, params);
    RCLCPP_INFO(logger(), "Matrices saved successfully");

    return 0;
}
