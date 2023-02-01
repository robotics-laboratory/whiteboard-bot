#include <camera/camera_visualization.hpp>
#include <camera/camera_calibration_import.hpp>

int main(int argc, char* argv[]) {
    std::string path_to_calibration_file = "/wbb/calibration/calibration_params.yaml";

    CalibrationParams calibration_params = import_camera_calibration(path_to_calibration_file);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_unique<CameraNode>(calibration_params));
    rclcpp::shutdown();
    return 0;
}
