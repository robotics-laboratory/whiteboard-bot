#include <camera/camera_node.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_unique<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
