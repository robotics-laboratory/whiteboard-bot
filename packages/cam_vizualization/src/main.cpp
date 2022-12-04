#include <rclcpp/rclcpp.hpp>
#include "cam_vizualization.h"

int main(int argc, char* argv[]) {
    cv::VideoCapture inputVideo;
    inputVideo.open(0);
    std::cout << "Camera is ok!" << std::endl;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CamVizualization>(inputVideo));
    rclcpp::shutdown();
    return 0;
}
