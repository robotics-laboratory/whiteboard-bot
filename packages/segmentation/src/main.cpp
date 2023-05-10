#include "segmentation/segmentation_node.h"

#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<wbb::SegmentationNode>());
    rclcpp::shutdown();
    return 0;
}
