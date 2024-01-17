#include "planner/planner_node.h"

#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_unique<wbb::PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
