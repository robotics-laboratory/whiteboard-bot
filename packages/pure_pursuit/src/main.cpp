#include <rclcpp/rclcpp.hpp>

#include "pure_pursuit/pursuit.h"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<wbb::PurePursuit>());
    rclcpp::shutdown();
    return 0;
}
