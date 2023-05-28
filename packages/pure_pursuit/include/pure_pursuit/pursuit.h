#pragma once

#include <chrono>

#include <wbb_msgs/msg/image_path.hpp>
#include <wbb_msgs/msg/image_pose.hpp>
#include <wbb_msgs/msg/control.hpp>
#include <rclcpp/rclcpp.hpp>

namespace wbb
{

class PurePursuit : public rclcpp::Node
{
  public:
    PurePursuit() : Node("pure_pursuit");
  private:
    void handleTrajectory(wbb_msgs::msg::ImagePath::SharedPtr trajectory);

    void handleBotPose(wbb_msgs::msg::ImagePos::SharedPtr bot_pose);

    void sendControlCommand();

    struct Slols
    {
        rclcpp::Subscription<wbb_msgs::msg::ImagePath>::SharedPtr trajectory = nullptr;
        rclcpp::Subscription<wbb_msgs::msg::ImagePose>::SharedPtr bot_pose = nullptr;
    } slot_;
    struct Signals
    {
        rclcpp::Publisher<wbb_msgs::msg::Control>::SharedPtr control = nullptr;
    } signal_;

    struct State
    {
        wbb_msgs::msg::ImagePath::SharedPtr trajectory = nullptr;
        wbb_msgs::msg::ImagePose::SharedPtr bot_pose = nullptr;
    } state_;

    std::chrono::duration<double> timeout_{0.20};
    rclcpp::TimerBase::SharedPtr timer_ = nullptr;
};

} // namespace wbb




