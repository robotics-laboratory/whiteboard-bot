#pragma once

#include <chrono>
#include <cmath>
#include <functional>

#include <wbb_msgs/msg/image_path.hpp>
#include <wbb_msgs/msg/image_pose.hpp>
#include <wbb_msgs/msg/control.hpp>
#include <wbb_msgs/msg/image_pixel_scale.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/image_marker.hpp>

namespace wbb
{

using namespace std::placeholders;

class PurePursuit : public rclcpp::Node
{
  public:
    PurePursuit();
  private:
    void handleTrajectory(wbb_msgs::msg::ImagePath::SharedPtr trajectory);

    void handleBotPose(wbb_msgs::msg::ImagePose::SharedPtr bot_pose);

    void handlePixelScale(wbb_msgs::msg::ImagePixelScale::SharedPtr scale);

    double calculateDistance(wbb_msgs::msg::ImagePoint::SharedPtr first,
                             wbb_msgs::msg::ImagePose::SharedPtr second);

    double calculateCurvature(wbb_msgs::msg::ImagePoint::SharedPtr lookahead,
                              wbb_msgs::msg::ImagePose::SharedPtr bot_pose, double scale);

//    wbb_msgs::msg::ImagePoint findClosest(wbb_msgs::msg::ImagePath::SharedPtr trajectory,
//                                                     wbb_msgs::msg::ImagePose::SharedPtr bot_pose);

    wbb_msgs::msg::ImagePoint::SharedPtr checkSegment(wbb_msgs::msg::ImagePoint start,
                                                      wbb_msgs::msg::ImagePoint end,
                                                      wbb_msgs::msg::ImagePose::SharedPtr bot_pose);

    wbb_msgs::msg::ImagePoint::SharedPtr findLookahead(wbb_msgs::msg::ImagePath::SharedPtr trajectory,
                                                       wbb_msgs::msg::ImagePose::SharedPtr bot_pose);

    void visualizeLookahead(wbb_msgs::msg::ImagePoint::SharedPtr lookahead, double scale);

    void visualizeRadius(double curvature, wbb_msgs::msg::ImagePose::SharedPtr bot_pose, double scale);

    void visualizeLARadius(wbb_msgs::msg::ImagePose::SharedPtr bot_pose, double scale);

    void sendControlCommand();

    struct Slols
    {
        rclcpp::Subscription<wbb_msgs::msg::ImagePath>::SharedPtr trajectory = nullptr;
        rclcpp::Subscription<wbb_msgs::msg::ImagePose>::SharedPtr bot_pose = nullptr;
        rclcpp::Subscription<wbb_msgs::msg::ImagePixelScale>::SharedPtr scale = nullptr;
    } slot_;
    struct Signals
    {
        rclcpp::Publisher<wbb_msgs::msg::Control>::SharedPtr control = nullptr;
        rclcpp::Publisher<visualization_msgs::msg::ImageMarker>::SharedPtr visual = nullptr;
    } signal_;

    struct State
    {
        wbb_msgs::msg::ImagePath::SharedPtr trajectory = nullptr;
        wbb_msgs::msg::ImagePose::SharedPtr bot_pose = nullptr;
        wbb_msgs::msg::ImagePixelScale::SharedPtr scale = nullptr;
    } state_;

    std::chrono::duration<double> timeout_{0.20};
    double lookahead_distance;
    rclcpp::TimerBase::SharedPtr timer_ = nullptr;
};

} // namespace wbb




