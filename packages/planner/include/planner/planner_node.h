#pragma once

#include "detection/detection.h"
#include "planner/graph_helper.h"
#include "wbb_msgs/msg/segmentation_grid.hpp"
#include "wbb_msgs/msg/image_pose.hpp"
#include "wbb_msgs/msg/image_path.hpp"
#include "wbb_msgs/srv/erase.hpp"

#include <foxglove_msgs/msg/image_marker_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <deque>
#include <memory>

namespace wbb {

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    void onErase(
        const wbb_msgs::srv::Erase::Request::SharedPtr, wbb_msgs::srv::Erase::Response::SharedPtr);

    void handleSegmentation(wbb_msgs::msg::SegmentationGrid msg);
    void handleBotPose(wbb_msgs::msg::ImagePose msg);

    bool isBotInSegment(const cv::Point& segment_center);

    void publishTrajectory();
    void publishPath();
    void publishFullState();

    struct Services {
        rclcpp::Service<wbb_msgs::srv::Erase>::SharedPtr erase = nullptr;
    } service_;

    struct Slots {
        rclcpp::Subscription<wbb_msgs::msg::SegmentationGrid>::SharedPtr segmentation = nullptr;
        rclcpp::Subscription<wbb_msgs::msg::ImagePose>::SharedPtr bot_pose = nullptr;
    } slot_;

    struct Signals {
        rclcpp::Publisher<foxglove_msgs::msg::ImageMarkerArray>::SharedPtr trajectory = nullptr;
        rclcpp::Publisher<wbb_msgs::msg::ImagePath>::SharedPtr path = nullptr;
    } signal_;

    struct Timers {
        rclcpp::TimerBase::SharedPtr main = nullptr;
    } timer_;

    SegmentationGrid segmentation_{};
    BotPose bot_pose_{};
    std::deque<cv::Point> path_{};
};

}  // namespace wbb
