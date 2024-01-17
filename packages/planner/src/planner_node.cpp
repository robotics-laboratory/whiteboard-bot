#include "planner/planner_node.h"
#include "planner/graph_helper.h"
#include "wbb_msgs/msg/image_point.hpp"

#include <std_msgs/msg/header.hpp>

#include <stdio.h>

namespace {
int kTrajectoryId = 2;
}  // namespace

namespace wbb {

PlannerNode::PlannerNode() : Node("PlannerNode") {
    service_.erase = this->create_service<wbb_msgs::srv::Erase>(
        "/erase",
        std::bind(&PlannerNode::onErase, this, std::placeholders::_1, std::placeholders::_2));

    slot_.segmentation = this->create_subscription<wbb_msgs::msg::SegmentationGrid>(
        "/board/segmentation",
        10,
        std::bind(&PlannerNode::handleSegmentation, this, std::placeholders::_1));

    slot_.bot_pose = this->create_subscription<wbb_msgs::msg::ImagePose>(
        "/robot/ego", 10, std::bind(&PlannerNode::handleBotPose, this, std::placeholders::_1));

    signal_.path = this->create_publisher<wbb_msgs::msg::ImagePath>("/robot/motion", 10);
    signal_.trajectory =
        this->create_publisher<foxglove_msgs::msg::ImageMarkerArray>("/robot/motion/markers", 10);

    const auto period = std::chrono::milliseconds(this->declare_parameter<int>("period", 200));

    timer_.main = this->create_wall_timer(period, std::bind(&PlannerNode::publishFullState, this));
}

void PlannerNode::handleSegmentation(wbb_msgs::msg::SegmentationGrid msg) {
    segmentation_.width = std::move(msg.width);
    segmentation_.height = std::move(msg.height);
    segmentation_.resolution = std::move(msg.resolution);
    segmentation_.data = std::move(msg.data);
}

void PlannerNode::handleBotPose(wbb_msgs::msg::ImagePose msg) {
    bot_pose_.x = std::move(msg.x);
    bot_pose_.y = std::move(msg.y);
    bot_pose_.theta = std::move(msg.theta);
}

bool PlannerNode::isBotInSegment(const cv::Point& segment_center) {
    return (
        std::abs(bot_pose_.x - segment_center.x) * 2 <= segmentation_.resolution &&
        std::abs(bot_pose_.y - segment_center.y) * 2 <= segmentation_.resolution);
}

void PlannerNode::publishTrajectory() {
    foxglove_msgs::msg::ImageMarkerArray msg;

    std::vector<cv::Point2f> coords;

    if (!path_.empty()) {
        coords.emplace_back(bot_pose_.x, bot_pose_.y);

        for (const auto& point : path_) {
            coords.emplace_back(point.x, point.y);
        }
    }

    const auto line_msg =
        msg::makeLineStrip(kTrajectoryId, coords, Color(0, 0, 1, 1), 3, now(), false, true);
    const auto point_msg = msg::makePoints(kTrajectoryId, coords, Color(0, 0, 1, 1), 5);

    msg.markers.push_back(line_msg);
    msg.markers.push_back(point_msg);

    signal_.trajectory->publish(msg);
}

void PlannerNode::publishPath() {
    wbb_msgs::msg::ImagePath msg;

    for (const auto& point : path_) {
        wbb_msgs::msg::ImagePoint point_msg;

        point_msg.x = point.x;
        point_msg.y = point.y;

        msg.points.push_back(point_msg);
    }

    signal_.path->publish(msg);
}

void PlannerNode::publishFullState() {
    if (path_.empty()) {
        return;
    }

    if (isBotInSegment(path_.front())) {
        path_.pop_front();
    }

    publishPath();
    publishTrajectory();
}

void PlannerNode::onErase(
    const wbb_msgs::srv::Erase::Request::SharedPtr, wbb_msgs::srv::Erase::Response::SharedPtr) {
    path_.clear();

    const auto& path_vector = graph::getPath(bot_pose_, segmentation_);
    std::move(begin(path_vector), end(path_vector), back_inserter(path_));
}

}  // namespace wbb