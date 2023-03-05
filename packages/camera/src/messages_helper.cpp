#include "camera/messages_helper.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include <chrono>

namespace wbb {

visualization_msgs::msg::ImageMarker getMarkerMsg(const std::vector<cv::Point2f>& coords, int id) {
    visualization_msgs::msg::ImageMarker msg;
    geometry_msgs::msg::Point point;
    std_msgs::msg::ColorRGBA color;

    for (size_t i = 0; i <= 4; ++i) {
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
        msg.outline_colors.push_back(color);

        point.x = coords[i % 4].x;
        point.y = coords[i % 4].y;
        msg.points.push_back(point);
    }

    msg.header = std_msgs::msg::Header();
    msg.ns = "marker";
    msg.id = id;
    msg.type = visualization_msgs::msg::ImageMarker::LINE_STRIP;
    msg.action = visualization_msgs::msg::ImageMarker::ADD;
    msg.outline_color = color;
    msg.scale = 1;
    msg.lifetime = rclcpp::Duration::from_seconds(0);
    return msg;
}

foxglove_msgs::msg::ImageMarkerArray getMarkerArrayMsg(
    const std::vector<std::vector<cv::Point2f>>& markers) {
    foxglove_msgs::msg::ImageMarkerArray msg;

    for (const auto& marker : markers) {
        const auto marker_msg = wbb::getMarkerMsg(marker, 0);
        msg.markers.push_back(marker_msg);
    }

    return msg;
}

wbb_interfaces::msg::ImagePose getImagePoseMsg(const wbb::BotPose& pos) {
    wbb_interfaces::msg::ImagePose msg;

    msg.x = pos.x;
    msg.y = pos.y;
    msg.theta = pos.theta;

    return msg;
}

wbb_interfaces::msg::ImageMarkerPose getImageMarkerPoseMsg(const std::vector<cv::Point2f>& marker) {
    wbb_interfaces::msg::ImageMarkerPose msg;

    for (const auto& point : marker) {
        msg.x.push_back(round<int>(point.x));
        msg.y.push_back(round<int>(point.y));
    }

    return msg;
}

wbb_interfaces::msg::ImageMarkerPoseArray getImageMarkerPoseArrayMsg(
    const std::vector<std::vector<cv::Point2f>>& markers) {
    wbb_interfaces::msg::ImageMarkerPoseArray msg;

    for (const auto& marker : markers) {
        const auto marker_msg = getImageMarkerPoseMsg(marker);
        msg.markers.push_back(marker_msg);
    }

    return msg;
}

}  // namespace wbb