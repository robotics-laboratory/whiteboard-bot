#include "camera/messages_helper.h"

#include <wbb_msgs/msg/image_point.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include <chrono>

namespace wbb::msg {

visualization_msgs::msg::ImageMarker makeLineStrip(
    int id, const std::vector<cv::Point2f>& coords, bool close) {
    visualization_msgs::msg::ImageMarker msg;
    geometry_msgs::msg::Point point;

    std_msgs::msg::ColorRGBA color;
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;
    color.a = 1.0;

    size_t amount = coords.size();
    if (close) {
        amount += 1;
    }

    for (size_t i = 0; i < amount; ++i) {
        msg.outline_colors.push_back(color);
        point.x = coords[i % coords.size()].x;
        point.y = coords[i % coords.size()].y;
        msg.points.push_back(point);
    }

    msg.header = std_msgs::msg::Header();
    msg.header.frame_id = "";
    // msg.header.stamp = now();
    msg.ns = "marker";
    msg.id = id;
    msg.type = visualization_msgs::msg::ImageMarker::LINE_STRIP;
    msg.action = visualization_msgs::msg::ImageMarker::ADD;
    msg.outline_color = color;
    msg.scale = 1;
    msg.lifetime = rclcpp::Duration::from_seconds(0);
    return msg;
}

foxglove_msgs::msg::ImageMarkerArray makeLineStripArray(
    const std::vector<std::vector<cv::Point2f>>& markers, bool close) {
    foxglove_msgs::msg::ImageMarkerArray msg;

    for (const auto& marker : markers) {
        const auto marker_msg = makeLineStrip(0, marker, close);
        msg.markers.push_back(marker_msg);
    }

    return msg;
}

wbb_msgs::msg::ImagePose toImagePose(const wbb::BotPose& pos) {
    wbb_msgs::msg::ImagePose msg;

    msg.x = pos.x;
    msg.y = pos.y;
    msg.theta = pos.theta;

    return msg;
}

wbb_msgs::msg::ImageMarkerPos toImageMarkerPos(const std::vector<cv::Point2f>& marker) {
    wbb_msgs::msg::ImageMarkerPos msg;
    wbb_msgs::msg::ImagePoint point;

    for (const auto& corner : marker) {
        point.x = round<int>(corner.x);
        point.y = round<int>(corner.y);
        msg.corners.push_back(point);
    }

    return msg;
}

wbb_msgs::msg::ImageMarkerPosArray toImageMarkerPosArray(
    const std::vector<std::vector<cv::Point2f>>& markers) {
    wbb_msgs::msg::ImageMarkerPosArray msg;

    for (const auto& marker : markers) {
        const auto marker_msg = toImageMarkerPos(marker);
        msg.markers.push_back(marker_msg);
    }

    return msg;
}

}  // namespace wbb::msg