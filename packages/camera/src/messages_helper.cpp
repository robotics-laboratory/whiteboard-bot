#include "camera/messages_helper.h"

#include <wbb_msgs/msg/image_point.hpp>

#include <std_msgs/msg/header.hpp>

#include <chrono>

namespace wbb::msg {

std_msgs::msg::Header getHeader(const rclcpp::Time& capturing_time) {
    std_msgs::msg::Header header;
    header.frame_id = "";
    header.stamp = capturing_time;

    return header;
}

visualization_msgs::msg::ImageMarker makeLineStrip(
    int id, const std::vector<cv::Point2f>& coords, const rclcpp::Time& capturing_time, bool close,
    bool add) {
    visualization_msgs::msg::ImageMarker msg;
    geometry_msgs::msg::Point point;

    std_msgs::msg::ColorRGBA color;
    color.r = 1.0;
    color.g = 0.0;
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

    msg.header = getHeader(capturing_time);
    msg.ns = "marker";
    msg.id = id;
    msg.type = visualization_msgs::msg::ImageMarker::LINE_STRIP;
    if (add) {
        msg.action = visualization_msgs::msg::ImageMarker::ADD;
    } else {
        msg.action = visualization_msgs::msg::ImageMarker::REMOVE;
    }
    msg.outline_color = color;
    msg.scale = 2;
    return msg;
}

foxglove_msgs::msg::ImageMarkerArray makeLineStripArray(
    int id, const std::vector<std::vector<cv::Point2f>>& markers,
    const rclcpp::Time& capturing_time, bool close, bool add) {
    foxglove_msgs::msg::ImageMarkerArray msg;

    for (const auto& marker : markers) {
        const auto marker_msg = makeLineStrip(id, marker, capturing_time, close, add);
        msg.markers.push_back(marker_msg);
    }

    return msg;
}

wbb_msgs::msg::ImagePose toImagePose(const wbb::BotPose& pose, const rclcpp::Time& capturing_time) {
    wbb_msgs::msg::ImagePose msg;

    msg.header = getHeader(capturing_time);
    msg.x = pose.x;
    msg.y = pose.y;
    msg.theta = pose.theta;

    return msg;
}

wbb_msgs::msg::ImageMarkerPos toImageMarkerPos(const Marker& marker) {
    wbb_msgs::msg::ImageMarkerPos msg;
    wbb_msgs::msg::ImagePoint point;

    for (const auto& corner : marker.corners) {
        point.x = round<int>(corner.x);
        point.y = round<int>(corner.y);
        msg.corners.push_back(point);
    }

    msg.id = marker.id;

    return msg;
}

wbb_msgs::msg::ImageMarkerPosArray toImageMarkerPosArray(
    const std::vector<Marker>& markers) {
    wbb_msgs::msg::ImageMarkerPosArray msg;

    for (const auto& marker : markers) {
        const auto marker_msg = toImageMarkerPos(marker);
        msg.markers.push_back(marker_msg);
    }

    return msg;
}

}  // namespace wbb::msg
