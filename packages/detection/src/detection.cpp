#include "detection/detection.h"

#include <wbb_msgs/msg/image_point.hpp>

#include <boost/assert.hpp>
#include <std_msgs/msg/header.hpp>

#include <chrono>

namespace wbb {

Marker toMarker(const wbb_msgs::msg::ImageMarkerPos& msg) {
    BOOST_ASSERT(msg.corners.size() == 4);

    Marker marker;

    marker.id = msg.id;
    for (const auto& corner : msg.corners) {
        marker.corners.emplace_back(corner.x, corner.y);
    }

    return marker;
}

std::vector<Marker> toMarkerArray(const wbb_msgs::msg::ImageMarkerPosArray& msg) {
    std::vector<Marker> markers{};

    for (const auto& marker : msg.markers) {
        markers.push_back(toMarker(marker));
    }

    return markers;
}

std::optional<Marker> toBotBox(const wbb_msgs::msg::ImageMarkerPos& msg) {
    if (msg.corners.empty()) {
        return std::nullopt;
    }

    return toMarker(msg);
}

namespace msg {
std_msgs::msg::Header makeHeader(const rclcpp::Time& captured_time) {
    std_msgs::msg::Header header;
    header.frame_id = "";
    header.stamp = captured_time;

    return header;
}

visualization_msgs::msg::ImageMarker makeLineStrip(
    int id, const std::vector<cv::Point2f>& coords, const Color& color, float scale,
    const rclcpp::Time& captured_time, bool close, bool add) {
    visualization_msgs::msg::ImageMarker msg;

    msg.header = makeHeader(captured_time);
    msg.ns = "marker";
    msg.id = id;
    msg.type = visualization_msgs::msg::ImageMarker::LINE_STRIP;

    if (!add) {
        msg.action = visualization_msgs::msg::ImageMarker::REMOVE;
        return msg;
    }

    msg.action = visualization_msgs::msg::ImageMarker::ADD;

    geometry_msgs::msg::Point point;
    std_msgs::msg::ColorRGBA outline_color;

    outline_color.r = color.r;
    outline_color.g = color.g;
    outline_color.b = color.b;
    outline_color.a = color.a;

    msg.outline_color = outline_color;
    msg.scale = scale;

    size_t amount = coords.size();
    if (close) {
        amount += 1;
    }

    for (size_t i = 0; i < amount; ++i) {
        msg.outline_colors.push_back(outline_color);
        point.x = coords[i % coords.size()].x;
        point.y = coords[i % coords.size()].y;
        msg.points.push_back(point);
    }

    return msg;
}

foxglove_msgs::msg::ImageMarkerArray makeLineStripArray(
    int id, const std::vector<std::vector<cv::Point2f>>& markers, const Color& color, float scale,
    const rclcpp::Time& captured_time, bool close, bool add) {
    foxglove_msgs::msg::ImageMarkerArray msg;

    for (const auto& marker : markers) {
        const auto marker_msg = makeLineStrip(id, marker, color, scale, captured_time, close, add);
        msg.markers.push_back(marker_msg);
    }

    return msg;
}

visualization_msgs::msg::ImageMarker makePoints(
    int id, const std::vector<cv::Point2f>& coords, const Color& color, float scale) {
    visualization_msgs::msg::ImageMarker msg;

    msg.ns = "";
    msg.id = id;
    msg.type = visualization_msgs::msg::ImageMarker::POINTS;
    msg.action = visualization_msgs::msg::ImageMarker::ADD;

    geometry_msgs::msg::Point point;
    std_msgs::msg::ColorRGBA outline_color;

    outline_color.r = color.r;
    outline_color.g = color.g;
    outline_color.b = color.b;
    outline_color.a = color.a;

    msg.outline_color = outline_color;
    msg.scale = scale;

    for (const auto& point_coords : coords) {
        msg.outline_colors.push_back(outline_color);
        point.x = point_coords.x;
        point.y = point_coords.y;
        msg.points.push_back(point);
    }

    return msg;
}

visualization_msgs::msg::ImageMarker makePolygon(const std::vector<std::pair<int, int>>& coords) {
    visualization_msgs::msg::ImageMarker msg;

    msg.ns = "";
    msg.type = visualization_msgs::msg::ImageMarker::POLYGON;
    msg.action = visualization_msgs::msg::ImageMarker::ADD;

    std_msgs::msg::ColorRGBA color;

    color.r = 1.0;
    color.g = 0.0;
    color.b = 0.0;
    color.a = 0.4;

    msg.filled = 1;
    msg.outline_color = color;
    msg.fill_color = color;

    for (const auto& cur_point : coords) {
        geometry_msgs::msg::Point point;
        point.x = cur_point.first;
        point.y = cur_point.second;
        msg.points.push_back(point);
    }

    return msg;
}

foxglove_msgs::msg::ImageMarkerArray makePolygonArray(
    const std::vector<std::vector<std::pair<int, int>>>& markers_coords) {
    foxglove_msgs::msg::ImageMarkerArray msg;

    for (const auto& coords : markers_coords) {
        msg.markers.push_back(makePolygon(coords));
    }

    return msg;
}

wbb_msgs::msg::ImagePose toImagePose(const wbb::BotPose& pose, const rclcpp::Time& captured_time) {
    wbb_msgs::msg::ImagePose msg;

    msg.header = makeHeader(captured_time);
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

wbb_msgs::msg::ImageMarkerPosArray toImageMarkerPosArray(const std::vector<Marker>& markers) {
    wbb_msgs::msg::ImageMarkerPosArray msg;

    for (const auto& marker : markers) {
        const auto marker_msg = toImageMarkerPos(marker);
        msg.markers.push_back(marker_msg);
    }

    return msg;
}

}  // namespace msg

}  // namespace wbb
