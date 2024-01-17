#pragma once

#include <wbb_msgs/msg/image_pose.hpp>
#include <wbb_msgs/msg/image_marker_pos.hpp>
#include <wbb_msgs/msg/image_marker_pos_array.hpp>

#include <foxglove_msgs/msg/image_marker_array.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/image_marker.hpp>

namespace wbb {

struct BotPose {
    int x = 0;
    int y = 0;
    float theta = 0.;
};

struct SegmentationGrid {
    int resolution = 0;
    int width = 0;
    int height = 0;
    std::vector<uint8_t> data{};
};

struct Color {
    Color() = default;
    Color(double r, double g, double b, double a) : r(r), g(g), b(b), a(a) {}

    double r = 0.;
    double g = 0.;
    double b = 0.;
    double a = 1.;
};

struct Marker {
    Marker() = default;
    Marker(int id, const std::vector<cv::Point2f>& corners) : id(id), corners(corners) {}

    int id = 0;
    std::vector<cv::Point2f> corners{};
};

struct DetectionResult {
    std::optional<Marker> ego = std::nullopt;
    std::vector<Marker> corners{};
};

Marker toMarker(const wbb_msgs::msg::ImageMarkerPos& msg);

std::vector<Marker> toMarkerArray(const wbb_msgs::msg::ImageMarkerPosArray& msg);

std::optional<Marker> toBotBox(const wbb_msgs::msg::ImageMarkerPos& msg);

namespace msg {

std_msgs::msg::Header makeHeader(const rclcpp::Time& capturing_time);

visualization_msgs::msg::ImageMarker makeLineStrip(
    int id, const std::vector<cv::Point2f>& coords, const Color& color, float scale,
    const rclcpp::Time& capturing_time, bool close = true, bool add = true);

foxglove_msgs::msg::ImageMarkerArray makeLineStripArray(
    int id, const std::vector<std::vector<cv::Point2f>>& markers, const Color& color, float scale,
    const rclcpp::Time& capturing_time, bool close = true, bool add = true);

visualization_msgs::msg::ImageMarker makePoints(
    int id, const std::vector<cv::Point2f>& coords, const Color& color, float scale);

visualization_msgs::msg::ImageMarker makePolygon(const std::vector<std::pair<int, int>>& coords);

foxglove_msgs::msg::ImageMarkerArray makePolygonArray(
    const std::vector<std::vector<std::pair<int, int>>>& markers_coords);

wbb_msgs::msg::ImagePose toImagePose(const wbb::BotPose& pose, const rclcpp::Time& capturing_time);

wbb_msgs::msg::ImageMarkerPos toImageMarkerPos(const Marker& marker);

wbb_msgs::msg::ImageMarkerPosArray toImageMarkerPosArray(const std::vector<Marker>& markers_coords);

}  // namespace msg

}  // namespace wbb
