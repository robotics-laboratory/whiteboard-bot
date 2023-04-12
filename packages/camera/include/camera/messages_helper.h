#pragma once

#include "camera/params.h"

#include <wbb_msgs/msg/image_pose.hpp>
#include <wbb_msgs/msg/image_marker_pos.hpp>
#include <wbb_msgs/msg/image_marker_pos_array.hpp>

#include <foxglove_msgs/msg/image_marker_array.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/image_marker.hpp>

namespace wbb::msg {

std_msgs::msg::Header getHeader(const rclcpp::Time& capturing_time);

visualization_msgs::msg::ImageMarker makeLineStrip(
    int id, const std::vector<cv::Point2f>& coords, const rclcpp::Time& capturing_time,
    bool close = true, bool add = true);

foxglove_msgs::msg::ImageMarkerArray makeLineStripArray(
    int id, const std::vector<std::vector<cv::Point2f>>& markers,
    const rclcpp::Time& capturing_time, bool close = true, bool add = true);

wbb_msgs::msg::ImagePose toImagePose(const wbb::BotPose& pose, const rclcpp::Time& capturing_time);

wbb_msgs::msg::ImageMarkerPos toImageMarkerPos(const Marker& marker);

wbb_msgs::msg::ImageMarkerPosArray toImageMarkerPosArray(
    const std::vector<Marker>& markers_coords);

}  // namespace wbb::msg