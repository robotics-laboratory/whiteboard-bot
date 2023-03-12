#pragma once

#include "camera/params.h"

#include <wbb_msgs/msg/image_pose.hpp>
#include <wbb_msgs/msg/image_marker_pos.hpp>
#include <wbb_msgs/msg/image_marker_pos_array.hpp>

#include <foxglove_msgs/msg/image_marker_array.hpp>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/msg/image_marker.hpp>

namespace wbb::msg {

visualization_msgs::msg::ImageMarker makeLineStrip(
    int id, const std::vector<cv::Point2f>& coords, bool close = true);

foxglove_msgs::msg::ImageMarkerArray makeLineStripArray(
    const std::vector<std::vector<cv::Point2f>>& markers, bool close = true);

wbb_msgs::msg::ImagePose toImagePose(const wbb::BotPose& pos);

wbb_msgs::msg::ImageMarkerPos toImageMarkerPos(const std::vector<cv::Point2f>& marker);

wbb_msgs::msg::ImageMarkerPosArray toImageMarkerPosArray(
    const std::vector<std::vector<cv::Point2f>>& markers_coords);

}  // namespace wbb::msg