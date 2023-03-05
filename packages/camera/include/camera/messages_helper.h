#pragma once

#include "camera/params.h"

#include <wbb_interfaces/msg/image_pose.hpp>
#include <wbb_interfaces/msg/image_marker_pose.hpp>
#include <wbb_interfaces/msg/image_marker_pose_array.hpp>

#include <foxglove_msgs/msg/image_marker_array.hpp>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/msg/image_marker.hpp>

namespace wbb {

visualization_msgs::msg::ImageMarker getMarkerMsg(const std::vector<cv::Point2f>& coords, int id);

foxglove_msgs::msg::ImageMarkerArray getMarkerArrayMsg(
    const std::vector<std::vector<cv::Point2f>>& markers_coords);

wbb_interfaces::msg::ImagePose getImagePoseMsg(const wbb::BotPose& pos);

wbb_interfaces::msg::ImageMarkerPose getImageMarkerPoseMsg(const std::vector<cv::Point2f>& marker);

wbb_interfaces::msg::ImageMarkerPoseArray getImageMarkerPoseArrayMsg(
    const std::vector<std::vector<cv::Point2f>>& markers_coords);

}  // namespace wbb