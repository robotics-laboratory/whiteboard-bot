#pragma once

#include "detection/detection.h"
#include "wbb_msgs/msg/segmentation_grid.hpp"
#include "wbb_msgs/msg/image_marker_pos.hpp"
#include "wbb_msgs/msg/image_marker_pos_array.hpp"

#include <foxglove_msgs/msg/image_marker_array.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>

namespace wbb {

struct SegmentationGrid {
    int resolution = 0;
    int width = 0;
    int height = 0;
    std::vector<unsigned char> data{};
};

class SegmentationNode : public rclcpp::Node {
  public:
    SegmentationNode();

  private:
    void handleImage(sensor_msgs::msg::Image::ConstSharedPtr msg);
    void handleMarkers(wbb_msgs::msg::ImageMarkerPosArray msg);
    void handleBotBox(wbb_msgs::msg::ImageMarkerPos msg);

    cv::Mat applyThreshold(const cv::Mat& image);
    SegmentationGrid segment(const cv::Mat& threshold_image);
    std::vector<std::vector<std::pair<int, int>>> getPreviewMarkerCoords(
        const SegmentationGrid& segmentation);

    void makeSegmentationOnTimer();
    void publishSegmentation(const SegmentationGrid& segmentation);
    void publishPreviewMarkers(const SegmentationGrid& segmentation);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_ = nullptr;
    rclcpp::Subscription<wbb_msgs::msg::ImageMarkerPosArray>::SharedPtr markers_subscription_ =
        nullptr;
    rclcpp::Subscription<wbb_msgs::msg::ImageMarkerPos>::SharedPtr bot_box_subscription_ = nullptr;

    rclcpp::Publisher<foxglove_msgs::msg::ImageMarkerArray>::SharedPtr preview_publisher_ = nullptr;
    rclcpp::Publisher<wbb_msgs::msg::SegmentationGrid>::SharedPtr segmentation_publisher_ = nullptr;

    std::vector<Marker> markers_ = std::vector<Marker>(4);
    std::optional<Marker> bot_box_ = std::nullopt;
    cv::Mat board_image_{};
    int resolution_ = 0;
    int delta_ = 0;
};

}  // namespace wbb
