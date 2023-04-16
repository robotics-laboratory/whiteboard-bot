#pragma once
#include "wbb_msgs/msg/segmentation_grid.hpp"
#include "wbb_msgs/msg/image_marker_pos.hpp"
#include "wbb_msgs/msg/image_marker_pos_array.hpp"
#include "camera/detection.h"

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include <memory>

namespace wbb {

class SegmentationNode : public rclcpp::Node {
  public:
    SegmentationNode();

  private:
    void handleImage(sensor_msgs::msg::Image::ConstSharedPtr msg);
    void handleMarkers(wbb_msgs::msg::ImageMarkerPosArray msg);
    void handleBotBox(wbb_msgs::msg::ImageMarkerPos msg);

    cv::Mat applyThreshold(const cv::Mat& image);

    void publishCompressedImage(const cv::Mat& image);

    void makeSegmentationOnTimer();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<wbb_msgs::msg::ImageMarkerPosArray>::SharedPtr
        markers_subscription_;
    rclcpp::Subscription<wbb_msgs::msg::ImageMarkerPos>::SharedPtr bot_box_subscription_;

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_publisher_;
    rclcpp::Publisher<wbb_msgs::msg::SegmentationGrid>::SharedPtr segmentation_publisher_;

    std::vector<Marker> markers_{};
    Marker bot_box_{};
    cv::Mat board_image_{};
};

}  // namespace wbb
