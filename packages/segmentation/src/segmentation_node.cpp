#include "segmentation/segmentation_node.h"

#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/header.hpp>

#include <boost/assert.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include <chrono>
#include <stdio.h>

namespace {
int getIndex(int x, int y, int width) { return y * width + x; }
}  // namespace

namespace wbb {

SegmentationNode::SegmentationNode() : Node("SegmentationNode") {
    preview_publisher_ = this->create_publisher<foxglove_msgs::msg::ImageMarkerArray>(
        "/board/preview/segmetation", 10);

    segmentation_publisher_ =
        this->create_publisher<wbb_msgs::msg::SegmentationGrid>("/board/segmentation", 10);

    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/board/image", 10, std::bind(&SegmentationNode::handleImage, this, std::placeholders::_1));

    markers_subscription_ = this->create_subscription<wbb_msgs::msg::ImageMarkerPosArray>(
        "/board/image/corners",
        10,
        std::bind(&SegmentationNode::handleMarkers, this, std::placeholders::_1));

    bot_box_subscription_ = this->create_subscription<wbb_msgs::msg::ImageMarkerPos>(
        "/board/image/ego",
        10,
        std::bind(&SegmentationNode::handleBotBox, this, std::placeholders::_1));

    const auto segmentation_period =
        std::chrono::milliseconds(this->declare_parameter<int>("segmentation_period", 1000));
    timer_ = this->create_wall_timer(
        segmentation_period, std::bind(&SegmentationNode::makeSegmentationOnTimer, this));

    resolution_ = this->declare_parameter<int>("resolution", 30);
    delta_ = this->declare_parameter<int>("delta", 3);
}

void SegmentationNode::handleImage(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Get image");
    auto cv_image = cv_bridge::toCvShare(msg);
    board_image_ = cv_image->image;
}

void SegmentationNode::handleMarkers(wbb_msgs::msg::ImageMarkerPosArray msg) {
    RCLCPP_DEBUG(this->get_logger(), "Get markers");
    markers_ = toMarkerArray(msg);
}

void SegmentationNode::handleBotBox(wbb_msgs::msg::ImageMarkerPos msg) {
    RCLCPP_DEBUG(this->get_logger(), "Get bot box");
    bot_box_ = toBotBox(msg);
}

cv::Mat SegmentationNode::applyThreshold(const cv::Mat& image) {
    cv::Mat grayscale_image;

    cv::cvtColor(image, grayscale_image, cv::COLOR_BGR2GRAY);

    cv::Mat adapt_threshold_image;
    cv::adaptiveThreshold(
        grayscale_image,
        adapt_threshold_image,
        255,
        cv::ADAPTIVE_THRESH_GAUSSIAN_C,
        cv::THRESH_BINARY_INV,
        15,
        5);

    cv::Mat threshold_image;
    cv::threshold(grayscale_image, threshold_image, 100, 255, cv::THRESH_BINARY_INV);

    cv::Mat result_image;
    cv::max(threshold_image, adapt_threshold_image, result_image);

    if (bot_box_) {
        std::vector<cv::Point> border_coords;
        for (const auto& corner : bot_box_->corners) {
            border_coords.push_back(cv::Point(ceil<int>(corner.x), ceil<int>(corner.y)));
        }
        cv::fillConvexPoly(result_image, border_coords, cv::Scalar(0, 0, 0));
    }

    RCLCPP_DEBUG(this->get_logger(), "Thresholded successfully");

    return result_image;
}

SegmentationGrid SegmentationNode::segment(const cv::Mat& threshold_image) {
    int last_pixel_width = threshold_image.cols - 1;
    int last_pixel_height = threshold_image.rows - 1;

    SegmentationGrid segmentation;

    BOOST_ASSERT(threshold_image.cols % resolution_ == 0);
    BOOST_ASSERT(threshold_image.rows % resolution_ == 0);

    segmentation.resolution = resolution_;
    segmentation.width = threshold_image.cols / resolution_;
    segmentation.height = threshold_image.rows / resolution_;

    for (int i = 0; i < last_pixel_height; i += resolution_) {
        for (int j = 0; j < last_pixel_width; j += resolution_) {
            int min_x = j;
            int max_x = min_x + resolution_;

            int min_y = i;
            int max_y = min_y + resolution_;

            cv::Mat cur_part = threshold_image(cv::Range(min_y, max_y), cv::Range(min_x, max_x));

            double min, max;
            cv::minMaxLoc(cur_part, &min, &max);

            uint8_t cur_segment_flag = 0;

            if (min < max) {
                cur_segment_flag = 1;
            }

            segmentation.data.push_back(cur_segment_flag);
        }
    }

    std::vector<std::pair<int, int>> corner_segments_coords = {
        {0, 0},
        {segmentation.width - delta_, 0},
        {segmentation.width - delta_, segmentation.height - delta_},
        {0, segmentation.height - delta_}};

    for (int i = 0; i < delta_; ++i) {
        for (int j = 0; j < delta_; ++j) {
            for (const auto& corner : corner_segments_coords) {
                segmentation
                    .data[getIndex(corner.first + j, corner.second + i, segmentation.width)] = 0;
            }
        }
    }

    RCLCPP_DEBUG(this->get_logger(), "Segmentated successfully");

    return segmentation;
}

std::vector<std::vector<std::pair<int, int>>> SegmentationNode::getPreviewMarkerCoords(
    const SegmentationGrid& segmentation) {
    std::vector<std::vector<std::pair<int, int>>> preview_marker_coords{};

    for (int i = 0; i < segmentation.height; ++i) {
        for (int j = 0; j < segmentation.width; ++j) {
            if (!segmentation.data[getIndex(j, i, segmentation.width)]) {
                continue;
            }

            int x = j * segmentation.resolution;
            int y = i * segmentation.resolution;

            std::vector<std::pair<int, int>> cur_marker_coords = {
                {x, y},
                {x + segmentation.resolution, y},
                {x + segmentation.resolution, y + segmentation.resolution},
                {x, y + segmentation.resolution}};

            preview_marker_coords.push_back(cur_marker_coords);
        }
    }

    return preview_marker_coords;
}

void SegmentationNode::publishSegmentation(const SegmentationGrid& segmentation) {
    wbb_msgs::msg::SegmentationGrid segmentation_msg;

    segmentation_msg.width = segmentation.width;
    segmentation_msg.height = segmentation.height;
    segmentation_msg.resolution = segmentation.resolution;
    segmentation_msg.data = std::move(segmentation.data);

    segmentation_publisher_->publish(segmentation_msg);
}
void SegmentationNode::publishPreviewMarkers(const SegmentationGrid& segmentation) {
    std::vector<std::vector<std::pair<int, int>>> preview_marker_coords =
        getPreviewMarkerCoords(segmentation);
    foxglove_msgs::msg::ImageMarkerArray preview_msg = msg::makePolygonArray(preview_marker_coords);

    preview_publisher_->publish(preview_msg);
}

void SegmentationNode::makeSegmentationOnTimer() {
    if (board_image_.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "Got empty image. Skipping segmentation...");
        return;
    }

    cv::Mat threshold_image = applyThreshold(board_image_);

    SegmentationGrid segmentation = segment(threshold_image);

    publishSegmentation(segmentation);
    publishPreviewMarkers(segmentation);
}

}  // namespace wbb
