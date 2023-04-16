#include "segmentation/segmentation_node.h"

#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/header.hpp>

#include <chrono>
#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace {

const int resolution = 30;  //  TODO: сделать скейл в зависимости от размеров стерки
const int delta = 30;  // TODO: сделать дельту зависимой от размера стерки

} // namespace

namespace wbb {

SegmentationNode::SegmentationNode() : Node("SegmentationNode") {
    image_publisher_ =
        this->create_publisher<sensor_msgs::msg::CompressedImage>("/board/preview/segmetation", 10);

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

    markers_ = std::vector<Marker>(4);
    bot_box_.corners = std::vector<cv::Point2f>(4);
}

void SegmentationNode::handleImage(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    auto cv_image = cv_bridge::toCvShare(msg);
    board_image_ = cv_image->image;
}

void SegmentationNode::handleMarkers(wbb_msgs::msg::ImageMarkerPosArray msg) {
    for (const auto& marker : msg.markers) {
        Marker cur_marker{};
        cur_marker.id = marker.id;

        for (const auto& corner : marker.corners) {
            cur_marker.corners.push_back(cv::Point(corner.x, corner.y));
        }
        markers_[cur_marker.id] = std::move(cur_marker);
    }
}

void SegmentationNode::handleBotBox(wbb_msgs::msg::ImageMarkerPos msg) {
    if (!msg.corners.size()) {
        return;
    }

    for (size_t i = 0; i < msg.corners.size(); ++i) {
        bot_box_.corners[i] = cv::Point(msg.corners[i].x, msg.corners[i].y);
    }
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
    return result_image;
}

void SegmentationNode::publishCompressedImage(const cv::Mat& image) {
    sensor_msgs::msg::CompressedImage compressed_msg;
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toCompressedImageMsg(compressed_msg);
    image_publisher_->publish(compressed_msg);
}

void SegmentationNode::makeSegmentationOnTimer() {
    if (board_image_.cols == 0) {
        return;
    }

    cv::Mat board_copy = board_image_.clone();
    cv::Mat threshold_image = applyThreshold(board_copy).clone();

    int frame_width = board_image_.cols - 1;
    int frame_height = board_image_.rows - 1;

    for (const auto& marker : markers_) {
        std::vector<cv::Point> rect_coords{};
        cv::Point corner = cv::Point(
            ceil<int>(marker.corners[(marker.id + 2) % 4].x),
            ceil<int>(marker.corners[(marker.id + 2) % 4].y));

        if (marker.id == 0 || marker.id == 3) {
            corner.x += delta;
        } else {
            corner.x -= delta;
        }
        corner.x = std::max(std::min(corner.x, frame_width), 0);

        if (marker.id == 0 || marker.id == 1) {
            corner.y += delta;
        } else {
            corner.y -= delta;
        }
        corner.y = std::max(std::min(corner.y, frame_height), 0);

        switch (marker.id) {
            case 0:
                rect_coords = {
                    cv::Point(0, 0), cv::Point(corner.x, 0), corner, cv::Point(0, corner.y)};
                break;
            case 1:
                rect_coords = {
                    cv::Point(corner.x, 0),
                    cv::Point(frame_width, 0),
                    cv::Point(frame_width, corner.y),
                    corner};
                break;
            case 2:
                rect_coords = {
                    corner,
                    cv::Point(frame_width, corner.y),
                    cv::Point(frame_width, frame_height),
                    cv::Point(corner.x, frame_height)};
                break;
            default:
                rect_coords = {
                    corner,
                    cv::Point(corner.x, frame_height),
                    cv::Point(0, frame_height),
                    cv::Point(0, corner.y)};
        }

        cv::fillConvexPoly(threshold_image, rect_coords, cv::Scalar(0, 0, 0));
    }

    if (bot_box_.corners.size() > 0) {
        std::vector<cv::Point> border_coords;
        for (const auto& corner : bot_box_.corners) {
            border_coords.push_back(cv::Point(ceil<int>(corner.x), ceil<int>(corner.y)));
        }
        cv::fillConvexPoly(threshold_image, border_coords, cv::Scalar(0, 0, 0));
    }

    wbb_msgs::msg::SegmentationGrid segmentation_msg;

    segmentation_msg.resolution = resolution;
    segmentation_msg.width = (frame_width + resolution) / resolution;
    segmentation_msg.height = (frame_height + resolution) / resolution;

    for (int i = 0; i * resolution < frame_height; ++i) {
        for (int j = 0; j * resolution < frame_width; ++j) {
            int min_x = j * resolution;
            int max_x = std::min((j + 1) * resolution, frame_width);

            int min_y = i * resolution;
            int max_y = std::min((i + 1) * resolution, frame_height);

            cv::Mat cur_part = threshold_image(cv::Range(min_y, max_y), cv::Range(min_x, max_x));

            double min, max;
            cv::minMaxLoc(cur_part, &min, &max);

            cv::Mat croped = board_copy(cv::Range(min_y, max_y), cv::Range(min_x, max_x));
            cv::Mat mat(croped.rows, croped.cols, board_copy.type(), cv::Scalar(0, 0, 255));

            if (min < max) {
                segmentation_msg.data.push_back(1);
                cv::addWeighted(croped, 0.6, mat, 0.4, 0, croped);
            } else {
                segmentation_msg.data.push_back(0);
            }
        }
    }

    publishCompressedImage(board_copy);
    segmentation_publisher_->publish(segmentation_msg);
}

}  // namespace wbb
