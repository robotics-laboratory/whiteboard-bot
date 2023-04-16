#pragma once

#include <opencv2/opencv.hpp>

namespace wbb {

struct BotPose {
    int x = 0;
    int y = 0;
    float theta = 0.;
};

struct Marker {
    Marker() = default;
    Marker(const int& id, const std::vector<cv::Point2f>& corners) : id(id), corners(corners) {}

    int id = 0;
    std::vector<cv::Point2f> corners{};
};

struct DetectionResult {
    std::optional<Marker> ego = std::nullopt;
    std::vector<Marker> corners{};
};

}  // namespace wbb
