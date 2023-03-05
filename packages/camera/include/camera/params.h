#pragma once

#include <opencv2/opencv.hpp>

namespace wbb {

struct IntrinsicCameraParameters {
    IntrinsicCameraParameters(
            const cv::Matx33f& camera_matrix,
            const cv::Vec<float, 5>& distortion)
        : camera_matrix(camera_matrix)
        , distortion(distortion)
    {}

    IntrinsicCameraParameters() = default;

    cv::Matx33f camera_matrix = cv::Matx33f::eye();
    cv::Vec<float, 5> distortion = {0, 0, 0, 0, 0};
};

struct BotPose {
    int x = 0;
    int y = 0;
    float theta = 0.;
};

IntrinsicCameraParameters importCameraCalibration(const std::string& path_to_yaml);

void exportCameraCalibration(
    const std::string& path_to_yaml,
    const IntrinsicCameraParameters& params);

IntrinsicCameraParameters calibrate(const std::vector<cv::String>& names);

}  // namespace wbb
