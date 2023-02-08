#pragma once

#include <opencv2/opencv.hpp>

struct IntrinsicCameraParameters {
    IntrinsicCameraParameters(const cv::Matx33f& camera_matrix, const cv::Vec<float, 5>& distorsion)
        : camera_matrix(camera_matrix), distorsion(distorsion){}
    IntrinsicCameraParameters() = default; 
    cv::Matx33f camera_matrix{cv::Matx33f::eye()};
    cv::Vec<float, 5> distorsion{0, 0, 0, 0, 0};
};

IntrinsicCameraParameters import_camera_calibration(const std::string& path_to_yaml);
void export_camera_calibration(const std::string& path_to_yaml, const IntrinsicCameraParameters& params);
IntrinsicCameraParameters calirate(const std::vector<cv::String>& names);
