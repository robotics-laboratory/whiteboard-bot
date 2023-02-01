#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

struct CalibrationParams {
    CalibrationParams(cv::Matx33f camera_matrix, cv::Vec<float, 5> distance_coefficients)
        : camera_matrix(camera_matrix), distance_coefficients(distance_coefficients);
    cv::Matx33f camera_matrix;
    cv::Vec<float, 5> distance_coefficients;
};
