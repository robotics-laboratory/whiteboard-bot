#pragma once

#include <iostream>
#include <fstream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

std::string PATH_TO_IMAGES = "/wbb/calibration/calibration_images/*.png";
cv::Size pattern_size(9, 6);  // Amount of cells on each side of chessboard
int board_size[2] = {pattern_size.width + 1, pattern_size.height + 1};
cv::Size frame_size(640, 480);  // Frame size

bool export_camera_calibration(
    std::string file_path, cv::Matx33f& camera_matrix, cv::Vec<float, 5>& distance_coefficients);
