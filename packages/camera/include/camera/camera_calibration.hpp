#pragma once

#include <camera/camera.hpp>

#include <iostream>
#include <fstream>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

cv::Size pattern_size(9, 6);  // Amount of cells on each side of chessboard
int board_size[2] = {pattern_size.width + 1, pattern_size.height + 1};

CalibrationParams export_camera_calibration(std::string path);
