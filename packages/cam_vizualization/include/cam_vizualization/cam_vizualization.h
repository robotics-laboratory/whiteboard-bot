#pragma once

#include <chrono>
#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <opencv2/aruco.hpp>

class CamVizualization : public rclcpp::Node {
public:
    CamVizualization(cv::VideoCapture cam);

private:
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    cv::VideoCapture cam;
    cv::Ptr <cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
};
