#include "cam_vizualization.h"

using namespace std::chrono_literals;

CamVizualization::CamVizualization(cv::VideoCapture cam) : Node("CamVizualization"), cam(cam) {
    publisher_ =
            this->create_publisher<sensor_msgs::msg::Image>("CamTopic", 10);
    timer_ = this->create_wall_timer(
            100ms, std::bind(&CamVizualization::timer_callback, this));
}

void CamVizualization::timer_callback() {
    cv_bridge::CvImagePtr cv_ptr;

    cv::Mat image;
    cam.read(image);
    std::vector<int> ids;
    std::vector <std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image, dictionary, corners, ids);

    if (ids.size() > 0) {
        cv::aruco::drawDetectedMarkers(image, corners, ids);
    }

    std::vector<uchar> buff;
    std::vector<int> param(2);
    param[0] = cv::IMWRITE_JPEG_QUALITY;
    param[1] = 80;
    cv::imencode(".jpg", mat, buff, param);

    sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image)
                    .toImageMsg();

    publisher_->publish(*msg.get());
    std::cout << "Published!" << std::endl;
}
