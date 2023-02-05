#include <camera/params.h>

#include <rclcpp/rclcpp.hpp>

static auto& logger() {
    static auto logger = rclcpp::get_logger("CameraCalibration");
    return logger;
}

IntrinsicCameraParameters calirate(const std::vector<cv::String>& calibrate) {
    std::vector<std::vector<cv::Point2f>> image_corners_coords(calibrate.size());
    std::vector<std::vector<cv::Point3f>> world_corners_coords;

    std::vector<cv::Point3f> sq_points;
    for (int i = 1; i < board_size[1]; i++) {
        for (int j = 1; j < board_size[0]; j++) {
            sq_points.push_back(cv::Point3f(j, i, 0));
        }
    }

    cv::Size frame_size(0, 0);

    size_t cur_index = 0;
    for (auto const& f : calibrate) {
        RCLCPP_DEBUG(logger(), "Image is processing now");

        cv::Mat cur_image = cv::imread(f);

        if (!cur_index) {
            frame_size = cv::Size(cur_image.cols, cur_image.rows);
        } else if (frame_size != cv::Size(cur_image.cols, cur_image.rows)) {
            RCLCPP_ERROR(logger(), "Image has different resolution");
            exit(-1);
        }

        cv::Mat gray_image;
        cv::cvtColor(cur_image, gray_image, cv::COLOR_RGB2GRAY);

        bool is_pattern_found = cv::findChessboardCorners(
            gray_image,
            pattern_size,
            image_corners_coords[cur_index],
            cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

        if (is_pattern_found) {
            cv::cornerSubPix(
                gray_image,
                image_corners_coords[cur_index],
                cv::Size(11, 11),
                cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            world_corners_coords.push_back(sq_points);
        }

        ++cur_index;
    }

    IntrinsicCameraParameters params(cv::Matx33f::eye(), cv::Vec<float, 5>(0, 0, 0, 0, 0));

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
    int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 + cv::CALIB_ZERO_TANGENT_DIST +
                cv::CALIB_FIX_PRINCIPAL_POINT;

    RCLCPP_INFO(logger(), "Calibrating...");
    cv::calibrateCamera(
        world_corners_coords,
        image_corners_coords,
        frame_size,
        params.camera_matrix,
        params.distorsion,
        rvecs,
        tvecs,
        flags);

    return params;
}
