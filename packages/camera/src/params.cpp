#include <camera/params.h>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

namespace wbb {

namespace {

const cv::Size pattern_size(9, 6);  // Amount of cells on each side of chessboard
const int board_size[2] = {pattern_size.width + 1, pattern_size.height + 1};

auto& logger() {
    static auto logger = rclcpp::get_logger("CameraCalibration");
    return logger;
}

}  // namespace

IntrinsicCameraParameters calibrate(const std::vector<cv::String>& names) {
    std::vector<std::vector<cv::Point2f>> image_corners_coords(names.size());
    std::vector<std::vector<cv::Point3f>> world_corners_coords;

    std::vector<cv::Point3f> sq_points;
    for (int i = 1; i < board_size[1]; i++) {
        for (int j = 1; j < board_size[0]; j++) {
            sq_points.push_back(cv::Point3f(j, i, 0));
        }
    }

    cv::Size frame_size(0, 0);

    for (size_t i = 0; i < names.size(); ++i) {
        const auto& name = names[i];

        RCLCPP_DEBUG(logger(), "Image: %s", name.c_str());

        const cv::Mat cur_image = cv::imread(name);

        if (!i) {
            frame_size = cv::Size(cur_image.cols, cur_image.rows);
        } else if (frame_size != cv::Size(cur_image.cols, cur_image.rows)) {
            RCLCPP_ERROR(logger(), "Image %s has different resolution", name.c_str());
            throw std::runtime_error("Image has different resolution");
        }

        cv::Mat gray_image;
        cv::cvtColor(cur_image, gray_image, cv::COLOR_RGB2GRAY);

        const bool is_pattern_found = cv::findChessboardCorners(
            gray_image,
            pattern_size,
            image_corners_coords[i],
            cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

        if (is_pattern_found) {
            cv::cornerSubPix(
                gray_image,
                image_corners_coords[i],
                cv::Size(11, 11),
                cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            world_corners_coords.push_back(sq_points);
        } else {
            RCLCPP_ERROR(logger(), "Unable find chessboard on image %s", name.c_str());
            throw std::runtime_error("Unable to find chessboard on image");
        }
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
        params.distortion,
        rvecs,
        tvecs,
        flags);

    return params;
}

IntrinsicCameraParameters importCameraCalibration(const std::string& path_to_yaml) {
    const YAML::Node config = YAML::LoadFile(path_to_yaml);

    cv::Matx33f camera_matrix;
    cv::Vec<float, 5> distortion;

    const std::vector<float> camera_matrix_values =
        config["camera_matrix"].as<std::vector<float>>();
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            camera_matrix(i, j) = camera_matrix_values[i * 3 + j];
        }
    }

    const std::vector<float> distortion_values = config["distortion"].as<std::vector<float>>();
    for (size_t i = 0; i < 5; ++i) {
        distortion[i] = distortion_values[i];
    }

    return IntrinsicCameraParameters(camera_matrix, distortion);
}

void exportCameraCalibration(
    const std::string& path_to_yaml, const IntrinsicCameraParameters& params) {
    std::ofstream output_file(path_to_yaml);

    if (!output_file) {
        RCLCPP_ERROR(logger(), "Unable to open output file");
        return;
    }

    YAML::Emitter output;

    output << YAML::BeginMap;

    output << YAML::Key << "camera_matrix";
    output << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            output << params.camera_matrix(i, j);
        }
    }

    output << YAML::EndSeq;

    output << YAML::Key << "distortion";
    output << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < 5; ++i) {
        output << params.distortion[i];
    }
    output << YAML::EndSeq;

    output << YAML::EndMap;

    output_file << output.c_str();
    output_file.close();
}

}  // namespace wbb