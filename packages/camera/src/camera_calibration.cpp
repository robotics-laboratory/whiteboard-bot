#include <camera/camera_calibration.hpp>

void print_manual(bool less) {
    if (less) {
        std::cout << "Error: the following arguments are required: path_to_images\n";
    } else {
        std::cout << "Error: too many arguments\n";
    }
    std::cout
        << "Usage: ros2 run camera camera_calibration path_to_images [path_to_calibration_file]\n";
}

bool export_camera_calibration(
    std::string file_path, cv::Matx33f& camera_matrix, cv::Vec<float, 5>& distance_coefficients) {
    std::ofstream output_file(file_path);

    if (!output_file) {
        return false;
    }

    YAML::Emitter output;

    output << YAML::BeginMap;

    output << YAML::Key << "camera_matrix";
    output << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            output << camera_matrix(i, j);
        }
    }
    output << YAML::EndSeq;

    output << YAML::Key << "distance_coefficients";
    output << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < 5; ++i) {
        output << distance_coefficients[i];
    }
    output << YAML::EndSeq;

    output << YAML::EndMap;

    output_file << output.c_str();
    output_file.close();

    return true;
}

int main(int argc, char** argv) {
    std::string path_to_images;
    std::string path_to_calibration_file = "/wbb/calibration/calibration_params.yaml";

    if (argc <= 1) {
        print_manual(true);
        RCLCPP_ERROR(
            rclcpp::get_logger("CameraCalibration"), "Not enough arguments for calibration");
        return -1;
    }

    if (argc >= 2) {
        path_to_images = argv[1];
    }

    if (argc == 3) {
        path_to_calibration_file = argv[2];
    } else if (argc > 3) {
        print_manual(false);
        RCLCPP_ERROR(rclcpp::get_logger("CameraCalibration"), "Too many arguments for calibration");
        return -1;
    }

    std::vector<cv::String> file_names;
    cv::glob(path_to_images + "/*.png", file_names, false);

    std::vector<std::vector<cv::Point2f>> image_corners_coords(file_names.size());
    std::vector<std::vector<cv::Point3f>> world_corners_coords;

    std::vector<cv::Point3f> sq_points;
    for (int i = 1; i < board_size[1]; i++) {
        for (int j = 1; j < board_size[0]; j++) {
            sq_points.push_back(cv::Point3f(j, i, 0));
        }
    }

    cv::Size frame_size(0, 0);

    size_t cur_index = 0;
    for (auto const& f : file_names) {
        std::cout << std::string(f) << std::endl;

        cv::Mat cur_image = cv::imread(file_names[cur_index]);

        if (frame_size == cv::Size(0, 0)) {
            frame_size = cv::Size(cur_image.cols, cur_image.rows);
        }

        cv::Mat gray_image;

        cv::cvtColor(cur_image, gray_image, cv::COLOR_RGB2GRAY);

        bool is_pattern_found = cv::findChessboardCorners(
            gray_image,
            pattern_size,
            image_corners_coords[cur_index],
            cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

        if (is_pattern_found) {
            std::cout << cur_index << std::endl;
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

    cv::Matx33f camera_matrix(cv::Matx33f::eye());
    cv::Vec<float, 5> distance_coefficients(0, 0, 0, 0, 0);

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
    int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 + cv::CALIB_ZERO_TANGENT_DIST +
                cv::CALIB_FIX_PRINCIPAL_POINT;

    RCLCPP_INFO(rclcpp::get_logger("CameraCalibration"), "Calibrating...");
    cv::calibrateCamera(
        world_corners_coords,
        image_corners_coords,
        frame_size,
        camera_matrix,
        distance_coefficients,
        rvecs,
        tvecs,
        flags);

    if (export_camera_calibration(path_to_calibration_file, camera_matrix, distance_coefficients)) {
        RCLCPP_INFO(rclcpp::get_logger("CameraCalibration"), "Matrices saved successfully");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("CameraCalibration"), "Unable to export calibration");
        return -1;
    }

    return 0;
}
