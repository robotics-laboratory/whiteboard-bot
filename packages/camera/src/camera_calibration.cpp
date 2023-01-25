#include <camera/camera_calibration.hpp>
#include <camera/logger.hpp>

bool export_camera_calibration(
    std::string file_path, cv::Matx33f& camera_matrix, cv::Vec<float, 5>& distance_coefficients) {
    std::ofstream output_file(file_path);

    if (!output_file) {
        return false;
    }

    size_t rows = camera_matrix.rows;
    size_t columns = camera_matrix.cols;

    for (size_t i = 0; i < rows; i++) {
        for (size_t j = 0; j < columns; j++) {
            float value = camera_matrix(i, j);
            output_file << value << std::endl;
        }
    }

    rows = distance_coefficients.rows;

    for (size_t i = 0; i < rows; i++) {
        float value = distance_coefficients[i];
        output_file << value << std::endl;
    }

    output_file.close();
    return true;
}

int main(int argc, char** argv) {
    LOG_INIT_COUT();
    (void)argc;
    (void)argv;

    std::vector<cv::String> file_names;
    cv::glob(PATH_TO_IMAGES, file_names, false);

    std::vector<std::vector<cv::Point2f>> image_corners_coords(file_names.size());
    std::vector<std::vector<cv::Point3f>> world_corners_coords;

    std::vector<cv::Point3f> sq_points;
    for (int i = 1; i < board_size[1]; i++) {
        for (int j = 1; j < board_size[0]; j++) {
            sq_points.push_back(cv::Point3f(j, i, 0));
        }
    }

    size_t cur_index = 0;
    for (auto const& f : file_names) {
        std::cout << std::string(f) << std::endl;

        cv::Mat cur_image = cv::imread(file_names[cur_index]);
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

    log(LOG_INFO) << "Calibrating...\n";
    cv::calibrateCamera(
        world_corners_coords,
        image_corners_coords,
        frame_size,
        camera_matrix,
        distance_coefficients,
        rvecs,
        tvecs,
        flags);

    if (export_camera_calibration(
            "/wbb/calibration/camera_calibration_coefs", camera_matrix, distance_coefficients)) {
        log(LOG_INFO) << "Matrices saved successfully\n";
    } else {
        log(LOG_INFO) << "Unable to calibrate camera\n";
        return -1;
    }

    return 0;
}
