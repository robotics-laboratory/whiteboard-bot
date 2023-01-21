#include <cam_calibration/cam_calibration.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int checkerBoard[2] = {10, 7};
cv::Size frameSize(640, 480);

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
<<<<<<< HEAD
            float value = camera_matrix(i, j);
=======
            double value = camera_matrix(i, j);
>>>>>>> 338ddca948f7517ea16839be8f5b87a66a7f3c99
            output_file << value << std::endl;
        }
    }

<<<<<<< HEAD
    rows = distance_coefficients.rows;

    for (size_t i = 0; i < rows; i++) {
        double value = distance_coefficients[i];
=======
    columns = camera_matrix.cols;

    for (size_t j = 0; j < columns; j++) {
        double value = distance_coefficients[j];
>>>>>>> 338ddca948f7517ea16839be8f5b87a66a7f3c99
        output_file << value << std::endl;
    }
    output_file.close();
    return true;
}

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    std::vector<cv::String> files_name;
    cv::glob("/wbb/calibration/calibration_images/*.png", files_name, false);
    cv::Size patternSize(9, 6);
    std::vector<std::vector<cv::Point2f>> image_corners_coords(files_name.size());
    std::vector<std::vector<cv::Point3f>> world_corners_coords;

    std::vector<cv::Point3f> objp;
    for (int i = 1; i < checkerBoard[1]; i++) {
        for (int j = 1; j < checkerBoard[0]; j++) {
            objp.push_back(cv::Point3f(j, i, 0));
        }
    }

    std::size_t i = 0;
    for (auto const& f : files_name) {
        cv::Mat img = cv::imread(files_name[i]);
        cv::Mat gray;

        cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);

        bool patternFound = cv::findChessboardCorners(
            gray,
            patternSize,
            image_corners_coords[i],
            cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

        if (patternFound) {
            std::cout << "Found chess board on image " << i << std::endl;
            cv::cornerSubPix(
                gray,
                image_corners_coords[i],
                cv::Size(11, 11),
                cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            world_corners_coords.push_back(objp);
        }

        i++;
    }

    cv::Matx33f camera_matrix(cv::Matx33f::eye());
    cv::Vec<float, 5> distance_coefficients(0, 0, 0, 0, 0);

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
    int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 + cv::CALIB_ZERO_TANGENT_DIST +
                cv::CALIB_FIX_PRINCIPAL_POINT;

    std::cout << "Calibrating..." << std::endl;

<<<<<<< HEAD
=======
    std::cout << world_corners_coords.size() << " " << image_corners_coords.size() << std::endl;
>>>>>>> 338ddca948f7517ea16839be8f5b87a66a7f3c99
    float error = cv::calibrateCamera(
        world_corners_coords,
        image_corners_coords,
        frameSize,
        camera_matrix,
        distance_coefficients,
        rvecs,
        tvecs,
        flags);

    std::cout << "Reprojection error = " << error << "\nCamera matrix:\n"
              << camera_matrix << "\nDistance coefficients:\n"
              << distance_coefficients << std::endl;

    if (export_camera_calibration(
            "/wbb/calibration/camera_calibration_coefs", camera_matrix, distance_coefficients)) {
        std::cout << "Matrices successfully saved!" << std::endl;
    } else {
        std::cout << "Can't save matrices in given file!" << std::endl;
    }

    return 0;
}
