#include "aruco_imu_eskf_localization_cpp/camera_calibration.hpp"

#include <opencv2/calib3d.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <stdexcept>
#include <string>

namespace aruco_imu_eskf_localization_cpp
{

namespace
{

void collect_numeric_data(const YAML::Node & node, std::vector<double> & out)
{
  if (!node.IsSequence()) {
    out.push_back(node.as<double>());
    return;
  }
  for (const auto & child : node) {
    collect_numeric_data(child, out);
  }
}

std::vector<double> numeric_data(const YAML::Node & node)
{
  std::vector<double> out;
  collect_numeric_data(node, out);
  return out;
}

}  // namespace

CameraCalibration load_camera_calibration(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  CameraCalibration calibration;
  calibration.camera_model = root["camera_model"] ? root["camera_model"].as<std::string>() : "fisheye";
  calibration.used_legacy_default = !root["camera_model"];
  if (calibration.camera_model != "fisheye") {
    throw std::runtime_error("unsupported camera_model " + calibration.camera_model);
  }

  const auto camera_data = numeric_data(root["camera_matrix"]["data"]);
  if (camera_data.size() != 9) {
    throw std::runtime_error("camera_matrix.data must contain 9 values");
  }
  calibration.camera_matrix = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 9; ++i) {
    calibration.camera_matrix.at<double>(i / 3, i % 3) = camera_data.at(i);
  }

  const auto dist_data = numeric_data(root["distortion_coefficients"]["data"]);
  if (dist_data.size() != 4) {
    throw std::runtime_error("fisheye distortion_coefficients.data must contain 4 values");
  }
  calibration.distortion_coefficients = cv::Mat(4, 1, CV_64F);
  for (int i = 0; i < 4; ++i) {
    calibration.distortion_coefficients.at<double>(i, 0) = dist_data.at(i);
  }

  calibration.image_size = cv::Size(
    root["image_size"]["width"].as<int>(),
    root["image_size"]["height"].as<int>());
  return calibration;
}

FisheyeRectification build_fisheye_rectification(
  const CameraCalibration & calibration,
  double balance)
{
  FisheyeRectification rect;
  balance = std::clamp(balance, 0.0, 1.0);
  cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
    calibration.camera_matrix,
    calibration.distortion_coefficients,
    calibration.image_size,
    cv::Matx33d::eye(),
    rect.rectified_camera_matrix,
    balance);
  cv::fisheye::initUndistortRectifyMap(
    calibration.camera_matrix,
    calibration.distortion_coefficients,
    cv::Matx33d::eye(),
    rect.rectified_camera_matrix,
    calibration.image_size,
    CV_32FC1,
    rect.map1,
    rect.map2);
  rect.zero_distortion = cv::Mat::zeros(4, 1, CV_64F);
  return rect;
}

}  // namespace aruco_imu_eskf_localization_cpp
