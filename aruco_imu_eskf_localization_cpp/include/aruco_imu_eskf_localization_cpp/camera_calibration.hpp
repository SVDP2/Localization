#ifndef ARUCO_IMU_ESKF_LOCALIZATION_CPP__CAMERA_CALIBRATION_HPP_
#define ARUCO_IMU_ESKF_LOCALIZATION_CPP__CAMERA_CALIBRATION_HPP_

#include <opencv2/core.hpp>

#include <string>

namespace aruco_imu_eskf_localization_cpp
{

struct CameraCalibration
{
  std::string camera_model{"fisheye"};
  cv::Mat camera_matrix;
  cv::Mat distortion_coefficients;
  cv::Size image_size;
  bool used_legacy_default{false};
};

struct FisheyeRectification
{
  cv::Mat rectified_camera_matrix;
  cv::Mat zero_distortion;
  cv::Mat map1;
  cv::Mat map2;
};

CameraCalibration load_camera_calibration(const std::string & path);
FisheyeRectification build_fisheye_rectification(
  const CameraCalibration & calibration,
  double balance);

}  // namespace aruco_imu_eskf_localization_cpp

#endif  // ARUCO_IMU_ESKF_LOCALIZATION_CPP__CAMERA_CALIBRATION_HPP_
