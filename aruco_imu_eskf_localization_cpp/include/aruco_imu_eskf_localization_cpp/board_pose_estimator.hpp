#ifndef ARUCO_IMU_ESKF_LOCALIZATION_CPP__BOARD_POSE_ESTIMATOR_HPP_
#define ARUCO_IMU_ESKF_LOCALIZATION_CPP__BOARD_POSE_ESTIMATOR_HPP_

#include <Eigen/Dense>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace aruco_imu_eskf_localization_cpp
{

struct BoardPoseEstimate
{
  cv::Vec3d rvec_board_from_camera;
  cv::Vec3d tvec_board_from_camera;
  int visible_markers{0};
  double reprojection_rmse_px{0.0};
  double image_area_px{0.0};
  bool used_single_marker_fallback{false};
};

struct BoardPoseEstimatorOptions
{
  int min_markers{2};
  int min_markers_to_initialize{2};
  double max_position_jump_m{0.35};
  double max_rotation_jump_deg{55.0};
  double max_heading_jump_deg{40.0};
  double max_reprojection_rmse_px{6.0};
  double front_halfspace_min_z_m{0.05};
  double max_view_angle_deg{75.0};
  double feasible_x_min_m{-3.50};
  double feasible_x_max_m{-0.10};
  double feasible_abs_y_max_m{1.00};
  double feasible_z_min_m{-0.50};
  double feasible_z_max_m{0.80};
  double reference_rotation_gate_deg{0.0};
};

class BoardDefinition
{
public:
  static BoardDefinition from_yaml_file(const std::string & path);

  std::vector<int> marker_ids() const;
  std::vector<cv::Point3d> marker_object_points(int marker_id) const;
  std::vector<std::pair<int, std::vector<cv::Point2d>>> known_detections(
    const std::vector<std::vector<cv::Point2f>> & corners,
    const std::vector<int> & ids) const;

  std::optional<BoardPoseEstimate> estimate_pose(
    const std::vector<std::vector<cv::Point2f>> & corners,
    const std::vector<int> & ids,
    const cv::Mat & camera_matrix,
    const cv::Mat & dist_coeffs,
    const BoardPoseEstimatorOptions & options,
    const std::optional<Eigen::Isometry3d> & previous_board_to_base,
    const std::optional<Eigen::Isometry3d> & reference_board_to_base,
    const Eigen::Isometry3d & camera_to_base);

  const std::string & last_rejection_reason() const {return last_rejection_reason_;}

private:
  std::map<int, double> marker_sizes_m_;
  std::map<int, Eigen::Vector3d> marker_centers_m_;
  std::string description_;
  std::string last_rejection_reason_;

  bool passes_motion_gate(
    const Eigen::Isometry3d & board_to_base,
    const Eigen::Isometry3d & previous_board_to_base,
    const BoardPoseEstimatorOptions & options) const;
  bool passes_feasible_box(
    const Eigen::Isometry3d & board_to_base,
    const BoardPoseEstimatorOptions & options) const;
  double reprojection_rmse(
    const std::vector<cv::Point3d> & object_points,
    const std::vector<cv::Point2d> & image_points,
    const cv::Vec3d & rvec_camera_from_board,
    const cv::Vec3d & tvec_camera_from_board,
    const cv::Mat & camera_matrix,
    const cv::Mat & dist_coeffs) const;
  double image_area_px(
    const std::vector<std::pair<int, std::vector<cv::Point2d>>> & detections) const;
};

Eigen::Isometry3d board_pose_to_transform(
  const cv::Vec3d & rvec_board_from_camera,
  const cv::Vec3d & tvec_board_from_camera);
Eigen::Matrix<double, 6, 6> measurement_covariance_from_estimate(
  const BoardPoseEstimate & estimate);

}  // namespace aruco_imu_eskf_localization_cpp

#endif  // ARUCO_IMU_ESKF_LOCALIZATION_CPP__BOARD_POSE_ESTIMATOR_HPP_
