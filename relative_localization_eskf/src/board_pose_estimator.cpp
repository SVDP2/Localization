#include "relative_localization_eskf/board_pose_estimator.hpp"

#include "relative_localization_eskf/geometry.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace relative_localization_eskf
{

namespace
{

Eigen::Isometry3d camera_from_board_transform(const cv::Vec3d & rvec, const cv::Vec3d & tvec)
{
  cv::Mat r_cv;
  cv::Rodrigues(rvec, r_cv);
  Eigen::Matrix3d r;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      r(row, col) = r_cv.at<double>(row, col);
    }
  }
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.linear() = r;
  tf.translation() = Eigen::Vector3d(tvec[0], tvec[1], tvec[2]);
  return tf;
}

void invert_observation(
  const cv::Vec3d & rvec_camera_from_board,
  const cv::Vec3d & tvec_camera_from_board,
  cv::Vec3d & rvec_board_from_camera,
  cv::Vec3d & tvec_board_from_camera)
{
  const Eigen::Isometry3d camera_from_board =
    camera_from_board_transform(rvec_camera_from_board, tvec_camera_from_board);
  const Eigen::Isometry3d board_from_camera = camera_from_board.inverse();

  Eigen::AngleAxisd aa(board_from_camera.linear());
  Eigen::Vector3d rotvec = aa.axis() * aa.angle();
  if (!std::isfinite(rotvec.norm())) {
    rotvec.setZero();
  }
  rvec_board_from_camera = cv::Vec3d(rotvec.x(), rotvec.y(), rotvec.z());
  tvec_board_from_camera = cv::Vec3d(
    board_from_camera.translation().x(),
    board_from_camera.translation().y(),
    board_from_camera.translation().z());
}

double rotation_delta_deg(const Eigen::Matrix3d & a, const Eigen::Matrix3d & b)
{
  const Eigen::AngleAxisd aa(a * b.transpose());
  return std::abs(aa.angle()) * 180.0 / M_PI;
}

}  // namespace

BoardDefinition BoardDefinition::from_yaml_file(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  BoardDefinition board;
  for (const auto & marker : root["markers"]) {
    const int id = marker["id"].as<int>();
    board.marker_sizes_m_[id] = marker["size"].as<double>() / 1000.0;
  }
  const auto positions = root["board_geometry"]["marker_positions"];
  for (const auto & item : positions) {
    const int id = item.first.as<int>();
    const auto p = item.second.as<std::vector<double>>();
    if (p.size() != 3) {
      throw std::runtime_error("marker position must contain 3 values");
    }
    board.marker_centers_m_[id] = Eigen::Vector3d(p[0], p[1], p[2]) / 1000.0;
  }
  if (root["board_geometry"]["description"]) {
    board.description_ = root["board_geometry"]["description"].as<std::string>();
  }
  return board;
}

std::vector<int> BoardDefinition::marker_ids() const
{
  std::vector<int> ids;
  ids.reserve(marker_sizes_m_.size());
  for (const auto & item : marker_sizes_m_) {
    ids.push_back(item.first);
  }
  return ids;
}

std::vector<cv::Point3d> BoardDefinition::marker_object_points(int marker_id) const
{
  const double half = marker_sizes_m_.at(marker_id) * 0.5;
  const Eigen::Vector3d c = marker_centers_m_.at(marker_id);
  return {
    cv::Point3d(c.x() - half, c.y() + half, c.z()),
    cv::Point3d(c.x() + half, c.y() + half, c.z()),
    cv::Point3d(c.x() + half, c.y() - half, c.z()),
    cv::Point3d(c.x() - half, c.y() - half, c.z()),
  };
}

std::vector<std::pair<int, std::vector<cv::Point2d>>> BoardDefinition::known_detections(
  const std::vector<std::vector<cv::Point2f>> & corners,
  const std::vector<int> & ids) const
{
  std::vector<std::pair<int, std::vector<cv::Point2d>>> detections;
  for (size_t i = 0; i < ids.size() && i < corners.size(); ++i) {
    const int id = ids[i];
    if (marker_sizes_m_.count(id) == 0 || marker_centers_m_.count(id) == 0) {
      continue;
    }
    std::vector<cv::Point2d> pts;
    pts.reserve(corners[i].size());
    for (const auto & p : corners[i]) {
      pts.emplace_back(p.x, p.y);
    }
    detections.emplace_back(id, pts);
  }
  return detections;
}

std::optional<BoardPoseEstimate> BoardDefinition::estimate_pose(
  const std::vector<std::vector<cv::Point2f>> & corners,
  const std::vector<int> & ids,
  const cv::Mat & camera_matrix,
  const cv::Mat & dist_coeffs,
  const BoardPoseEstimatorOptions & options,
  const std::optional<Eigen::Isometry3d> & previous_board_to_base,
  const std::optional<Eigen::Isometry3d> & reference_board_to_base,
  const Eigen::Isometry3d & camera_to_base)
{
  last_rejection_reason_.clear();
  const auto detections = known_detections(corners, ids);
  if (static_cast<int>(detections.size()) < std::max(1, options.min_markers)) {
    last_rejection_reason_ = "known_markers_below_min";
    return std::nullopt;
  }
  if (!previous_board_to_base && static_cast<int>(detections.size()) < options.min_markers_to_initialize) {
    last_rejection_reason_ = "need_more_markers_to_initialize";
    return std::nullopt;
  }

  std::vector<cv::Point3d> object_points;
  std::vector<cv::Point2d> image_points;
  for (const auto & detection : detections) {
    const auto obj = marker_object_points(detection.first);
    object_points.insert(object_points.end(), obj.begin(), obj.end());
    image_points.insert(image_points.end(), detection.second.begin(), detection.second.end());
  }

  std::vector<cv::Vec3d> rvecs;
  std::vector<cv::Vec3d> tvecs;
  std::vector<double> reprojection_errors;
  const cv::SolvePnPMethod method =
    detections.size() == 1 ? cv::SOLVEPNP_IPPE_SQUARE : cv::SOLVEPNP_IPPE;
  bool ok = cv::solvePnPGeneric(
    object_points,
    image_points,
    camera_matrix,
    dist_coeffs,
    rvecs,
    tvecs,
    false,
    method,
    cv::noArray(),
    cv::noArray(),
    reprojection_errors);
  if (!ok || rvecs.empty()) {
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    ok = cv::solvePnP(
      object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec, false,
      cv::SOLVEPNP_ITERATIVE);
    if (!ok) {
      last_rejection_reason_ = "solve_pnp_failed";
      return std::nullopt;
    }
    rvecs.push_back(rvec);
    tvecs.push_back(tvec);
  }

  double best_score = std::numeric_limits<double>::infinity();
  std::optional<BoardPoseEstimate> best;
  std::string last_candidate_rejection = "no_candidate_evaluated";
  for (size_t i = 0; i < rvecs.size(); ++i) {
    cv::Vec3d rvec_bc;
    cv::Vec3d tvec_bc;
    invert_observation(rvecs[i], tvecs[i], rvec_bc, tvec_bc);
    const Eigen::Isometry3d board_from_camera = board_pose_to_transform(rvec_bc, tvec_bc);
    const Eigen::Isometry3d board_from_base = board_from_camera * camera_to_base;

    if (board_from_camera.translation().z() < options.front_halfspace_min_z_m) {
      last_candidate_rejection =
        "view_gate_z z=" + std::to_string(board_from_camera.translation().z()) +
        " min=" + std::to_string(options.front_halfspace_min_z_m);
      continue;
    }
    const double distance = board_from_camera.translation().norm();
    const double view_angle = std::acos(
      std::clamp(board_from_camera.translation().z() / std::max(distance, 1.0e-9), -1.0, 1.0)) *
      180.0 / M_PI;
    if (view_angle > options.max_view_angle_deg) {
      last_candidate_rejection =
        "view_angle_gate angle_deg=" + std::to_string(view_angle) +
        " max=" + std::to_string(options.max_view_angle_deg);
      continue;
    }
    if (!passes_feasible_box(board_from_base, options)) {
      const Eigen::Isometry3d leader_from_base = transform_leader_rear_from_board() * board_from_base;
      const Eigen::Vector3d p = leader_from_base.translation();
      last_candidate_rejection =
        "feasible_box_gate x=" + std::to_string(p.x()) +
        " y=" + std::to_string(p.y()) +
        " z=" + std::to_string(p.z());
      continue;
    }
    if (previous_board_to_base && !passes_motion_gate(board_from_base, *previous_board_to_base, options)) {
      const double dp = (board_from_base.translation() - previous_board_to_base->translation()).norm();
      last_candidate_rejection =
        "motion_position_gate dp=" + std::to_string(dp) +
        " max=" + std::to_string(options.max_position_jump_m);
      continue;
    }
    if (
      reference_board_to_base &&
      options.reference_rotation_gate_deg > 0.0 &&
      rotation_delta_deg(board_from_base.linear(), reference_board_to_base->linear()) >
      options.reference_rotation_gate_deg)
    {
      const double dr = rotation_delta_deg(board_from_base.linear(), reference_board_to_base->linear());
      last_candidate_rejection =
        "reference_rotation_gate dr_deg=" + std::to_string(dr) +
        " max=" + std::to_string(options.reference_rotation_gate_deg);
      continue;
    }

    const double rmse = reprojection_rmse(
      object_points, image_points, rvecs[i], tvecs[i], camera_matrix, dist_coeffs);
    if (rmse > options.max_reprojection_rmse_px) {
      last_candidate_rejection =
        "reprojection_gate rmse=" + std::to_string(rmse) +
        " max=" + std::to_string(options.max_reprojection_rmse_px);
      continue;
    }

    double score = rmse;
    if (previous_board_to_base) {
      score += 2.0 * (board_from_base.translation() - previous_board_to_base->translation()).norm();
      score += 0.02 * rotation_delta_deg(board_from_base.linear(), previous_board_to_base->linear());
    }
    if (reference_board_to_base) {
      score += 0.02 * rotation_delta_deg(board_from_base.linear(), reference_board_to_base->linear());
    }
    if (score < best_score) {
      best_score = score;
      BoardPoseEstimate estimate;
      estimate.rvec_board_from_camera = rvec_bc;
      estimate.tvec_board_from_camera = tvec_bc;
      estimate.visible_markers = static_cast<int>(detections.size());
      estimate.reprojection_rmse_px = rmse;
      estimate.image_area_px = image_area_px(detections);
      estimate.used_single_marker_fallback = detections.size() == 1;
      best = estimate;
    }
  }

  if (!best) {
    last_rejection_reason_ = last_candidate_rejection;
  }
  return best;
}

bool BoardDefinition::passes_motion_gate(
  const Eigen::Isometry3d & board_to_base,
  const Eigen::Isometry3d & previous_board_to_base,
  const BoardPoseEstimatorOptions & options) const
{
  const double dp = (board_to_base.translation() - previous_board_to_base.translation()).norm();
  (void)previous_board_to_base;
  return dp <= options.max_position_jump_m;
}

bool BoardDefinition::passes_feasible_box(
  const Eigen::Isometry3d & board_to_base,
  const BoardPoseEstimatorOptions & options) const
{
  const Eigen::Isometry3d leader_from_base = transform_leader_rear_from_board() * board_to_base;
  const Eigen::Vector3d p = leader_from_base.translation();
  return p.x() >= options.feasible_x_min_m &&
    p.x() <= options.feasible_x_max_m &&
    std::abs(p.y()) <= options.feasible_abs_y_max_m &&
    p.z() >= options.feasible_z_min_m &&
    p.z() <= options.feasible_z_max_m;
}

double BoardDefinition::reprojection_rmse(
  const std::vector<cv::Point3d> & object_points,
  const std::vector<cv::Point2d> & image_points,
  const cv::Vec3d & rvec_camera_from_board,
  const cv::Vec3d & tvec_camera_from_board,
  const cv::Mat & camera_matrix,
  const cv::Mat & dist_coeffs) const
{
  std::vector<cv::Point2d> projected;
  cv::projectPoints(
    object_points, rvec_camera_from_board, tvec_camera_from_board, camera_matrix, dist_coeffs,
    projected);
  double sq = 0.0;
  for (size_t i = 0; i < projected.size() && i < image_points.size(); ++i) {
    const cv::Point2d d = projected[i] - image_points[i];
    sq += d.dot(d);
  }
  return std::sqrt(sq / std::max<size_t>(projected.size(), 1));
}

double BoardDefinition::image_area_px(
  const std::vector<std::pair<int, std::vector<cv::Point2d>>> & detections) const
{
  double area = 0.0;
  for (const auto & detection : detections) {
    const auto & points = detection.second;
    if (points.size() < 3) {
      continue;
    }
    double signed_area = 0.0;
    for (size_t i = 0; i < points.size(); ++i) {
      const auto & a = points[i];
      const auto & b = points[(i + 1) % points.size()];
      signed_area += a.x * b.y - b.x * a.y;
    }
    area += 0.5 * std::abs(signed_area);
  }
  return area;
}

Eigen::Isometry3d board_pose_to_transform(
  const cv::Vec3d & rvec_board_from_camera,
  const cv::Vec3d & tvec_board_from_camera)
{
  return camera_from_board_transform(rvec_board_from_camera, tvec_board_from_camera);
}

Eigen::Matrix<double, 6, 6> measurement_covariance_from_estimate(
  const BoardPoseEstimate & estimate)
{
  double lateral = 0.03;
  double vertical = 0.04;
  double depth = 0.08;
  double roll = 12.0 * M_PI / 180.0;
  double pitch = roll;
  double yaw = 8.0 * M_PI / 180.0;
  if (estimate.visible_markers >= 3) {
    lateral = 0.015;
    vertical = 0.020;
    depth = 0.040;
    roll = 8.0 * M_PI / 180.0;
    pitch = roll;
    yaw = 5.0 * M_PI / 180.0;
  } else if (estimate.visible_markers <= 1) {
    lateral = 0.070;
    vertical = 0.090;
    depth = 0.180;
    roll = 22.0 * M_PI / 180.0;
    pitch = roll;
    yaw = 14.0 * M_PI / 180.0;
  }

  const double reproj_scale = std::clamp(1.0 + 0.4 * std::max(estimate.reprojection_rmse_px - 0.5, 0.0), 1.0, 4.0);
  const double area_scale = std::clamp(std::sqrt(16000.0 / std::max(estimate.image_area_px, 1.0)), 1.0, 4.0);
  const double fallback_t = estimate.used_single_marker_fallback ? 1.6 : 1.0;
  const double fallback_r = estimate.used_single_marker_fallback ? 2.0 : 1.0;
  const double t_scale = reproj_scale * area_scale * fallback_t;
  const double r_scale = reproj_scale * area_scale * fallback_r;

  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
  cov(0, 0) = std::pow(lateral * t_scale, 2);
  cov(1, 1) = std::pow(vertical * t_scale, 2);
  cov(2, 2) = std::pow(depth * t_scale, 2);
  cov(3, 3) = std::pow(roll * r_scale, 2);
  cov(4, 4) = std::pow(pitch * r_scale, 2);
  cov(5, 5) = std::pow(yaw * r_scale, 2);
  return cov;
}

}  // namespace relative_localization_eskf
