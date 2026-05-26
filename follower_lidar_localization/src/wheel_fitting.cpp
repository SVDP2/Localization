#include "follower_lidar_localization/wheel_fitting.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace follower_lidar_localization
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double norm(const Point2 & point)
{
  return std::hypot(point.x, point.y);
}

Point2 operator+(const Point2 & a, const Point2 & b)
{
  return {a.x + b.x, a.y + b.y};
}

Point2 operator-(const Point2 & a, const Point2 & b)
{
  return {a.x - b.x, a.y - b.y};
}

Point2 operator*(const Point2 & a, double scale)
{
  return {a.x * scale, a.y * scale};
}

double dot(const Point2 & a, const Point2 & b)
{
  return a.x * b.x + a.y * b.y;
}

double cross(const Point2 & a, const Point2 & b)
{
  return a.x * b.y - a.y * b.x;
}

Point2 normalized(const Point2 & point)
{
  const double n = norm(point);
  if (!std::isfinite(n) || n < 1.0e-9) {
    return {1.0, 0.0};
  }
  return {point.x / n, point.y / n};
}

double angle_of(const Point2 & point)
{
  return std::atan2(point.y, point.x);
}

bool finite_pose(const Pose2 & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

bool pose_in_gate(const Pose2 & pose, const FitConfig & config)
{
  return finite_pose(pose) &&
         pose.x >= config.pose_x_min_m &&
         pose.x <= config.pose_x_max_m &&
         std::abs(pose.y) <= config.pose_abs_y_max_m &&
         std::abs(normalize_angle(pose.yaw)) <= config.max_abs_yaw_rad;
}

std::optional<SegmentCandidate> fit_cluster(
  const std::vector<ScanPoint2> & scan_points,
  int begin,
  int end,
  const FitConfig & config)
{
  const int count = end - begin;
  if (count < config.min_cluster_points) {
    return std::nullopt;
  }

  Point2 mean;
  for (int i = begin; i < end; ++i) {
    mean = mean + scan_points[i].point;
  }
  mean = mean * (1.0 / static_cast<double>(count));

  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (int i = begin; i < end; ++i) {
    const Point2 d = scan_points[i].point - mean;
    covariance(0, 0) += d.x * d.x;
    covariance(0, 1) += d.x * d.y;
    covariance(1, 0) += d.y * d.x;
    covariance(1, 1) += d.y * d.y;
  }
  covariance /= std::max(1, count - 1);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return std::nullopt;
  }
  Eigen::Vector2d major = solver.eigenvectors().col(1);
  Point2 direction = normalized({major.x(), major.y()});

  double min_projection = std::numeric_limits<double>::max();
  double max_projection = std::numeric_limits<double>::lowest();
  double residual_sum = 0.0;
  for (int i = begin; i < end; ++i) {
    const Point2 d = scan_points[i].point - mean;
    const double projection = dot(d, direction);
    min_projection = std::min(min_projection, projection);
    max_projection = std::max(max_projection, projection);
    const double residual = cross(d, direction);
    residual_sum += residual * residual;
  }

  const double length = max_projection - min_projection;
  const double rms = std::sqrt(residual_sum / static_cast<double>(count));
  if (!std::isfinite(length) || !std::isfinite(rms) ||
    length < config.candidate_min_length_m ||
    length > config.candidate_max_length_m ||
    rms > config.candidate_max_rms_m)
  {
    return std::nullopt;
  }

  SegmentCandidate candidate;
  candidate.center = mean + direction * (0.5 * (min_projection + max_projection));
  candidate.direction = direction;
  candidate.start = mean + direction * min_projection;
  candidate.end = mean + direction * max_projection;
  candidate.length = length;
  candidate.rms = rms;
  candidate.point_count = count;
  return candidate;
}

struct Hypothesis
{
  Pose2 pose;
  std::vector<SegmentAssignment> assignments;
  int visible_segments{0};
  double mean_center_residual_m{0.0};
  double score{std::numeric_limits<double>::infinity()};
};

ModelSegment transform_model_segment(const ModelSegment & segment, const Pose2 & pose)
{
  ModelSegment transformed = segment;
  transformed.center = transform_point(pose, segment.center);
  transformed.direction = rotate_point(segment.direction, pose.yaw);
  return transformed;
}

Point2 inverse_transform_point(const Pose2 & pose, const Point2 & point)
{
  return rotate_point(point - Point2{pose.x, pose.y}, -pose.yaw);
}

int wheel_box_occupancy_count(
  const Pose2 & pose,
  const std::vector<ScanPoint2> & scan_points,
  const FitConfig & config)
{
  if (!config.enable_wheel_box_occupancy_bonus || config.wheel_box_occupancy_bonus <= 0.0) {
    return 0;
  }

  const double half_track = 0.5 * config.track_width_m;
  const double half_width = 0.5 * config.wheel_width_m;
  const double margin = std::max(0.0, config.wheel_box_margin_m);
  const int min_points = std::max(1, config.wheel_box_min_points);
  const std::array<Point2, 4> wheel_centers{
    Point2{0.0, half_track},
    Point2{0.0, -half_track},
    Point2{config.wheelbase_m, half_track},
    Point2{config.wheelbase_m, -half_track},
  };
  std::array<int, 4> point_counts{0, 0, 0, 0};

  for (const auto & sample : scan_points) {
    if (!sample.valid) {
      continue;
    }
    const Point2 local = inverse_transform_point(pose, sample.point);
    for (size_t i = 0; i < wheel_centers.size(); ++i) {
      const auto & center = wheel_centers[i];
      if (local.x >= center.x - config.wheel_radius_m - margin &&
        local.x <= center.x + config.wheel_radius_m + margin &&
        local.y >= center.y - half_width - margin &&
        local.y <= center.y + half_width + margin)
      {
        point_counts[i] += 1;
      }
    }
  }

  int occupied = 0;
  for (const int count : point_counts) {
    if (count >= min_points) {
      occupied += 1;
    }
  }
  return occupied;
}

Hypothesis evaluate_pose(
  const Pose2 & pose,
  const std::vector<ModelSegment> & model,
  const std::vector<SegmentCandidate> & candidates,
  const std::vector<ScanPoint2> & scan_points,
  const FitConfig & config,
  const std::optional<Pose2> & prior)
{
  Hypothesis hypothesis;
  hypothesis.pose = pose;
  if (!pose_in_gate(pose, config)) {
    return hypothesis;
  }

  std::vector<bool> candidate_used(candidates.size(), false);
  double distance_sum = 0.0;
  double score = 0.0;

  for (size_t model_index = 0; model_index < model.size(); ++model_index) {
    const auto transformed = transform_model_segment(model[model_index], pose);
    const double model_angle = angle_of(transformed.direction);

    int best_candidate = -1;
    double best_score = std::numeric_limits<double>::infinity();
    double best_distance = 0.0;
    double best_angle_error = 0.0;

    for (size_t candidate_index = 0; candidate_index < candidates.size(); ++candidate_index) {
      if (candidate_used[candidate_index]) {
        continue;
      }
      const auto & candidate = candidates[candidate_index];
      const double center_distance = norm(candidate.center - transformed.center);
      const double angle_error = line_angle_distance(model_angle, angle_of(candidate.direction));
      if (center_distance > config.assignment_max_center_distance_m ||
        angle_error > config.assignment_max_angle_error_rad)
      {
        continue;
      }
      const double candidate_score = center_distance + 0.20 * angle_error + 2.0 * candidate.rms;
      if (candidate_score < best_score) {
        best_score = candidate_score;
        best_candidate = static_cast<int>(candidate_index);
        best_distance = center_distance;
        best_angle_error = angle_error;
      }
    }

    if (best_candidate >= 0) {
      candidate_used[best_candidate] = true;
      hypothesis.assignments.push_back(
        SegmentAssignment{
            static_cast<int>(model_index),
            best_candidate,
            best_distance,
            best_angle_error});
      distance_sum += best_distance;
      score += best_score;
    }
  }

  hypothesis.visible_segments = static_cast<int>(hypothesis.assignments.size());
  if (hypothesis.visible_segments > 0) {
    hypothesis.mean_center_residual_m =
      distance_sum / static_cast<double>(hypothesis.visible_segments);
    score /= static_cast<double>(hypothesis.visible_segments);
  }

  if (prior) {
    score += config.prior_position_weight * pose_distance(pose, *prior);
    score += config.prior_yaw_weight * angle_distance(pose.yaw, prior->yaw);
  }
  const int occupied_wheel_boxes = wheel_box_occupancy_count(pose, scan_points, config);
  score -= config.wheel_box_occupancy_bonus * static_cast<double>(occupied_wheel_boxes);
  score -= 0.20 * static_cast<double>(hypothesis.visible_segments);
  hypothesis.score = score;
  return hypothesis;
}

std::vector<Pose2> make_pair_hypotheses(
  const ModelSegment & model,
  const SegmentCandidate & candidate)
{
  std::vector<Pose2> hypotheses;
  const double model_angle = angle_of(model.direction);
  const double candidate_angle = angle_of(candidate.direction);
  for (const double direction_flip : {0.0, kPi}) {
    Pose2 pose;
    pose.yaw = normalize_angle(candidate_angle + direction_flip - model_angle);
    const Point2 rotated_model_center = rotate_point(model.center, pose.yaw);
    pose.x = candidate.center.x - rotated_model_center.x;
    pose.y = candidate.center.y - rotated_model_center.y;
    hypotheses.push_back(pose);
  }
  return hypotheses;
}

}  // namespace

std::vector<ModelSegment> make_leader_wheel_model(const FitConfig & config)
{
  const double half_track = 0.5 * config.track_width_m;
  const double rear_edge_x = -config.wheel_radius_m;
  const double front_edge_x = config.wheelbase_m - config.wheel_radius_m;
  const Point2 lateral_direction{0.0, 1.0};
  const Point2 longitudinal_direction{1.0, 0.0};
  std::vector<ModelSegment> model{
    {"rear_left", {rear_edge_x, half_track}, lateral_direction, config.wheel_width_m},
    {"rear_right", {rear_edge_x, -half_track}, lateral_direction, config.wheel_width_m},
    {"front_left", {front_edge_x, half_track}, lateral_direction, config.wheel_width_m},
    {"front_right", {front_edge_x, -half_track}, lateral_direction, config.wheel_width_m},
  };
  if (config.enable_l_shape_segments) {
    const double left_inner_y = half_track - 0.5 * config.wheel_width_m;
    const double right_inner_y = -half_track + 0.5 * config.wheel_width_m;
    model.push_back(
      {"rear_left_inner", {0.0, left_inner_y}, longitudinal_direction,
        2.0 * config.wheel_radius_m});
    model.push_back(
      {"rear_right_inner", {0.0, right_inner_y}, longitudinal_direction,
        2.0 * config.wheel_radius_m});
    model.push_back(
      {"front_left_inner", {config.wheelbase_m, left_inner_y}, longitudinal_direction,
        2.0 * config.wheel_radius_m});
    model.push_back(
      {"front_right_inner", {config.wheelbase_m, right_inner_y}, longitudinal_direction,
        2.0 * config.wheel_radius_m});
  }
  return model;
}

std::vector<SegmentCandidate> extract_segment_candidates(
  const std::vector<ScanPoint2> & scan_points,
  const FitConfig & config)
{
  std::vector<SegmentCandidate> candidates;
  int cluster_begin = -1;
  Point2 previous;
  bool have_previous = false;

  auto close_cluster = [&](int end_index) {
      if (cluster_begin >= 0) {
        auto candidate = fit_cluster(scan_points, cluster_begin, end_index, config);
        if (candidate) {
          candidates.push_back(*candidate);
        }
      }
      cluster_begin = -1;
      have_previous = false;
    };

  for (int i = 0; i < static_cast<int>(scan_points.size()); ++i) {
    const auto & sample = scan_points[i];
    const bool in_roi = sample.valid &&
      sample.point.x >= config.roi_x_min_m &&
      sample.point.x <= config.roi_x_max_m &&
      std::abs(sample.point.y) <= config.roi_abs_y_max_m;

    if (!in_roi) {
      close_cluster(i);
      continue;
    }

    if (cluster_begin < 0) {
      cluster_begin = i;
    } else if (have_previous && norm(sample.point - previous) > config.cluster_gap_m) {
      close_cluster(i);
      cluster_begin = i;
    }
    previous = sample.point;
    have_previous = true;
  }
  close_cluster(static_cast<int>(scan_points.size()));
  return candidates;
}

FitResult fit_leader_wheel_pose(
  const std::vector<ScanPoint2> & scan_points,
  const FitConfig & config,
  const std::optional<Pose2> & prior)
{
  FitResult result;
  result.model_segments = make_leader_wheel_model(config);
  result.candidates = extract_segment_candidates(scan_points, config);

  if (result.candidates.empty()) {
    result.status = "NO_SEGMENT_CANDIDATES";
    return result;
  }

  std::vector<Hypothesis> hypotheses;
  for (const auto & model : result.model_segments) {
    for (const auto & candidate : result.candidates) {
      for (const auto & pose : make_pair_hypotheses(model, candidate)) {
        hypotheses.push_back(
          evaluate_pose(
            pose, result.model_segments, result.candidates, scan_points, config, prior));
      }
    }
  }

  if (prior) {
    hypotheses.push_back(
      evaluate_pose(
        *prior, result.model_segments, result.candidates, scan_points, config, prior));
  }

  auto best_it = std::max_element(
    hypotheses.begin(), hypotheses.end(),
    [](const Hypothesis & a, const Hypothesis & b) {
      if (a.visible_segments != b.visible_segments) {
        return a.visible_segments < b.visible_segments;
      }
      return a.score > b.score;
    });

  if (best_it == hypotheses.end()) {
    result.status = "NO_HYPOTHESES";
    return result;
  }

  result.pose = best_it->pose;
  result.assignments = best_it->assignments;
  result.visible_segments = best_it->visible_segments;
  result.mean_center_residual_m = best_it->mean_center_residual_m;
  result.score = best_it->score;
  result.valid = result.visible_segments >= config.min_visible_segments &&
    std::isfinite(result.score);
  result.status = result.valid ? "OK" : "INSUFFICIENT_VISIBLE_SEGMENTS";
  return result;
}

double normalize_angle(double angle_rad)
{
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

double angle_distance(double a_rad, double b_rad)
{
  return std::abs(normalize_angle(a_rad - b_rad));
}

double line_angle_distance(double a_rad, double b_rad)
{
  const double direct = angle_distance(a_rad, b_rad);
  const double flipped = angle_distance(a_rad, b_rad + kPi);
  return std::min(direct, flipped);
}

Point2 rotate_point(const Point2 & point, double yaw_rad)
{
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  return {c * point.x - s * point.y, s * point.x + c * point.y};
}

Point2 transform_point(const Pose2 & pose, const Point2 & point)
{
  return rotate_point(point, pose.yaw) + Point2{pose.x, pose.y};
}

Pose2 compose_pose(const Pose2 & a, const Pose2 & b)
{
  const Point2 rotated = rotate_point({b.x, b.y}, a.yaw);
  return {a.x + rotated.x, a.y + rotated.y, normalize_angle(a.yaw + b.yaw)};
}

Pose2 inverse_pose(const Pose2 & pose)
{
  const double inv_yaw = -pose.yaw;
  const Point2 inv_translation = rotate_point({-pose.x, -pose.y}, inv_yaw);
  return {inv_translation.x, inv_translation.y, normalize_angle(inv_yaw)};
}

double pose_distance(const Pose2 & a, const Pose2 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

WheelPoseTracker::WheelPoseTracker(const TrackerConfig & config)
: config_(config)
{
}

TrackerOutput WheelPoseTracker::update(const FitResult & result, double stamp_sec)
{
  predict(stamp_sec);

  if (!result.valid) {
    output_.measurement_applied = false;
    output_.status = result.status;
    if (!initialized_) {
      output_.mode = TrackerMode::Lost;
      output_.has_pose = false;
      output_.status = "LOST_" + result.status;
      return output_;
    }
    if (output_.mode != TrackerMode::Coasting || !have_coast_start_) {
      coast_start_stamp_sec_ = stamp_sec;
      have_coast_start_ = true;
    }
    output_.coasting_frames += 1;
    const double coast_sec = std::max(0.0, stamp_sec - coast_start_stamp_sec_);
    if (coast_sec > config_.max_coast_sec) {
      output_.mode = TrackerMode::Lost;
      output_.status = "LOST_" + result.status;
      output_.covariance_scale = 100.0;
    } else {
      output_.mode = TrackerMode::Coasting;
      output_.status = "COASTING_" + result.status;
      output_.covariance_scale = std::min(50.0, output_.covariance_scale * 1.8);
    }
    return output_;
  }

  if (!initialized_) {
    initialize(result.pose, stamp_sec, TrackerMode::Tracking, "TRACKING_INITIALIZED");
    output_.measurement_applied = true;
    return output_;
  }

  if (measurement_near_pose(
      result.pose, output_.pose, config_.tracking_gate_m, config_.tracking_yaw_gate_rad))
  {
    apply_tracking_measurement(result.pose, stamp_sec);
    return output_;
  }

  enter_reacquiring(result.pose, stamp_sec);
  return output_;
}

TrackerOutput WheelPoseTracker::predict_only(double stamp_sec)
{
  predict(stamp_sec);
  return output_;
}

void WheelPoseTracker::predict(double stamp_sec)
{
  if (!initialized_) {
    last_stamp_sec_ = stamp_sec;
    have_stamp_ = true;
    last_prediction_dt_sec_ = 0.0;
    return;
  }
  if (!have_stamp_) {
    last_stamp_sec_ = stamp_sec;
    have_stamp_ = true;
    last_prediction_dt_sec_ = 0.0;
    return;
  }
  const double dt = std::max(0.0, std::min(0.25, stamp_sec - last_stamp_sec_));
  last_prediction_dt_sec_ = dt;
  output_.pose.x += velocity_.x * dt;
  output_.pose.y += velocity_.y * dt;
  output_.pose.yaw = normalize_angle(output_.pose.yaw + velocity_.yaw * dt);
  last_stamp_sec_ = stamp_sec;
  have_stamp_ = true;
}

bool WheelPoseTracker::measurement_near_pose(
  const Pose2 & measurement,
  const Pose2 & reference,
  double position_gate,
  double yaw_gate) const
{
  return pose_distance(measurement, reference) <= position_gate &&
         angle_distance(measurement.yaw, reference.yaw) <= yaw_gate;
}

void WheelPoseTracker::initialize(
  const Pose2 & pose,
  double stamp_sec,
  TrackerMode mode,
  const std::string & status)
{
  output_.has_pose = true;
  output_.measurement_applied = false;
  output_.mode = mode;
  output_.pose = pose;
  output_.covariance_scale = 1.0;
  output_.coasting_frames = 0;
  output_.reacquire_hits = 0;
  output_.status = status;
  velocity_ = Pose2{};
  initialized_ = true;
  have_coast_start_ = false;
  last_stamp_sec_ = stamp_sec;
  have_stamp_ = true;
  last_prediction_dt_sec_ = 0.0;
}

void WheelPoseTracker::apply_tracking_measurement(const Pose2 & measurement, double stamp_sec)
{
  const double dt = std::max(1.0e-3, last_prediction_dt_sec_);
  const Pose2 previous = output_.pose;
  const double alpha = std::clamp(config_.measurement_blend, 0.0, 1.0);
  output_.pose.x = (1.0 - alpha) * output_.pose.x + alpha * measurement.x;
  output_.pose.y = (1.0 - alpha) * output_.pose.y + alpha * measurement.y;
  output_.pose.yaw = normalize_angle(
    output_.pose.yaw + alpha * normalize_angle(measurement.yaw - output_.pose.yaw));

  const Pose2 measured_velocity{
    (output_.pose.x - previous.x) / dt,
    (output_.pose.y - previous.y) / dt,
    normalize_angle(output_.pose.yaw - previous.yaw) / dt};
  const double beta = std::clamp(config_.velocity_blend, 0.0, 1.0);
  velocity_.x = (1.0 - beta) * velocity_.x + beta * measured_velocity.x;
  velocity_.y = (1.0 - beta) * velocity_.y + beta * measured_velocity.y;
  velocity_.yaw = (1.0 - beta) * velocity_.yaw + beta * measured_velocity.yaw;

  output_.has_pose = true;
  output_.measurement_applied = true;
  output_.mode = TrackerMode::Tracking;
  output_.covariance_scale = 1.0;
  output_.coasting_frames = 0;
  output_.reacquire_hits = 0;
  output_.status = "TRACKING";
  have_coast_start_ = false;
  last_stamp_sec_ = stamp_sec;
  have_stamp_ = true;
}

void WheelPoseTracker::enter_reacquiring(const Pose2 & measurement, double stamp_sec)
{
  if (output_.mode != TrackerMode::Reacquiring ||
    !measurement_near_pose(
      measurement, reacquire_pose_, config_.reacquire_gate_m, config_.reacquire_yaw_gate_rad))
  {
    reacquire_pose_ = measurement;
    output_.reacquire_hits = 1;
  } else {
    reacquire_pose_ = measurement;
    output_.reacquire_hits += 1;
  }

  output_.measurement_applied = false;
  output_.mode = TrackerMode::Reacquiring;
  output_.status = "REACQUIRING";
  output_.covariance_scale = std::min(50.0, output_.covariance_scale * 2.0);

  if (output_.reacquire_hits >= config_.min_reacquire_hits) {
    initialize(measurement, stamp_sec, TrackerMode::Tracking, "TRACKING_REACQUIRED");
    output_.measurement_applied = true;
  }
}

const char * tracker_mode_name(TrackerMode mode)
{
  switch (mode) {
    case TrackerMode::Lost:
      return "LOST";
    case TrackerMode::Tracking:
      return "TRACKING";
    case TrackerMode::Coasting:
      return "COASTING";
    case TrackerMode::Reacquiring:
      return "REACQUIRING";
  }
  return "UNKNOWN";
}

}  // namespace follower_lidar_localization
