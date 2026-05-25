#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace follower_lidar_localization
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};

struct Pose2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct ScanPoint2
{
  Point2 point;
  bool valid{false};
};

struct FitConfig
{
  double wheelbase_m{0.720};
  double track_width_m{0.700};
  double wheel_radius_m{0.1325};
  double wheel_width_m{0.110};

  double roi_x_min_m{0.15};
  double roi_x_max_m{2.50};
  double roi_abs_y_max_m{1.20};
  double pose_x_min_m{0.0};
  double pose_x_max_m{3.00};
  double pose_abs_y_max_m{1.50};
  double max_abs_yaw_rad{1.22};

  double cluster_gap_m{0.060};
  int min_cluster_points{4};
  double candidate_min_length_m{0.050};
  double candidate_max_length_m{0.180};
  double candidate_max_rms_m{0.025};

  int min_visible_segments{2};
  double assignment_max_center_distance_m{0.180};
  double assignment_max_angle_error_rad{0.65};
  double prior_position_weight{0.25};
  double prior_yaw_weight{0.10};
};

struct TrackerConfig
{
  double tracking_gate_m{0.45};
  double tracking_yaw_gate_rad{0.70};
  double reacquire_gate_m{0.75};
  double reacquire_yaw_gate_rad{1.05};
  double measurement_blend{0.65};
  double velocity_blend{0.35};
  double max_coast_sec{0.80};
  int min_reacquire_hits{2};
};

struct ModelSegment
{
  std::string name;
  Point2 center;
  Point2 direction;
  double length{0.0};
};

struct SegmentCandidate
{
  Point2 center;
  Point2 direction;
  Point2 start;
  Point2 end;
  double length{0.0};
  double rms{0.0};
  int point_count{0};
};

struct SegmentAssignment
{
  int model_index{-1};
  int candidate_index{-1};
  double center_distance_m{0.0};
  double angle_error_rad{0.0};
};

struct FitResult
{
  bool valid{false};
  std::string status{"NO_FIT"};
  Pose2 pose;
  int visible_segments{0};
  double mean_center_residual_m{0.0};
  double score{0.0};
  std::vector<ModelSegment> model_segments;
  std::vector<SegmentCandidate> candidates;
  std::vector<SegmentAssignment> assignments;
};

enum class TrackerMode
{
  Lost,
  Tracking,
  Coasting,
  Reacquiring,
};

struct TrackerOutput
{
  bool has_pose{false};
  bool measurement_applied{false};
  TrackerMode mode{TrackerMode::Lost};
  Pose2 pose;
  double covariance_scale{1.0};
  int coasting_frames{0};
  int reacquire_hits{0};
  std::string status{"LOST"};
};

std::vector<ModelSegment> make_leader_wheel_model(const FitConfig & config);
std::vector<SegmentCandidate> extract_segment_candidates(
  const std::vector<ScanPoint2> & scan_points,
  const FitConfig & config);
FitResult fit_leader_wheel_pose(
  const std::vector<ScanPoint2> & scan_points,
  const FitConfig & config,
  const std::optional<Pose2> & prior = std::nullopt);

double normalize_angle(double angle_rad);
double angle_distance(double a_rad, double b_rad);
double line_angle_distance(double a_rad, double b_rad);
Point2 rotate_point(const Point2 & point, double yaw_rad);
Point2 transform_point(const Pose2 & pose, const Point2 & point);
Pose2 compose_pose(const Pose2 & a, const Pose2 & b);
Pose2 inverse_pose(const Pose2 & pose);
double pose_distance(const Pose2 & a, const Pose2 & b);

class WheelPoseTracker
{
public:
  explicit WheelPoseTracker(const TrackerConfig & config = TrackerConfig{});

  TrackerOutput update(const FitResult & result, double stamp_sec);
  TrackerOutput predict_only(double stamp_sec);
  const TrackerOutput & output() const {return output_;}

private:
  void predict(double stamp_sec);
  bool measurement_near_pose(
    const Pose2 & measurement,
    const Pose2 & reference,
    double position_gate,
    double yaw_gate) const;
  void initialize(
    const Pose2 & pose, double stamp_sec, TrackerMode mode,
    const std::string & status);
  void apply_tracking_measurement(const Pose2 & measurement, double stamp_sec);
  void enter_reacquiring(const Pose2 & measurement, double stamp_sec);

  TrackerConfig config_;
  TrackerOutput output_;
  Pose2 velocity_;
  Pose2 reacquire_pose_;
  bool initialized_{false};
  bool have_stamp_{false};
  bool have_coast_start_{false};
  double last_stamp_sec_{0.0};
  double last_prediction_dt_sec_{0.0};
  double coast_start_stamp_sec_{0.0};
};

const char * tracker_mode_name(TrackerMode mode);

}  // namespace follower_lidar_localization
