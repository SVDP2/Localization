#include "follower_lidar_localization/wheel_fitting.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

namespace follower_lidar_localization
{
namespace
{

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Point point(double x, double y, double z = 0.0)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

std_msgs::msg::ColorRGBA color(double r, double g, double b, double a)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

std::string number(double value, int precision = 3)
{
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.*f", precision, value);
  return std::string(buffer);
}

Point2 rotate_quaternion_2d(
  const geometry_msgs::msg::Quaternion & q_msg,
  const Point2 & point_in)
{
  double x = q_msg.x;
  double y = q_msg.y;
  double z = q_msg.z;
  double w = q_msg.w;
  const double q_norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (!std::isfinite(q_norm) || q_norm < 1.0e-12) {
    return point_in;
  }
  x /= q_norm;
  y /= q_norm;
  z /= q_norm;
  w /= q_norm;

  const double xx = x * x;
  const double yy = y * y;
  const double zz = z * z;
  const double xy = x * y;
  const double wz = w * z;
  const double xz = x * z;
  const double wy = w * y;
  const double yz = y * z;
  const double wx = w * x;
  (void)xz;
  (void)wy;
  (void)yz;
  (void)wx;

  const double r00 = 1.0 - 2.0 * (yy + zz);
  const double r01 = 2.0 * (xy - wz);
  const double r10 = 2.0 * (xy + wz);
  const double r11 = 1.0 - 2.0 * (xx + zz);
  return {
    r00 * point_in.x + r01 * point_in.y,
    r10 * point_in.x + r11 * point_in.y};
}

diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

}  // namespace

class LeaderWheelFittingNode : public rclcpp::Node
{
public:
  LeaderWheelFittingNode()
  : Node("leader_wheel_fitting_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameters();
    load_parameters();

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    leader_base_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      leader_base_odom_topic_, 10);
    leader_rear_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      leader_rear_odom_topic_, 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, 10);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {scan_callback(msg);});

    if (use_aruco_prior_ && !aruco_prior_topic_.empty()) {
      aruco_prior_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        aruco_prior_topic_, 10,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {aruco_prior_callback(msg);});
    }

    RCLCPP_INFO(
      get_logger(),
      "leader_wheel_fitting_node started: scan=%s base_frame=%s leader_base_odom=%s",
      scan_topic_.c_str(), base_frame_.c_str(), leader_base_odom_topic_.c_str());
  }

private:
  void declare_parameters()
  {
    declare_parameter("scan_topic", "/follower/scan");
    declare_parameter("aruco_prior_topic", "/follower/localization/leader_base/odom");
    declare_parameter("use_aruco_prior", true);
    declare_parameter("aruco_prior_timeout_sec", 0.50);
    declare_parameter("base_frame", "follower/base_link");
    declare_parameter("leader_base_detection_frame", "leader/base_link");
    declare_parameter("leader_rear_frame", "leader/leader_rear");
    declare_parameter(
      "leader_base_odom_topic",
      "/follower/localization/lidar_wheels/leader_base_detection");
    declare_parameter("leader_rear_odom_topic", "/follower/localization/lidar_wheels/odom");
    declare_parameter("marker_topic", "/follower/localization/lidar_wheels/markers");
    declare_parameter("diagnostics_topic", "/follower/localization/lidar_wheels/diagnostics");
    declare_parameter("tf_timeout_sec", 0.05);

    declare_parameter("wheelbase_m", 0.722);
    declare_parameter("track_width_m", 0.660);
    declare_parameter("wheel_radius_m", 0.115);
    declare_parameter("wheel_width_m", 0.100);
    declare_parameter("roi_x_min_m", 0.15);
    declare_parameter("roi_x_max_m", 2.50);
    declare_parameter("roi_abs_y_max_m", 1.20);
    declare_parameter("pose_x_min_m", 0.0);
    declare_parameter("pose_x_max_m", 3.00);
    declare_parameter("pose_abs_y_max_m", 1.50);
    declare_parameter("max_abs_yaw_deg", 70.0);
    declare_parameter("cluster_gap_m", 0.060);
    declare_parameter("min_cluster_points", 4);
    declare_parameter("candidate_min_length_m", 0.050);
    declare_parameter("candidate_max_length_m", 0.320);
    declare_parameter("candidate_max_rms_m", 0.025);
    declare_parameter("min_visible_segments", 2);
    declare_parameter("assignment_max_center_distance_m", 0.180);
    declare_parameter("assignment_max_angle_error_deg", 37.0);
    declare_parameter("enable_l_shape_segments", false);
    declare_parameter("enable_wheel_box_occupancy_bonus", true);
    declare_parameter("wheel_box_margin_m", 0.035);
    declare_parameter("wheel_box_min_points", 2);
    declare_parameter("wheel_box_occupancy_bonus", 0.025);

    declare_parameter("leader_rear_x_m", -0.275);
    declare_parameter("leader_rear_y_m", 0.0);
    declare_parameter("leader_rear_z_m", 0.0525);
    declare_parameter("leader_rear_yaw_deg", 0.0);

    declare_parameter("tracking_gate_m", 0.45);
    declare_parameter("tracking_yaw_gate_deg", 40.0);
    declare_parameter("reacquire_gate_m", 0.75);
    declare_parameter("reacquire_yaw_gate_deg", 60.0);
    declare_parameter("measurement_blend", 0.65);
    declare_parameter("velocity_blend", 0.35);
    declare_parameter("max_coast_sec", 0.80);
    declare_parameter("min_reacquire_hits", 2);
  }

  void load_parameters()
  {
    scan_topic_ = get_parameter("scan_topic").as_string();
    aruco_prior_topic_ = get_parameter("aruco_prior_topic").as_string();
    use_aruco_prior_ = get_parameter("use_aruco_prior").as_bool();
    aruco_prior_timeout_sec_ = get_parameter("aruco_prior_timeout_sec").as_double();
    base_frame_ = get_parameter("base_frame").as_string();
    leader_base_detection_frame_ = get_parameter("leader_base_detection_frame").as_string();
    leader_rear_frame_ = get_parameter("leader_rear_frame").as_string();
    leader_base_odom_topic_ = get_parameter("leader_base_odom_topic").as_string();
    leader_rear_odom_topic_ = get_parameter("leader_rear_odom_topic").as_string();
    marker_topic_ = get_parameter("marker_topic").as_string();
    diagnostics_topic_ = get_parameter("diagnostics_topic").as_string();
    tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();

    fit_config_.wheelbase_m = get_parameter("wheelbase_m").as_double();
    fit_config_.track_width_m = get_parameter("track_width_m").as_double();
    fit_config_.wheel_radius_m = get_parameter("wheel_radius_m").as_double();
    fit_config_.wheel_width_m = get_parameter("wheel_width_m").as_double();
    fit_config_.roi_x_min_m = get_parameter("roi_x_min_m").as_double();
    fit_config_.roi_x_max_m = get_parameter("roi_x_max_m").as_double();
    fit_config_.roi_abs_y_max_m = get_parameter("roi_abs_y_max_m").as_double();
    fit_config_.pose_x_min_m = get_parameter("pose_x_min_m").as_double();
    fit_config_.pose_x_max_m = get_parameter("pose_x_max_m").as_double();
    fit_config_.pose_abs_y_max_m = get_parameter("pose_abs_y_max_m").as_double();
    fit_config_.max_abs_yaw_rad = deg_to_rad(get_parameter("max_abs_yaw_deg").as_double());
    fit_config_.cluster_gap_m = get_parameter("cluster_gap_m").as_double();
    fit_config_.min_cluster_points = static_cast<int>(get_parameter("min_cluster_points").as_int());
    fit_config_.candidate_min_length_m = get_parameter("candidate_min_length_m").as_double();
    fit_config_.candidate_max_length_m = get_parameter("candidate_max_length_m").as_double();
    fit_config_.candidate_max_rms_m = get_parameter("candidate_max_rms_m").as_double();
    fit_config_.min_visible_segments =
      static_cast<int>(get_parameter("min_visible_segments").as_int());
    fit_config_.assignment_max_center_distance_m =
      get_parameter("assignment_max_center_distance_m").as_double();
    fit_config_.assignment_max_angle_error_rad =
      deg_to_rad(get_parameter("assignment_max_angle_error_deg").as_double());
    fit_config_.enable_l_shape_segments = get_parameter("enable_l_shape_segments").as_bool();
    fit_config_.enable_wheel_box_occupancy_bonus =
      get_parameter("enable_wheel_box_occupancy_bonus").as_bool();
    fit_config_.wheel_box_margin_m = get_parameter("wheel_box_margin_m").as_double();
    fit_config_.wheel_box_min_points =
      static_cast<int>(get_parameter("wheel_box_min_points").as_int());
    fit_config_.wheel_box_occupancy_bonus =
      get_parameter("wheel_box_occupancy_bonus").as_double();

    leader_base_to_rear_.x = get_parameter("leader_rear_x_m").as_double();
    leader_base_to_rear_.y = get_parameter("leader_rear_y_m").as_double();
    leader_base_to_rear_.yaw = deg_to_rad(get_parameter("leader_rear_yaw_deg").as_double());
    leader_rear_z_m_ = get_parameter("leader_rear_z_m").as_double();

    TrackerConfig tracker_config;
    tracker_config.tracking_gate_m = get_parameter("tracking_gate_m").as_double();
    tracker_config.tracking_yaw_gate_rad =
      deg_to_rad(get_parameter("tracking_yaw_gate_deg").as_double());
    tracker_config.reacquire_gate_m = get_parameter("reacquire_gate_m").as_double();
    tracker_config.reacquire_yaw_gate_rad =
      deg_to_rad(get_parameter("reacquire_yaw_gate_deg").as_double());
    tracker_config.measurement_blend = get_parameter("measurement_blend").as_double();
    tracker_config.velocity_blend = get_parameter("velocity_blend").as_double();
    tracker_config.max_coast_sec = get_parameter("max_coast_sec").as_double();
    tracker_config.min_reacquire_hits =
      static_cast<int>(get_parameter("min_reacquire_hits").as_int());
    tracker_ = WheelPoseTracker(tracker_config);
  }

  void aruco_prior_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    const Pose2 reference_from_follower{p.x, p.y, yaw_from_quaternion(q)};
    const Pose2 follower_from_reference = inverse_pose(reference_from_follower);

    if (msg->header.frame_id == leader_rear_frame_) {
      const Pose2 leader_rear_from_leader_base = inverse_pose(leader_base_to_rear_);
      latest_prior_ = compose_pose(follower_from_reference, leader_rear_from_leader_base);
    } else {
      latest_prior_ = follower_from_reference;
    }
    latest_prior_stamp_ = rclcpp::Time(msg->header.stamp);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp);
    std::vector<ScanPoint2> samples;
    std::string tf_status;
    if (!scan_to_base_samples(*msg, samples, tf_status)) {
      FitResult empty_result;
      empty_result.status = tf_status;
      const auto tracker_output = tracker_.predict_only(time_seconds(stamp));
      publish_markers(*msg, empty_result, tracker_output);
      publish_diagnostics(*msg, empty_result, tracker_output, tf_status);
      return;
    }

    const auto prior = fresh_prior(stamp);
    const FitResult fit_result = fit_leader_wheel_pose(samples, fit_config_, prior);
    const auto tracker_output = tracker_.update(fit_result, time_seconds(stamp));

    publish_markers(*msg, fit_result, tracker_output);
    publish_diagnostics(*msg, fit_result, tracker_output, "OK");
    if (tracker_output.has_pose && tracker_output.mode != TrackerMode::Lost) {
      publish_odometry(*msg, fit_result, tracker_output);
    }
  }

  bool scan_to_base_samples(
    const sensor_msgs::msg::LaserScan & msg,
    std::vector<ScanPoint2> & samples,
    std::string & status)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(
        base_frame_,
        msg.header.frame_id,
        tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));
    } catch (const std::exception & e) {
      status = "TF_UNAVAILABLE:" + std::string(e.what());
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "LiDAR TF %s <- %s unavailable: %s",
        base_frame_.c_str(), msg.header.frame_id.c_str(), e.what());
      return false;
    }

    samples.reserve(msg.ranges.size());
    double angle = msg.angle_min;
    for (const float range_f : msg.ranges) {
      ScanPoint2 sample;
      const double range = static_cast<double>(range_f);
      if (std::isfinite(range) &&
        range >= static_cast<double>(msg.range_min) &&
        range <= static_cast<double>(msg.range_max))
      {
        const Point2 point_lidar{range * std::cos(angle), range * std::sin(angle)};
        Point2 point_base = rotate_quaternion_2d(tf.transform.rotation, point_lidar);
        point_base.x += tf.transform.translation.x;
        point_base.y += tf.transform.translation.y;
        sample.point = point_base;
        sample.valid = std::isfinite(point_base.x) && std::isfinite(point_base.y);
      }
      samples.push_back(sample);
      angle += msg.angle_increment;
    }
    status = "OK";
    return true;
  }

  std::optional<Pose2> fresh_prior(const rclcpp::Time & stamp) const
  {
    if (!use_aruco_prior_ || !latest_prior_ || latest_prior_stamp_.nanoseconds() == 0) {
      return std::nullopt;
    }
    const double age = std::abs((stamp - latest_prior_stamp_).nanoseconds() * 1.0e-9);
    if (age > aruco_prior_timeout_sec_) {
      return std::nullopt;
    }
    return latest_prior_;
  }

  void publish_odometry(
    const sensor_msgs::msg::LaserScan & scan,
    const FitResult & result,
    const TrackerOutput & tracker_output)
  {
    nav_msgs::msg::Odometry leader_base_odom;
    leader_base_odom.header.stamp = scan.header.stamp;
    leader_base_odom.header.frame_id = base_frame_;
    leader_base_odom.child_frame_id = leader_base_detection_frame_;
    fill_pose(leader_base_odom, tracker_output.pose, 0.0);
    fill_covariance(leader_base_odom, result, tracker_output, false);
    leader_base_odom_pub_->publish(leader_base_odom);

    const Pose2 follower_from_leader_rear =
      compose_pose(tracker_output.pose, leader_base_to_rear_);
    const Pose2 leader_rear_from_follower = inverse_pose(follower_from_leader_rear);

    nav_msgs::msg::Odometry leader_rear_odom;
    leader_rear_odom.header.stamp = scan.header.stamp;
    leader_rear_odom.header.frame_id = leader_rear_frame_;
    leader_rear_odom.child_frame_id = base_frame_;
    fill_pose(leader_rear_odom, leader_rear_from_follower, -leader_rear_z_m_);
    fill_covariance(leader_rear_odom, result, tracker_output, true);
    leader_rear_odom_pub_->publish(leader_rear_odom);
  }

  void fill_pose(nav_msgs::msg::Odometry & odom, const Pose2 & pose, double z) const
  {
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = z;
    odom.pose.pose.orientation = quaternion_from_yaw(pose.yaw);
  }

  void fill_covariance(
    nav_msgs::msg::Odometry & odom,
    const FitResult & result,
    const TrackerOutput & tracker_output,
    bool leader_rear_output) const
  {
    std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
    const double visibility_scale =
      result.visible_segments > 0 ? 4.0 / static_cast<double>(result.visible_segments) : 8.0;
    const double residual_scale = std::max(1.0, result.mean_center_residual_m / 0.030);
    const double scale = std::max(1.0, tracker_output.covariance_scale) *
      visibility_scale * residual_scale;
    odom.pose.covariance[0] = 0.0025 * scale;
    odom.pose.covariance[7] = 0.0025 * scale;
    odom.pose.covariance[14] = leader_rear_output ? 4.0 : 1.0;
    odom.pose.covariance[21] = 10.0;
    odom.pose.covariance[28] = 10.0;
    odom.pose.covariance[35] = 0.0100 * scale;
  }

  void publish_markers(
    const sensor_msgs::msg::LaserScan & scan,
    const FitResult & result,
    const TrackerOutput & tracker_output)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.header.stamp = scan.header.stamp;
    clear.header.frame_id = base_frame_;
    clear.ns = "leader_wheel_fitting";
    clear.id = 0;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    int id = 1;
    add_candidate_marker(array, scan, result, id);
    if (tracker_output.has_pose && tracker_output.mode != TrackerMode::Lost) {
      add_model_marker(array, scan, tracker_output, result, id);
      add_wheel_box_marker(array, scan, tracker_output, id);
      add_pose_text_marker(array, scan, tracker_output, result, id);
    }
    marker_pub_->publish(array);
  }

  void add_candidate_marker(
    visualization_msgs::msg::MarkerArray & array,
    const sensor_msgs::msg::LaserScan & scan,
    const FitResult & result,
    int & id) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = scan.header.stamp;
    marker.header.frame_id = base_frame_;
    marker.ns = "leader_wheel_fitting/candidates";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.018;
    marker.color = color(0.10, 0.55, 1.0, 1.0);
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 300000000;
    for (const auto & candidate : result.candidates) {
      marker.points.push_back(point(candidate.start.x, candidate.start.y));
      marker.points.push_back(point(candidate.end.x, candidate.end.y));
    }
    array.markers.push_back(marker);
  }

  void add_model_marker(
    visualization_msgs::msg::MarkerArray & array,
    const sensor_msgs::msg::LaserScan & scan,
    const TrackerOutput & tracker_output,
    const FitResult & result,
    int & id) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = scan.header.stamp;
    marker.header.frame_id = base_frame_;
    marker.ns = "leader_wheel_fitting/model";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.026;
    marker.color = tracker_output.mode == TrackerMode::Tracking ?
      color(0.20, 0.90, 0.30, 1.0) : color(1.0, 0.70, 0.10, 1.0);
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 300000000;
    const auto model = result.model_segments.empty() ?
      make_leader_wheel_model(fit_config_) : result.model_segments;
    for (const auto & segment : model) {
      const Point2 model_start{
        segment.center.x - segment.direction.x * (0.5 * segment.length),
        segment.center.y - segment.direction.y * (0.5 * segment.length)};
      const Point2 model_end{
        segment.center.x + segment.direction.x * (0.5 * segment.length),
        segment.center.y + segment.direction.y * (0.5 * segment.length)};
      const Point2 start = transform_point(
        tracker_output.pose,
        model_start);
      const Point2 end = transform_point(
        tracker_output.pose,
        model_end);
      marker.points.push_back(point(start.x, start.y));
      marker.points.push_back(point(end.x, end.y));
    }
    array.markers.push_back(marker);
  }

  void add_wheel_box_marker(
    visualization_msgs::msg::MarkerArray & array,
    const sensor_msgs::msg::LaserScan & scan,
    const TrackerOutput & tracker_output,
    int & id) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = scan.header.stamp;
    marker.header.frame_id = base_frame_;
    marker.ns = "leader_wheel_fitting/wheel_boxes";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.012;
    marker.color = color(0.10, 1.0, 0.65, 0.65);
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 300000000;

    const double half_track = 0.5 * fit_config_.track_width_m;
    const double half_width = 0.5 * fit_config_.wheel_width_m;
    const std::array<Point2, 4> centers{
      Point2{0.0, half_track},
      Point2{0.0, -half_track},
      Point2{fit_config_.wheelbase_m, half_track},
      Point2{fit_config_.wheelbase_m, -half_track},
    };
    for (const auto & center : centers) {
      const std::array<Point2, 4> corners{
        Point2{center.x - fit_config_.wheel_radius_m, center.y - half_width},
        Point2{center.x + fit_config_.wheel_radius_m, center.y - half_width},
        Point2{center.x + fit_config_.wheel_radius_m, center.y + half_width},
        Point2{center.x - fit_config_.wheel_radius_m, center.y + half_width},
      };
      for (size_t i = 0; i < corners.size(); ++i) {
        const Point2 a = transform_point(tracker_output.pose, corners[i]);
        const Point2 b = transform_point(
          tracker_output.pose, corners[(i + 1) % corners.size()]);
        marker.points.push_back(point(a.x, a.y, 0.01));
        marker.points.push_back(point(b.x, b.y, 0.01));
      }
    }
    array.markers.push_back(marker);
  }

  void add_pose_text_marker(
    visualization_msgs::msg::MarkerArray & array,
    const sensor_msgs::msg::LaserScan & scan,
    const TrackerOutput & tracker_output,
    const FitResult & result,
    int & id) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = scan.header.stamp;
    marker.header.frame_id = base_frame_;
    marker.ns = "leader_wheel_fitting/status";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = point(tracker_output.pose.x, tracker_output.pose.y, 0.35);
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.08;
    marker.color = color(1.0, 1.0, 1.0, 1.0);
    marker.text = std::string(tracker_mode_name(tracker_output.mode)) +
      " wheels=" + std::to_string(result.visible_segments) +
      " res=" + number(result.mean_center_residual_m, 3);
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 300000000;
    array.markers.push_back(marker);
  }

  void publish_diagnostics(
    const sensor_msgs::msg::LaserScan & scan,
    const FitResult & result,
    const TrackerOutput & tracker_output,
    const std::string & tf_status)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = scan.header.stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "follower_lidar_localization/leader_wheel_fitting";
    status.hardware_id = "follower_2d_lidar";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    if (tracker_output.mode == TrackerMode::Coasting ||
      tracker_output.mode == TrackerMode::Reacquiring ||
      !result.valid)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }
    if (tracker_output.mode == TrackerMode::Lost || tf_status.rfind("TF_UNAVAILABLE", 0) == 0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }
    status.message = tracker_output.status;
    status.values = {
      kv("tf_status", tf_status),
      kv("fit_status", result.status),
      kv("tracker_mode", tracker_mode_name(tracker_output.mode)),
      kv("candidate_count", std::to_string(result.candidates.size())),
      kv("visible_segments", std::to_string(result.visible_segments)),
      kv("mean_center_residual_m", number(result.mean_center_residual_m)),
      kv("score", number(result.score)),
      kv("measurement_applied", tracker_output.measurement_applied ? "true" : "false"),
      kv("coasting_frames", std::to_string(tracker_output.coasting_frames)),
      kv("reacquire_hits", std::to_string(tracker_output.reacquire_hits)),
    };
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  static double deg_to_rad(double deg)
  {
    return deg * 3.14159265358979323846 / 180.0;
  }

  double time_seconds(const rclcpp::Time & stamp) const
  {
    if (stamp.nanoseconds() == 0) {
      return now().seconds();
    }
    return stamp.seconds();
  }

  std::string scan_topic_;
  std::string aruco_prior_topic_;
  bool use_aruco_prior_{true};
  double aruco_prior_timeout_sec_{0.5};
  std::string base_frame_;
  std::string leader_base_detection_frame_;
  std::string leader_rear_frame_;
  std::string leader_base_odom_topic_;
  std::string leader_rear_odom_topic_;
  std::string marker_topic_;
  std::string diagnostics_topic_;
  double tf_timeout_sec_{0.05};
  FitConfig fit_config_;
  WheelPoseTracker tracker_;
  Pose2 leader_base_to_rear_;
  double leader_rear_z_m_{0.0525};

  std::optional<Pose2> latest_prior_;
  rclcpp::Time latest_prior_stamp_{0, 0, RCL_ROS_TIME};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr aruco_prior_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr leader_base_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr leader_rear_odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

}  // namespace follower_lidar_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<follower_lidar_localization::LeaderWheelFittingNode>());
  rclcpp::shutdown();
  return 0;
}
