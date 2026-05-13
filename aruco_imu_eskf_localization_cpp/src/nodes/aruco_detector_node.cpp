#include "aruco_imu_eskf_localization_cpp/board_pose_estimator.hpp"
#include "aruco_imu_eskf_localization_cpp/camera_calibration.hpp"
#include "aruco_imu_eskf_localization_cpp/geometry.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <deque>
#include <memory>
#include <optional>
#include <string>

namespace aruco_imu_eskf_localization_cpp
{
namespace
{

int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
}

std::string package_path_or_absolute(const std::string & path)
{
  if (path.empty() || path.front() == '/') {
    return path;
  }
  return ament_index_cpp::get_package_share_directory("aruco_imu_eskf_localization_cpp") +
    "/" + path;
}

int aruco_dict_id(const std::string & name)
{
  if (name == "DICT_4X4_50") {return cv::aruco::DICT_4X4_50;}
  if (name == "DICT_4X4_100") {return cv::aruco::DICT_4X4_100;}
  if (name == "DICT_5X5_100") {return cv::aruco::DICT_5X5_100;}
  if (name == "DICT_6X6_250") {return cv::aruco::DICT_6X6_250;}
  if (name == "DICT_APRILTAG_36h11") {return cv::aruco::DICT_APRILTAG_36h11;}
  return cv::aruco::DICT_6X6_250;
}

}  // namespace

class ArucoDetectorNode : public rclcpp::Node
{
public:
  ArucoDetectorNode()
  : Node("aruco_detector_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter("marker_config_file", "config/markers_board.yaml");
    declare_parameter("aruco_dict", "DICT_6X6_250");
    declare_parameter("camera_calibration_file", "config/cam_intrinsic.yaml");
    declare_parameter("image_topic", "image_raw");
    declare_parameter("board_pose_topic", "localization/aruco/board_pose");
    declare_parameter("pose_prior_topic", "localization/relative/pose");
    declare_parameter("debug_image_topic", "localization/aruco/debug_image");
    declare_parameter("diagnostics_topic", "diagnostics");
    declare_parameter("base_frame", "follower/base_link");
    declare_parameter("camera_frame", "follower/usb_cam");
    declare_parameter("board_frame", "leader/board");
    declare_parameter("leader_rear_frame", "leader/leader_rear");
    declare_parameter("publish_tf", false);
    declare_parameter("publish_raw_aruco_tf", true);
    declare_parameter("raw_aruco_child_frame", "follower/base_link_aruco_raw");
    declare_parameter("rectify_balance", 0.0);
    declare_parameter("pose_prior_timeout_sec", 0.25);
    declare_parameter("pose_prior_rotation_gate_deg", 0.0);
    declare_parameter("min_markers_for_board", 2);
    declare_parameter("min_markers_to_initialize", 2);
    declare_parameter("max_position_jump_m", 0.35);
    declare_parameter("max_rotation_jump_deg", 55.0);
    declare_parameter("max_heading_jump_deg", 40.0);
    declare_parameter("max_reprojection_rmse_px", 6.0);
    declare_parameter("front_halfspace_min_z_m", 0.05);
    declare_parameter("max_view_angle_deg", 75.0);
    declare_parameter("feasible_x_min_m", -3.50);
    declare_parameter("feasible_x_max_m", -0.10);
    declare_parameter("feasible_abs_y_max_m", 1.00);
    declare_parameter("feasible_z_min_m", -0.50);
    declare_parameter("feasible_z_max_m", 0.80);

    base_frame_ = get_parameter("base_frame").as_string();
    camera_frame_ = get_parameter("camera_frame").as_string();
    board_frame_ = get_parameter("board_frame").as_string();
    leader_rear_frame_ = get_parameter("leader_rear_frame").as_string();
    raw_aruco_child_frame_ = get_parameter("raw_aruco_child_frame").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    publish_raw_aruco_tf_ = get_parameter("publish_raw_aruco_tf").as_bool();
    prior_timeout_sec_ = get_parameter("pose_prior_timeout_sec").as_double();
    options_.reference_rotation_gate_deg = get_parameter("pose_prior_rotation_gate_deg").as_double();
    options_.min_markers = get_parameter("min_markers_for_board").as_int();
    options_.min_markers_to_initialize = get_parameter("min_markers_to_initialize").as_int();
    options_.max_position_jump_m = get_parameter("max_position_jump_m").as_double();
    options_.max_rotation_jump_deg = get_parameter("max_rotation_jump_deg").as_double();
    options_.max_heading_jump_deg = get_parameter("max_heading_jump_deg").as_double();
    options_.max_reprojection_rmse_px = get_parameter("max_reprojection_rmse_px").as_double();
    options_.front_halfspace_min_z_m = get_parameter("front_halfspace_min_z_m").as_double();
    options_.max_view_angle_deg = get_parameter("max_view_angle_deg").as_double();
    options_.feasible_x_min_m = get_parameter("feasible_x_min_m").as_double();
    options_.feasible_x_max_m = get_parameter("feasible_x_max_m").as_double();
    options_.feasible_abs_y_max_m = get_parameter("feasible_abs_y_max_m").as_double();
    options_.feasible_z_min_m = get_parameter("feasible_z_min_m").as_double();
    options_.feasible_z_max_m = get_parameter("feasible_z_max_m").as_double();

    const auto marker_path = package_path_or_absolute(get_parameter("marker_config_file").as_string());
    const auto calib_path = package_path_or_absolute(get_parameter("camera_calibration_file").as_string());
    board_ = BoardDefinition::from_yaml_file(marker_path);
    calibration_ = load_camera_calibration(calib_path);
    rectification_ = build_fisheye_rectification(
      calibration_, get_parameter("rectify_balance").as_double());
    dictionary_ = cv::aruco::getPredefinedDictionary(aruco_dict_id(get_parameter("aruco_dict").as_string()));
    detector_params_ = cv::aruco::DetectorParameters::create();

    board_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("board_pose_topic").as_string(), 10);
    debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      get_parameter("debug_image_topic").as_string(), 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      get_parameter("diagnostics_topic").as_string(), 10);
    if (publish_tf_ || publish_raw_aruco_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      get_parameter("image_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {image_callback(msg);});
    prior_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("pose_prior_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {prior_callback(msg);});

    RCLCPP_INFO(get_logger(), "C++ ArUco detector publishing %s", get_parameter("board_pose_topic").as_string().c_str());
    RCLCPP_INFO(
      get_logger(),
      "ArUco detector gates: min_markers=%d init_markers=%d reference_rotation_gate=%.1f deg",
      options_.min_markers, options_.min_markers_to_initialize,
      options_.reference_rotation_gate_deg);
  }

private:
  void prior_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    Eigen::Isometry3d board_from_base = Eigen::Isometry3d::Identity();
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    board_from_base.translation() = Eigen::Vector3d(p.x, p.y, p.z);
    board_from_base.linear() = normalize_quaternion(q.x, q.y, q.z, q.w).toRotationMatrix();
    prior_samples_.emplace_back(stamp_ns(msg->header.stamp), board_from_base);
    while (prior_samples_.size() > 120) {
      prior_samples_.pop_front();
    }
  }

  std::optional<Eigen::Isometry3d> latest_prior(int64_t current_ns) const
  {
    const int64_t timeout_ns = static_cast<int64_t>(std::max(prior_timeout_sec_, 0.0) * 1.0e9);
    for (auto it = prior_samples_.rbegin(); it != prior_samples_.rend(); ++it) {
      if (it->first >= current_ns) {
        continue;
      }
      if ((current_ns - it->first) > timeout_ns) {
        break;
      }
      return it->second;
    }
    return std::nullopt;
  }

  std::optional<Eigen::Isometry3d> camera_from_base()
  {
    if (cached_camera_from_base_) {
      return cached_camera_from_base_;
    }
    try {
      const auto tf = tf_buffer_.lookupTransform(base_frame_, camera_frame_, tf2::TimePointZero);
      cached_camera_from_base_ = transform_from_msg(tf).inverse();
      return cached_camera_from_base_;
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "camera TF %s <- %s unavailable: %s",
        base_frame_.c_str(), camera_frame_.c_str(), e.what());
      return std::nullopt;
    }
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    try {
      image_callback_impl(msg);
    } catch (const cv::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "OpenCV detector exception: %s", e.what());
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "detector exception: %s", e.what());
    }
  }

  void image_callback_impl(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    detector_known_markers_ = 0;
    detector_pose_published_ = false;
    detector_rejection_reason_ = "processing";

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN(get_logger(), "cv_bridge failed: %s", e.what());
      return;
    }

    auto cam_from_base = camera_from_base();
    if (!cam_from_base) {
      return;
    }

    cv::Mat rectified;
    cv::remap(cv_ptr->image, rectified, rectification_.map1, rectification_.map2, cv::INTER_LINEAR);

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> rejected;
    cv::aruco::detectMarkers(rectified, dictionary_, corners, ids, detector_params_, rejected);
    detector_detected_markers_ = static_cast<int>(ids.size());
    detector_known_markers_ = static_cast<int>(board_.known_detections(corners, ids).size());
    if (corners.empty()) {
      detector_rejection_reason_ = "no_markers_detected";
      publish_debug(msg->header, rectified, corners, ids);
      publish_detector_diagnostics(msg->header.stamp);
      return;
    }

    const auto prior = latest_prior(stamp_ns(msg->header.stamp));
    const auto estimate = board_.estimate_pose(
      corners, ids, rectification_.rectified_camera_matrix, rectification_.zero_distortion,
      options_, previous_board_from_base_, prior, *cam_from_base);
    if (!estimate) {
      detector_rejection_reason_ = board_.last_rejection_reason();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "board pose rejected: %s (detected=%d known=%d)",
        detector_rejection_reason_.c_str(), detector_detected_markers_, detector_known_markers_);
      publish_debug(msg->header, rectified, corners, ids);
      publish_detector_diagnostics(msg->header.stamp);
      return;
    }
    detector_rejection_reason_ = "published";

    const Eigen::Isometry3d board_from_camera =
      board_pose_to_transform(estimate->rvec_board_from_camera, estimate->tvec_board_from_camera);
    const Eigen::Isometry3d board_from_base = board_from_camera * (*cam_from_base);
    previous_board_from_base_ = board_from_base;

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header = msg->header;
    pose.header.frame_id = board_frame_;
    pose.pose.pose.position.x = board_from_camera.translation().x();
    pose.pose.pose.position.y = board_from_camera.translation().y();
    pose.pose.pose.position.z = board_from_camera.translation().z();
    pose.pose.pose.orientation = quaternion_msg_from_eigen(Eigen::Quaterniond(board_from_camera.linear()));
    const auto cov = measurement_covariance_from_estimate(*estimate);
    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) {
        pose.pose.covariance[r * 6 + c] = cov(r, c);
      }
    }
    board_pose_pub_->publish(pose);
    detector_pose_published_ = true;

    if (tf_broadcaster_) {
      if (publish_raw_aruco_tf_) {
        const Eigen::Isometry3d leader_from_base = transform_leader_rear_from_board() * board_from_base;
        geometry_msgs::msg::TransformStamped raw_tf_msg;
        raw_tf_msg.header.stamp = pose.header.stamp;
        raw_tf_msg.header.frame_id = leader_rear_frame_;
        raw_tf_msg.child_frame_id = raw_aruco_child_frame_;
        raw_tf_msg.transform.translation.x = leader_from_base.translation().x();
        raw_tf_msg.transform.translation.y = leader_from_base.translation().y();
        raw_tf_msg.transform.translation.z = leader_from_base.translation().z();
        raw_tf_msg.transform.rotation = quaternion_msg_from_eigen(
          Eigen::Quaterniond(leader_from_base.linear()));
        tf_broadcaster_->sendTransform(raw_tf_msg);
      }
    }

    if (tf_broadcaster_ && publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header = pose.header;
      tf_msg.child_frame_id = camera_frame_;
      tf_msg.transform.translation.x = pose.pose.pose.position.x;
      tf_msg.transform.translation.y = pose.pose.pose.position.y;
      tf_msg.transform.translation.z = pose.pose.pose.position.z;
      tf_msg.transform.rotation = pose.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);
    }

    publish_debug(msg->header, rectified, corners, ids);
    publish_detector_diagnostics(msg->header.stamp);
  }

  static void add_diag(
    diagnostic_msgs::msg::DiagnosticStatus & status,
    const std::string & key,
    const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  }

  void publish_detector_diagnostics(const builtin_interfaces::msg::Time & stamp)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "aruco_detector";
    status.level = detector_pose_published_ ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = detector_pose_published_ ? "pose_published" : detector_rejection_reason_;
    add_diag(status, "detected_markers", std::to_string(detector_detected_markers_));
    add_diag(status, "known_markers", std::to_string(detector_known_markers_));
    add_diag(status, "pose_published", detector_pose_published_ ? "true" : "false");
    add_diag(status, "rejection_reason", detector_rejection_reason_);
    add_diag(status, "min_markers_for_board", std::to_string(options_.min_markers));
    add_diag(status, "min_markers_to_initialize", std::to_string(options_.min_markers_to_initialize));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  void publish_debug(
    const std_msgs::msg::Header & header,
    cv::Mat image,
    const std::vector<std::vector<cv::Point2f>> & corners,
    const std::vector<int> & ids)
  {
    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(image, corners, ids);
    }
    cv_bridge::CvImage out(header, "bgr8", image);
    debug_image_pub_->publish(*out.toImageMsg());
  }

  BoardDefinition board_;
  CameraCalibration calibration_;
  FisheyeRectification rectification_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  BoardPoseEstimatorOptions options_;

  std::string base_frame_;
  std::string camera_frame_;
  std::string board_frame_;
  std::string leader_rear_frame_;
  std::string raw_aruco_child_frame_;
  bool publish_tf_{false};
  bool publish_raw_aruco_tf_{true};
  double prior_timeout_sec_{0.25};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr prior_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr board_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::optional<Eigen::Isometry3d> cached_camera_from_base_;
  std::optional<Eigen::Isometry3d> previous_board_from_base_;
  std::deque<std::pair<int64_t, Eigen::Isometry3d>> prior_samples_;
  int detector_detected_markers_{0};
  int detector_known_markers_{0};
  bool detector_pose_published_{false};
  std::string detector_rejection_reason_{"not_started"};
};

}  // namespace aruco_imu_eskf_localization_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aruco_imu_eskf_localization_cpp::ArucoDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
