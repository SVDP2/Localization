#ifndef ESKF_LOCALIZATION__ESKF_LOCALIZATION_NODE_HPP_
#define ESKF_LOCALIZATION__ESKF_LOCALIZATION_NODE_HPP_

#include <memory>
#include <mutex>
#include <limits>
#include <cstddef>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tier4_map_msgs/msg/map_projector_info.hpp>

#include <Eigen/Core>

#include <sophus/se3.hpp>

#include "kiss_icp/pipeline/KissICP.hpp"

#include "eskf_localization/eskf/eskf_core.hpp"
#include "eskf_localization/eskf/gnss_update_handler.hpp"
#include "eskf_localization/eskf/odom_builder.hpp"
#include "eskf_localization/parameters.hpp"
#include "eskf_localization/preprocess/gnss_heading_arbitrator.hpp"
#include "eskf_localization/preprocess/gnss_preprocessor.hpp"
#include "eskf_localization/preprocess/imu_calibration_manager.hpp"
#include "eskf_localization/preprocess/imu_preprocessor.hpp"
#include "eskf_localization/types.hpp"
#include "eskf_localization/util/diagnostics_publisher.hpp"
#include "eskf_localization/util/map_projector.hpp"
#include "eskf_localization/util/tf_cache.hpp"
#include "eskf_localization/util/time_processing.hpp"

namespace eskf_localization
{

class ESKFLocalizationNode : public rclcpp::Node
{
public:
  explicit ESKFLocalizationNode(const rclcpp::NodeOptions & t_options);

private:
  // Callback functions
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr t_msg);
  void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr t_msg);
  void gnss_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr t_msg);
  void map_projector_info_callback(
    const tier4_map_msgs::msg::MapProjectorInfo::SharedPtr t_msg);
  void velocity_callback(
    const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr t_msg);
  void steering_callback(
    const autoware_vehicle_msgs::msg::SteeringReport::SharedPtr t_msg);
  void heading_callback(const std_msgs::msg::Float64::SharedPtr t_msg);
  void
  pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr t_msg);
  void initialpose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr t_msg);

  // Timer callback for periodic output
  void publish_timer_callback();

  // Parameter loading
  void load_parameters();

  // Autoware-compatible trigger service
  void service_trigger_node(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pose_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
    m_gnss_odom_pub;   // GNSS-only position
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    m_gnss_pose_cov_pub;   // GNSS pose (for pose_initializer)
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    m_diag_pub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_preprocessed_imu_pub;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr
    m_gnss_velocity_pub;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr m_gnss_sub;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    m_gnss_vel_sub;
  rclcpp::Subscription<tier4_map_msgs::msg::MapProjectorInfo>::SharedPtr
    m_map_projector_sub;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr
    m_velocity_sub;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr
    m_steering_sub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_heading_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    m_pointcloud_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    m_initialpose_sub;

  // Trigger service (activate/deactivate)
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_trigger_node_;

  // TF listener
  std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

  // TF broadcaster (for debug output)
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

  // Timer
  rclcpp::TimerBase::SharedPtr m_publish_timer;

  // Parameters
  ESKFLocalizationNodeParams m_node_params;
  TimeProcessor m_time_processor{m_node_params.time};

  // Phase 3: TF extrinsic cache (module)
  TfCache m_tf_cache;

  // Phase 5: Map projection info
  MapProjector m_map_projector;
  GnssPreprocessor m_gnss_preprocessor;
  mutable std::mutex m_heading_arbitrator_mutex;
  GnssHeadingArbitrator m_heading_arbitrator;

  // Phase 6: IMU preprocessing
  ImuPreprocessor m_imu_preprocessor;
  ImuCalibrationManager m_imu_calibration_manager;
  static constexpr double IMU_CALIBRATION_DURATION_SEC = 30.0;

  // Phase 1: Input counters for diagnostics
  size_t m_imu_count;
  size_t m_gnss_count;
  size_t m_gnss_vel_count;
  size_t m_velocity_count;
  size_t m_steering_count;
  bool m_map_projector_received;

  // Phase 2: Time processing state
  rclcpp::Time m_last_imu_stamp;
  rclcpp::Time m_last_gnss_stamp;
  rclcpp::Time m_last_gnss_vel_stamp;
  rclcpp::Time m_last_velocity_stamp;
  rclcpp::Time m_last_steering_stamp;
  rclcpp::Time m_last_heading_stamp;
  rclcpp::Time m_last_pointcloud_stamp;

  // ============================================================
  // Initialization/activation control (Autoware compatible)
  // ============================================================
  bool m_is_activated_{true};             // runtime gating flag (SetBool)
  bool m_use_external_initialpose_{false}; // init.mode == external_initialpose

  // Measurement time-alignment tolerance (seconds). If measurement stamp is
  // within this tolerance behind m_state_stamp, treat it as aligned.
  // Phase 2: IMU dt statistics (for diagnostics)
  ImuDtStats m_imu_dt_stats;

  // Phase 2: Latest sensor data buffers
  sensor_msgs::msg::NavSatFix::SharedPtr m_latest_gnss;
  geometry_msgs::msg::TwistStamped::SharedPtr m_latest_gnss_vel;
  autoware_vehicle_msgs::msg::VelocityReport::SharedPtr m_latest_velocity;
  autoware_vehicle_msgs::msg::SteeringReport::SharedPtr m_latest_steering;
  std_msgs::msg::Float64::SharedPtr m_latest_heading;
  double m_latest_heading_yaw_rad{
    0.0};   // 표준 ENU yaw (변환 완료, CCW+ from East)
  bool m_heading_received{false};
  // After each ESKF (re-)initialization, suppress heading rate-gate briefly
  // so the filter can settle without immediate rate-based rejection.
  rclcpp::Time m_eskf_init_stamp_{0, 0, RCL_ROS_TIME};
  static constexpr double k_heading_rate_gate_init_grace_sec_{5.0};
  // Rate-gate recovery bookkeeping (to avoid "stuck skip" loops)
  int m_heading_rate_gate_skip_count_{0};
  rclcpp::Time m_heading_rate_gate_skip_window_start_{0, 0, RCL_ROS_TIME};

  // Diagnostics helper: last yaw measurement variance actually used
  double m_last_yaw_meas_var_rad2{0.01};
  double m_last_heading_status_inflate_dbg{1.0};
  double m_last_heading_recover_inflate_dbg{1.0};
  double m_last_heading_yaw_var_pre_nis_dbg{std::numeric_limits<double>::quiet_NaN()};
  double m_last_heading_yaw_var_applied_dbg{std::numeric_limits<double>::quiet_NaN()};
  std::string m_last_heading_yaw_var_source_dbg{"init"};
  EskfYawUpdateDebug m_last_heading_yaw_update_dbg{};

  // P1-3: Diagnostics counter (was static local)
  size_t m_diag_counter{0};
  EskfDiagnosticsPublisher m_diag_builder;

  // Phase 2: Helper functions
  void reset_imu_dt_stats();
  bool validate_stamp_and_order(
    const rclcpp::Time & current_stamp,
    const rclcpp::Time & now,
    const rclcpp::Time & last_stamp,
    const char * label);

  enum class GnssRecoverChannel
  {
    kPosition,
    kVelocity
  };

  // GNSS status upgrade transition smoothing
  // - holdoff: keep updates blocked for a short settling window
  // - recover inflate: immediate rise on worse input, exponential decay on recovery
  void update_gnss_recover_state(int curr_status, const rclcpp::Time & stamp);
  bool compute_gnss_recover_inflate(
    GnssRecoverChannel channel,
    const rclcpp::Time & stamp,
    bool & skip_update,
    double & out_inflate);
  double apply_gnss_inflate_envelope(
    GnssRecoverChannel channel,
    const rclcpp::Time & stamp,
    double target_inflate);
  void update_heading_recover_state(int curr_status, const rclcpp::Time & stamp);
  void raise_heading_recover_inflate(
    const std::string & reason,
    const rclcpp::Time & stamp);
  double compute_heading_measurement_inflate(
    const rclcpp::Time & stamp,
    double * status_inflate = nullptr,
    double * recover_inflate = nullptr);

  // ============================================================
  // Phase 7: ESKF core state (prediction + update + injection)
  // ============================================================
  mutable std::mutex m_state_mutex;
  EskfCore m_eskf{m_node_params.eskf};
  rclcpp::Time m_state_stamp{0, 0, RCL_ROS_TIME};

  // Initialization helper: cache GNSS position until reliable yaw is available.
  bool m_pending_init_position{false};
  Eigen::Vector3d m_pending_init_p_map{Eigen::Vector3d::Zero()};
  rclcpp::Time m_pending_init_stamp{0, 0, RCL_ROS_TIME};

  // Phase 7: For output/debug
  OdomBuilder m_odom_builder;
  GnssUpdateHandler m_gnss_update_handler;
  geometry_msgs::msg::Vector3 m_last_omega_base{};
  bool m_have_last_omega_base{false};
  EskfGnssPosUpdateDebug m_last_gnss_pos_update_dbg{};
  EskfGnssVelUpdateDebug m_last_gnss_vel_update_dbg{};
  double m_last_gnss_pos_recover_inflate_dbg{1.0};
  double m_last_gnss_vel_recover_inflate_dbg{1.0};

  // ============================================================
  // GNSS transition (status upgrade) smoothing state
  // ============================================================
  mutable std::mutex m_gnss_recover_mutex_;
  int m_prev_gnss_status_{std::numeric_limits<int>::min()};
  bool m_gnss_recover_active_{false};
  rclcpp::Time m_gnss_recover_holdoff_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time m_gnss_recover_pos_last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time m_gnss_recover_vel_last_stamp_{0, 0, RCL_ROS_TIME};
  double m_gnss_recover_pos_inflate_{1.0};
  double m_gnss_recover_vel_inflate_{1.0};
  rclcpp::Time m_gnss_env_pos_last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time m_gnss_env_vel_last_stamp_{0, 0, RCL_ROS_TIME};
  double m_gnss_env_pos_inflate_{1.0};
  double m_gnss_env_vel_inflate_{1.0};

  // Heading transition/skip robustness envelope
  mutable std::mutex m_heading_recover_mutex_;
  int m_prev_heading_status_{std::numeric_limits<int>::min()};
  rclcpp::Time m_heading_recover_holdoff_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time m_heading_recover_last_stamp_{0, 0, RCL_ROS_TIME};
  double m_heading_recover_inflate_{1.0};
  double m_heading_status_inflate_{1.0};

  // ============================================================
  // Phase 2: KISS-ICP tight coupling (optional)
  // ============================================================
  std::unique_ptr<kiss_icp::pipeline::KissICP> m_kiss_icp_{};
  bool m_kiss_initialized_{false};
  double m_kiss_trust_{0.0}; // low-pass filtered trust in [0,1]
  rclcpp::Time m_last_kiss_update_stamp{0, 0, RCL_ROS_TIME};
  bool m_have_kiss_yaw_ref_{false};
  double m_last_kiss_yaw_ref_rad_{0.0}; // yaw reference at last applied update

  // Auto-reset bookkeeping
  bool m_kiss_reset_candidate_{false};
  rclcpp::Time m_kiss_reset_candidate_since{0, 0, RCL_ROS_TIME};
  size_t m_kiss_reset_count_{0};

  struct KissDiagSnapshot
  {
    bool enabled{false};
    bool initialized{false};
    bool yaw_enabled{false};
    bool vy_enabled{false};
    int source_points{0};
    double dt_ms{0.0};
    double trust{0.0};
    double target_trust{0.0};
    double yaw_rate_radps{0.0};
    double vy_mps{0.0};
    double yaw_var_eff{0.0};
    double vy_var_eff{0.0};
    bool yaw_applied{false};
    std::string yaw_reason{};
    double yaw_nis{0.0};
    double yaw_R{0.0};
    bool vy_applied{false};
    std::string vy_reason{};
    double vy_nis{0.0};
    double vy_R{0.0};
    std::string skip_reason{};
    double time_alignment_error_ms{0.0};
    bool reset_candidate{false};
    size_t reset_count{0};
  };
  mutable std::mutex m_kiss_diag_mutex_;
  KissDiagSnapshot m_kiss_diag_{};
};

} // namespace eskf_localization

#endif // ESKF_LOCALIZATION__ESKF_LOCALIZATION_NODE_HPP_
