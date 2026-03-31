#include "eskf_localization/eskf_localization_node.hpp"

#include <chrono>
#include <cmath>
#include <cstring>

namespace eskf_localization
{

// NOTE: 노드의 수명주기(초기화/구독/퍼블리셔/타이머/TF 설정)를 한 곳에 모아
// 시스템 전체 배선 구조를 빠르게 파악할 수 있도록 구성한다.

ESKFLocalizationNode::ESKFLocalizationNode(const rclcpp::NodeOptions & t_options)
: Node("eskf_localization", t_options), m_imu_count(0), m_gnss_count(0),
  m_gnss_vel_count(0), m_velocity_count(0), m_steering_count(0),
  m_map_projector_received(false), m_last_imu_stamp(0, 0, RCL_ROS_TIME),
  m_last_gnss_stamp(0, 0, RCL_ROS_TIME),
  m_last_gnss_vel_stamp(0, 0, RCL_ROS_TIME),
  m_last_velocity_stamp(0, 0, RCL_ROS_TIME),
  m_last_steering_stamp(0, 0, RCL_ROS_TIME),
  m_last_heading_stamp(0, 0, RCL_ROS_TIME),
  m_last_pointcloud_stamp(0, 0, RCL_ROS_TIME)
{
  RCLCPP_INFO(this->get_logger(), "Initializing ESKF Localization Node");

  load_parameters(); // 파라미터 로딩은 별도 로더에서 일괄 처리

  const auto & io = m_node_params.io;
  OdomBuilderConfig odom_config;
  odom_config.map_frame = io.map_frame;
  odom_config.base_frame = io.base_frame;
  odom_config.gyro_noise_std = m_node_params.eskf.gyro_noise_std;
  m_odom_builder.set_config(odom_config);

  // TF 버퍼/리스너는 초기화 초기에 생성해 다른 모듈에서 사용 가능하도록 한다.
  m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

  m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(
    io.output_odom_topic, 10);
  m_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/localization/pose_twist_fusion_filter/pose", 10);
  m_gnss_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state_gnss", 10);
  if (m_node_params.gnss.publish_pose_with_covariance) {
    m_gnss_pose_cov_pub =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      m_node_params.gnss.pose_with_covariance_topic, 10);
  }
  m_diag_pub = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10);
  m_preprocessed_imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(
    "/sensing/imu/imu_data", 10);
  m_gnss_velocity_pub = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
    "/sensing/gnss/velocity_status", 10);

  m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  m_tf_cache.set_tf_buffer(m_tf_buffer);
  m_tf_cache.set_base_frame(io.base_frame);
  m_tf_cache.cache_common_extrinsics(this->get_logger());

  // 센서 구독/퍼블리셔 설정은 노드 배선의 핵심
  m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
    io.imu_topic, 1,
    std::bind(
      &ESKFLocalizationNode::imu_callback, this,
      std::placeholders::_1));

  m_gnss_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    io.gnss_topic, 1,
    std::bind(
      &ESKFLocalizationNode::gnss_callback, this,
      std::placeholders::_1));

  m_gnss_vel_sub =
    this->create_subscription<geometry_msgs::msg::TwistStamped>(
    io.gnss_vel_topic, 1,
    std::bind(
      &ESKFLocalizationNode::gnss_vel_callback, this,
      std::placeholders::_1));

  m_map_projector_sub =
    this->create_subscription<tier4_map_msgs::msg::MapProjectorInfo>(
    io.map_projector_info_topic, rclcpp::QoS(1).transient_local(),
    std::bind(
      &ESKFLocalizationNode::map_projector_info_callback, this,
      std::placeholders::_1));

  m_velocity_sub =
    this->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
    io.velocity_topic, 1,
    std::bind(
      &ESKFLocalizationNode::velocity_callback, this,
      std::placeholders::_1));

  m_steering_sub =
    this->create_subscription<autoware_vehicle_msgs::msg::SteeringReport>(
    io.steering_topic, 1,
    std::bind(
      &ESKFLocalizationNode::steering_callback, this,
      std::placeholders::_1));

  m_heading_sub = this->create_subscription<std_msgs::msg::Float64>(
    io.heading_topic, 1,
    std::bind(
      &ESKFLocalizationNode::heading_callback, this,
      std::placeholders::_1));

  // Autoware-compatible initial pose input (typically remapped to /initialpose3d)
  m_initialpose_sub =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    m_node_params.init.external_initialpose_topic, 1,
    std::bind(
      &ESKFLocalizationNode::initialpose_callback, this,
      std::placeholders::_1));

  // Autoware-compatible trigger service (activate/deactivate)
  service_trigger_node_ = this->create_service<std_srvs::srv::SetBool>(
    "trigger_node_srv",
    std::bind(
      &ESKFLocalizationNode::service_trigger_node, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile());

  // KISS-ICP tight coupling: subscribe pointcloud inside ESKF process
  if (m_node_params.kiss_icp.enable) {
    kiss_icp::pipeline::KISSConfig config;
    config.deskew = m_node_params.kiss_icp.deskew;
    config.max_range = m_node_params.kiss_icp.max_range;
    config.min_range = m_node_params.kiss_icp.min_range;
    config.voxel_size = m_node_params.kiss_icp.voxel_size;
    config.max_points_per_voxel = m_node_params.kiss_icp.max_points_per_voxel;
    config.max_num_iterations = m_node_params.kiss_icp.max_num_iterations;
    config.convergence_criterion = m_node_params.kiss_icp.convergence_criterion;
    config.max_num_threads = m_node_params.kiss_icp.max_num_threads;
    m_kiss_icp_ = std::make_unique<kiss_icp::pipeline::KissICP>(config);

    m_pointcloud_sub =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
      m_node_params.kiss_icp.pointcloud_topic, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(
        &ESKFLocalizationNode::pointcloud_callback, this,
        std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "KISS-ICP tight coupling: ENABLED");
    RCLCPP_INFO(
      this->get_logger(), " KISS pointcloud: %s",
      m_node_params.kiss_icp.pointcloud_topic.c_str());
    RCLCPP_INFO(
      this->get_logger(), " KISS fuse yaw: %s",
      m_node_params.kiss_icp.enable_yaw_update ? "ON" : "OFF");
    RCLCPP_INFO(
      this->get_logger(), " KISS fuse vy: %s",
      m_node_params.kiss_icp.enable_vy_update ? "ON" : "OFF");
  } else {
    RCLCPP_INFO(this->get_logger(), "KISS-ICP tight coupling: DISABLED");
  }
  {
    std::scoped_lock<std::mutex> diag_lock(m_kiss_diag_mutex_);
    m_kiss_diag_.enabled = m_node_params.kiss_icp.enable;
    m_kiss_diag_.yaw_enabled = m_node_params.kiss_icp.enable_yaw_update;
    m_kiss_diag_.vy_enabled = m_node_params.kiss_icp.enable_vy_update;
  }

  const auto period_ms = static_cast<int>(1000.0 / io.publish_rate);
  // 출력 주기는 파라미터 기반으로 타이머 설정
  m_publish_timer = this->create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&ESKFLocalizationNode::publish_timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "ESKF Localization Node initialized successfully");
  RCLCPP_INFO(this->get_logger(), " IMU topic: %s", io.imu_topic.c_str());
  RCLCPP_INFO(this->get_logger(), " GNSS topic: %s", io.gnss_topic.c_str());
  RCLCPP_INFO(this->get_logger(), " Output topic: %s", io.output_odom_topic.c_str());
  RCLCPP_INFO(this->get_logger(), " GNSS velocity topic: /sensing/gnss/velocity_status");
  RCLCPP_INFO(this->get_logger(), " Publish rate: %.1f Hz", io.publish_rate);
  RCLCPP_INFO(this->get_logger(), " TF publish: %s", io.publish_tf ? "ON" : "OFF");
  RCLCPP_INFO(this->get_logger(), " Heading topic: %s", io.heading_topic.c_str());
  RCLCPP_INFO(
    this->get_logger(), " Init mode: %s",
    m_node_params.init.mode.c_str());
  RCLCPP_INFO(
    this->get_logger(), " Require trigger: %s",
    m_node_params.init.require_trigger ? "true" : "false");
  RCLCPP_INFO(
    this->get_logger(), " External initialpose topic: %s",
    m_node_params.init.external_initialpose_topic.c_str());
  if (m_node_params.gnss.publish_pose_with_covariance) {
    RCLCPP_INFO(
      this->get_logger(), " GNSS pose_with_covariance: %s",
      m_node_params.gnss.pose_with_covariance_topic.c_str());
  }

  // IMU 캘리브레이션 경로/시작 여부는 파라미터에 의해 결정
  if (m_node_params.init_imu_calibration) {
    RCLCPP_INFO(
      this->get_logger(), "Starting IMU calibration (duration: %.1f seconds)",
      IMU_CALIBRATION_DURATION_SEC);
    RCLCPP_WARN(this->get_logger(), "Please ensure the vehicle is stationary during calibration!");
    m_imu_calibration_manager.start(this->now(), IMU_CALIBRATION_DURATION_SEC);
  } else {
    if (m_imu_preprocessor.load_calibration(
        m_node_params.calibration_file_path))
    {
      RCLCPP_INFO(
        this->get_logger(), "Loaded IMU calibration from file: %s",
        m_node_params.calibration_file_path.c_str());
      RCLCPP_INFO(
        this->get_logger(), " Gyro bias: [%.6f, %.6f, %.6f] rad/s",
        m_imu_preprocessor.calibration().gyro_bias.x,
        m_imu_preprocessor.calibration().gyro_bias.y,
        m_imu_preprocessor.calibration().gyro_bias.z);
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to load IMU calibration from file: %s. Please run with " "init_imu_calibration:=true to calibrate.",
        m_node_params.calibration_file_path.c_str());
    }
  }
}

bool ESKFLocalizationNode::validate_stamp_and_order(
  const rclcpp::Time & current_stamp, const rclcpp::Time & now,
  const rclcpp::Time & last_stamp, const char * label)
{
  auto clock = this->get_clock();
  const auto stamp_result = m_time_processor.validate_stamp(current_stamp, now);
  if (stamp_result == StampValidationResult::kInvalidStamp) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock, 5000, "%s: Invalid stamp (zero or negative), skipping", label);
    return false;
  }

  const double delay = (now - current_stamp).seconds();
  if (stamp_result == StampValidationResult::kTooOld) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock, 5000, "%s: Delay %.3fs exceeds max %.3fs, skipping", label, delay,
      m_node_params.time.max_delay);
    return false;
  }

  if (stamp_result == StampValidationResult::kFutureStamp) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock, 5000, "%s: Future stamp (%.3fs), skipping", label, delay);
    return false;
  }

  if (m_time_processor.validate_order(last_stamp, current_stamp) ==
    OrderValidationResult::kOutOfOrderOrDuplicate)
  {
    const double dt = (current_stamp - last_stamp).seconds();
    if (label != nullptr && std::strcmp(label, "Velocity") == 0 &&
      std::isfinite(dt) && std::abs(dt) <= 1e-12)
    {
      return false;
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock, 5000, "%s: Out-of-order or duplicate stamp (dt=%.6fs), skipping", label,
      dt);
    return false;
  }

  return true;
}

void ESKFLocalizationNode::reset_imu_dt_stats() {m_imu_dt_stats.reset();}

} // namespace eskf_localization
