#include "aruco_imu_eskf_localization_cpp/rviz_overlay/aruco_imu_eskf_debug_display.hpp"

#include <QFont>
#include <QPainter>
#include <QPolygonF>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_rendering/render_system.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace aruco_imu_eskf_localization_cpp
{
namespace
{

double quat_to_yaw(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double rad_to_deg(double rad)
{
  return rad * 180.0 / M_PI;
}

bool starts_with(const std::string & value, const char * prefix)
{
  return value.rfind(prefix, 0) == 0;
}

enum class StateGroup
{
  Applied,
  Neutral,
  Warning,
  Rejected
};

struct StateCell
{
  const char * key;
  const char * label;
  StateGroup group;
};

constexpr std::array<StateCell, 15> kDetectorStates{{
  {"published", "Pub", StateGroup::Applied},
  {"not_received", "No Rx", StateGroup::Neutral},
  {"not_started", "Idle", StateGroup::Neutral},
  {"processing", "Proc", StateGroup::Neutral},
  {"no_markers", "No Mk", StateGroup::Warning},
  {"known_below_min", "Known<Min", StateGroup::Warning},
  {"need_init", "Need Init", StateGroup::Warning},
  {"pnp_failed", "PnP Fail", StateGroup::Warning},
  {"view_z", "View Z", StateGroup::Warning},
  {"view_angle", "View Ang", StateGroup::Warning},
  {"feasible_box", "Box", StateGroup::Warning},
  {"motion_gate", "Motion", StateGroup::Warning},
  {"reference_rotation", "Ref Rot", StateGroup::Warning},
  {"reprojection", "Reproj", StateGroup::Warning},
  {"unknown", "Unknown", StateGroup::Warning},
}};

constexpr std::array<StateCell, 7> kArucoStates{{
  {"initialized_translation_only", "Init XYZ", StateGroup::Applied},
  {"aruco_position_smoothing", "Pos Upd", StateGroup::Applied},
  {"not_received", "No Rx", StateGroup::Neutral},
  {"out_of_order", "Order", StateGroup::Rejected},
  {"outside_history", "History", StateGroup::Rejected},
  {"not_initialized", "No Init", StateGroup::Rejected},
  {"unknown", "Unknown", StateGroup::Rejected},
}};

constexpr std::array<StateCell, 19> kLidarStates{{
  {"yaw_update", "Yaw", StateGroup::Applied},
  {"recovery_yaw_update", "Recover", StateGroup::Applied},
  {"disabled", "Off", StateGroup::Neutral},
  {"eskf_not_initialized", "No ESKF", StateGroup::Neutral},
  {"bootstrap_dt", "Boot", StateGroup::Neutral},
  {"init_yaw_ref", "Yaw Ref", StateGroup::Neutral},
  {"out_of_order", "Order", StateGroup::Rejected},
  {"lidar_tf_unavailable", "No TF", StateGroup::Rejected},
  {"empty_points", "Empty", StateGroup::Rejected},
  {"min_source_points", "Pts<Min", StateGroup::Rejected},
  {"missing_filter_stamp", "No Stamp", StateGroup::Rejected},
  {"invalid_dt", "Bad dt", StateGroup::Rejected},
  {"yaw_rate_gate", "Rate", StateGroup::Rejected},
  {"outside_history", "History", StateGroup::Rejected},
  {"yaw_gate", "Yaw Gate", StateGroup::Rejected},
  {"invalid_measurement", "Bad Meas", StateGroup::Rejected},
  {"invalid_covariance", "Bad Cov", StateGroup::Rejected},
  {"recovery_failed", "Rec Fail", StateGroup::Rejected},
  {"unknown", "Unknown", StateGroup::Rejected},
}};

constexpr std::array<StateCell, 6> kCalibrationStates{{
  {"disabled", "Off", StateGroup::Neutral},
  {"waiting_stationary", "Wait", StateGroup::Warning},
  {"collecting", "Collect", StateGroup::Warning},
  {"failed_motion_detected", "Motion", StateGroup::Rejected},
  {"valid", "Valid", StateGroup::Applied},
  {"unknown", "Unknown", StateGroup::Rejected},
}};

template<size_t N>
bool has_state_key(const std::array<StateCell, N> & states, const std::string & key)
{
  return std::any_of(states.begin(), states.end(), [&key](const StateCell & state) {
    return key == state.key;
  });
}

std::string normalize_detector_reason(const std::string & reason)
{
  if (reason == "known_markers_below_min") {
    return "known_below_min";
  }
  if (reason == "need_more_markers_to_initialize") {
    return "need_init";
  }
  if (reason == "solve_pnp_failed") {
    return "pnp_failed";
  }
  if (reason == "no_markers_detected") {
    return "no_markers";
  }
  if (starts_with(reason, "view_gate_z")) {
    return "view_z";
  }
  if (starts_with(reason, "view_angle_gate")) {
    return "view_angle";
  }
  if (starts_with(reason, "feasible_box_gate")) {
    return "feasible_box";
  }
  if (starts_with(reason, "motion_position_gate")) {
    return "motion_gate";
  }
  if (starts_with(reason, "reference_rotation_gate")) {
    return "reference_rotation";
  }
  if (starts_with(reason, "reprojection_gate")) {
    return "reprojection";
  }
  return has_state_key(kDetectorStates, reason) ? reason : "unknown";
}

std::string normalize_aruco_reason(const std::string & reason)
{
  if (reason == "none" || reason.empty()) {
    return "not_received";
  }
  return has_state_key(kArucoStates, reason) ? reason : "unknown";
}

std::string normalize_lidar_reason(const std::string & reason)
{
  if (reason != "recovery_yaw_update" && starts_with(reason, "recovery_")) {
    return "recovery_failed";
  }
  return has_state_key(kLidarStates, reason) ? reason : "unknown";
}

std::string normalize_calibration_status(const std::string & status)
{
  return has_state_key(kCalibrationStates, status) ? status : "unknown";
}

QColor color_for_group(StateGroup group)
{
  switch (group) {
    case StateGroup::Applied:
      return QColor(40, 190, 90, 210);
    case StateGroup::Warning:
      return QColor(230, 165, 45, 210);
    case StateGroup::Rejected:
      return QColor(220, 75, 70, 210);
    case StateGroup::Neutral:
    default:
      return QColor(95, 110, 125, 190);
  }
}

QColor blend_color(const QColor & a, const QColor & b, double t)
{
  t = std::clamp(t, 0.0, 1.0);
  return QColor(
    static_cast<int>(a.red() * (1.0 - t) + b.red() * t),
    static_cast<int>(a.green() * (1.0 - t) + b.green() * t),
    static_cast<int>(a.blue() * (1.0 - t) + b.blue() * t),
    static_cast<int>(a.alpha() * (1.0 - t) + b.alpha() * t));
}

double seconds_since(const rclcpp::Time & now, const rclcpp::Time & then)
{
  if (now.nanoseconds() == 0 || then.nanoseconds() == 0) {
    return std::numeric_limits<double>::infinity();
  }
  const double dt = (now - then).seconds();
  return dt >= 0.0 ? dt : std::numeric_limits<double>::infinity();
}

template<size_t N>
void draw_state_grid(
  QPainter & painter,
  const QRectF & rect,
  const QString & title,
  const std::array<StateCell, N> & states,
  const std::string & active_key,
  const rclcpp::Time & last_change_time,
  const rclcpp::Time & now,
  int columns)
{
  painter.setPen(QColor(255, 255, 255));
  QFont title_font = painter.font();
  title_font.setPointSize(9);
  title_font.setBold(true);
  painter.setFont(title_font);
  painter.drawText(QRectF(rect.x(), rect.y(), rect.width(), 14), Qt::AlignLeft, title);

  QFont cell_font = painter.font();
  cell_font.setPointSize(7);
  cell_font.setBold(false);
  painter.setFont(cell_font);

  columns = std::max(1, columns);
  const int rows = static_cast<int>((states.size() + columns - 1) / columns);
  const double gap = 4.0;
  const double title_h = 18.0;
  const double cell_w = (rect.width() - gap * (columns - 1)) / columns;
  const double cell_h = (rect.height() - title_h - gap * (rows - 1)) / rows;
  const double flash = std::clamp(1.0 - seconds_since(now, last_change_time) / 0.4, 0.0, 1.0);

  for (size_t i = 0; i < states.size(); ++i) {
    const int row = static_cast<int>(i) / columns;
    const int col = static_cast<int>(i) % columns;
    const QRectF cell(
      rect.x() + col * (cell_w + gap),
      rect.y() + title_h + row * (cell_h + gap),
      cell_w,
      cell_h);
    const bool active = active_key == states[i].key;
    QColor fill = active ? color_for_group(states[i].group) : QColor(58, 58, 58, 145);
    if (active && flash > 0.0) {
      fill = blend_color(fill, QColor(255, 255, 255, 245), flash * 0.55);
    }
    painter.setPen(QColor(210, 210, 210, active ? 230 : 120));
    painter.setBrush(fill);
    painter.drawRect(cell);
    painter.setPen(QColor(255, 255, 255, active ? 255 : 150));
    painter.drawText(cell.adjusted(2, 0, -2, 0), Qt::AlignCenter, states[i].label);
  }
}

void draw_innovation_bar(
  QPainter & painter,
  const QRectF & rect,
  double value,
  double gate,
  const QString & label,
  const QString & unit)
{
  painter.setPen(QColor(255, 255, 255));
  QFont font = painter.font();
  font.setPointSize(9);
  font.setBold(true);
  painter.setFont(font);

  if (!std::isfinite(value) || !std::isfinite(gate) || gate <= 0.0) {
    painter.drawText(QRectF(rect.x(), rect.y(), rect.width(), 16), Qt::AlignLeft, label + ": --");
    return;
  }

  const double ratio = std::abs(value) / gate;
  const QColor fill = ratio <= 0.75 ? QColor(45, 190, 95, 220) :
    ratio <= 1.0 ? QColor(230, 165, 45, 220) : QColor(220, 75, 70, 220);
  painter.drawText(
    QRectF(rect.x(), rect.y(), rect.width(), 16),
    Qt::AlignLeft,
    QString("%1: %2 / %3 %4")
    .arg(label)
    .arg(value, 0, 'f', 2)
    .arg(gate, 0, 'f', 2)
    .arg(unit));

  const QRectF bar(rect.x(), rect.y() + 20.0, rect.width(), 10.0);
  painter.setPen(QColor(210, 210, 210, 140));
  painter.setBrush(QColor(58, 58, 58, 150));
  painter.drawRect(bar);
  const double overflow_scale = 1.35;
  const double fill_w = bar.width() * std::min(ratio, overflow_scale) / overflow_scale;
  painter.setPen(Qt::NoPen);
  painter.setBrush(fill);
  painter.drawRect(QRectF(bar.x(), bar.y(), fill_w, bar.height()));
  const double gate_x = bar.x() + bar.width() / overflow_scale;
  painter.setPen(QPen(QColor(255, 255, 255, 210), 1));
  painter.drawLine(QPointF(gate_x, bar.y() - 2.0), QPointF(gate_x, bar.bottom() + 2.0));
}

void draw_heartbeat_dot(
  QPainter & painter,
  double x,
  double y,
  const QString & label,
  const rclcpp::Time & last_receive_time,
  const rclcpp::Time & now)
{
  const double age_sec = seconds_since(now, last_receive_time);
  const QColor color = age_sec < 0.2 ? QColor(45, 195, 95) :
    age_sec < 1.0 ? QColor(230, 170, 45) : QColor(220, 75, 70);
  painter.setPen(Qt::NoPen);
  painter.setBrush(color);
  painter.drawEllipse(QPointF(x, y), 5.0, 5.0);

  painter.setPen(QColor(255, 255, 255));
  const QString age = std::isfinite(age_sec) ?
    QString("%1 ms").arg(age_sec * 1000.0, 0, 'f', 0) : QString("-- ms");
  painter.drawText(QRectF(x + 8.0, y - 9.0, 92.0, 18.0), Qt::AlignLeft | Qt::AlignVCenter, label + " " + age);
}

void draw_status_cell(
  QPainter & painter,
  double x,
  double y,
  double w,
  double h,
  const QString & label,
  bool active)
{
  painter.setPen(QColor(220, 220, 220, 180));
  painter.setBrush(active ? QColor(40, 180, 80, 190) : QColor(70, 70, 70, 140));
  painter.drawRect(QRectF(x, y, w, h));
  painter.setPen(QColor(255, 255, 255));
  painter.drawText(QRectF(x, y, w, h), Qt::AlignCenter, label);
}

void draw_compass(
  QPainter & painter,
  const QPointF & center,
  double radius,
  double yaw_rad,
  const QColor & color,
  const QString & label)
{
  painter.setPen(QPen(color, 2));
  painter.setBrush(QColor(color.red(), color.green(), color.blue(), 45));
  painter.drawEllipse(center, radius, radius);
  painter.setPen(QColor(255, 255, 255));
  painter.drawText(static_cast<int>(center.x() - 18), static_cast<int>(center.y() - radius - 8), label);
  painter.drawText(static_cast<int>(center.x() + radius + 4), static_cast<int>(center.y() + 4), "E");

  painter.save();
  painter.translate(center);
  painter.rotate(-rad_to_deg(yaw_rad));
  painter.setPen(QPen(color, 2));
  painter.drawLine(QPointF(0, 0), QPointF(radius * 0.8, 0));
  QPolygonF arrow;
  arrow << QPointF(radius * 0.8, 0) << QPointF(radius * 0.62, -5) << QPointF(radius * 0.62, 5);
  painter.setBrush(color);
  painter.drawPolygon(arrow);
  painter.restore();

  painter.setPen(QColor(255, 255, 255));
  painter.drawText(
    static_cast<int>(center.x() - 34),
    static_cast<int>(center.y() + radius + 16),
    QString("%1 deg").arg(rad_to_deg(yaw_rad), 0, 'f', 1));
}

std::string kv_value(
  const diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key,
  const std::string & fallback = "")
{
  for (const auto & kv : status.values) {
    if (kv.key == key) {
      return kv.value;
    }
  }
  return fallback;
}

double kv_double(
  const diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key,
  double fallback = 0.0)
{
  try {
    return std::stod(kv_value(status, key, std::to_string(fallback)));
  } catch (...) {
    return fallback;
  }
}

bool kv_bool(const diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key)
{
  return kv_value(status, key, "false") == "true";
}

int kv_int(
  const diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key,
  int fallback = 0)
{
  try {
    return std::stoi(kv_value(status, key, std::to_string(fallback)));
  } catch (...) {
    return fallback;
  }
}

}  // namespace

ArucoImuEskfDebugDisplay::ArucoImuEskfDebugDisplay()
{
  property_width_ = new rviz_common::properties::IntProperty(
    "Width", 480, "Width of the overlay", this, SLOT(updateOverlaySize()));
  property_height_ = new rviz_common::properties::IntProperty(
    "Height", 700, "Height of the overlay", this, SLOT(updateOverlaySize()));
  property_left_ = new rviz_common::properties::IntProperty(
    "Left", 10, "Left position of the overlay", this, SLOT(updateOverlayPosition()));
  property_top_ = new rviz_common::properties::IntProperty(
    "Top", 10, "Top position of the overlay", this, SLOT(updateOverlayPosition()));
}

ArucoImuEskfDebugDisplay::~ArucoImuEskfDebugDisplay()
{
  std::lock_guard<std::mutex> lock(mutex_);
  overlay_.reset();
  odom_sub_.reset();
  imu_sub_.reset();
  diagnostics_sub_.reset();
}

void ArucoImuEskfDebugDisplay::onInitialize()
{
  rviz_common::Display::onInitialize();
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);

  static int count = 0;
  overlay_ = std::make_shared<rviz_overlay::OverlayObject>(
    "ArucoImuEskfDebugOverlay" + std::to_string(count++));
  overlay_->show();
  updateOverlaySize();
  updateOverlayPosition();

  auto rviz_ros_node = context_->getRosNodeAbstraction();
  odom_topic_property_ = std::make_unique<rviz_common::properties::RosTopicProperty>(
    "ESKF Odom Topic", "/follower/localization/leader_rear/odom", "nav_msgs/msg/Odometry",
    "Fused leader_rear -> base odometry", this, SLOT(topic_updated_odom()));
  odom_topic_property_->initialize(rviz_ros_node);
  imu_topic_property_ = std::make_unique<rviz_common::properties::RosTopicProperty>(
    "IMU Topic", "/follower/imu", "sensor_msgs/msg/Imu", "IMU orientation and gyro",
    this, SLOT(topic_updated_imu()));
  imu_topic_property_->initialize(rviz_ros_node);
  diagnostics_topic_property_ = std::make_unique<rviz_common::properties::RosTopicProperty>(
    "Diagnostics Topic", "/follower/diagnostics", "diagnostic_msgs/msg/DiagnosticArray",
    "ArUco IMU ESKF diagnostics", this, SLOT(topic_updated_diagnostics()));
  diagnostics_topic_property_->initialize(rviz_ros_node);
}

void ArucoImuEskfDebugDisplay::onEnable()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (overlay_) {
      overlay_->show();
    }
  }
  setupRosSubscriptions();
}

void ArucoImuEskfDebugDisplay::onDisable()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (overlay_) {
    overlay_->hide();
  }
}

void ArucoImuEskfDebugDisplay::reset()
{
  rviz_common::Display::reset();
}

void ArucoImuEskfDebugDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!overlay_ || !overlay_->isVisible() || overlay_->getTextureWidth() == 0) {
    return;
  }
  auto buffer = overlay_->getBuffer();
  QImage hud = buffer.getQImage(*overlay_);
  hud.fill(QColor(0, 0, 0, 60));
  drawWidget(hud);
}

rclcpp::Time ArucoImuEskfDebugDisplay::currentTime() const
{
  rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock.now();
}

void ArucoImuEskfDebugDisplay::updateOverlaySize()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!overlay_) {
    return;
  }
  overlay_->updateTextureSize(property_width_->getInt(), property_height_->getInt());
  overlay_->setDimensions(property_width_->getInt(), property_height_->getInt());
}

void ArucoImuEskfDebugDisplay::updateOverlayPosition()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!overlay_) {
    return;
  }
  overlay_->setPosition(
    property_left_->getInt(), property_top_->getInt(),
    rviz_overlay::HorizontalAlignment::LEFT,
    rviz_overlay::VerticalAlignment::TOP);
}

void ArucoImuEskfDebugDisplay::setupRosSubscriptions()
{
  topic_updated_odom();
  topic_updated_imu();
  topic_updated_diagnostics();
}

void ArucoImuEskfDebugDisplay::topic_updated_odom()
{
  std::lock_guard<std::mutex> lock(mutex_);
  odom_sub_.reset();
  auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
  if (!rviz_ros_node) {
    return;
  }
  auto node = rviz_ros_node->get_raw_node();
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_property_->getStdString(), rclcpp::QoS(10),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {odom_callback(msg);});
}

void ArucoImuEskfDebugDisplay::topic_updated_imu()
{
  std::lock_guard<std::mutex> lock(mutex_);
  imu_sub_.reset();
  auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
  if (!rviz_ros_node) {
    return;
  }
  auto node = rviz_ros_node->get_raw_node();
  imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_property_->getStdString(), rclcpp::QoS(10),
    [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {imu_callback(msg);});
}

void ArucoImuEskfDebugDisplay::topic_updated_diagnostics()
{
  std::lock_guard<std::mutex> lock(mutex_);
  diagnostics_sub_.reset();
  auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
  if (!rviz_ros_node) {
    return;
  }
  auto node = rviz_ros_node->get_raw_node();
  diagnostics_sub_ = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    diagnostics_topic_property_->getStdString(), rclcpp::QoS(10),
    [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {diagnostics_callback(msg);});
}

void ArucoImuEskfDebugDisplay::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  data_.last_odom_receive_time = currentTime();
  const auto & p = msg->pose.pose.position;
  const auto & q = msg->pose.pose.orientation;
  data_.odom_x = p.x;
  data_.odom_y = p.y;
  data_.odom_z = p.z;
  data_.filter_yaw_rad = quat_to_yaw(q.x, q.y, q.z, q.w);
}

void ArucoImuEskfDebugDisplay::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  data_.last_imu_receive_time = currentTime();
  const auto & q = msg->orientation;
  data_.imu_yaw_reference_only_rad = quat_to_yaw(q.x, q.y, q.z, q.w);
  data_.raw_gyro_z_radps = msg->angular_velocity.z;
  data_.imu_orientation_valid = true;
}

void ArucoImuEskfDebugDisplay::diagnostics_callback(
  const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const rclcpp::Time now = currentTime();
  data_.last_diag_receive_time = now;
  for (const auto & status : msg->status) {
    if (status.name == "aruco_detector") {
      data_.detector_pose_published = kv_bool(status, "pose_published");
      data_.detector_detected_markers = kv_int(status, "detected_markers");
      data_.detector_known_markers = kv_int(status, "known_markers");
      data_.detector_min_markers = kv_int(status, "min_markers_for_board");
      data_.detector_min_init_markers = kv_int(status, "min_markers_to_initialize");
      const std::string next_reason = kv_value(status, "rejection_reason", "not_received");
      if (normalize_detector_reason(next_reason) !=
        normalize_detector_reason(data_.detector_rejection_reason))
      {
        data_.last_detector_change_time = now;
      }
      data_.detector_rejection_reason = next_reason;
      continue;
    }
    if (status.name != "aruco_imu_eskf_localization") {
      continue;
    }
    data_.eskf_initialized = kv_bool(status, "eskf_initialized");
    data_.aruco_applied = kv_bool(status, "aruco_update_applied");
    const std::string next_aruco_reason = kv_value(status, "aruco_update_reason", "not_received");
    if (normalize_aruco_reason(next_aruco_reason) != normalize_aruco_reason(data_.aruco_reason)) {
      data_.last_aruco_change_time = now;
    }
    data_.aruco_reason = next_aruco_reason;
    data_.last_skip_reason = kv_value(status, "last_skip_reason", "none");
    data_.filter_yaw_rad = kv_double(status, "filter_yaw_rad", data_.filter_yaw_rad);
    data_.imu_yaw_reference_only_rad = kv_double(
      status, "imu_yaw_reference_only_rad", data_.imu_yaw_reference_only_rad);
    data_.imu_orientation_valid = kv_bool(status, "imu_orientation_valid");
    data_.raw_gyro_z_radps = kv_double(status, "raw_gyro_z_radps", data_.raw_gyro_z_radps);
    data_.corrected_gyro_z_radps =
      kv_double(status, "corrected_gyro_z_radps", data_.corrected_gyro_z_radps);
    data_.gyro_z_bias_radps = kv_double(status, "gyro_z_bias_radps", data_.gyro_z_bias_radps);
    data_.gyro_bias_valid = kv_bool(status, "gyro_bias_valid");
    data_.stationary_detected = kv_bool(status, "stationary_detected");
    const std::string next_calibration_status =
      kv_value(status, "calibration_status", data_.calibration_status);
    if (normalize_calibration_status(next_calibration_status) !=
      normalize_calibration_status(data_.calibration_status))
    {
      data_.last_calibration_change_time = now;
    }
    data_.calibration_status = next_calibration_status;
    data_.aruco_position_innovation_m = kv_double(status, "aruco_position_innovation_m");
    data_.aruco_position_gate_m = kv_double(status, "aruco_position_gate_m");
    data_.aruco_position_covariance_scale = kv_double(status, "aruco_position_covariance_scale", 1.0);
    data_.aruco_rotation_innovation_deg = kv_double(status, "aruco_rotation_innovation_deg");
    data_.lidar_icp_enabled = kv_bool(status, "lidar_icp_enabled");
    data_.lidar_icp_initialized = kv_bool(status, "lidar_icp_initialized");
    data_.lidar_icp_update_applied = kv_bool(status, "lidar_icp_update_applied");
    data_.lidar_icp_recovery_active = kv_bool(status, "lidar_icp_recovery_active");
    const std::string next_lidar_reason =
      kv_value(status, "lidar_icp_update_reason", data_.lidar_icp_update_reason);
    if (normalize_lidar_reason(next_lidar_reason) !=
      normalize_lidar_reason(data_.lidar_icp_update_reason))
    {
      data_.last_lidar_change_time = now;
    }
    data_.lidar_icp_update_reason = next_lidar_reason;
    data_.lidar_icp_yaw_innovation_rad =
      kv_double(status, "lidar_icp_yaw_innovation_rad", data_.lidar_icp_yaw_innovation_rad);
    data_.lidar_icp_yaw_gate_rad =
      kv_double(status, "lidar_icp_yaw_gate_rad", data_.lidar_icp_yaw_gate_rad);
  }
}

void ArucoImuEskfDebugDisplay::drawWidget(QImage & hud)
{
  const rclcpp::Time now = currentTime();
  QPainter painter(&hud);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(QColor(255, 255, 255));
  const double left = 12.0;
  const double width = std::max(120, hud.width() - 24);

  QFont title = painter.font();
  title.setPointSize(14);
  title.setBold(true);
  painter.setFont(title);
  painter.drawText(12, 24, "ARUCO IMU ESKF DEBUG");

  QFont font = painter.font();
  font.setPointSize(9);
  font.setBold(false);
  painter.setFont(font);
  draw_compass(painter, QPointF(95, 92), 34, data_.filter_yaw_rad, QColor(60, 220, 100), "ESKF");
  draw_compass(
    painter, QPointF(220, 92), 34, data_.imu_yaw_reference_only_rad,
    QColor(255, 170, 40), "IMU ref");

  painter.setFont(font);
  painter.setPen(QColor(255, 255, 255));
  painter.drawText(12, 148, "Heartbeat");
  draw_heartbeat_dot(painter, 82, 144, "odom", data_.last_odom_receive_time, now);
  draw_heartbeat_dot(painter, 196, 144, "imu", data_.last_imu_receive_time, now);
  draw_heartbeat_dot(painter, 300, 144, "diag", data_.last_diag_receive_time, now);

  QFont section = painter.font();
  section.setPointSize(10);
  section.setBold(true);
  painter.setFont(section);
  painter.drawText(12, 180, "Stage Flags");
  painter.setFont(font);
  const double flag_gap = 4.0;
  const double flag_w = (width - flag_gap * 5.0) / 6.0;
  draw_status_cell(
    painter, left + 0.0 * (flag_w + flag_gap), 190, flag_w, 25, "ESKF",
    data_.eskf_initialized);
  draw_status_cell(
    painter, left + 1.0 * (flag_w + flag_gap), 190, flag_w, 25, "ArUco",
    data_.aruco_applied);
  draw_status_cell(
    painter, left + 2.0 * (flag_w + flag_gap), 190, flag_w, 25, "Bias",
    data_.gyro_bias_valid);
  draw_status_cell(
    painter, left + 3.0 * (flag_w + flag_gap), 190, flag_w, 25, "Detect",
    data_.detector_pose_published);
  draw_status_cell(
    painter, left + 4.0 * (flag_w + flag_gap), 190, flag_w, 25, "LiDAR",
    data_.lidar_icp_update_applied);
  draw_status_cell(
    painter, left + 5.0 * (flag_w + flag_gap), 190, flag_w, 25, "Recover",
    data_.lidar_icp_recovery_active);

  draw_state_grid(
    painter, QRectF(left, 226, width, 82), "Detector",
    kDetectorStates, normalize_detector_reason(data_.detector_rejection_reason),
    data_.last_detector_change_time, now, 5);
  draw_state_grid(
    painter, QRectF(left, 316, width, 62), "ESKF ArUco",
    kArucoStates, normalize_aruco_reason(data_.aruco_reason),
    data_.last_aruco_change_time, now, 4);
  draw_state_grid(
    painter, QRectF(left, 386, width, 104), "LiDAR ICP",
    kLidarStates, normalize_lidar_reason(data_.lidar_icp_update_reason),
    data_.last_lidar_change_time, now, 5);
  draw_state_grid(
    painter, QRectF(left, 498, width, 44), "Calibration",
    kCalibrationStates, normalize_calibration_status(data_.calibration_status),
    data_.last_calibration_change_time, now, 6);

  draw_innovation_bar(
    painter, QRectF(left, 554, width, 34), data_.aruco_position_innovation_m,
    data_.aruco_position_gate_m, "ArUco pos innovation", "m");
  draw_innovation_bar(
    painter, QRectF(left, 596, width, 34), rad_to_deg(data_.lidar_icp_yaw_innovation_rad),
    rad_to_deg(data_.lidar_icp_yaw_gate_rad), "LiDAR yaw innovation", "deg");

  painter.setPen(QColor(255, 255, 255));
  QFont kpi = painter.font();
  kpi.setPointSize(10);
  kpi.setBold(true);
  painter.setFont(kpi);
  painter.drawText(
    12, 648,
    QString("Markers: %1 / %2   (min %3 / init %4)")
    .arg(data_.detector_detected_markers)
    .arg(data_.detector_known_markers)
    .arg(data_.detector_min_markers)
    .arg(data_.detector_min_init_markers));
  painter.drawText(
    12, 670,
    QString("Rel xyz: %1, %2, %3 m")
    .arg(data_.odom_x, 0, 'f', 2).arg(data_.odom_y, 0, 'f', 2).arg(data_.odom_z, 0, 'f', 2));
  painter.drawText(
    12, 692,
    QString("Gyro z raw/corr/bias: %1 / %2 / %3 rad/s")
    .arg(data_.raw_gyro_z_radps, 0, 'f', 3)
    .arg(data_.corrected_gyro_z_radps, 0, 'f', 3)
    .arg(data_.gyro_z_bias_radps, 0, 'f', 3));

  painter.end();
}

}  // namespace aruco_imu_eskf_localization_cpp

PLUGINLIB_EXPORT_CLASS(
  aruco_imu_eskf_localization_cpp::ArucoImuEskfDebugDisplay,
  rviz_common::Display)
