#include "aruco_imu_eskf_localization_cpp/rviz_overlay/aruco_imu_eskf_debug_display.hpp"

#include <QFont>
#include <QPainter>
#include <QPolygonF>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_rendering/render_system.hpp>

#include <cmath>
#include <iomanip>
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
    "Width", 430, "Width of the overlay", this, SLOT(updateOverlaySize()));
  property_height_ = new rviz_common::properties::IntProperty(
    "Height", 450, "Height of the overlay", this, SLOT(updateOverlaySize()));
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
  const auto & q = msg->orientation;
  data_.imu_yaw_reference_only_rad = quat_to_yaw(q.x, q.y, q.z, q.w);
  data_.raw_gyro_z_radps = msg->angular_velocity.z;
  data_.imu_orientation_valid = true;
}

void ArucoImuEskfDebugDisplay::diagnostics_callback(
  const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & status : msg->status) {
    if (status.name == "aruco_detector") {
      data_.detector_pose_published = kv_bool(status, "pose_published");
      data_.detector_detected_markers = kv_int(status, "detected_markers");
      data_.detector_known_markers = kv_int(status, "known_markers");
      data_.detector_min_markers = kv_int(status, "min_markers_for_board");
      data_.detector_min_init_markers = kv_int(status, "min_markers_to_initialize");
      data_.detector_rejection_reason = kv_value(status, "rejection_reason", "none");
      continue;
    }
    if (status.name != "aruco_imu_eskf_localization") {
      continue;
    }
    data_.eskf_initialized = kv_bool(status, "eskf_initialized");
    data_.aruco_applied = kv_bool(status, "aruco_update_applied");
    data_.aruco_reason = kv_value(status, "aruco_update_reason", "none");
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
    data_.calibration_status = kv_value(status, "calibration_status", data_.calibration_status);
    data_.aruco_position_innovation_m = kv_double(status, "aruco_position_innovation_m");
    data_.aruco_position_gate_m = kv_double(status, "aruco_position_gate_m");
    data_.aruco_position_covariance_scale = kv_double(status, "aruco_position_covariance_scale", 1.0);
    data_.aruco_rotation_innovation_deg = kv_double(status, "aruco_rotation_innovation_deg");
    return;
  }
}

void ArucoImuEskfDebugDisplay::drawWidget(QImage & hud)
{
  QPainter painter(&hud);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setPen(QColor(255, 255, 255));

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

  painter.setPen(QColor(255, 255, 255));
  painter.drawText(12, 162, QString("Relative xyz: %1, %2, %3 m")
    .arg(data_.odom_x, 0, 'f', 2).arg(data_.odom_y, 0, 'f', 2).arg(data_.odom_z, 0, 'f', 2));
  painter.drawText(
    12, 182,
    QString("Gyro z raw/corr/bias: %1 / %2 / %3 rad/s")
    .arg(data_.raw_gyro_z_radps, 0, 'f', 3)
    .arg(data_.corrected_gyro_z_radps, 0, 'f', 3)
    .arg(data_.gyro_z_bias_radps, 0, 'f', 3));
  painter.drawText(
    12, 202,
    QString("Gyro cal: %1, valid %2, stationary %3")
    .arg(QString::fromStdString(data_.calibration_status).left(24))
    .arg(data_.gyro_bias_valid ? "yes" : "no")
    .arg(data_.stationary_detected ? "yes" : "no"));

  QFont section = painter.font();
  section.setPointSize(11);
  section.setBold(true);
  painter.setFont(section);
  painter.drawText(12, 240, "Update State");
  painter.setFont(font);
  draw_status_cell(painter, 12, 252, 92, 28, "ESKF Init", data_.eskf_initialized);
  draw_status_cell(painter, 110, 252, 92, 28, "ArUco", data_.aruco_applied);
  draw_status_cell(painter, 208, 252, 92, 28, "Gyro Bias", data_.gyro_bias_valid);
  draw_status_cell(painter, 306, 252, 92, 28, "Detector", data_.detector_pose_published);

  painter.setPen(QColor(255, 255, 255));
  const QString aruco_reason = QString::fromStdString(data_.aruco_reason).left(42);
  const QString skip_reason = QString::fromStdString(data_.last_skip_reason).left(42);
  const QString detector_reason = QString::fromStdString(data_.detector_rejection_reason).left(42);
  painter.drawText(
    12, 302,
    QString("Detector markers: %1 detected, %2 known, min %3/init %4")
    .arg(data_.detector_detected_markers)
    .arg(data_.detector_known_markers)
    .arg(data_.detector_min_markers)
    .arg(data_.detector_min_init_markers));
  painter.drawText(12, 322, "Detector reason: " + detector_reason);
  painter.drawText(12, 344, "ESKF ArUco reason: " + aruco_reason);
  painter.drawText(12, 364, "Last skip: " + skip_reason);
  painter.drawText(
    12, 390,
    QString("Position innovation: %1 / %2 m")
    .arg(data_.aruco_position_innovation_m, 0, 'f', 3)
    .arg(data_.aruco_position_gate_m, 0, 'f', 3));
  painter.drawText(
    12, 410,
    QString("ArUco covariance scale: %1").arg(data_.aruco_position_covariance_scale, 0, 'f', 2));
  painter.drawText(
    12, 430,
    QString("ArUco rotation residual: %1 deg (debug only)")
    .arg(data_.aruco_rotation_innovation_deg, 0, 'f', 1));

  painter.end();
}

}  // namespace aruco_imu_eskf_localization_cpp

PLUGINLIB_EXPORT_CLASS(
  aruco_imu_eskf_localization_cpp::ArucoImuEskfDebugDisplay,
  rviz_common::Display)
