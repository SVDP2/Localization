// Copyright 2024 ESKF Localization
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "eskf_localization/rviz_overlay/gnss_debug_display.hpp"

#include <QPainter>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_rendering/render_system.hpp>

#include <memory>
#include <sstream>
#include <string>

namespace eskf_localization
{

GnssDebugDisplay::GnssDebugDisplay()
{
  RCLCPP_INFO(rclcpp::get_logger("GnssDebugDisplay"), "Constructor called");

  property_width_ = new rviz_common::properties::IntProperty(
    "Width", 450, "Width of the overlay", this, SLOT(updateOverlaySize()));
  property_height_ = new rviz_common::properties::IntProperty(
    "Height", 680, "Height of the overlay", this, SLOT(updateOverlaySize()));
  property_left_ = new rviz_common::properties::IntProperty(
    "Left", 10, "Left position of the overlay", this,
    SLOT(updateOverlayPosition()));
  property_top_ = new rviz_common::properties::IntProperty(
    "Top", 10, "Top position of the overlay", this,
    SLOT(updateOverlayPosition()));

  // Initialize helpers
  gnss_helper_ = std::make_unique<rviz_overlay::GnssDataHelper>();
  vehicle_helper_ = std::make_unique<rviz_overlay::VehicleDataHelper>();
  imu_helper_ = std::make_unique<rviz_overlay::ImuDataHelper>();
  status_helper_ = std::make_unique<rviz_overlay::StatusChartHelper>();
  localization_helper_ =
    std::make_unique<rviz_overlay::LocalizationDataHelper>();
  update_status_helper_ = std::make_unique<rviz_overlay::UpdateStatusHelper>();

  RCLCPP_INFO(rclcpp::get_logger("GnssDebugDisplay"), "Constructor finished");
}

GnssDebugDisplay::~GnssDebugDisplay()
{
  RCLCPP_INFO(rclcpp::get_logger("GnssDebugDisplay"), "Destructor called");
  std::lock_guard<std::mutex> lock(mutex_);
  overlay_.reset();
  gnss_vel_sub_.reset();
  navsatfix_sub_.reset();
  heading_sub_.reset();
  velocity_sub_.reset();
  steering_sub_.reset();
  kinematic_state_sub_.reset();
  imu_sub_.reset();
  gnss_helper_.reset();
  vehicle_helper_.reset();
  imu_helper_.reset();
  status_helper_.reset();
  localization_helper_.reset();
  update_status_helper_.reset();
}

void GnssDebugDisplay::onInitialize()
{
  RCLCPP_INFO(rclcpp::get_logger("GnssDebugDisplay"), "onInitialize START");

  rviz_common::Display::onInitialize();

  RCLCPP_INFO(rclcpp::get_logger("GnssDebugDisplay"), "Base class initialized");

  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);

  RCLCPP_INFO(rclcpp::get_logger("GnssDebugDisplay"), "Overlays prepared");

  static int count = 0;
  std::stringstream ss;
  ss << "GnssDebugDisplayObject" << count++;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    overlay_ = std::make_shared<rviz_overlay::OverlayObject>(ss.str());
  }

  RCLCPP_INFO(rclcpp::get_logger("GnssDebugDisplay"), "Overlay object created");

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (overlay_) {
      overlay_->show();
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("GnssDebugDisplay"),
    "Overlay shown, calling updateOverlaySize");

  updateOverlaySize();

  RCLCPP_INFO(
    rclcpp::get_logger("GnssDebugDisplay"),
    "updateOverlaySize done, calling updateOverlayPosition");

  updateOverlayPosition();

  RCLCPP_INFO(
    rclcpp::get_logger("GnssDebugDisplay"),
    "Overlay shown and positioned");

  // Initialize topic properties
  RCLCPP_INFO(
    rclcpp::get_logger("GnssDebugDisplay"),
    "Initializing topic properties");

  auto rviz_ros_node = context_->getRosNodeAbstraction();

  gnss_vel_topic_property_ =
    std::make_unique<rviz_common::properties::RosTopicProperty>(
    "GNSS Vel Topic", "/sensing/gnss/vel",
    "geometry_msgs/msg/TwistStamped", "Topic for GNSS velocity (ENU) Data",
    this, SLOT(topic_updated_gnss_vel()));
  gnss_vel_topic_property_->initialize(rviz_ros_node);

  navsatfix_topic_property_ =
    std::make_unique<rviz_common::properties::RosTopicProperty>(
    "NavSatFix Topic", "/sensing/gnss/navsatfix",
    "sensor_msgs/msg/NavSatFix", "Topic for GNSS NavSatFix Data", this,
    SLOT(topic_updated_navsatfix()));
  navsatfix_topic_property_->initialize(rviz_ros_node);

  heading_topic_property_ =
    std::make_unique<rviz_common::properties::RosTopicProperty>(
    "Heading Topic", "/sensing/gnss/heading", "std_msgs/msg/Float64",
    "Topic for GNSS Heading Data", this, SLOT(topic_updated_heading()));
  heading_topic_property_->initialize(rviz_ros_node);

  velocity_topic_property_ =
    std::make_unique<rviz_common::properties::RosTopicProperty>(
    "Velocity Topic", "/vehicle/status/velocity_status",
    "autoware_vehicle_msgs/msg/VelocityReport",
    "Topic for Vehicle Velocity Data", this,
    SLOT(topic_updated_velocity()));
  velocity_topic_property_->initialize(rviz_ros_node);

  steering_topic_property_ =
    std::make_unique<rviz_common::properties::RosTopicProperty>(
    "Steering Topic", "/vehicle/status/steering_status",
    "autoware_vehicle_msgs/msg/SteeringReport",
    "Topic for Vehicle Steering Data", this,
    SLOT(topic_updated_steering()));
  steering_topic_property_->initialize(rviz_ros_node);

  kinematic_state_topic_property_ =
    std::make_unique<rviz_common::properties::RosTopicProperty>(
    "Kinematic State Topic", "/localization/kinematic_state",
    "nav_msgs/msg/Odometry", "Topic for ESKF Localization State", this,
    SLOT(topic_updated_kinematic_state()));
  kinematic_state_topic_property_->initialize(rviz_ros_node);

  diagnostics_topic_property_ =
    std::make_unique<rviz_common::properties::RosTopicProperty>(
    "Diagnostics Topic", "/diagnostics",
    "diagnostic_msgs/msg/DiagnosticArray", "Topic for ESKF Diagnostics",
    this, SLOT(topic_updated_diagnostics()));
  diagnostics_topic_property_->initialize(rviz_ros_node);

  imu_topic_property_ =
    std::make_unique<rviz_common::properties::RosTopicProperty>(
    "IMU Topic", "/sensing/imu/imu_data", "sensor_msgs/msg/Imu",
    "Topic for Preprocessed IMU Data", this, SLOT(topic_updated_imu()));
  imu_topic_property_->initialize(rviz_ros_node);

  RCLCPP_INFO(
    rclcpp::get_logger("GnssDebugDisplay"),
    "Topic properties initialized");
  RCLCPP_INFO(rclcpp::get_logger("GnssDebugDisplay"), "onInitialize FINISHED");
}

void GnssDebugDisplay::setupRosSubscriptions()
{
  RCLCPP_INFO(
    rclcpp::get_logger("GnssDebugDisplay"),
    "Setting up ROS subscriptions");
  topic_updated_gnss_vel();
  topic_updated_navsatfix();
  topic_updated_heading();
  topic_updated_velocity();
  topic_updated_steering();
  topic_updated_kinematic_state();
  topic_updated_diagnostics();
  topic_updated_imu();
  RCLCPP_INFO(
    rclcpp::get_logger("GnssDebugDisplay"),
    "ROS subscriptions setup complete");
}

void GnssDebugDisplay::onEnable()
{
  RCLCPP_INFO(rclcpp::get_logger("GnssDebugDisplay"), "onEnable called");

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (overlay_) {
      overlay_->show();
    }
  }

  // Call setupRosSubscriptions OUTSIDE of mutex lock to avoid deadlock
  setupRosSubscriptions();

  RCLCPP_INFO(rclcpp::get_logger("GnssDebugDisplay"), "onEnable finished");
}

void GnssDebugDisplay::onDisable()
{
  RCLCPP_INFO(rclcpp::get_logger("GnssDebugDisplay"), "onDisable called");
  std::lock_guard<std::mutex> lock(mutex_);
  if (overlay_) {
    overlay_->hide();
  }
}

void GnssDebugDisplay::reset()
{
  RCLCPP_INFO(rclcpp::get_logger("GnssDebugDisplay"), "reset called");
  rviz_common::Display::reset();
}

void GnssDebugDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  std::lock_guard<std::mutex> lock(mutex_);

  if (!overlay_) {
    return;
  }

  if (!overlay_->isVisible()) {
    return;
  }

  if (overlay_->getTextureWidth() == 0 || overlay_->getTextureHeight() == 0) {
    return;
  }

  rviz_overlay::ScopedPixelBuffer buffer = overlay_->getBuffer();
  QImage hud = buffer.getQImage(*overlay_);
  hud.fill(QColor(0, 0, 0, 35)); // 86% transparent dark background

  drawWidget(hud);
}

void GnssDebugDisplay::updateOverlaySize()
{
  RCLCPP_INFO(
    rclcpp::get_logger("GnssDebugDisplay"),
    "updateOverlaySize called");
  std::lock_guard<std::mutex> lock(mutex_);
  if (!overlay_) {
    return;
  }
  overlay_->updateTextureSize(
    property_width_->getInt(),
    property_height_->getInt());
  overlay_->setDimensions(
    property_width_->getInt(),
    property_height_->getInt());
}

void GnssDebugDisplay::updateOverlayPosition()
{
  RCLCPP_INFO(
    rclcpp::get_logger("GnssDebugDisplay"),
    "updateOverlayPosition called");
  std::lock_guard<std::mutex> lock(mutex_);
  if (!overlay_) {
    return;
  }
  overlay_->setPosition(
    property_left_->getInt(), property_top_->getInt(),
    rviz_overlay::HorizontalAlignment::LEFT,
    rviz_overlay::VerticalAlignment::TOP);
}

void GnssDebugDisplay::topic_updated_gnss_vel()
{
  std::lock_guard<std::mutex> lock(mutex_);
  gnss_vel_sub_.reset();

  std::string topic_name = gnss_vel_topic_property_->getStdString();
  if (topic_name.empty()) {
    return;
  }

  try {
    auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
    if (!rviz_ros_node) {
      RCLCPP_WARN(
        rclcpp::get_logger("GnssDebugDisplay"),
        "Failed to lock ROS node for GNSS velocity");
      return;
    }
    auto node = rviz_ros_node->get_raw_node();
    if (!node) {
      RCLCPP_WARN(
        rclcpp::get_logger("GnssDebugDisplay"),
        "Failed to get raw node for GNSS velocity");
      return;
    }
    gnss_vel_sub_ =
      node->create_subscription<geometry_msgs::msg::TwistStamped>(
      topic_name, rclcpp::QoS(10),
      [this](
        const geometry_msgs::msg::TwistStamped::ConstSharedPtr
        msg) {gnss_vel_callback(msg);});
    RCLCPP_INFO(
      rclcpp::get_logger("GnssDebugDisplay"),
      "Subscribed to GNSS velocity topic: %s", topic_name.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GnssDebugDisplay"),
      "Exception in topic_updated_gnss_vel: %s", e.what());
  }
}

void GnssDebugDisplay::topic_updated_navsatfix()
{
  std::lock_guard<std::mutex> lock(mutex_);
  navsatfix_sub_.reset();

  std::string topic_name = navsatfix_topic_property_->getStdString();
  if (topic_name.empty()) {
    return;
  }

  try {
    auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
    if (!rviz_ros_node) {
      return;
    }
    auto node = rviz_ros_node->get_raw_node();
    if (!node) {
      return;
    }
    navsatfix_sub_ = node->create_subscription<sensor_msgs::msg::NavSatFix>(
      topic_name, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
        navsatfix_callback(msg);
      });
    RCLCPP_INFO(
      rclcpp::get_logger("GnssDebugDisplay"),
      "Subscribed to NavSatFix topic: %s", topic_name.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GnssDebugDisplay"),
      "Exception in topic_updated_navsatfix: %s", e.what());
  }
}

void GnssDebugDisplay::topic_updated_heading()
{
  std::lock_guard<std::mutex> lock(mutex_);
  heading_sub_.reset();

  std::string topic_name = heading_topic_property_->getStdString();
  if (topic_name.empty()) {
    return;
  }

  try {
    auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
    if (!rviz_ros_node) {
      return;
    }
    auto node = rviz_ros_node->get_raw_node();
    if (!node) {
      return;
    }
    heading_sub_ = node->create_subscription<std_msgs::msg::Float64>(
      topic_name, rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64::ConstSharedPtr msg) {
        heading_callback(msg);
      });
    RCLCPP_INFO(
      rclcpp::get_logger("GnssDebugDisplay"),
      "Subscribed to Heading topic: %s", topic_name.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GnssDebugDisplay"),
      "Exception in topic_updated_heading: %s", e.what());
  }
}

void GnssDebugDisplay::topic_updated_velocity()
{
  std::lock_guard<std::mutex> lock(mutex_);
  velocity_sub_.reset();

  std::string topic_name = velocity_topic_property_->getStdString();
  if (topic_name.empty()) {
    return;
  }

  try {
    auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
    if (!rviz_ros_node) {
      return;
    }
    auto node = rviz_ros_node->get_raw_node();
    if (!node) {
      return;
    }
    velocity_sub_ =
      node->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
      topic_name, rclcpp::QoS(10),
      [this](
        const autoware_vehicle_msgs::msg::VelocityReport::ConstSharedPtr
        msg) {velocity_callback(msg);});
    RCLCPP_INFO(
      rclcpp::get_logger("GnssDebugDisplay"),
      "Subscribed to Velocity topic: %s", topic_name.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GnssDebugDisplay"),
      "Exception in topic_updated_velocity: %s", e.what());
  }
}

void GnssDebugDisplay::topic_updated_steering()
{
  std::lock_guard<std::mutex> lock(mutex_);
  steering_sub_.reset();

  std::string topic_name = steering_topic_property_->getStdString();
  if (topic_name.empty()) {
    return;
  }

  try {
    auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
    if (!rviz_ros_node) {
      return;
    }
    auto node = rviz_ros_node->get_raw_node();
    if (!node) {
      return;
    }
    steering_sub_ =
      node->create_subscription<autoware_vehicle_msgs::msg::SteeringReport>(
      topic_name, rclcpp::QoS(10),
      [this](
        const autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr
        msg) {steering_callback(msg);});
    RCLCPP_INFO(
      rclcpp::get_logger("GnssDebugDisplay"),
      "Subscribed to Steering topic: %s", topic_name.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GnssDebugDisplay"),
      "Exception in topic_updated_steering: %s", e.what());
  }
}

void GnssDebugDisplay::gnss_vel_callback(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (gnss_helper_) {
    const float ve = msg ? msg->twist.linear.x : 0.0f;
    const float vn = msg ? msg->twist.linear.y : 0.0f;
    gnss_helper_->updateVelocityData(vn, ve);
  }
}

void GnssDebugDisplay::navsatfix_callback(
  const sensor_msgs::msg::NavSatFix::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (status_helper_) {
    status_helper_->updateStatusData(msg->status.status);
  }
}

void GnssDebugDisplay::heading_callback(
  const std_msgs::msg::Float64::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (gnss_helper_) {
    gnss_helper_->updateHeadingData(msg->data);
  }
}

void GnssDebugDisplay::velocity_callback(
  const autoware_vehicle_msgs::msg::VelocityReport::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (vehicle_helper_) {
    vehicle_helper_->updateVelocityData(msg->longitudinal_velocity);
  }
}

void GnssDebugDisplay::steering_callback(
  const autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (vehicle_helper_) {
    vehicle_helper_->updateSteeringData(msg->steering_tire_angle);
  }
}

void GnssDebugDisplay::topic_updated_kinematic_state()
{
  std::lock_guard<std::mutex> lock(mutex_);
  kinematic_state_sub_.reset();

  std::string topic_name = kinematic_state_topic_property_->getStdString();
  if (topic_name.empty()) {
    return;
  }

  try {
    auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
    if (!rviz_ros_node) {
      return;
    }
    auto node = rviz_ros_node->get_raw_node();
    if (!node) {
      return;
    }
    kinematic_state_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      topic_name, rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        kinematic_state_callback(msg);
      });
    RCLCPP_INFO(
      rclcpp::get_logger("GnssDebugDisplay"),
      "Subscribed to Kinematic State topic: %s", topic_name.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GnssDebugDisplay"),
      "Exception in topic_updated_kinematic_state: %s", e.what());
  }
}

void GnssDebugDisplay::kinematic_state_callback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (localization_helper_) {
    const auto & q = msg->pose.pose.orientation;
    localization_helper_->updateOrientationData(q.x, q.y, q.z, q.w);
  }
}

void GnssDebugDisplay::topic_updated_diagnostics()
{
  std::lock_guard<std::mutex> lock(mutex_);
  diagnostics_sub_.reset();

  std::string topic_name = diagnostics_topic_property_->getStdString();
  if (topic_name.empty()) {
    return;
  }

  try {
    auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
    if (!rviz_ros_node) {
      return;
    }
    auto node = rviz_ros_node->get_raw_node();
    if (!node) {
      return;
    }
    diagnostics_sub_ =
      node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      topic_name, rclcpp::QoS(10),
      [this](const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr
      msg) {diagnostics_callback(msg);});
    RCLCPP_INFO(
      rclcpp::get_logger("GnssDebugDisplay"),
      "Subscribed to Diagnostics topic: %s", topic_name.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GnssDebugDisplay"),
      "Exception in topic_updated_diagnostics: %s", e.what());
  }
}

void GnssDebugDisplay::diagnostics_callback(
  const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Find eskf_localization status
  for (const auto & status : msg->status) {
    if (status.name != "eskf_localization") {
      continue;
    }

    // Parse key-value pairs
    bool gnss_pos_applied = false;
    std::string gnss_pos_reason;
    bool gnss_vel_applied = false;
    std::string gnss_vel_reason;
    bool heading_applied = false;
    std::string heading_reason;
    bool eskf_initialized = false;
    int8_t gnss_status = -1;
    double pos_R_xx = 0.0;
    double pos_R_yy = 0.0;
    double pos_R_zz = 0.0;
    double vel_R_xx = 0.0;
    double vel_R_yy = 0.0;
    double vel_R_zz = 0.0;
    double heading_R = 0.0;

    bool kiss_enabled = false;
    bool kiss_initialized = false;
    bool kiss_yaw_enabled = false;
    bool kiss_vy_enabled = false;
    bool kiss_yaw_applied = false;
    std::string kiss_yaw_reason;
    bool kiss_vy_applied = false;
    std::string kiss_vy_reason;
    std::string kiss_skip_reason;
    double kiss_time_alignment_error_ms = 0.0;
    int kiss_source_points = 0;
    double kiss_dt_ms = 0.0;
    double kiss_trust = 0.0;
    double kiss_target_trust = 0.0;
    double kiss_yaw_rate_radps = 0.0;
    double kiss_vy_mps = 0.0;
    bool kiss_reset_candidate = false;
    size_t kiss_reset_count = 0;

    for (const auto & kv : status.values) {
      try {
        if (kv.key == "eskf_initialized") {
          eskf_initialized = (kv.value == "true");
        } else if (kv.key == "gnss_pos_update_applied") {
          gnss_pos_applied = (kv.value == "true");
        } else if (kv.key == "gnss_pos_update_reason") {
          gnss_pos_reason = kv.value;
        } else if (kv.key == "gnss_vel_update_applied") {
          gnss_vel_applied = (kv.value == "true");
        } else if (kv.key == "gnss_vel_update_reason") {
          gnss_vel_reason = kv.value;
        } else if (kv.key == "heading_yaw_update_applied") {
          heading_applied = (kv.value == "true");
        } else if (kv.key == "heading_yaw_update_reason") {
          heading_reason = kv.value;
        } else if (kv.key == "gnss_status") {
          gnss_status = static_cast<int8_t>(std::stoi(kv.value));
        }
        else if (kv.key == "gnss_pos_R_xx") {
          pos_R_xx = std::stod(kv.value);
        } else if (kv.key == "gnss_pos_R_yy") {
          pos_R_yy = std::stod(kv.value);
        } else if (kv.key == "gnss_pos_R_zz") {
          pos_R_zz = std::stod(kv.value);
        } else if (kv.key == "gnss_vel_R_xx") {
          vel_R_xx = std::stod(kv.value);
        } else if (kv.key == "gnss_vel_R_yy") {
          vel_R_yy = std::stod(kv.value);
        } else if (kv.key == "gnss_vel_R_zz") {
          vel_R_zz = std::stod(kv.value);
        } else if (kv.key == "heading_yaw_var") {
          heading_R = std::stod(kv.value);
        } else if (kv.key == "kiss_enabled") {
          kiss_enabled = (kv.value == "true");
        } else if (kv.key == "kiss_initialized") {
          kiss_initialized = (kv.value == "true");
        } else if (kv.key == "kiss_yaw_enabled") {
          kiss_yaw_enabled = (kv.value == "true");
        } else if (kv.key == "kiss_vy_enabled") {
          kiss_vy_enabled = (kv.value == "true");
        } else if (kv.key == "kiss_yaw_applied") {
          kiss_yaw_applied = (kv.value == "true");
        } else if (kv.key == "kiss_yaw_reason") {
          kiss_yaw_reason = kv.value;
        } else if (kv.key == "kiss_vy_applied") {
          kiss_vy_applied = (kv.value == "true");
        } else if (kv.key == "kiss_vy_reason") {
          kiss_vy_reason = kv.value;
        } else if (kv.key == "kiss_skip_reason") {
          kiss_skip_reason = kv.value;
        } else if (kv.key == "kiss_time_alignment_error_ms") {
          kiss_time_alignment_error_ms = std::stod(kv.value);
        } else if (kv.key == "kiss_source_points") {
          kiss_source_points = std::stoi(kv.value);
        } else if (kv.key == "kiss_dt_ms") {
          kiss_dt_ms = std::stod(kv.value);
        } else if (kv.key == "kiss_trust") {
          kiss_trust = std::stod(kv.value);
        } else if (kv.key == "kiss_target_trust") {
          kiss_target_trust = std::stod(kv.value);
        } else if (kv.key == "kiss_yaw_rate_radps") {
          kiss_yaw_rate_radps = std::stod(kv.value);
        } else if (kv.key == "kiss_vy_mps") {
          kiss_vy_mps = std::stod(kv.value);
        } else if (kv.key == "kiss_reset_candidate") {
          kiss_reset_candidate = (kv.value == "true");
        } else if (kv.key == "kiss_reset_count") {
          kiss_reset_count = static_cast<size_t>(std::stoull(kv.value));
        }
      } catch (...) {
        // Ignore parse errors
      }
    }

    eskf_initialized_from_diag_ = eskf_initialized;

    // Update status helper
    if (update_status_helper_) {
      rviz_overlay::UpdateStatusData status_data;
      status_data.pos_applied = gnss_pos_applied;
      status_data.pos_reason = gnss_pos_reason;
      status_data.pos_R_xx = pos_R_xx;
      status_data.pos_R_yy = pos_R_yy;
      status_data.pos_R_zz = pos_R_zz;

      status_data.vel_applied = gnss_vel_applied;
      status_data.vel_reason = gnss_vel_reason;
      status_data.vel_R_xx = vel_R_xx;
      status_data.vel_R_yy = vel_R_yy;
      status_data.vel_R_zz = vel_R_zz;

      status_data.heading_applied = heading_applied;
      status_data.heading_reason = heading_reason;
      status_data.heading_R = heading_R;

      status_data.gnss_status = gnss_status;
      status_data.eskf_initialized = eskf_initialized;

      status_data.kiss_enabled = kiss_enabled;
      status_data.kiss_initialized = kiss_initialized;
      status_data.kiss_yaw_enabled = kiss_yaw_enabled;
      status_data.kiss_vy_enabled = kiss_vy_enabled;
      status_data.kiss_yaw_applied = kiss_yaw_applied;
      status_data.kiss_yaw_reason = kiss_yaw_reason;
      status_data.kiss_vy_applied = kiss_vy_applied;
      status_data.kiss_vy_reason = kiss_vy_reason;
      status_data.kiss_skip_reason = kiss_skip_reason;
      status_data.kiss_time_alignment_error_ms = kiss_time_alignment_error_ms;
      status_data.kiss_source_points = kiss_source_points;
      status_data.kiss_dt_ms = kiss_dt_ms;
      status_data.kiss_trust = kiss_trust;
      status_data.kiss_target_trust = kiss_target_trust;
      status_data.kiss_yaw_rate_radps = kiss_yaw_rate_radps;
      status_data.kiss_vy_mps = kiss_vy_mps;
      status_data.kiss_reset_candidate = kiss_reset_candidate;
      status_data.kiss_reset_count = kiss_reset_count;

      update_status_helper_->updateStatusData(status_data);
    }

    break;
  }
}

void GnssDebugDisplay::topic_updated_imu()
{
  std::lock_guard<std::mutex> lock(mutex_);
  imu_sub_.reset();

  std::string topic_name = imu_topic_property_->getStdString();
  if (topic_name.empty()) {
    return;
  }

  try {
    auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
    if (!rviz_ros_node) {
      return;
    }
    auto node = rviz_ros_node->get_raw_node();
    if (!node) {
      return;
    }
    imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
      topic_name, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        imu_callback(msg);
      });
    RCLCPP_INFO(
      rclcpp::get_logger("GnssDebugDisplay"),
      "Subscribed to IMU topic: %s", topic_name.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GnssDebugDisplay"),
      "Exception in topic_updated_imu: %s", e.what());
  }
}

void GnssDebugDisplay::imu_callback(
  const sensor_msgs::msg::Imu::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (imu_helper_) {
    const auto & q = msg->orientation;
    const auto & gyro = msg->angular_velocity;
    const auto & accel = msg->linear_acceleration;
    imu_helper_->updateImuData(q.x, q.y, q.z, q.w, gyro.z, accel.x, accel.y);
  }
}

void GnssDebugDisplay::drawWidget(QImage & hud)
{
  QPainter painter(&hud);
  painter.setRenderHint(QPainter::Antialiasing, true);

  QColor text_color(255, 255, 255);
  QRectF rect(0, 0, hud.width(), hud.height());

  // Draw title
  painter.setPen(text_color);
  QFont title_font = painter.font();
  title_font.setPointSize(14);
  title_font.setBold(true);
  painter.setFont(title_font);
  painter.drawText(10, 20, "GNSS DEBUG OVERLAY");

  // Draw status chart (Fix status)
  if (status_helper_) {
    status_helper_->drawStatusChart(painter, rect);
  }

  // Heading Comparison section header
  QFont section_font = painter.font();
  section_font.setPointSize(12);
  section_font.setBold(true);
  painter.setFont(section_font);
  painter.drawText(10, 70, "[Heading Comparison]");

  // Draw all heading compasses (GNSS Computed, GNSS Sensor, IMU, ESKF)
  if (gnss_helper_) {
    gnss_helper_->drawGnssInfo(painter, rect, text_color);
  }
  if (imu_helper_) {
    // Pass ESKF initialization state and yaw to IMU helper for offset
    // correction
    if (localization_helper_) {
      imu_helper_->setEskfInitialized(
        eskf_initialized_from_diag_,
        localization_helper_->getYaw());
    }
    imu_helper_->drawImuInfo(painter, rect, text_color);
  }
  if (localization_helper_) {
    localization_helper_->drawLocalizationInfo(painter, rect, text_color);
  }

  // Sensor Data section header
  painter.setFont(section_font);
  painter.drawText(10, 200, "[Sensor Data]");

  // Draw sensor data (GNSS speed, vehicle speed/steering, IMU gyro, G-G
  // diagram)
  if (vehicle_helper_) {
    vehicle_helper_->drawVehicleInfo(painter, rect, text_color);
  }

  // Draw update status grid
  if (update_status_helper_) {
    update_status_helper_->drawUpdateStatus(painter, rect, text_color);
  }

  painter.end();
}

} // namespace eskf_localization

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  eskf_localization::GnssDebugDisplay,
  rviz_common::Display)
