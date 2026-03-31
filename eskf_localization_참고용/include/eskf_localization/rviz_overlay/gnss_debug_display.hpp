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

#ifndef ESKF_LOCALIZATION__RVIZ_OVERLAY__GNSS_DEBUG_DISPLAY_HPP_
#define ESKF_LOCALIZATION__RVIZ_OVERLAY__GNSS_DEBUG_DISPLAY_HPP_

#ifndef Q_MOC_RUN
#include "eskf_localization/rviz_overlay/gnss_data_helper.hpp"
#include "eskf_localization/rviz_overlay/imu_data_helper.hpp"
#include "eskf_localization/rviz_overlay/localization_data_helper.hpp"
#include "eskf_localization/rviz_overlay/overlay_utils.hpp"
#include "eskf_localization/rviz_overlay/status_chart_helper.hpp"
#include "eskf_localization/rviz_overlay/update_status_helper.hpp"
#include "eskf_localization/rviz_overlay/vehicle_data_helper.hpp"

#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>

#include <QImage>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>

#include <memory>
#include <mutex>
#endif

namespace eskf_localization
{

class GnssDebugDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  GnssDebugDisplay();
  ~GnssDebugDisplay() override;

protected:
  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;
  void onEnable() override;
  void onDisable() override;

private Q_SLOTS:
  void updateOverlaySize();
  void updateOverlayPosition();
  void topic_updated_gnss_vel();
  void topic_updated_navsatfix();
  void topic_updated_heading();
  void topic_updated_velocity();
  void topic_updated_steering();
  void topic_updated_kinematic_state();
  void topic_updated_diagnostics();
  void topic_updated_imu();

 private:
  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    gnss_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr
    velocity_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr
    steering_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kinematic_state_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Topic properties
  std::unique_ptr<rviz_common::properties::RosTopicProperty>
  gnss_vel_topic_property_;
  std::unique_ptr<rviz_common::properties::RosTopicProperty>
  navsatfix_topic_property_;
  std::unique_ptr<rviz_common::properties::RosTopicProperty>
  heading_topic_property_;
  std::unique_ptr<rviz_common::properties::RosTopicProperty>
  velocity_topic_property_;
  std::unique_ptr<rviz_common::properties::RosTopicProperty>
  steering_topic_property_;
  std::unique_ptr<rviz_common::properties::RosTopicProperty>
  kinematic_state_topic_property_;
  std::unique_ptr<rviz_common::properties::RosTopicProperty>
  diagnostics_topic_property_;
  std::unique_ptr<rviz_common::properties::RosTopicProperty>
  imu_topic_property_;

  // Display helpers
  std::unique_ptr<rviz_overlay::GnssDataHelper> gnss_helper_;
  std::unique_ptr<rviz_overlay::VehicleDataHelper> vehicle_helper_;
  std::unique_ptr<rviz_overlay::ImuDataHelper> imu_helper_;
  std::unique_ptr<rviz_overlay::StatusChartHelper> status_helper_;
  std::unique_ptr<rviz_overlay::LocalizationDataHelper> localization_helper_;
  std::unique_ptr<rviz_overlay::UpdateStatusHelper> update_status_helper_;

  bool eskf_initialized_from_diag_{false};

  // Overlay
  std::shared_ptr<rviz_overlay::OverlayObject> overlay_;
  rviz_common::properties::IntProperty * property_width_;
  rviz_common::properties::IntProperty * property_height_;
  rviz_common::properties::IntProperty * property_left_;
  rviz_common::properties::IntProperty * property_top_;

  std::mutex mutex_;

  // Callbacks
  void gnss_vel_callback(
    const geometry_msgs::msg::TwistStamped::ConstSharedPtr & msg);
  void
  navsatfix_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr & msg);
  void heading_callback(const std_msgs::msg::Float64::ConstSharedPtr & msg);
  void velocity_callback(
    const autoware_vehicle_msgs::msg::VelocityReport::ConstSharedPtr & msg);
  void steering_callback(
    const autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr & msg);
  void
  kinematic_state_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void diagnostics_callback(
    const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & msg);
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr & msg);

  void setupRosSubscriptions();
  void drawWidget(QImage & hud);
};

} // namespace eskf_localization

#endif // ESKF_LOCALIZATION__RVIZ_OVERLAY__GNSS_DEBUG_DISPLAY_HPP_
