#ifndef ARUCO_IMU_ESKF_LOCALIZATION_CPP__RVIZ_OVERLAY__ARUCO_IMU_ESKF_DEBUG_DISPLAY_HPP_
#define ARUCO_IMU_ESKF_LOCALIZATION_CPP__RVIZ_OVERLAY__ARUCO_IMU_ESKF_DEBUG_DISPLAY_HPP_

#ifndef Q_MOC_RUN
#include "aruco_imu_eskf_localization_cpp/rviz_overlay/overlay_utils.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <QImage>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>

#include <memory>
#include <mutex>
#include <string>
#endif

namespace aruco_imu_eskf_localization_cpp
{

struct OverlayData
{
  bool eskf_initialized{false};
  bool aruco_applied{false};
  std::string aruco_reason{"not_received"};
  std::string last_skip_reason{"none"};
  double filter_yaw_rad{0.0};
  double imu_yaw_reference_only_rad{0.0};
  bool imu_orientation_valid{false};
  double raw_gyro_z_radps{0.0};
  double corrected_gyro_z_radps{0.0};
  double gyro_z_bias_radps{0.0};
  bool gyro_bias_valid{false};
  bool stationary_detected{false};
  std::string calibration_status{"disabled"};
  double aruco_position_innovation_m{0.0};
  double aruco_position_gate_m{0.0};
  double aruco_position_covariance_scale{1.0};
  double aruco_rotation_innovation_deg{0.0};
  bool detector_pose_published{false};
  int detector_detected_markers{0};
  int detector_known_markers{0};
  int detector_min_markers{0};
  int detector_min_init_markers{0};
  std::string detector_rejection_reason{"not_received"};
  double odom_x{0.0};
  double odom_y{0.0};
  double odom_z{0.0};
};

class ArucoImuEskfDebugDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  ArucoImuEskfDebugDisplay();
  ~ArucoImuEskfDebugDisplay() override;

protected:
  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;
  void onEnable() override;
  void onDisable() override;

private Q_SLOTS:
  void updateOverlaySize();
  void updateOverlayPosition();
  void topic_updated_odom();
  void topic_updated_imu();
  void topic_updated_diagnostics();

private:
  void setupRosSubscriptions();
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg);
  void drawWidget(QImage & hud);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;

  std::unique_ptr<rviz_common::properties::RosTopicProperty> odom_topic_property_;
  std::unique_ptr<rviz_common::properties::RosTopicProperty> imu_topic_property_;
  std::unique_ptr<rviz_common::properties::RosTopicProperty> diagnostics_topic_property_;
  rviz_common::properties::IntProperty * property_width_{nullptr};
  rviz_common::properties::IntProperty * property_height_{nullptr};
  rviz_common::properties::IntProperty * property_left_{nullptr};
  rviz_common::properties::IntProperty * property_top_{nullptr};

  std::shared_ptr<rviz_overlay::OverlayObject> overlay_;
  OverlayData data_;
  std::mutex mutex_;
};

}  // namespace aruco_imu_eskf_localization_cpp

#endif  // ARUCO_IMU_ESKF_LOCALIZATION_CPP__RVIZ_OVERLAY__ARUCO_IMU_ESKF_DEBUG_DISPLAY_HPP_
