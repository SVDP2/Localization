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

#ifndef ESKF_LOCALIZATION__RVIZ_OVERLAY__IMU_DATA_HELPER_HPP_
#define ESKF_LOCALIZATION__RVIZ_OVERLAY__IMU_DATA_HELPER_HPP_

#include <QColor>
#include <QPainter>
#include <QPointF>
#include <QString>

namespace eskf_localization
{
namespace rviz_overlay
{

/**
 * Helper class for rendering IMU sensor data.
 * Displays IMU heading compass, angular velocity, and G-G diagram.
 */
class ImuDataHelper
{
public:
  ImuDataHelper();

  /**
   * Update IMU data from preprocessed IMU message.
   */
  void updateImuData(
    double qx, double qy, double qz, double qw, double gyro_z,
    double accel_x, double accel_y);

  /**
   * Set ESKF initialization state and apply yaw offset correction.
   * @param initialized Whether ESKF is initialized
   * @param eskf_yaw Current ESKF estimated yaw (only used at first init)
   */
  void setEskfInitialized(bool initialized, double eskf_yaw);

  /**
   * Check if ESKF is initialized.
   */
  bool isEskfInitialized() const {return eskf_initialized_;}

  /**
   * Draw IMU sensor info including heading compass, gyro, and G-G diagram.
   */
  void drawImuInfo(QPainter & painter, const QRectF & rect, const QColor & color);

private:
  double yaw_rad_;        // Yaw from IMU orientation (radians)
  double yaw_offset_;     // Yaw offset = ESKF_yaw - IMU_yaw at init
  double gyro_z_;         // Angular velocity Z (rad/s)
  double accel_x_;        // Linear acceleration X (m/s²)
  double accel_y_;        // Linear acceleration Y (m/s²)
  bool eskf_initialized_; // Whether ESKF has been initialized
  bool offset_applied_;   // Whether offset has been applied (one-time)
  int init_frame_count_;  // Frame counter to wait before applying offset

  double getCorrectedYaw() const;
  double quaternionToYaw(double qx, double qy, double qz, double qw) const;

  void drawCompassHeading(
    QPainter & painter, const QPointF & center,
    double radius, double heading_rad,
    double value_for_display);

  void drawCompassInitializing(
    QPainter & painter, const QPointF & center,
    double radius);

  void drawGGDiagram(
    QPainter & painter, const QPointF & center, double size,
    double accel_x, double accel_y);
};

} // namespace rviz_overlay
} // namespace eskf_localization

#endif // ESKF_LOCALIZATION__RVIZ_OVERLAY__IMU_DATA_HELPER_HPP_
