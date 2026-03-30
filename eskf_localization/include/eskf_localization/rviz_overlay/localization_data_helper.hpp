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

#ifndef ESKF_LOCALIZATION__RVIZ_OVERLAY__LOCALIZATION_DATA_HELPER_HPP_
#define ESKF_LOCALIZATION__RVIZ_OVERLAY__LOCALIZATION_DATA_HELPER_HPP_

#include <QColor>
#include <QPainter>
#include <QPointF>
#include <QString>

namespace eskf_localization
{
namespace rviz_overlay
{

/**
 * Helper class for rendering localization orientation data from Odometry.
 * Displays compass visualization for the estimated heading from ESKF.
 */
class LocalizationDataHelper
{
public:
  LocalizationDataHelper();

  /**
   * Update orientation from Odometry quaternion.
   * @param qx Quaternion x component
   * @param qy Quaternion y component
   * @param qz Quaternion z component
   * @param qw Quaternion w component
   */
  void updateOrientationData(double qx, double qy, double qz, double qw);

  /**
   * Draw localization info including heading compass.
   * @param painter QPainter to draw with
   * @param rect Available drawing area
   * @param color Text color
   */
  void drawLocalizationInfo(
    QPainter & painter, const QRectF & rect,
    const QColor & color);

  /**
   * Draw compass with heading from orientation quaternion.
   * Uses North=0, CW+ convention (ENU frame yaw).
   */
  void drawCompassHeading(
    QPainter & painter, const QPointF & center,
    double radius, double heading_rad,
    double value_for_display);

private:
  double yaw_rad_; // Yaw extracted from quaternion (radians)

public:
  /**
   * Get the current yaw value.
   */
  double getYaw() const {return yaw_rad_;}

private:
  /**
   * Convert quaternion to yaw (rotation around Z-axis in ENU).
   */
  double quaternionToYaw(double qx, double qy, double qz, double qw) const;
};

} // namespace rviz_overlay
} // namespace eskf_localization

#endif // ESKF_LOCALIZATION__RVIZ_OVERLAY__LOCALIZATION_DATA_HELPER_HPP_
