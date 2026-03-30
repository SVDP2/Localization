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

#include "eskf_localization/rviz_overlay/localization_data_helper.hpp"

#include <QFont>
#include <QPolygonF>

#include <cmath>

namespace eskf_localization
{
namespace rviz_overlay
{

LocalizationDataHelper::LocalizationDataHelper()
: yaw_rad_(0.0) {}

void LocalizationDataHelper::updateOrientationData(
  double qx, double qy,
  double qz, double qw)
{
  yaw_rad_ = quaternionToYaw(qx, qy, qz, qw);
}

double LocalizationDataHelper::quaternionToYaw(
  double qx, double qy, double qz,
  double qw) const
{
  // Standard quaternion to Euler yaw conversion (ENU frame, Z-up)
  // yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
  double siny_cosp = 2.0 * (qw * qz + qx * qy);
  double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  return std::atan2(siny_cosp, cosy_cosp);
}

void LocalizationDataHelper::drawLocalizationInfo(
  QPainter & painter,
  const QRectF & rect,
  const QColor & color)
{
  painter.setPen(color);
  QFont font = painter.font();
  int y_offset = static_cast<int>(rect.top());

  // ESKF heading compass (moved to heading comparison)
  font.setPointSize(9);
  painter.setFont(font);
  painter.drawText(static_cast<int>(rect.left()) + 370, y_offset + 68, "ESKF");

  // Draw compass for ESKF heading
  QPointF compass_center(rect.left() + 390, y_offset + 120);
  drawCompassHeading(painter, compass_center, 30.0, yaw_rad_, yaw_rad_);
}

void LocalizationDataHelper::drawCompassHeading(
  QPainter & painter,
  const QPointF & center,
  double radius,
  double heading_rad,
  double value_for_display)
{
  // Draw compass circle background
  painter.setPen(QPen(QColor(100, 200, 100), 2)); // Green border for ESKF
  painter.setBrush(QColor(30, 60, 30, 64));      // Darker green background
  painter.drawEllipse(center, radius, radius);

  // Draw East (E) marker at right (3 o'clock position) - ENU convention
  painter.setPen(QColor(255, 255, 255));
  QFont font = painter.font();
  font.setPointSize(10);
  font.setBold(true);
  painter.setFont(font);
  painter.drawText(
    static_cast<int>(center.x() + radius + 5),
    static_cast<int>(center.y() + 5), "E");

  // Draw heading arrow
  // ENU Convention: heading_rad=0 points East (right), positive = CCW
  // Qt rotate is CW+, so we use -heading_rad for CCW+ display
  painter.save();
  painter.translate(center);
  painter.rotate(-heading_rad * 180.0 / M_PI); // Negative for CCW+

  // Draw arrow line (pointing right = East at 0 rad)
  painter.setPen(QPen(QColor(0, 255, 100), 2)); // Green arrow for ESKF
  painter.drawLine(QPointF(0, 0), QPointF(radius * 0.8, 0));

  // Draw arrow head (pointing right before rotation)
  QPolygonF arrowHead;
  arrowHead << QPointF(radius * 0.8, 0) << QPointF(radius * 0.65, -5)
            << QPointF(radius * 0.65, 5);
  painter.setBrush(QColor(0, 255, 100));
  painter.drawPolygon(arrowHead);

  painter.restore();

  // Draw angle value below compass (2 lines: rad primary, deg secondary)
  painter.setPen(QColor(255, 255, 255));
  font.setBold(false);
  font.setPointSize(8);
  painter.setFont(font);
  // Line 1: rad (primary unit)
  QString line1 = QString("%1 rad").arg(value_for_display, 0, 'f', 3);
  painter.drawText(
    static_cast<int>(center.x() - 30),
    static_cast<int>(center.y() + radius + 15), line1);
  // Line 2: deg (converted)
  double deg = value_for_display * 180.0 / M_PI;
  QString line2 = QString("(%1 deg)").arg(deg, 0, 'f', 1);
  painter.drawText(
    static_cast<int>(center.x() - 30),
    static_cast<int>(center.y() + radius + 27), line2);
}

} // namespace rviz_overlay
} // namespace eskf_localization
