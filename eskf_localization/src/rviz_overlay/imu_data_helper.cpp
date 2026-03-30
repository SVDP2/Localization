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

#include "eskf_localization/rviz_overlay/imu_data_helper.hpp"

#include <QFont>
#include <QPolygonF>

#include <cmath>

namespace eskf_localization
{
namespace rviz_overlay
{

ImuDataHelper::ImuDataHelper()
: yaw_rad_(0.0), yaw_offset_(0.0), gyro_z_(0.0), accel_x_(0.0),
  accel_y_(0.0), eskf_initialized_(false), offset_applied_(false),
  init_frame_count_(0) {}

void ImuDataHelper::updateImuData(
  double qx, double qy, double qz, double qw,
  double gyro_z, double accel_x,
  double accel_y)
{
  yaw_rad_ = quaternionToYaw(qx, qy, qz, qw);
  gyro_z_ = gyro_z;
  accel_x_ = accel_x;
  accel_y_ = accel_y;
}

void ImuDataHelper::setEskfInitialized(bool initialized, double eskf_yaw)
{
  if (initialized && !offset_applied_) {
    if (++init_frame_count_ > 30) {  // Wait ~1 sec (30 frames) before applying offset
      yaw_offset_ = eskf_yaw - yaw_rad_;
      offset_applied_ = true;
    }
  }
  eskf_initialized_ = initialized;
}

double ImuDataHelper::getCorrectedYaw() const
{
  if (eskf_initialized_ && offset_applied_) {
    return yaw_rad_ + yaw_offset_;
  }
  return yaw_rad_;
}

double ImuDataHelper::quaternionToYaw(
  double qx, double qy, double qz,
  double qw) const
{
  // Standard quaternion to Euler yaw conversion (ENU frame, Z-up)
  double siny_cosp = 2.0 * (qw * qz + qx * qy);
  double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  return std::atan2(siny_cosp, cosy_cosp);
}

void ImuDataHelper::drawImuInfo(
  QPainter & painter, const QRectF & rect,
  const QColor & color)
{
  painter.setPen(color);
  QFont font = painter.font();
  int y_offset = static_cast<int>(rect.top());

  // IMU Heading compass
  font.setPointSize(9);
  painter.setFont(font);
  painter.drawText(static_cast<int>(rect.left()) + 265, y_offset + 68, "IMU");
  QPointF compass_center(rect.left() + 280, y_offset + 120);

  if (eskf_initialized_) {
    // Show corrected yaw after ESKF initialization
    double corrected_yaw = getCorrectedYaw();
    drawCompassHeading(
      painter, compass_center, 30.0, corrected_yaw,
      corrected_yaw);
  } else {
    // Show grayed-out initializing state before ESKF
    drawCompassInitializing(painter, compass_center, 30.0);
  }

  // Angular velocity Z (moved to sensor data section)
  font.setPointSize(11);
  painter.setFont(font);
  painter.drawText(
    static_cast<int>(rect.left()) + 10, y_offset + 280,
    QString("IMU Gyro (Z): %1 rad/s").arg(gyro_z_, 0, 'f', 3));
}

void ImuDataHelper::drawCompassHeading(
  QPainter & painter, const QPointF & center,
  double radius, double heading_rad,
  double value_for_display)
{
  // Draw compass circle background (orange/gold color for IMU)
  painter.setPen(QPen(QColor(255, 165, 0), 2)); // Orange border
  painter.setBrush(QColor(80, 50, 0, 64));     // Dark orange background
  painter.drawEllipse(center, radius, radius);

  // Draw East (E) marker at right (3 o'clock position) - ENU convention
  painter.setPen(QColor(255, 255, 255));
  QFont font = painter.font();
  font.setPointSize(9);
  font.setBold(true);
  painter.setFont(font);
  painter.drawText(
    static_cast<int>(center.x() + radius + 3),
    static_cast<int>(center.y() + 4), "E");

  // Draw heading arrow
  // ENU Convention: heading_rad=0 points East (right), positive = CCW
  // Qt rotate is CW+, so we use -heading_rad for CCW+ display
  painter.save();
  painter.translate(center);
  painter.rotate(-heading_rad * 180.0 / M_PI); // Negative for CCW+

  // Draw arrow line (pointing right = East at 0 rad)
  painter.setPen(QPen(QColor(255, 165, 0), 2)); // Orange arrow
  painter.drawLine(QPointF(0, 0), QPointF(radius * 0.75, 0));

  // Draw arrow head (pointing right before rotation)
  QPolygonF arrowHead;
  arrowHead << QPointF(radius * 0.75, 0) << QPointF(radius * 0.6, -4)
            << QPointF(radius * 0.6, 4);
  painter.setBrush(QColor(255, 165, 0));
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

void ImuDataHelper::drawCompassInitializing(
  QPainter & painter,
  const QPointF & center,
  double radius)
{
  // Draw grayed-out compass circle for "Initializing" state
  painter.setPen(QPen(QColor(128, 128, 128), 2)); // Gray border
  painter.setBrush(QColor(60, 60, 60, 64));      // Dark gray background
  painter.drawEllipse(center, radius, radius);

  // Draw East (E) marker in gray
  painter.setPen(QColor(150, 150, 150));
  QFont font = painter.font();
  font.setPointSize(9);
  font.setBold(true);
  painter.setFont(font);
  painter.drawText(
    static_cast<int>(center.x() + radius + 3),
    static_cast<int>(center.y() + 4), "E");

  // Draw "..." loading indicator in center
  painter.setPen(QColor(180, 180, 180));
  font.setPointSize(10);
  font.setBold(false);
  painter.setFont(font);
  painter.drawText(
    static_cast<int>(center.x() - 8),
    static_cast<int>(center.y() + 4), "...");

  // Draw "Initializing" text below compass
  painter.setPen(QColor(180, 180, 180));
  font.setPointSize(7);
  painter.setFont(font);
  painter.drawText(
    static_cast<int>(center.x() - 30),
    static_cast<int>(center.y() + radius + 15), "Initializing");
  painter.drawText(
    static_cast<int>(center.x() - 30),
    static_cast<int>(center.y() + radius + 27), "(wait ESKF)");
}

void ImuDataHelper::drawGGDiagram(
  QPainter & painter, const QPointF & center,
  double size, double accel_x, double accel_y)
{
  const double half_size = size / 2.0;
  const double g = 9.80665;         // Gravity constant
  const double max_range = 2.0 * g; // ±2g range

  // Draw background square
  painter.setPen(QPen(QColor(150, 150, 150), 1));
  painter.setBrush(QColor(30, 30, 30, 150));
  QRectF diagram_rect(center.x() - half_size, center.y() - half_size, size,
    size);
  painter.drawRect(diagram_rect);

  // Draw grid circles (0.5g, 1.0g, 1.5g, 2.0g)
  painter.setPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
  painter.setBrush(Qt::NoBrush);
  for (int i = 1; i <= 4; ++i) {
    double g_level = i * 0.5 * g;
    double radius = (g_level / max_range) * half_size;
    painter.drawEllipse(center, radius, radius);
  }

  // Draw axis lines
  painter.setPen(QPen(QColor(120, 120, 120), 1));
  // X-axis (lateral)
  painter.drawLine(
    QPointF(center.x() - half_size, center.y()),
    QPointF(center.x() + half_size, center.y()));
  // Y-axis (longitudinal)
  painter.drawLine(
    QPointF(center.x(), center.y() - half_size),
    QPointF(center.x(), center.y() + half_size));

  // Draw axis labels
  painter.setPen(QColor(200, 200, 200));
  QFont font = painter.font();
  font.setPointSize(8);
  painter.setFont(font);
  painter.drawText(
    static_cast<int>(center.x() + half_size + 3),
    static_cast<int>(center.y() + 4), "X");
  painter.drawText(
    static_cast<int>(center.x() - 3),
    static_cast<int>(center.y() - half_size - 3), "Y");

  // Calculate point position
  // X-axis: lateral (left/right)
  // Y-axis: longitudinal (forward/backward)
  // Invert Y for screen coordinates (Y-up)
  double point_x = center.x() + (accel_x / max_range) * half_size;
  double point_y = center.y() - (accel_y / max_range) * half_size;

  // Clamp to diagram bounds
  point_x = std::max(
    center.x() - half_size,
    std::min(point_x, center.x() + half_size));
  point_y = std::max(
    center.y() - half_size,
    std::min(point_y, center.y() + half_size));

  // Draw acceleration point
  painter.setPen(QPen(QColor(255, 165, 0), 2)); // Orange
  painter.setBrush(QColor(255, 165, 0, 200));
  painter.drawEllipse(QPointF(point_x, point_y), 4, 4);

  // Draw acceleration magnitude below diagram
  double accel_mag = std::sqrt(accel_x * accel_x + accel_y * accel_y);
  QString accel_text = QString("%1 m/s²").arg(accel_mag, 0, 'f', 2);
  painter.setPen(QColor(255, 255, 255));
  font.setPointSize(8);
  painter.setFont(font);
  painter.drawText(
    static_cast<int>(center.x() - 25),
    static_cast<int>(center.y() + half_size + 15), accel_text);
}

} // namespace rviz_overlay
} // namespace eskf_localization
