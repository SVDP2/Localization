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

#include "eskf_localization/rviz_overlay/gnss_data_helper.hpp"

#include <QFont>
#include <QPolygonF>

#include <cmath>

namespace eskf_localization
{
namespace rviz_overlay
{

GnssDataHelper::GnssDataHelper()
: vn_(0.0), ve_(0.0), gnss_heading_(0.0) {}

void GnssDataHelper::updateVelocityData(float vn, float ve)
{
  vn_ = vn;
  ve_ = ve;
}

void GnssDataHelper::updateHeadingData(double heading_deg)
{
  gnss_heading_ = heading_deg;
}

double GnssDataHelper::computeSpeed() const
{
  return std::sqrt(vn_ * vn_ + ve_ * ve_);
}

double GnssDataHelper::computeHeadingRad() const
{
  return std::atan2(ve_, vn_);
}

void GnssDataHelper::drawGnssInfo(
  QPainter & painter, const QRectF & rect,
  const QColor & color)
{
  painter.setPen(color);
  QFont font = painter.font();
  font.setPointSize(11);
  painter.setFont(font);

  double speed_ms = computeSpeed();
  double speed_kmh = speed_ms * 3.6;
  double heading_computed_rad = computeHeadingRad();

  int y_offset = static_cast<int>(rect.top());

  // GNSS speed (moved to sensor data section)
  QString speed_text = QString("GNSS Speed: %1 m/s (%2 km/h)")
    .arg(speed_ms, 0, 'f', 2)
    .arg(speed_kmh, 0, 'f', 2);
  painter.drawText(
    static_cast<int>(rect.left()) + 10, y_offset + 220,
    speed_text);

  // Draw compass for computed heading (moved to heading comparison)
  font.setPointSize(9);
  font.setBold(false);
  painter.setFont(font);
  painter.drawText(
    static_cast<int>(rect.left()) + 25, y_offset + 68,
    "GNSS Computed");
  QPointF compass_center_computed(rect.left() + 60, y_offset + 120);
  drawCompassHeading(
    painter, compass_center_computed, 30.0,
    heading_computed_rad, heading_computed_rad);

  // Draw compass for sensor heading (East = 0 deg, CW+)
  painter.drawText(
    static_cast<int>(rect.left()) + 135, y_offset + 68,
    "GNSS Sensor");
  QPointF compass_center_sensor(rect.left() + 170, y_offset + 120);
  drawCompassHeadingEastZero(
    painter, compass_center_sensor, 30.0,
    gnss_heading_, gnss_heading_);
}

void GnssDataHelper::drawCompassHeading(
  QPainter & painter,
  const QPointF & center, double radius,
  double heading_rad,
  double value_for_display)
{
  // Draw compass circle background
  painter.setPen(QPen(QColor(200, 200, 200), 2));
  painter.setBrush(QColor(50, 50, 50, 64));
  painter.drawEllipse(center, radius, radius);

  // Draw North (N) marker at top
  painter.setPen(QColor(255, 255, 255));
  QFont font = painter.font();
  font.setPointSize(10);
  font.setBold(true);
  painter.setFont(font);
  painter.drawText(
    static_cast<int>(center.x() - 5),
    static_cast<int>(center.y() - radius - 5), "N");

  // Draw heading arrow
  painter.save();
  painter.translate(center);
  painter.rotate(heading_rad * 180.0 / M_PI); // Convert radians to degrees

  // Draw arrow line
  painter.setPen(QPen(QColor(255, 0, 0), 2));
  painter.drawLine(QPointF(0, 0), QPointF(0, -radius * 0.8));

  // Draw arrow head
  QPolygonF arrowHead;
  arrowHead << QPointF(0, -radius * 0.8) << QPointF(-5, -radius * 0.65)
            << QPointF(5, -radius * 0.65);
  painter.setBrush(QColor(255, 0, 0));
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

void GnssDataHelper::drawCompassHeadingEastZero(
  QPainter & painter,
  const QPointF & center,
  double radius,
  double heading_deg,
  double value_for_display)
{
  // Draw compass circle background
  painter.setPen(QPen(QColor(200, 200, 200), 2));
  painter.setBrush(QColor(50, 50, 50, 64));
  painter.drawEllipse(center, radius, radius);

  // Draw East (E) marker at right (3 o'clock position)
  painter.setPen(QColor(255, 255, 255));
  QFont font = painter.font();
  font.setPointSize(10);
  font.setBold(true);
  painter.setFont(font);
  painter.drawText(
    static_cast<int>(center.x() + radius + 5),
    static_cast<int>(center.y() + 5), "E");

  // Convert heading: East = 0 deg, CW+ to Qt rotation
  // Qt rotation: 0 deg = right (3 o'clock), CW+
  // So heading_deg directly maps to Qt rotation
  double qt_rotation_deg = heading_deg;

  // Draw heading arrow
  painter.save();
  painter.translate(center);
  painter.rotate(qt_rotation_deg); // Rotate clockwise from East

  // Draw arrow line
  painter.setPen(QPen(QColor(255, 0, 0), 2));
  painter.drawLine(QPointF(0, 0), QPointF(radius * 0.8, 0));

  // Draw arrow head (pointing right before rotation)
  QPolygonF arrowHead;
  arrowHead << QPointF(radius * 0.8, 0) << QPointF(radius * 0.65, -5)
            << QPointF(radius * 0.65, 5);
  painter.setBrush(QColor(255, 0, 0));
  painter.drawPolygon(arrowHead);

  painter.restore();

  // Draw angle value below compass (2 lines: deg primary, rad secondary)
  painter.setPen(QColor(255, 255, 255));
  font.setBold(false);
  font.setPointSize(8);
  painter.setFont(font);
  // Line 1: deg (primary unit for GNSS sensor)
  QString line1 = QString("%1 deg").arg(value_for_display, 0, 'f', 1);
  painter.drawText(
    static_cast<int>(center.x() - 30),
    static_cast<int>(center.y() + radius + 15), line1);
  // Line 2: rad (converted)
  double rad = value_for_display * M_PI / 180.0;
  QString line2 = QString("(%1 rad)").arg(rad, 0, 'f', 3);
  painter.drawText(
    static_cast<int>(center.x() - 30),
    static_cast<int>(center.y() + radius + 27), line2);
}

} // namespace rviz_overlay
} // namespace eskf_localization
