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

#include "eskf_localization/rviz_overlay/vehicle_data_helper.hpp"

#include <QFont>

#include <cmath>

namespace eskf_localization
{
namespace rviz_overlay
{

VehicleDataHelper::VehicleDataHelper()
: longitudinal_vel_(0.0), steering_angle_(0.0) {}

void VehicleDataHelper::updateVelocityData(float longitudinal)
{
  longitudinal_vel_ = longitudinal;
}

void VehicleDataHelper::updateSteeringData(float steering_angle_rad)
{
  steering_angle_ = steering_angle_rad;
}

void VehicleDataHelper::drawVehicleInfo(
  QPainter & painter, const QRectF & rect,
  const QColor & color)
{
  painter.setPen(color);
  QFont font = painter.font();
  font.setPointSize(11);
  painter.setFont(font);

  int y_offset = static_cast<int>(rect.top());

  // Vehicle speed (moved to sensor data section)
  double speed_ms = static_cast<double>(longitudinal_vel_);
  double speed_kmh = speed_ms * 3.6;
  QString speed_text = QString("Vehicle Speed: %1 m/s (%2 km/h)")
    .arg(speed_ms, 0, 'f', 2)
    .arg(speed_kmh, 0, 'f', 2);
  painter.drawText(
    static_cast<int>(rect.left()) + 10, y_offset + 240,
    speed_text);

  // Steering angle
  double steering_deg = steering_angle_ * 180.0 / M_PI;
  QString steering_text =
    QString("Steering Angle: %1 rad (%2\u00B0)")
    .arg(static_cast<double>(steering_angle_), 0, 'f', 4)
    .arg(steering_deg, 0, 'f', 2);
  painter.drawText(
    static_cast<int>(rect.left()) + 10, y_offset + 260,
    steering_text);
}

} // namespace rviz_overlay
} // namespace eskf_localization
