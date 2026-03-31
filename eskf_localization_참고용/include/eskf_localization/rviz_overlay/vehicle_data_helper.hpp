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

#ifndef ESKF_LOCALIZATION__RVIZ_OVERLAY__VEHICLE_DATA_HELPER_HPP_
#define ESKF_LOCALIZATION__RVIZ_OVERLAY__VEHICLE_DATA_HELPER_HPP_

#include <QColor>
#include <QPainter>
#include <QString>

namespace eskf_localization
{
namespace rviz_overlay
{

/**
 * Helper class for rendering vehicle OBD data (velocity and steering angle).
 * Note: lateral_velocity is excluded as it is always 0.0 (invalid field).
 */
class VehicleDataHelper
{
public:
  VehicleDataHelper();

  void updateVelocityData(float longitudinal);
  void updateSteeringData(float steering_angle_rad);

  void drawVehicleInfo(QPainter & painter, const QRectF & rect, const QColor & color);

private:
  float longitudinal_vel_;
  float steering_angle_;
};

}  // namespace rviz_overlay
}  // namespace eskf_localization

#endif  // ESKF_LOCALIZATION__RVIZ_OVERLAY__VEHICLE_DATA_HELPER_HPP_
