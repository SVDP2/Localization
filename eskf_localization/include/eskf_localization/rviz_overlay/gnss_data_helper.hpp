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

#ifndef ESKF_LOCALIZATION__RVIZ_OVERLAY__GNSS_DATA_HELPER_HPP_
#define ESKF_LOCALIZATION__RVIZ_OVERLAY__GNSS_DATA_HELPER_HPP_

#include <QColor>
#include <QPainter>
#include <QPointF>
#include <QString>

namespace eskf_localization
{
namespace rviz_overlay
{

/**
 * Helper class for rendering GNSS velocity and heading data.
 * Includes compass visualization for heading display.
 */
class GnssDataHelper
{
public:
  GnssDataHelper();

  void updateVelocityData(float vn, float ve);
  void updateHeadingData(double heading_deg);

  void drawGnssInfo(QPainter & painter, const QRectF & rect, const QColor & color);
  void drawCompassHeading(
    QPainter & painter, const QPointF & center, double radius, double heading_rad,
    double value_for_display);
  void drawCompassHeadingEastZero(
    QPainter & painter, const QPointF & center, double radius, double heading_deg,
    double value_for_display);

private:
  float vn_;
  float ve_;
  double gnss_heading_;

  double computeSpeed() const;
  double computeHeadingRad() const;
};

}  // namespace rviz_overlay
}  // namespace eskf_localization

#endif  // ESKF_LOCALIZATION__RVIZ_OVERLAY__GNSS_DATA_HELPER_HPP_
