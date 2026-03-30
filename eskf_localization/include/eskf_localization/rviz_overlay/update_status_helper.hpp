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

#ifndef ESKF_LOCALIZATION__RVIZ_OVERLAY__UPDATE_STATUS_HELPER_HPP_
#define ESKF_LOCALIZATION__RVIZ_OVERLAY__UPDATE_STATUS_HELPER_HPP_

#include <QColor>
#include <QPainter>
#include <QRectF>
#include <QString>

#include <cstddef>
#include <string>

namespace eskf_localization
{
namespace rviz_overlay
{

struct UpdateStatusData
{
  // Position update
  bool pos_applied{false};
  std::string pos_reason{};
  double pos_R_xx{0.0};
  double pos_R_yy{0.0};
  double pos_R_zz{0.0};

  // Velocity update
  bool vel_applied{false};
  std::string vel_reason{};
  double vel_R_xx{0.0};
  double vel_R_yy{0.0};
  double vel_R_zz{0.0};

  // Heading update
  bool heading_applied{false};
  std::string heading_reason{};
  double heading_R{0.0};

  // System status
  int8_t gnss_status{-1};
  bool eskf_initialized{false};

  // KISS-ICP status
  bool kiss_enabled{false};
  bool kiss_initialized{false};
  bool kiss_yaw_enabled{false};
  bool kiss_vy_enabled{false};
  bool kiss_yaw_applied{false};
  std::string kiss_yaw_reason{};
  bool kiss_vy_applied{false};
  std::string kiss_vy_reason{};
  std::string kiss_skip_reason{};
  double kiss_time_alignment_error_ms{0.0};
  int kiss_source_points{0};
  double kiss_dt_ms{0.0};
  double kiss_trust{0.0};
  double kiss_target_trust{0.0};
  double kiss_yaw_rate_radps{0.0};
  double kiss_vy_mps{0.0};
  bool kiss_reset_candidate{false};
  size_t kiss_reset_count{0};
};

/**
 * Helper class for rendering ESKF update status grid.
 * Displays Position/Velocity/Heading update states in a 3x2 grid format.
 */
class UpdateStatusHelper
{
public:
  UpdateStatusHelper();

  void updateStatusData(const UpdateStatusData & data);

  void drawUpdateStatus(QPainter & painter, const QRectF & rect, const QColor & text_color);

private:
  UpdateStatusData data_;

  // Color constants
  static constexpr int ACTIVE_R = 50;
  static constexpr int ACTIVE_G = 200;
  static constexpr int ACTIVE_B = 50;
  static constexpr int ACTIVE_ALPHA = 180;

  static constexpr int INACTIVE_R = 80;
  static constexpr int INACTIVE_G = 80;
  static constexpr int INACTIVE_B = 80;
  static constexpr int INACTIVE_ALPHA = 120;

  void drawPositionGrid(
    QPainter & painter, double x, double y, double width, double height,
    const QColor & text_color);
  void drawVelocityGrid(
    QPainter & painter, double x, double y, double width, double height,
    const QColor & text_color);
  void drawHeadingGrid(
    QPainter & painter, double x, double y, double width, double height,
    const QColor & text_color);

  void drawGridCell(
    QPainter & painter, double x, double y, double width, double height,
    const QString & label, bool active);

  QString getCategoryFromReason(const std::string & reason, bool applied) const;
  QString formatReason(const std::string & reason, bool applied) const;

  QColor getGnssStatusColor() const;
  QString getGnssStatusText() const;
};

}  // namespace rviz_overlay
}  // namespace eskf_localization

#endif  // ESKF_LOCALIZATION__RVIZ_OVERLAY__UPDATE_STATUS_HELPER_HPP_
