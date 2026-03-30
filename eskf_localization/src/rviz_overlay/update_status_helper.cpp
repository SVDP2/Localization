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

#include "eskf_localization/rviz_overlay/update_status_helper.hpp"

#include <QFont>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace eskf_localization
{
namespace rviz_overlay
{

UpdateStatusHelper::UpdateStatusHelper()
{
}

void UpdateStatusHelper::updateStatusData(const UpdateStatusData & data)
{
  data_ = data;
}

void UpdateStatusHelper::drawUpdateStatus(
  QPainter & painter, const QRectF & rect, const QColor & text_color)
{
  const double base_x = rect.left() + 10;
  const double base_y = rect.top() + 320;
  const double grid_width = 240;
  const double grid_height = 95;

  QFont section_font = painter.font();
  section_font.setPointSize(11);
  section_font.setBold(true);
  painter.setFont(section_font);
  painter.setPen(text_color);

  // Position section
  painter.drawText(base_x, base_y, "Position Update");
  drawPositionGrid(
    painter, base_x, base_y + 5, grid_width, grid_height, text_color);

  // Velocity section
  const double vel_y = base_y + grid_height + 20;
  painter.setPen(text_color);
  painter.setFont(section_font);
  painter.drawText(base_x, vel_y, "Velocity Update");
  drawVelocityGrid(
    painter, base_x, vel_y + 5, grid_width, grid_height, text_color);

  // Heading section
  const double head_y = vel_y + grid_height + 20;
  painter.setPen(text_color);
  painter.setFont(section_font);
  painter.drawText(base_x, head_y, "Heading Update");
  drawHeadingGrid(
    painter, base_x, head_y + 5, grid_width, grid_height, text_color);

  // Right column: Status and R values
  const double right_x = base_x + grid_width + 40;
  painter.setPen(text_color);
  painter.setFont(section_font);
  painter.drawText(right_x, base_y, "Sensor Status");

  QFont data_font = painter.font();
  data_font.setPointSize(9);
  data_font.setBold(false);
  painter.setFont(data_font);

  double info_y = base_y + 20;

  // GNSS Status with colored circle
  const double circle_radius = 6.0;
  const QPointF gnss_circle_center(right_x + 50, info_y - 4);
  painter.setBrush(getGnssStatusColor());
  painter.setPen(Qt::NoPen);
  painter.drawEllipse(gnss_circle_center, circle_radius, circle_radius);

  painter.setPen(text_color);
  painter.drawText(right_x, info_y, "GNSS:");
  painter.drawText(right_x + 65, info_y, getGnssStatusText());
  info_y += 18;

  // ESKF Init Status
  const QPointF init_circle_center(right_x + 50, info_y - 4);
  QColor init_color = data_.eskf_initialized ? QColor(50, 200, 50) : QColor(200, 50, 50);
  painter.setBrush(init_color);
  painter.setPen(Qt::NoPen);
  painter.drawEllipse(init_circle_center, circle_radius, circle_radius);

  painter.setPen(text_color);
  painter.drawText(right_x, info_y, "ESKF:");
  painter.drawText(right_x + 65, info_y, data_.eskf_initialized ? "Init" : "Not Init");
  info_y += 25;

  // Position R values
  painter.setPen(text_color);
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);

  double pos_R_avg = (data_.pos_R_xx + data_.pos_R_yy + data_.pos_R_zz) / 3.0;
  oss << "Pos R:  " << pos_R_avg << " m²";
  painter.drawText(right_x, info_y, QString::fromStdString(oss.str()));
  info_y += 15;

  // Velocity R values
  oss.str("");
  double vel_R_avg = (data_.vel_R_xx + data_.vel_R_yy + data_.vel_R_zz) / 3.0;
  oss << "Vel R:  " << vel_R_avg << " m²/s²";
  painter.drawText(right_x, info_y, QString::fromStdString(oss.str()));
  info_y += 15;

  // Yaw R value
  oss.str("");
  oss << "Yaw R:  " << data_.heading_R << " rad²";
  painter.drawText(right_x, info_y, QString::fromStdString(oss.str()));

  // KISS-ICP tile
  info_y += 24;
  painter.setPen(text_color);
  painter.setFont(section_font);
  painter.drawText(right_x, info_y, "KISS Status");

  const bool kiss_yaw_active = data_.kiss_yaw_enabled && data_.kiss_yaw_applied;
  const bool kiss_vy_active = data_.kiss_vy_enabled && data_.kiss_vy_applied;
  const bool kiss_off = !data_.kiss_enabled;
  const bool has_skip_reason =
    !data_.kiss_skip_reason.empty() && data_.kiss_skip_reason != "none";
  const bool has_update_issue =
    has_skip_reason ||
    (data_.kiss_yaw_enabled && !data_.kiss_yaw_applied) ||
    (data_.kiss_vy_enabled && !data_.kiss_vy_applied);
  const bool kiss_error_active = !kiss_off && has_update_issue;
  const double right_col_width = std::max(120.0, rect.right() - right_x - 10.0);
  const double kiss_cell_w = std::clamp(right_col_width / 3.0, 40.0, 62.0);
  const double kiss_cell_h = 16.0;
  const double kiss_grid_y = info_y + 6.0;
  drawGridCell(painter, right_x, kiss_grid_y, kiss_cell_w, kiss_cell_h, "YAW", kiss_yaw_active);
  drawGridCell(
    painter, right_x + kiss_cell_w, kiss_grid_y, kiss_cell_w, kiss_cell_h,
    "VY", kiss_vy_active);
  drawGridCell(
    painter, right_x + 2.0 * kiss_cell_w, kiss_grid_y, kiss_cell_w, kiss_cell_h,
    "ERROR", kiss_error_active);

  QFont kiss_font = painter.font();
  kiss_font.setPointSize(8);
  kiss_font.setBold(false);
  painter.setFont(kiss_font);
  painter.setPen(text_color);

  double kiss_info_y = kiss_grid_y + kiss_cell_h + 12.0;
  const QString kiss_state = !data_.kiss_enabled ? "OFF" : (data_.kiss_initialized ? "RUN" : "WAIT_INIT");
  painter.drawText(right_x, kiss_info_y, "State: " + kiss_state);
  kiss_info_y += 12.0;

  std::ostringstream kiss_oss;
  kiss_oss << std::fixed << std::setprecision(2);
  kiss_oss << "Trust " << data_.kiss_trust << "/" << data_.kiss_target_trust;
  painter.drawText(right_x, kiss_info_y, QString::fromStdString(kiss_oss.str()));
  kiss_info_y += 12.0;

  kiss_oss.str("");
  kiss_oss.clear();
  kiss_oss << std::fixed << std::setprecision(2);
  kiss_oss << "Yaw: " << data_.kiss_yaw_rate_radps << "  Vy: " << data_.kiss_vy_mps;
  painter.drawText(right_x, kiss_info_y, QString::fromStdString(kiss_oss.str()));
  kiss_info_y += 12.0;

  std::string reason = data_.kiss_skip_reason;
  if (reason.empty() || reason == "none") {
    if (data_.kiss_yaw_enabled && !data_.kiss_yaw_applied &&
      !data_.kiss_yaw_reason.empty())
    {
      reason = data_.kiss_yaw_reason;
    } else if (data_.kiss_vy_enabled && !data_.kiss_vy_applied &&
      !data_.kiss_vy_reason.empty())
    {
      reason = data_.kiss_vy_reason;
    }
  }
  if (reason.empty()) {
    reason = "none";
  }
  QString reason_text = QString::fromStdString(reason);
  if (reason_text.length() > 22) {
    reason_text = reason_text.left(22) + "..";
  }
  painter.drawText(right_x, kiss_info_y, "Reason: " + reason_text);

  if (
    reason.find("time_alignment") != std::string::npos &&
    data_.kiss_time_alignment_error_ms > 0.0)
  {
    kiss_info_y += 12.0;
    std::ostringstream align_oss;
    align_oss << std::fixed << std::setprecision(1);
    align_oss << "AlignErr: " << data_.kiss_time_alignment_error_ms << " ms";
    painter.drawText(right_x, kiss_info_y, QString::fromStdString(align_oss.str()));
  }
}

void UpdateStatusHelper::drawPositionGrid(
  QPainter & painter, double x, double y, double width, double height,
  const QColor & text_color)
{
  const double cell_width = width / 3.0;
  const double cell_height = (height - 15) / 3.0;

  QString category = getCategoryFromReason(data_.pos_reason, data_.pos_applied);

  // Row 1
  drawGridCell(painter, x, y, cell_width, cell_height, "Applied", category == "Applied");
  drawGridCell(painter, x + cell_width, y, cell_width, cell_height, "Holdoff", category == "Holdoff");
  drawGridCell(painter, x + 2 * cell_width, y, cell_width, cell_height, "StatusLo", category == "StatusLo");

  // Row 2
  drawGridCell(painter, x, y + cell_height, cell_width, cell_height, "TimeErr", category == "TimeErr");
  drawGridCell(painter, x + cell_width, y + cell_height, cell_width, cell_height, "NotInit", category == "NotInit");
  drawGridCell(painter, x + 2 * cell_width, y + cell_height, cell_width, cell_height, "Error", category == "Error");

  // Reason row
  painter.setPen(text_color);
  QFont reason_font = painter.font();
  reason_font.setPointSize(8);
  painter.setFont(reason_font);
  QString reason_text = "Reason: " + formatReason(data_.pos_reason, data_.pos_applied);
  painter.drawText(x, y + 2 * cell_height + 12, reason_text);
}

void UpdateStatusHelper::drawVelocityGrid(
  QPainter & painter, double x, double y, double width, double height,
  const QColor & text_color)
{
  const double cell_width = width / 3.0;
  const double cell_height = (height - 15) / 3.0;

  QString category = getCategoryFromReason(data_.vel_reason, data_.vel_applied);

  // Row 1
  drawGridCell(painter, x, y, cell_width, cell_height, "Applied", category == "Applied");
  drawGridCell(painter, x + cell_width, y, cell_width, cell_height, "Holdoff", category == "Holdoff");
  drawGridCell(painter, x + 2 * cell_width, y, cell_width, cell_height, "StatusLo", category == "StatusLo");

  // Row 2
  drawGridCell(painter, x, y + cell_height, cell_width, cell_height, "TimeErr", category == "TimeErr");
  drawGridCell(painter, x + cell_width, y + cell_height, cell_width, cell_height, "BadData", category == "BadData");
  drawGridCell(painter, x + 2 * cell_width, y + cell_height, cell_width, cell_height, "Error", category == "Error");

  // Reason row
  painter.setPen(text_color);
  QFont reason_font = painter.font();
  reason_font.setPointSize(8);
  painter.setFont(reason_font);
  QString reason_text = "Reason: " + formatReason(data_.vel_reason, data_.vel_applied);
  painter.drawText(x, y + 2 * cell_height + 12, reason_text);
}

void UpdateStatusHelper::drawHeadingGrid(
  QPainter & painter, double x, double y, double width, double height,
  const QColor & text_color)
{
  const double cell_width = width / 3.0;
  const double cell_height = (height - 15) / 3.0;

  QString category = getCategoryFromReason(data_.heading_reason, data_.heading_applied);

  // Row 1
  drawGridCell(painter, x, y, cell_width, cell_height, "Normal", category == "Normal");
  drawGridCell(painter, x + cell_width, y, cell_width, cell_height, "Bypass", category == "Bypass");
  drawGridCell(painter, x + 2 * cell_width, y, cell_width, cell_height, "RateGate", category == "RateGate");

  // Row 2
  drawGridCell(painter, x, y + cell_height, cell_width, cell_height, "Holdoff", category == "Holdoff");
  drawGridCell(painter, x + cell_width, y + cell_height, cell_width, cell_height, "StatusLo", category == "StatusLo");
  drawGridCell(painter, x + 2 * cell_width, y + cell_height, cell_width, cell_height, "Timeout", category == "Timeout");

  // Row 3
  drawGridCell(painter, x, y + 2 * cell_height, cell_width, cell_height, "Disabled", category == "Disabled");
  drawGridCell(painter, x + cell_width, y + 2 * cell_height, cell_width, cell_height, "Error", category == "Error");

  // Reason row
  painter.setPen(text_color);
  QFont reason_font = painter.font();
  reason_font.setPointSize(8);
  painter.setFont(reason_font);
  QString reason_text = "Reason: " + formatReason(data_.heading_reason, data_.heading_applied);
  painter.drawText(x, y + 3 * cell_height + 12, reason_text);
}

void UpdateStatusHelper::drawGridCell(
  QPainter & painter, double x, double y, double width, double height,
  const QString & label, bool active)
{
  QColor bg_color;
  if (active) {
    bg_color = QColor(ACTIVE_R, ACTIVE_G, ACTIVE_B, ACTIVE_ALPHA);
  } else {
    bg_color = QColor(INACTIVE_R, INACTIVE_G, INACTIVE_B, INACTIVE_ALPHA);
  }

  painter.setBrush(bg_color);
  painter.setPen(QColor(200, 200, 200, 150));
  painter.drawRect(QRectF(x, y, width - 2, height - 2));

  painter.setPen(QColor(255, 255, 255));
  QFont cell_font = painter.font();
  cell_font.setPointSize(8);
  painter.setFont(cell_font);

  QRectF text_rect(x, y, width - 2, height - 2);
  painter.drawText(text_rect, Qt::AlignCenter, label);
}

QString UpdateStatusHelper::getCategoryFromReason(const std::string & reason, bool applied) const
{
  if (applied) {
    // Heading has "Normal" cell
    if (reason == "heading") {
      return "Normal";
    }
    // Position/Velocity have "Applied" cell
    if (reason.empty() || reason == "gnss_pos" || reason == "gnss_vel") {
      return "Applied";
    }
    if (reason == "nis_inflated") {
      return "Applied";  // Still applied but with NIS inflate
    }
    if (reason.find("rate_gate_bypass") != std::string::npos) {
      return "Bypass";
    }
    return "Applied";
  }

  // Not applied
  if (reason.find("recover_holdoff") != std::string::npos ||
      reason.find("heading_recover_holdoff") != std::string::npos) {
    return "Holdoff";
  }
  if (reason.find("gnss_status") != std::string::npos) {
    return "StatusLo";
  }
  if (reason.find("time_alignment") != std::string::npos) {
    return "TimeErr";
  }
  if (reason.find("not_initialized") != std::string::npos ||
      reason.find("not_activated") != std::string::npos ||
      reason.find("wait_yaw_for_init") != std::string::npos ||
      reason.find("external_initialpose_mode") != std::string::npos) {
    return "NotInit";
  }
  if (reason.find("rate_gate_skip") != std::string::npos) {
    return "RateGate";
  }
  if (reason.find("heading_timeout") != std::string::npos) {
    return "Timeout";
  }
  if (reason == "disabled") {
    return "Disabled";
  }
  if (reason.find("non_finite") != std::string::npos) {
    return "BadData";
  }
  if (!reason.empty()) {
    return "Error";  // Catch-all for other errors
  }

  return "Applied";  // Default if reason is empty and applied is false (shouldn't happen)
}

QString UpdateStatusHelper::formatReason(const std::string & reason, bool applied) const
{
  if (applied && (reason.empty() || reason == "gnss_pos" || reason == "gnss_vel")) {
    return "normal";
  }
  if (applied && reason == "heading") {
    return "heading";
  }
  return QString::fromStdString(reason);
}

QColor UpdateStatusHelper::getGnssStatusColor() const
{
  switch (data_.gnss_status) {
    case -1:
      return QColor(255, 0, 0);  // Red - NO_FIX
    case 0:
      return QColor(255, 255, 0);  // Yellow - FIX
    case 1:
      return QColor(0, 255, 0);  // Green - SBAS_FIX
    case 2:
      return QColor(0, 150, 255);  // Blue - GBAS_FIX
    default:
      return QColor(128, 128, 128);  // Gray - Unknown
  }
}

QString UpdateStatusHelper::getGnssStatusText() const
{
  switch (data_.gnss_status) {
    case -1:
      return "NO_FIX";
    case 0:
      return "FIX";
    case 1:
      return "SBAS";
    case 2:
      return "GBAS";
    default:
      return "UNKNOWN";
  }
}

}  // namespace rviz_overlay
}  // namespace eskf_localization
