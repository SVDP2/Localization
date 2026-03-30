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

#include "eskf_localization/rviz_overlay/status_chart_helper.hpp"

namespace eskf_localization
{
namespace rviz_overlay
{

StatusChartHelper::StatusChartHelper()
: status_(-1)
{
}

void StatusChartHelper::updateStatusData(int8_t status)
{
  status_ = status;
}

QColor StatusChartHelper::getStatusColor() const
{
  // -1: NO_FIX (red), 0: FIX (yellow), 1: SBAS_FIX (green), 2: GBAS_FIX (blue)
  switch (status_) {
    case -1:
      return QColor(255, 0, 0);  // Red
    case 0:
      return QColor(255, 255, 0);  // Yellow
    case 1:
      return QColor(0, 255, 0);  // Green
    case 2:
      return QColor(0, 0, 255);  // Blue
    default:
      return QColor(128, 128, 128);  // Gray for unknown
  }
}

QString StatusChartHelper::getStatusText() const
{
  switch (status_) {
    case -1:
      return "NO_FIX";
    case 0:
      return "FIX";
    case 1:
      return "SBAS_FIX";
    case 2:
      return "GBAS_FIX";
    default:
      return "UNKNOWN";
  }
}

void StatusChartHelper::drawStatusChart(QPainter & painter, const QRectF & rect)
{
  const double indicator_radius = 10.0;
  const QPointF indicator_center(rect.left() + 170, rect.top() + 35);

  // Draw status text
  painter.setPen(QColor(255, 255, 255));
  QFont font = painter.font();
  font.setPointSize(12);
  font.setBold(false);
  painter.setFont(font);
  painter.drawText(rect.left() + 10, rect.top() + 40, "Fix Status: " + getStatusText());

  // Draw colored indicator circle
  painter.setBrush(getStatusColor());
  painter.setPen(Qt::NoPen);
  painter.drawEllipse(indicator_center, indicator_radius, indicator_radius);
}

}  // namespace rviz_overlay
}  // namespace eskf_localization
