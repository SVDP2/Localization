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

#ifndef ESKF_LOCALIZATION__RVIZ_OVERLAY__STATUS_CHART_HELPER_HPP_
#define ESKF_LOCALIZATION__RVIZ_OVERLAY__STATUS_CHART_HELPER_HPP_

#include <QColor>
#include <QPainter>
#include <QString>

#include <cstdint>

namespace eskf_localization
{
namespace rviz_overlay
{

/**
 * Helper class for rendering GNSS fix status as a colored indicator.
 */
class StatusChartHelper
{
public:
  StatusChartHelper();

  void updateStatusData(int8_t status);

  void drawStatusChart(QPainter & painter, const QRectF & rect);

private:
  int8_t status_;

  QColor getStatusColor() const;
  QString getStatusText() const;
};

}  // namespace rviz_overlay
}  // namespace eskf_localization

#endif  // ESKF_LOCALIZATION__RVIZ_OVERLAY__STATUS_CHART_HELPER_HPP_
