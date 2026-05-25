#include "follower_lidar_localization/wheel_fitting.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <vector>

namespace follower_lidar_localization
{
namespace
{

std::vector<ScanPoint2> sample_segment(
  const Point2 & start,
  const Point2 & end,
  int count)
{
  std::vector<ScanPoint2> points;
  for (int i = 0; i < count; ++i) {
    const double t = count == 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(count - 1);
    points.push_back(
      ScanPoint2{
          {start.x + t * (end.x - start.x), start.y + t * (end.y - start.y)},
          true});
  }
  points.push_back(ScanPoint2{{0.0, 0.0}, false});
  return points;
}

std::vector<ScanPoint2> sample_model_segments(
  const FitConfig & config,
  const Pose2 & pose,
  const std::vector<int> & visible_indices)
{
  std::vector<ScanPoint2> points;
  const auto model = make_leader_wheel_model(config);
  for (const int index : visible_indices) {
    const auto & segment = model.at(index);
    const Point2 start_model{
      segment.center.x - 0.5 * segment.length * segment.direction.x,
      segment.center.y - 0.5 * segment.length * segment.direction.y};
    const Point2 end_model{
      segment.center.x + 0.5 * segment.length * segment.direction.x,
      segment.center.y + 0.5 * segment.length * segment.direction.y};
    auto segment_points = sample_segment(
      transform_point(pose, start_model),
      transform_point(pose, end_model),
      7);
    points.insert(points.end(), segment_points.begin(), segment_points.end());
  }
  return points;
}

}  // namespace

TEST(WheelFitting, FitsFourVisibleWheelSegments)
{
  FitConfig config;
  const Pose2 expected{1.05, -0.12, 0.18};
  const auto points = sample_model_segments(config, expected, {0, 1, 2, 3});

  const auto result = fit_leader_wheel_pose(points, config);

  ASSERT_TRUE(result.valid) << result.status;
  EXPECT_EQ(result.visible_segments, 4);
  EXPECT_NEAR(result.pose.x, expected.x, 0.03);
  EXPECT_NEAR(result.pose.y, expected.y, 0.03);
  EXPECT_NEAR(angle_distance(result.pose.yaw, expected.yaw), 0.0, 0.04);
}

TEST(WheelFitting, PriorDisambiguatesTwoRearSegments)
{
  FitConfig config;
  const Pose2 expected{1.20, 0.08, -0.12};
  const auto points = sample_model_segments(config, expected, {0, 1});

  const auto result = fit_leader_wheel_pose(points, config, expected);

  ASSERT_TRUE(result.valid) << result.status;
  EXPECT_EQ(result.visible_segments, 2);
  EXPECT_NEAR(result.pose.x, expected.x, 0.04);
  EXPECT_NEAR(result.pose.y, expected.y, 0.04);
  EXPECT_NEAR(angle_distance(result.pose.yaw, expected.yaw), 0.0, 0.05);
}

TEST(WheelFitting, OneSegmentDoesNotCreateValidPose)
{
  FitConfig config;
  const Pose2 expected{1.0, 0.0, 0.0};
  const auto points = sample_model_segments(config, expected, {0});

  const auto result = fit_leader_wheel_pose(points, config, expected);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.visible_segments, 1);
}

TEST(WheelFitting, IgnoresOutlierSegment)
{
  FitConfig config;
  const Pose2 expected{0.95, 0.05, 0.08};
  auto points = sample_model_segments(config, expected, {0, 1, 2, 3});
  auto outlier = sample_segment({1.8, 0.9}, {1.8, 1.01}, 7);
  points.insert(points.end(), outlier.begin(), outlier.end());

  const auto result = fit_leader_wheel_pose(points, config, expected);

  ASSERT_TRUE(result.valid) << result.status;
  EXPECT_GE(result.visible_segments, 4);
  EXPECT_NEAR(result.pose.x, expected.x, 0.04);
  EXPECT_NEAR(result.pose.y, expected.y, 0.04);
  EXPECT_NEAR(angle_distance(result.pose.yaw, expected.yaw), 0.0, 0.05);
}

TEST(WheelTracker, CoastsDuringShortDropout)
{
  TrackerConfig tracker_config;
  tracker_config.max_coast_sec = 0.5;
  WheelPoseTracker tracker(tracker_config);

  FitResult result;
  result.valid = true;
  result.status = "OK";
  result.pose = {1.0, 0.0, 0.0};
  result.visible_segments = 4;

  auto output = tracker.update(result, 10.0);
  ASSERT_TRUE(output.has_pose);
  EXPECT_EQ(output.mode, TrackerMode::Tracking);

  FitResult dropout;
  dropout.valid = false;
  dropout.status = "NO_SEGMENT_CANDIDATES";
  output = tracker.update(dropout, 10.1);

  EXPECT_TRUE(output.has_pose);
  EXPECT_EQ(output.mode, TrackerMode::Coasting);
}

}  // namespace follower_lidar_localization
