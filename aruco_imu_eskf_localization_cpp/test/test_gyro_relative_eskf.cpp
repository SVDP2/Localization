#include "aruco_imu_eskf_localization_cpp/gyro_relative_eskf.hpp"
#include "aruco_imu_eskf_localization_cpp/geometry.hpp"

#include <gtest/gtest.h>

using aruco_imu_eskf_localization_cpp::GyroRelativeEskf;
using aruco_imu_eskf_localization_cpp::GyroRelativeEskfOptions;
using aruco_imu_eskf_localization_cpp::yaw_from_quaternion;

TEST(GyroRelativeEskf, PredictIntegratesGyroYaw)
{
  GyroRelativeEskf filter;
  filter.initialize_position_only(
    0, Eigen::Vector3d::Zero(), 1.0e-3 * Eigen::Matrix3d::Identity());

  filter.predict(500000000LL, Eigen::Vector3d(0.0, 0.0, 0.4));

  const double yaw = yaw_from_quaternion(Eigen::Quaterniond(filter.pose_matrix().linear()));
  EXPECT_NEAR(yaw, 0.2, 0.02);
}

TEST(GyroRelativeEskf, ArucoPositionUpdateDoesNotChangeYaw)
{
  GyroRelativeEskf filter;
  filter.initialize_position_only(
    0, Eigen::Vector3d::Zero(), 0.1 * Eigen::Matrix3d::Identity());
  filter.predict(1000000000LL, Eigen::Vector3d(0.0, 0.0, 0.25));
  const double yaw_before = yaw_from_quaternion(Eigen::Quaterniond(filter.pose_matrix().linear()));

  const auto result = filter.update_position(
    Eigen::Vector3d(1.0, 0.4, 0.0),
    0.05 * Eigen::Matrix3d::Identity(),
    2.0);
  const double yaw_after = yaw_from_quaternion(Eigen::Quaterniond(filter.pose_matrix().linear()));

  EXPECT_TRUE(result.accepted);
  EXPECT_NEAR(yaw_before, yaw_after, 1.0e-12);
  EXPECT_GT(filter.pose_matrix().translation().x(), 0.0);
}

TEST(GyroRelativeEskf, RollPitchAndZRemainConstrained)
{
  GyroRelativeEskf filter;
  filter.initialize_position_only(
    0, Eigen::Vector3d(0.5, -0.2, 1.0), 0.1 * Eigen::Matrix3d::Identity());

  filter.predict(1000000000LL, Eigen::Vector3d(0.7, -0.4, 0.25));
  const auto result = filter.update_position(
    Eigen::Vector3d(1.0, 0.4, 2.0),
    0.05 * Eigen::Matrix3d::Identity(),
    2.0);

  const Eigen::Vector3d z_axis = filter.pose_matrix().linear() * Eigen::Vector3d::UnitZ();
  EXPECT_TRUE(result.accepted);
  EXPECT_NEAR(filter.pose_matrix().translation().z(), 0.0, 1.0e-12);
  EXPECT_NEAR(z_axis.x(), 0.0, 1.0e-12);
  EXPECT_NEAR(z_axis.y(), 0.0, 1.0e-12);
  EXPECT_NEAR(z_axis.z(), 1.0, 1.0e-12);
  EXPECT_NEAR(filter.linear_velocity_base_mps().norm(), 0.0, 1.0e-12);
}

TEST(GyroRelativeEskf, PositionGateRejectsLargeArucoJump)
{
  GyroRelativeEskf filter;
  filter.initialize_position_only(
    0, Eigen::Vector3d::Zero(), 0.1 * Eigen::Matrix3d::Identity());

  const auto result = filter.update_position(
    Eigen::Vector3d(4.0, 0.0, 0.0),
    0.05 * Eigen::Matrix3d::Identity(),
    1.0);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "position_gate");
  EXPECT_NEAR(filter.pose_matrix().translation().x(), 0.0, 1.0e-12);
}
