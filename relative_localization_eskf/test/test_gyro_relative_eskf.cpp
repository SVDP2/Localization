#include "relative_localization_eskf/gyro_relative_eskf.hpp"
#include "relative_localization_eskf/geometry.hpp"

#include <gtest/gtest.h>

using relative_localization_eskf::GyroRelativeEskf;
using relative_localization_eskf::GyroRelativeEskfOptions;
using relative_localization_eskf::yaw_from_quaternion;


TEST(GyroRelativeEskf, InitializePoseSetsMeasuredYaw)
{
  GyroRelativeEskf filter;
  filter.initialize_pose(
    0, Eigen::Vector3d(0.5, -0.2, 0.0),
    0.1 * Eigen::Matrix3d::Identity(), 0.7, 0.01);

  const double yaw = yaw_from_quaternion(Eigen::Quaterniond(filter.pose_matrix().linear()));

  EXPECT_TRUE(filter.initialized());
  EXPECT_NEAR(filter.pose_matrix().translation().x(), 0.5, 1.0e-12);
  EXPECT_NEAR(filter.pose_matrix().translation().y(), -0.2, 1.0e-12);
  EXPECT_NEAR(yaw, 0.7, 1.0e-12);
}

TEST(GyroRelativeEskf, PositionUpdateUsesMeasurementCovariance)
{
  GyroRelativeEskf filter;
  filter.initialize_position_only(
    0, Eigen::Vector3d::Zero(), 0.01 * Eigen::Matrix3d::Identity());

  const auto weak = filter.update_position(
    Eigen::Vector3d(1.0, 0.0, 0.0),
    1.0 * Eigen::Matrix3d::Identity(),
    2.0);
  const double weak_x = filter.pose_matrix().translation().x();

  filter.reset();
  filter.initialize_position_only(
    0, Eigen::Vector3d::Zero(), 0.01 * Eigen::Matrix3d::Identity());
  const auto strong = filter.update_position(
    Eigen::Vector3d(1.0, 0.0, 0.0),
    0.001 * Eigen::Matrix3d::Identity(),
    2.0);
  const double strong_x = filter.pose_matrix().translation().x();

  EXPECT_TRUE(weak.accepted);
  EXPECT_TRUE(strong.accepted);
  EXPECT_EQ(strong.reason, "position_update");
  EXPECT_LT(weak_x, strong_x);
  EXPECT_GT(strong_x, 0.8);
}

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

TEST(GyroRelativeEskf, YawUpdateCorrectsGyroDrift)
{
  GyroRelativeEskf filter;
  filter.initialize_position_only(
    0, Eigen::Vector3d::Zero(), 0.1 * Eigen::Matrix3d::Identity());
  filter.predict(1000000000LL, Eigen::Vector3d(0.0, 0.0, 0.3));

  const double yaw_before = yaw_from_quaternion(Eigen::Quaterniond(filter.pose_matrix().linear()));
  const auto result = filter.update_yaw(0.1, 0.01, 0.5);
  const double yaw_after = yaw_from_quaternion(Eigen::Quaterniond(filter.pose_matrix().linear()));

  EXPECT_TRUE(result.accepted);
  EXPECT_NEAR(result.yaw_innovation_rad, -0.2, 1.0e-12);
  EXPECT_LT(std::abs(yaw_after - 0.1), std::abs(yaw_before - 0.1));
}

TEST(GyroRelativeEskf, RepeatedYawUpdatesEstimateResidualGyroBias)
{
  GyroRelativeEskfOptions options;
  options.gyro_noise_std_radps = 0.005;
  options.initial_gyro_bias_std_radps = 0.05;
  GyroRelativeEskf filter(options);
  filter.initialize_position_only(
    0, Eigen::Vector3d::Zero(), 0.1 * Eigen::Matrix3d::Identity());

  int64_t stamp_ns = 0;
  for (int i = 0; i < 8; ++i) {
    stamp_ns += 500000000LL;
    filter.predict(stamp_ns, Eigen::Vector3d(0.0, 0.0, 0.04));
    const auto result = filter.update_yaw(0.0, 0.0004, 0.5);
    EXPECT_TRUE(result.accepted);
  }

  EXPECT_GT(filter.residual_gyro_z_bias_radps(), 0.02);
  EXPECT_LT(filter.residual_gyro_z_bias_radps(), 0.06);
  EXPECT_LT(std::abs(filter.angular_velocity_base_radps().z()), 0.03);
}

TEST(GyroRelativeEskf, YawGateRejectsLargeInnovation)
{
  GyroRelativeEskf filter;
  filter.initialize_position_only(
    0, Eigen::Vector3d::Zero(), 0.1 * Eigen::Matrix3d::Identity());
  filter.predict(1000000000LL, Eigen::Vector3d(0.0, 0.0, 0.1));

  const double yaw_before = yaw_from_quaternion(Eigen::Quaterniond(filter.pose_matrix().linear()));
  const auto result = filter.update_yaw(1.0, 0.01, 0.2);
  const double yaw_after = yaw_from_quaternion(Eigen::Quaterniond(filter.pose_matrix().linear()));

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "yaw_gate");
  EXPECT_NEAR(yaw_before, yaw_after, 1.0e-12);
}

TEST(GyroRelativeEskf, YawUpdateWrapsInnovationAcrossPi)
{
  GyroRelativeEskf filter;
  filter.initialize_position_only(
    0, Eigen::Vector3d::Zero(), 0.1 * Eigen::Matrix3d::Identity());
  filter.predict(1000000000LL, Eigen::Vector3d(0.0, 0.0, 3.13));

  const auto result = filter.update_yaw(-3.13, 0.01, 0.1);

  EXPECT_TRUE(result.accepted);
  EXPECT_NEAR(result.yaw_innovation_rad, 0.0231853071795864, 1.0e-9);
}
