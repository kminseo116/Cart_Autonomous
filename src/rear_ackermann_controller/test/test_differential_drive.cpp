#include <gtest/gtest.h>

#include "rear_ackermann_controller/differential_drive.hpp"

TEST(DifferentialDrive, StraightHasEqualWheelSpeeds)
{
  const auto speeds = rear_ackermann_controller::compute_wheel_speeds(
    0.2, 0.0, rear_ackermann_controller::VehicleParams{});
  EXPECT_DOUBLE_EQ(speeds.left_mps, 0.2);
  EXPECT_DOUBLE_EQ(speeds.right_mps, 0.2);
}

TEST(DifferentialDrive, InPlaceRotationHasOppositeWheelSpeeds)
{
  const auto speeds = rear_ackermann_controller::compute_wheel_speeds(
    0.0, 0.25, rear_ackermann_controller::VehicleParams{});
  EXPECT_DOUBLE_EQ(speeds.left_mps, -0.0625);
  EXPECT_DOUBLE_EQ(speeds.right_mps, 0.0625);
}

TEST(DifferentialDrive, WheelSpeedLimitPreservesCurvature)
{
  auto params = rear_ackermann_controller::VehicleParams{};
  params.rear_track_m = 0.5;
  params.max_wheel_speed_mps = 0.2;
  params.max_yaw_rate_rad_s = 0.5;
  const auto speeds = rear_ackermann_controller::compute_wheel_speeds(0.2, 0.2, params);

  EXPECT_NEAR(speeds.right_mps, 0.2, 1e-12);
  EXPECT_NEAR(speeds.left_mps / speeds.right_mps, 0.6, 1e-12);
}
