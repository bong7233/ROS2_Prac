// Unit tests for the pure differential-drive kinematics.
#include <cmath>

#include <gtest/gtest.h>

#include "amr_base_controller/diff_drive_kinematics.hpp"

using amr_base_controller::bodyTwistFromWheelVelocities;
using amr_base_controller::integrateOdometry;
using amr_base_controller::normalizeAngle;
using amr_base_controller::Pose2D;
using amr_base_controller::wheelVelocitiesFromTwist;

TEST(DiffDriveKinematics, StraightDrivesBothWheelsEqually)
{
  const auto w = wheelVelocitiesFromTwist(0.5, 0.0, 0.1, 0.5);
  EXPECT_NEAR(w.left_radps, 5.0, 1e-9);
  EXPECT_NEAR(w.right_radps, 5.0, 1e-9);
}

TEST(DiffDriveKinematics, RotationSpinsWheelsOpposite)
{
  const auto w = wheelVelocitiesFromTwist(0.0, 1.0, 0.1, 0.5);
  EXPECT_NEAR(w.left_radps, -w.right_radps, 1e-9);
  EXPECT_LT(w.left_radps, 0.0);  // spinning left (CCW) drives the left wheel back
}

TEST(DiffDriveKinematics, ForwardInverseRoundTrip)
{
  const double radius = 0.1;
  const double separation = 0.55;
  const auto w = wheelVelocitiesFromTwist(0.3, 0.4, radius, separation);
  const auto t =
    bodyTwistFromWheelVelocities(w.left_radps, w.right_radps, radius, separation);
  EXPECT_NEAR(t.linear_x, 0.3, 1e-9);
  EXPECT_NEAR(t.angular_z, 0.4, 1e-9);
}

TEST(DiffDriveKinematics, IntegrateStraightAlongX)
{
  const auto p = integrateOdometry(Pose2D{0.0, 0.0, 0.0}, 1.0, 0.0, 2.0);
  EXPECT_NEAR(p.x, 2.0, 1e-9);
  EXPECT_NEAR(p.y, 0.0, 1e-9);
  EXPECT_NEAR(p.heading, 0.0, 1e-9);
}

TEST(DiffDriveKinematics, IntegrateStraightAlongYWhenFacingNorth)
{
  const auto p = integrateOdometry(Pose2D{0.0, 0.0, M_PI_2}, 1.0, 0.0, 1.0);
  EXPECT_NEAR(p.x, 0.0, 1e-9);
  EXPECT_NEAR(p.y, 1.0, 1e-9);
}

TEST(DiffDriveKinematics, IntegrateRotationChangesHeadingOnly)
{
  const auto p = integrateOdometry(Pose2D{0.0, 0.0, 0.0}, 0.0, 1.0, 1.0);
  EXPECT_NEAR(p.heading, 1.0, 1e-9);
  EXPECT_NEAR(p.x, 0.0, 1e-9);
  EXPECT_NEAR(p.y, 0.0, 1e-9);
}

TEST(DiffDriveKinematics, NormalizeAngleWraps)
{
  EXPECT_NEAR(normalizeAngle(0.5), 0.5, 1e-9);
  EXPECT_NEAR(std::abs(normalizeAngle(3.0 * M_PI)), M_PI, 1e-9);
}
