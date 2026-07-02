// Pure differential-drive kinematics and odometry integration.
//
// Header-only and free of ROS types so the math can be unit tested with gtest
// without constructing a node, and so the node has a single source of truth for
// the wheel <-> body-twist conversions and dead-reckoning.
#ifndef AMR_BASE_CONTROLLER__DIFF_DRIVE_KINEMATICS_HPP_
#define AMR_BASE_CONTROLLER__DIFF_DRIVE_KINEMATICS_HPP_

#include <cmath>

namespace amr_base_controller
{

struct WheelVelocities
{
  double left_radps;
  double right_radps;
};

struct BodyTwist
{
  double linear_x;
  double angular_z;
};

struct Pose2D
{
  double x;
  double y;
  double heading;
};

// Wrap an angle to (-pi, pi].
inline double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// Inverse kinematics: body twist -> wheel angular velocities (rad/s).
inline WheelVelocities wheelVelocitiesFromTwist(
  const double linear_x,
  const double angular_z,
  const double wheel_radius_m,
  const double wheel_separation_m)
{
  const double left_linear_mps = linear_x - angular_z * wheel_separation_m * 0.5;
  const double right_linear_mps = linear_x + angular_z * wheel_separation_m * 0.5;
  return WheelVelocities{
    left_linear_mps / wheel_radius_m,
    right_linear_mps / wheel_radius_m};
}

// Forward kinematics: wheel angular velocities (rad/s) -> body twist.
inline BodyTwist bodyTwistFromWheelVelocities(
  const double left_radps,
  const double right_radps,
  const double wheel_radius_m,
  const double wheel_separation_m)
{
  const double left_linear_mps = left_radps * wheel_radius_m;
  const double right_linear_mps = right_radps * wheel_radius_m;
  return BodyTwist{
    (left_linear_mps + right_linear_mps) * 0.5,
    (right_linear_mps - left_linear_mps) / wheel_separation_m};
}

// Midpoint dead-reckoning: advance the pose by a body twist over dt seconds.
inline Pose2D integrateOdometry(
  const Pose2D & pose,
  const double linear_x,
  const double angular_z,
  const double dt)
{
  const double delta_theta = angular_z * dt;
  const double heading_midpoint = pose.heading + delta_theta * 0.5;
  return Pose2D{
    pose.x + linear_x * std::cos(heading_midpoint) * dt,
    pose.y + linear_x * std::sin(heading_midpoint) * dt,
    normalizeAngle(pose.heading + delta_theta)};
}

}  // namespace amr_base_controller

#endif  // AMR_BASE_CONTROLLER__DIFF_DRIVE_KINEMATICS_HPP_
