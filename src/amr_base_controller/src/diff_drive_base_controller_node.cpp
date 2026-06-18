#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "amr_interfaces/msg/motor_state.hpp"
#include "amr_interfaces/msg/wheel_command.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace amr_base_controller
{
namespace
{
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

double clampSymmetric(const double value, const double limit)
{
  return std::clamp(value, -limit, limit);
}
}  // namespace

class DiffDriveBaseControllerNode final : public rclcpp::Node
{
public:
  DiffDriveBaseControllerNode()
  : Node("diff_drive_base_controller"),
    updater_(this)
  {
    wheel_radius_m_ = declare_parameter<double>("wheel_radius_m", 0.1);
    wheel_separation_m_ = declare_parameter<double>("wheel_separation_m", 0.55);
    command_rate_hz_ = declare_parameter<double>("command_rate_hz", 50.0);
    command_timeout_ms_ = declare_parameter<double>("command_timeout_ms", 250.0);
    max_linear_velocity_mps_ = declare_parameter<double>("max_linear_velocity_mps", 1.0);
    max_angular_velocity_radps_ = declare_parameter<double>("max_angular_velocity_radps", 1.5);
    max_wheel_accel_radps2_ = declare_parameter<double>("max_wheel_accel_radps2", 40.0);
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    if (wheel_radius_m_ <= 0.0) {
      throw std::runtime_error("wheel_radius_m must be positive");
    }
    if (wheel_separation_m_ <= 0.0) {
      throw std::runtime_error("wheel_separation_m must be positive");
    }
    if (command_rate_hz_ <= 0.0) {
      throw std::runtime_error("command_rate_hz must be positive");
    }

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_safe",
      rclcpp::QoS(10),
      std::bind(&DiffDriveBaseControllerNode::onCmdVel, this, std::placeholders::_1));
    motor_state_sub_ = create_subscription<amr_interfaces::msg::MotorState>(
      "motor_state",
      rclcpp::QoS(10),
      std::bind(&DiffDriveBaseControllerNode::onMotorState, this, std::placeholders::_1));
    wheel_cmd_pub_ =
      create_publisher<amr_interfaces::msg::WheelCommand>("wheel_command", rclcpp::QoS(10));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    updater_.setHardwareID("diff_drive_base");
    updater_.add("base_controller", this, &DiffDriveBaseControllerNode::produceDiagnostics);

    last_cmd_time_ = now();
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / command_rate_hz_));
    command_timer_ =
      create_wall_timer(period, std::bind(&DiffDriveBaseControllerNode::publishWheelCommand, this));

    RCLCPP_INFO(
      get_logger(),
      "Diff-drive base controller started: radius %.3f m, separation %.3f m",
      wheel_radius_m_,
      wheel_separation_m_);
  }

private:
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = now();
    latest_cmd_ = *msg;
  }

  void publishWheelCommand()
  {
    const rclcpp::Time stamp = now();
    const bool timed_out = commandAgeMs(stamp) > command_timeout_ms_;

    const double linear_x = timed_out
      ? 0.0
      : clampSymmetric(latest_cmd_.linear.x, max_linear_velocity_mps_);
    const double angular_z = timed_out
      ? 0.0
      : clampSymmetric(latest_cmd_.angular.z, max_angular_velocity_radps_);

    const double left_linear_mps = linear_x - angular_z * wheel_separation_m_ * 0.5;
    const double right_linear_mps = linear_x + angular_z * wheel_separation_m_ * 0.5;

    auto msg = amr_interfaces::msg::WheelCommand();
    msg.header.stamp = stamp;
    msg.header.frame_id = base_frame_id_;
    msg.left_velocity_radps = left_linear_mps / wheel_radius_m_;
    msg.right_velocity_radps = right_linear_mps / wheel_radius_m_;
    msg.max_accel_radps2 = max_wheel_accel_radps2_;
    wheel_cmd_pub_->publish(msg);

    updater_.update();
  }

  void onMotorState(const amr_interfaces::msg::MotorState::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp);
    last_motor_state_time_ = now();
    have_motor_state_ = true;

    const double left_linear_mps = msg->left_wheel_velocity_radps * wheel_radius_m_;
    const double right_linear_mps = msg->right_wheel_velocity_radps * wheel_radius_m_;
    const double linear_x = (left_linear_mps + right_linear_mps) * 0.5;
    const double angular_z = (right_linear_mps - left_linear_mps) / wheel_separation_m_;

    if (!have_odom_stamp_) {
      last_odom_stamp_ = stamp;
      have_odom_stamp_ = true;
      publishOdometry(stamp, linear_x, angular_z);
      return;
    }

    const double dt = (stamp - last_odom_stamp_).seconds();
    last_odom_stamp_ = stamp;
    if (dt <= 0.0 || dt > 1.0) {
      publishOdometry(stamp, linear_x, angular_z);
      return;
    }

    const double delta_theta = angular_z * dt;
    const double heading_midpoint = heading_rad_ + delta_theta * 0.5;
    x_m_ += linear_x * std::cos(heading_midpoint) * dt;
    y_m_ += linear_x * std::sin(heading_midpoint) * dt;
    heading_rad_ = normalizeAngle(heading_rad_ + delta_theta);

    publishOdometry(stamp, linear_x, angular_z);
  }

  void publishOdometry(
    const rclcpp::Time & stamp,
    const double linear_x,
    const double angular_z)
  {
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;
    odom.pose.pose.position.x = x_m_;
    odom.pose.pose.position.y = y_m_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.z = std::sin(heading_rad_ * 0.5);
    odom.pose.pose.orientation.w = std::cos(heading_rad_ * 0.5);
    odom.twist.twist.linear.x = linear_x;
    odom.twist.twist.angular.z = angular_z;
    odom.pose.covariance[0] = 0.02;
    odom.pose.covariance[7] = 0.02;
    odom.pose.covariance[35] = 0.05;
    odom.twist.covariance[0] = 0.02;
    odom.twist.covariance[35] = 0.05;
    odom_pub_->publish(odom);

    if (publish_tf_) {
      auto tf = geometry_msgs::msg::TransformStamped();
      tf.header.stamp = stamp;
      tf.header.frame_id = odom_frame_id_;
      tf.child_frame_id = base_frame_id_;
      tf.transform.translation.x = x_m_;
      tf.transform.translation.y = y_m_;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  double commandAgeMs(const rclcpp::Time & stamp) const
  {
    return (stamp - last_cmd_time_).seconds() * 1000.0;
  }

  static double normalizeAngle(const double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    const double cmd_age_ms = commandAgeMs(now());
    const double motor_age_ms = have_motor_state_
      ? (now() - last_motor_state_time_).seconds() * 1000.0
      : -1.0;

    if (!have_motor_state_) {
      status.summary(DiagnosticStatus::WARN, "waiting for motor_state");
    } else if (cmd_age_ms > command_timeout_ms_) {
      status.summary(DiagnosticStatus::WARN, "cmd_vel_safe timeout");
    } else {
      status.summary(DiagnosticStatus::OK, "base controller is healthy");
    }

    status.add("x_m", x_m_);
    status.add("y_m", y_m_);
    status.add("heading_rad", heading_rad_);
    status.add("cmd_age_ms", cmd_age_ms);
    status.add("motor_state_age_ms", motor_age_ms);
    status.add("wheel_radius_m", wheel_radius_m_);
    status.add("wheel_separation_m", wheel_separation_m_);
    status.add("publish_tf", publish_tf_);
  }

  double wheel_radius_m_{0.1};
  double wheel_separation_m_{0.55};
  double command_rate_hz_{50.0};
  double command_timeout_ms_{250.0};
  double max_linear_velocity_mps_{1.0};
  double max_angular_velocity_radps_{1.5};
  double max_wheel_accel_radps2_{40.0};
  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_link"};
  bool publish_tf_{true};

  geometry_msgs::msg::Twist latest_cmd_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_motor_state_time_;
  rclcpp::Time last_odom_stamp_;
  bool have_motor_state_{false};
  bool have_odom_stamp_{false};

  double x_m_{0.0};
  double y_m_{0.0};
  double heading_rad_{0.0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<amr_interfaces::msg::MotorState>::SharedPtr motor_state_sub_;
  rclcpp::Publisher<amr_interfaces::msg::WheelCommand>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  diagnostic_updater::Updater updater_;
};
}  // namespace amr_base_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_base_controller::DiffDriveBaseControllerNode>());
  rclcpp::shutdown();
  return 0;
}
