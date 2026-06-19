#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "amr_interfaces/srv/inject_motor_fault.hpp"
#include "amr_interfaces/msg/motor_state.hpp"
#include "amr_interfaces/msg/wheel_command.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace amr_motor_driver
{
namespace
{
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
}  // namespace

class MockMotorDriverNode final : public rclcpp::Node
{
public:
  MockMotorDriverNode()
  : Node("mock_motor_driver"),
    updater_(this)
  {
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    command_timeout_ms_ = declare_parameter<double>("command_timeout_ms", 200.0);
    velocity_time_constant_s_ = declare_parameter<double>("velocity_time_constant_s", 0.08);
    max_velocity_radps_ = declare_parameter<double>("max_velocity_radps", 20.0);
    dc_bus_voltage_v_ = declare_parameter<double>("dc_bus_voltage_v", 24.0);
    drive_enabled_ = declare_parameter<bool>("enable_on_start", true);
    hardware_id_ = declare_parameter<std::string>("hardware_id", "mock_motor_drive");

    if (publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be positive");
    }
    if (command_timeout_ms_ <= 0.0) {
      throw std::runtime_error("command_timeout_ms must be positive");
    }
    if (velocity_time_constant_s_ <= 0.0) {
      throw std::runtime_error("velocity_time_constant_s must be positive");
    }
    if (max_velocity_radps_ <= 0.0) {
      throw std::runtime_error("max_velocity_radps must be positive");
    }

    wheel_cmd_sub_ = create_subscription<amr_interfaces::msg::WheelCommand>(
      "wheel_command",
      rclcpp::QoS(10),
      std::bind(&MockMotorDriverNode::onWheelCommand, this, std::placeholders::_1));
    motor_state_pub_ =
      create_publisher<amr_interfaces::msg::MotorState>("motor_state", rclcpp::QoS(10));
    motor_enable_srv_ = create_service<std_srvs::srv::SetBool>(
      "motor_enable",
      std::bind(
        &MockMotorDriverNode::onMotorEnable,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    clear_fault_srv_ = create_service<std_srvs::srv::Trigger>(
      "clear_motor_fault",
      std::bind(
        &MockMotorDriverNode::onClearFault,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    inject_fault_srv_ = create_service<amr_interfaces::srv::InjectMotorFault>(
      "inject_motor_fault",
      std::bind(
        &MockMotorDriverNode::onInjectFault,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    updater_.setHardwareID(hardware_id_);
    updater_.add("motor_drive", this, &MockMotorDriverNode::produceDiagnostics);

    last_update_time_ = now();
    last_command_time_ = now();
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = create_wall_timer(period, std::bind(&MockMotorDriverNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "Mock motor driver started: enabled=%s, max %.2f rad/s",
      drive_enabled_ ? "true" : "false",
      max_velocity_radps_);
  }

private:
  void onWheelCommand(const amr_interfaces::msg::WheelCommand::SharedPtr msg)
  {
    last_command_time_ = now();
    if (!drive_enabled_ || fault_active_) {
      target_left_radps_ = 0.0;
      target_right_radps_ = 0.0;
      return;
    }

    target_left_radps_ = std::clamp(
      msg->left_velocity_radps, -max_velocity_radps_, max_velocity_radps_);
    target_right_radps_ = std::clamp(
      msg->right_velocity_radps, -max_velocity_radps_, max_velocity_radps_);
  }

  void onMotorEnable(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    if (fault_active_ && request->data) {
      response->success = false;
      response->message = "cannot enable motor while fault is active";
      return;
    }

    drive_enabled_ = request->data;
    if (!drive_enabled_) {
      target_left_radps_ = 0.0;
      target_right_radps_ = 0.0;
    }
    response->success = true;
    response->message = drive_enabled_ ? "motor enabled" : "motor disabled";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void onClearFault(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    fault_active_ = false;
    fault_code_ = 0U;
    response->success = true;
    response->message = "motor fault cleared";
    RCLCPP_INFO(get_logger(), "Motor fault cleared");
  }

  void onInjectFault(
    const std::shared_ptr<amr_interfaces::srv::InjectMotorFault::Request> request,
    std::shared_ptr<amr_interfaces::srv::InjectMotorFault::Response> response)
  {
    fault_active_ = true;
    fault_code_ = request->fault_code == 0U ? 1001U : request->fault_code;
    target_left_radps_ = 0.0;
    target_right_radps_ = 0.0;
    drive_enabled_ = false;

    response->success = true;
    response->message = "motor fault injected: " + std::to_string(fault_code_);
    if (!request->description.empty()) {
      response->message += " (" + request->description + ")";
    }

    RCLCPP_ERROR(get_logger(), "FAE scenario: %s", response->message.c_str());
  }

  void onTimer()
  {
    const rclcpp::Time stamp = now();
    const double dt = std::max(0.0, (stamp - last_update_time_).seconds());
    last_update_time_ = stamp;

    const bool timed_out = commandAgeMs(stamp) > command_timeout_ms_;
    if (timed_out || !drive_enabled_ || fault_active_) {
      target_left_radps_ = 0.0;
      target_right_radps_ = 0.0;
    }

    const double alpha = std::clamp(dt / velocity_time_constant_s_, 0.0, 1.0);
    measured_left_radps_ += (target_left_radps_ - measured_left_radps_) * alpha;
    measured_right_radps_ += (target_right_radps_ - measured_right_radps_) * alpha;
    left_position_rad_ += measured_left_radps_ * dt;
    right_position_rad_ += measured_right_radps_ * dt;

    auto msg = amr_interfaces::msg::MotorState();
    msg.header.stamp = stamp;
    msg.header.frame_id = "base_link";
    msg.communication_ok = true;
    msg.drive_enabled = drive_enabled_;
    msg.fault_active = fault_active_;
    msg.fault_code = fault_code_;
    msg.left_wheel_velocity_radps = measured_left_radps_;
    msg.right_wheel_velocity_radps = measured_right_radps_;
    msg.left_wheel_position_rad = left_position_rad_;
    msg.right_wheel_position_rad = right_position_rad_;
    msg.dc_bus_voltage_v = dc_bus_voltage_v_;
    msg.command_age_ms = commandAgeMs(stamp);
    motor_state_pub_->publish(msg);

    updater_.force_update();
  }

  double commandAgeMs(const rclcpp::Time & stamp) const
  {
    return (stamp - last_command_time_).seconds() * 1000.0;
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    const double age_ms = commandAgeMs(now());
    if (fault_active_) {
      status.summary(DiagnosticStatus::ERROR, "motor drive fault is active");
    } else if (!drive_enabled_) {
      status.summary(DiagnosticStatus::WARN, "motor drive is disabled");
    } else if (age_ms > command_timeout_ms_) {
      status.summary(DiagnosticStatus::WARN, "wheel command timeout");
    } else {
      status.summary(DiagnosticStatus::OK, "motor drive is healthy");
    }

    status.add("driver_type", "mock");
    status.add("drive_enabled", drive_enabled_);
    status.add("fault_active", fault_active_);
    status.add("fault_code", fault_code_);
    status.add("left_wheel_velocity_radps", measured_left_radps_);
    status.add("right_wheel_velocity_radps", measured_right_radps_);
    status.add("command_age_ms", age_ms);
    status.add("command_timeout_ms", command_timeout_ms_);
    status.add("dc_bus_voltage_v", dc_bus_voltage_v_);
  }

  std::string hardware_id_;
  double publish_rate_hz_{50.0};
  double command_timeout_ms_{200.0};
  double velocity_time_constant_s_{0.08};
  double max_velocity_radps_{20.0};
  double dc_bus_voltage_v_{24.0};

  bool drive_enabled_{true};
  bool fault_active_{false};
  uint32_t fault_code_{0U};

  double target_left_radps_{0.0};
  double target_right_radps_{0.0};
  double measured_left_radps_{0.0};
  double measured_right_radps_{0.0};
  double left_position_rad_{0.0};
  double right_position_rad_{0.0};

  rclcpp::Time last_update_time_;
  rclcpp::Time last_command_time_;

  rclcpp::Subscription<amr_interfaces::msg::WheelCommand>::SharedPtr wheel_cmd_sub_;
  rclcpp::Publisher<amr_interfaces::msg::MotorState>::SharedPtr motor_state_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr motor_enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_fault_srv_;
  rclcpp::Service<amr_interfaces::srv::InjectMotorFault>::SharedPtr inject_fault_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater updater_;
};
}  // namespace amr_motor_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_motor_driver::MockMotorDriverNode>());
  rclcpp::shutdown();
  return 0;
}
