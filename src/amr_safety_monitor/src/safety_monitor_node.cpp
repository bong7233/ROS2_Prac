#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "amr_interfaces/msg/io_state.hpp"
#include "amr_interfaces/msg/motor_state.hpp"
#include "amr_interfaces/msg/safety_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace amr_safety_monitor
{
namespace
{
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

std::string joinReasons(const std::vector<std::string> & reasons)
{
  if (reasons.empty()) {
    return "clear";
  }

  std::ostringstream oss;
  for (std::size_t i = 0; i < reasons.size(); ++i) {
    if (i > 0U) {
      oss << ", ";
    }
    oss << reasons.at(i);
  }
  return oss.str();
}
}  // namespace

class SafetyMonitorNode final : public rclcpp::Node
{
public:
  SafetyMonitorNode()
  : Node("safety_monitor"),
    updater_(this)
  {
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    command_timeout_ms_ = declare_parameter<double>("command_timeout_ms", 300.0);
    state_timeout_ms_ = declare_parameter<double>("state_timeout_ms", 1000.0);
    low_battery_percentage_ = declare_parameter<double>("low_battery_percentage", 0.20);
    critical_battery_percentage_ = declare_parameter<double>("critical_battery_percentage", 0.10);
    require_battery_state_ = declare_parameter<bool>("require_battery_state", true);
    require_io_state_ = declare_parameter<bool>("require_io_state", true);
    require_motor_state_ = declare_parameter<bool>("require_motor_state", true);
    require_motor_enabled_ = declare_parameter<bool>("require_motor_enabled", true);

    if (publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be positive");
    }
    if (command_timeout_ms_ <= 0.0 || state_timeout_ms_ <= 0.0) {
      throw std::runtime_error("timeout parameters must be positive");
    }
    if (critical_battery_percentage_ >= low_battery_percentage_) {
      throw std::runtime_error("critical_battery_percentage must be lower than low_battery_percentage");
    }

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      rclcpp::QoS(10),
      std::bind(&SafetyMonitorNode::onCmdVel, this, std::placeholders::_1));
    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      "battery_state",
      rclcpp::QoS(10),
      std::bind(&SafetyMonitorNode::onBatteryState, this, std::placeholders::_1));
    io_sub_ = create_subscription<amr_interfaces::msg::IoState>(
      "io_state",
      rclcpp::QoS(10),
      std::bind(&SafetyMonitorNode::onIoState, this, std::placeholders::_1));
    motor_sub_ = create_subscription<amr_interfaces::msg::MotorState>(
      "motor_state",
      rclcpp::QoS(10),
      std::bind(&SafetyMonitorNode::onMotorState, this, std::placeholders::_1));

    safe_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_safe", rclcpp::QoS(10));
    safety_state_pub_ =
      create_publisher<amr_interfaces::msg::SafetyState>("safety_state", rclcpp::QoS(10));

    updater_.setHardwareID("software_safety_monitor");
    updater_.add("safety_monitor", this, &SafetyMonitorNode::produceDiagnostics);

    last_cmd_time_ = now();
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = create_wall_timer(period, std::bind(&SafetyMonitorNode::onTimer, this));

    RCLCPP_INFO(get_logger(), "Safety monitor started");
  }

private:
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_ = *msg;
    last_cmd_time_ = now();
  }

  void onBatteryState(const sensor_msgs::msg::BatteryState::SharedPtr msg)
  {
    have_battery_state_ = true;
    last_battery_time_ = now();

    if (std::isfinite(msg->percentage)) {
      battery_percentage_ = msg->percentage;
    }
    battery_low_ = battery_percentage_ <= low_battery_percentage_;
    battery_critical_ = battery_percentage_ <= critical_battery_percentage_;
  }

  void onIoState(const amr_interfaces::msg::IoState::SharedPtr msg)
  {
    have_io_state_ = true;
    last_io_time_ = now();
    io_communication_ok_ = msg->communication_ok;
    estop_active_ = msg->estop_active;
    protective_stop_active_ = msg->protective_stop_active;
  }

  void onMotorState(const amr_interfaces::msg::MotorState::SharedPtr msg)
  {
    have_motor_state_ = true;
    last_motor_time_ = now();
    motor_communication_ok_ = msg->communication_ok;
    motor_enabled_ = msg->drive_enabled;
    motor_fault_ = msg->fault_active;
    motor_fault_code_ = msg->fault_code;
  }

  void onTimer()
  {
    const rclcpp::Time stamp = now();
    const Decision decision = evaluateDecision(stamp);

    geometry_msgs::msg::Twist safe_cmd;
    if (decision.command_allowed) {
      safe_cmd = latest_cmd_;
    }
    safe_cmd_pub_->publish(safe_cmd);

    auto state = amr_interfaces::msg::SafetyState();
    state.header.stamp = stamp;
    state.header.frame_id = "base_link";
    state.command_allowed = decision.command_allowed;
    state.estop_active = estop_active_;
    state.protective_stop_active = protective_stop_active_;
    state.motor_fault = motor_fault_;
    state.battery_critical = battery_critical_;
    state.command_timeout = decision.command_timeout;
    state.communication_fault = decision.communication_fault;
    state.active_reason = decision.reason;
    safety_state_pub_->publish(state);

    last_decision_ = decision;
    updater_.force_update();
  }

  struct Decision
  {
    bool command_allowed{false};
    bool command_timeout{true};
    bool communication_fault{false};
    std::string reason{"not evaluated"};
  };

  Decision evaluateDecision(const rclcpp::Time & stamp) const
  {
    auto decision = Decision();
    std::vector<std::string> reasons;

    decision.command_timeout = commandAgeMs(stamp) > command_timeout_ms_;
    if (decision.command_timeout) {
      reasons.push_back("cmd_vel timeout");
    }
    if (estop_active_) {
      reasons.push_back("estop active");
    }
    if (protective_stop_active_) {
      reasons.push_back("protective stop active");
    }
    if (battery_critical_) {
      reasons.push_back("battery critical");
    }
    if (motor_fault_) {
      reasons.push_back("motor fault " + std::to_string(motor_fault_code_));
    }
    if (require_motor_enabled_ && !motor_enabled_) {
      reasons.push_back("motor disabled");
    }

    decision.communication_fault = communicationFault(stamp);
    if (decision.communication_fault) {
      reasons.push_back("state communication timeout");
    }

    decision.command_allowed = reasons.empty();
    decision.reason = joinReasons(reasons);
    return decision;
  }

  bool communicationFault(const rclcpp::Time & stamp) const
  {
    if (require_battery_state_ && isStateStale(have_battery_state_, last_battery_time_, stamp)) {
      return true;
    }
    if (require_io_state_ && isStateStale(have_io_state_, last_io_time_, stamp)) {
      return true;
    }
    if (require_motor_state_ && isStateStale(have_motor_state_, last_motor_time_, stamp)) {
      return true;
    }
    if (have_io_state_ && !io_communication_ok_) {
      return true;
    }
    if (have_motor_state_ && !motor_communication_ok_) {
      return true;
    }
    return false;
  }

  bool isStateStale(
    const bool have_state,
    const rclcpp::Time & last_state_time,
    const rclcpp::Time & stamp) const
  {
    if (!have_state) {
      return true;
    }
    return (stamp - last_state_time).seconds() * 1000.0 > state_timeout_ms_;
  }

  double commandAgeMs(const rclcpp::Time & stamp) const
  {
    return (stamp - last_cmd_time_).seconds() * 1000.0;
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    if (estop_active_ || battery_critical_ || motor_fault_ || last_decision_.communication_fault) {
      status.summary(DiagnosticStatus::ERROR, last_decision_.reason);
    } else if (protective_stop_active_ || last_decision_.command_timeout || !motor_enabled_) {
      status.summary(DiagnosticStatus::WARN, last_decision_.reason);
    } else {
      status.summary(DiagnosticStatus::OK, "safety clear");
    }

    status.add("command_allowed", last_decision_.command_allowed);
    status.add("reason", last_decision_.reason);
    status.add("cmd_age_ms", commandAgeMs(now()));
    status.add("battery_percentage", battery_percentage_);
    status.add("battery_low", battery_low_);
    status.add("battery_critical", battery_critical_);
    status.add("estop_active", estop_active_);
    status.add("protective_stop_active", protective_stop_active_);
    status.add("motor_enabled", motor_enabled_);
    status.add("motor_fault", motor_fault_);
    status.add("communication_fault", last_decision_.communication_fault);
  }

  double publish_rate_hz_{50.0};
  double command_timeout_ms_{300.0};
  double state_timeout_ms_{1000.0};
  double low_battery_percentage_{0.20};
  double critical_battery_percentage_{0.10};
  bool require_battery_state_{true};
  bool require_io_state_{true};
  bool require_motor_state_{true};
  bool require_motor_enabled_{true};

  geometry_msgs::msg::Twist latest_cmd_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_battery_time_;
  rclcpp::Time last_io_time_;
  rclcpp::Time last_motor_time_;
  bool have_battery_state_{false};
  bool have_io_state_{false};
  bool have_motor_state_{false};

  double battery_percentage_{1.0};
  bool battery_low_{false};
  bool battery_critical_{false};
  bool io_communication_ok_{false};
  bool motor_communication_ok_{false};
  bool estop_active_{false};
  bool protective_stop_active_{false};
  bool motor_enabled_{false};
  bool motor_fault_{false};
  uint32_t motor_fault_code_{0U};
  Decision last_decision_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<amr_interfaces::msg::IoState>::SharedPtr io_sub_;
  rclcpp::Subscription<amr_interfaces::msg::MotorState>::SharedPtr motor_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_pub_;
  rclcpp::Publisher<amr_interfaces::msg::SafetyState>::SharedPtr safety_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater updater_;
};
}  // namespace amr_safety_monitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_safety_monitor::SafetyMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
