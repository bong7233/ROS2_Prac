#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "amr_interfaces/msg/docking_state.hpp"
#include "amr_interfaces/msg/io_state.hpp"
#include "amr_interfaces/msg/motor_state.hpp"
#include "amr_interfaces/msg/robot_state.hpp"
#include "amr_interfaces/msg/safety_state.hpp"
#include "amr_interfaces/srv/set_mode.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace amr_system_manager
{
namespace
{
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using RobotState = amr_interfaces::msg::RobotState;

std::string modeName(const uint8_t mode)
{
  switch (mode) {
    case RobotState::MODE_BOOT:
      return "BOOT";
    case RobotState::MODE_INIT:
      return "INIT";
    case RobotState::MODE_MANUAL:
      return "MANUAL";
    case RobotState::MODE_AUTO_READY:
      return "AUTO_READY";
    case RobotState::MODE_AUTO_RUNNING:
      return "AUTO_RUNNING";
    case RobotState::MODE_PAUSED:
      return "PAUSED";
    case RobotState::MODE_CHARGING:
      return "CHARGING";
    case RobotState::MODE_FAULT:
      return "FAULT";
    case RobotState::MODE_ESTOP:
      return "ESTOP";
    default:
      return "UNKNOWN";
  }
}

bool validRequestedMode(const uint8_t mode)
{
  return mode == RobotState::MODE_MANUAL ||
         mode == RobotState::MODE_AUTO_READY ||
         mode == RobotState::MODE_PAUSED ||
         mode == RobotState::MODE_CHARGING;
}
}  // namespace

class SystemManagerNode final : public rclcpp::Node
{
public:
  SystemManagerNode()
  : Node("system_manager"),
    updater_(this)
  {
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);
    low_battery_percentage_ = declare_parameter<double>("low_battery_percentage", 0.20);
    critical_battery_percentage_ = declare_parameter<double>("critical_battery_percentage", 0.10);
    requested_mode_ = static_cast<uint8_t>(
      declare_parameter<int>("initial_mode", RobotState::MODE_MANUAL));
    docking_state_timeout_ms_ = declare_parameter<double>("docking_state_timeout_ms", 1000.0);
    auto_charge_when_docked_ = declare_parameter<bool>("auto_charge_when_docked", true);

    if (publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be positive");
    }

    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      "battery_state",
      rclcpp::QoS(10),
      std::bind(&SystemManagerNode::onBatteryState, this, std::placeholders::_1));
    io_sub_ = create_subscription<amr_interfaces::msg::IoState>(
      "io_state",
      rclcpp::QoS(10),
      std::bind(&SystemManagerNode::onIoState, this, std::placeholders::_1));
    motor_sub_ = create_subscription<amr_interfaces::msg::MotorState>(
      "motor_state",
      rclcpp::QoS(10),
      std::bind(&SystemManagerNode::onMotorState, this, std::placeholders::_1));
    safety_sub_ = create_subscription<amr_interfaces::msg::SafetyState>(
      "safety_state",
      rclcpp::QoS(10),
      std::bind(&SystemManagerNode::onSafetyState, this, std::placeholders::_1));
    docking_sub_ = create_subscription<amr_interfaces::msg::DockingState>(
      "docking_state",
      rclcpp::QoS(10),
      std::bind(&SystemManagerNode::onDockingState, this, std::placeholders::_1));

    robot_state_pub_ =
      create_publisher<amr_interfaces::msg::RobotState>("robot_state", rclcpp::QoS(10));
    set_mode_srv_ = create_service<amr_interfaces::srv::SetMode>(
      "set_mode",
      std::bind(
        &SystemManagerNode::onSetMode,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    reset_fault_srv_ = create_service<std_srvs::srv::Trigger>(
      "reset_fault",
      std::bind(
        &SystemManagerNode::onResetFault,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    updater_.setHardwareID("amr_system");
    updater_.add("system_manager", this, &SystemManagerNode::produceDiagnostics);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = create_wall_timer(period, std::bind(&SystemManagerNode::onTimer, this));

    RCLCPP_INFO(get_logger(), "System manager started in requested mode %s", modeName(requested_mode_).c_str());
  }

private:
  void onBatteryState(const sensor_msgs::msg::BatteryState::SharedPtr msg)
  {
    have_battery_ = true;
    if (std::isfinite(msg->percentage)) {
      battery_percentage_ = msg->percentage;
    }
    battery_low_ = battery_percentage_ <= low_battery_percentage_;
    battery_critical_ = battery_percentage_ <= critical_battery_percentage_;
  }

  void onIoState(const amr_interfaces::msg::IoState::SharedPtr msg)
  {
    have_io_ = true;
    io_communication_ok_ = msg->communication_ok;
    estop_active_ = msg->estop_active;
  }

  void onMotorState(const amr_interfaces::msg::MotorState::SharedPtr msg)
  {
    have_motor_ = true;
    motor_communication_ok_ = msg->communication_ok;
    motor_fault_ = msg->fault_active;
  }

  void onSafetyState(const amr_interfaces::msg::SafetyState::SharedPtr msg)
  {
    have_safety_ = true;
    safety_communication_fault_ = msg->communication_fault;
    safety_reason_ = msg->active_reason;
  }

  void onDockingState(const amr_interfaces::msg::DockingState::SharedPtr msg)
  {
    have_docking_ = true;
    last_docking_time_ = now();
    docking_aligned_ = msg->detected && msg->aligned;
  }

  void onSetMode(
    const std::shared_ptr<amr_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<amr_interfaces::srv::SetMode::Response> response)
  {
    if (!validRequestedMode(request->mode)) {
      response->success = false;
      response->message = "requested mode is not operator-selectable";
      return;
    }
    if (estop_active_) {
      response->success = false;
      response->message = "cannot change mode while estop is active";
      return;
    }
    if (motor_fault_ || battery_critical_ || safety_communication_fault_) {
      response->success = false;
      response->message = "cannot change mode while fault is active";
      return;
    }

    requested_mode_ = request->mode;
    auto_charging_ = false;  // operator override; don't auto-revert on undock
    response->success = true;
    response->message = "requested mode set to " + modeName(requested_mode_);
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void onResetFault(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (estop_active_) {
      response->success = false;
      response->message = "estop is still active";
      return;
    }
    if (battery_critical_) {
      response->success = false;
      response->message = "battery is still critical";
      return;
    }

    requested_mode_ = RobotState::MODE_MANUAL;
    response->success = true;
    response->message =
      "software fault reset accepted; clear device faults through each driver if needed";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void onTimer()
  {
    const rclcpp::Time stamp = now();
    docked_ = computeDocked(stamp);
    updateAutoCharging();

    auto msg = amr_interfaces::msg::RobotState();
    msg.header.stamp = stamp;
    msg.header.frame_id = "base_link";
    msg.mode = effectiveMode();
    msg.estop_active = estop_active_;
    msg.battery_low = battery_low_;
    msg.battery_critical = battery_critical_;
    msg.motor_fault = motor_fault_;
    msg.communication_fault = communicationFault();
    msg.fault_active = msg.mode == RobotState::MODE_FAULT || msg.mode == RobotState::MODE_ESTOP;
    msg.docked = docked_;
    msg.message = buildStateMessage(msg.mode);
    robot_state_pub_->publish(msg);
    last_mode_ = msg.mode;

    updater_.force_update();
  }

  bool computeDocked(const rclcpp::Time & stamp) const
  {
    if (!have_docking_) {
      return false;
    }
    if ((stamp - last_docking_time_).seconds() * 1000.0 > docking_state_timeout_ms_) {
      return false;
    }
    return docking_aligned_;
  }

  // Edge-triggered: enter CHARGING when the robot docks and return to MANUAL when
  // it undocks, without fighting an operator who changes mode in between.
  void updateAutoCharging()
  {
    if (auto_charge_when_docked_ && !estop_active_ && !battery_critical_ && !motor_fault_) {
      if (docked_ && !prev_docked_) {
        requested_mode_ = RobotState::MODE_CHARGING;
        auto_charging_ = true;
        RCLCPP_INFO(get_logger(), "Docked: switching to CHARGING");
      } else if (!docked_ && prev_docked_ && auto_charging_) {
        requested_mode_ = RobotState::MODE_MANUAL;
        auto_charging_ = false;
        RCLCPP_INFO(get_logger(), "Undocked: returning to MANUAL");
      }
    }
    prev_docked_ = docked_;
  }

  uint8_t effectiveMode() const
  {
    if (estop_active_) {
      return RobotState::MODE_ESTOP;
    }
    if (battery_critical_ || motor_fault_ || communicationFault()) {
      return RobotState::MODE_FAULT;
    }
    return requested_mode_;
  }

  bool communicationFault() const
  {
    if (!have_battery_ || !have_io_ || !have_motor_ || !have_safety_) {
      return true;
    }
    return !io_communication_ok_ || !motor_communication_ok_ || safety_communication_fault_;
  }

  std::string buildStateMessage(const uint8_t mode) const
  {
    if (mode == RobotState::MODE_ESTOP) {
      return "estop active";
    }
    if (mode == RobotState::MODE_FAULT) {
      if (battery_critical_) {
        return "battery critical";
      }
      if (motor_fault_) {
        return "motor fault";
      }
      if (communicationFault()) {
        return "communication fault: " + safety_reason_;
      }
      return "fault active";
    }
    std::string text = battery_low_
      ? modeName(mode) + " with low battery"
      : modeName(mode);
    if (docked_) {
      text += " (docked)";
    }
    return text;
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    if (last_mode_ == RobotState::MODE_ESTOP || last_mode_ == RobotState::MODE_FAULT) {
      status.summary(DiagnosticStatus::ERROR, buildStateMessage(last_mode_));
    } else if (battery_low_) {
      status.summary(DiagnosticStatus::WARN, "battery is low");
    } else {
      status.summary(DiagnosticStatus::OK, "system state is healthy");
    }

    status.add("mode", modeName(last_mode_));
    status.add("requested_mode", modeName(requested_mode_));
    status.add("battery_percentage", battery_percentage_);
    status.add("battery_low", battery_low_);
    status.add("battery_critical", battery_critical_);
    status.add("estop_active", estop_active_);
    status.add("motor_fault", motor_fault_);
    status.add("communication_fault", communicationFault());
    status.add("docked", docked_);
    status.add("safety_reason", safety_reason_);
  }

  double publish_rate_hz_{5.0};
  double low_battery_percentage_{0.20};
  double critical_battery_percentage_{0.10};
  double docking_state_timeout_ms_{1000.0};
  uint8_t requested_mode_{RobotState::MODE_MANUAL};
  uint8_t last_mode_{RobotState::MODE_BOOT};

  bool have_battery_{false};
  bool have_io_{false};
  bool have_motor_{false};
  bool have_safety_{false};
  double battery_percentage_{1.0};
  bool battery_low_{false};
  bool battery_critical_{false};
  bool io_communication_ok_{false};
  bool motor_communication_ok_{false};
  bool safety_communication_fault_{false};
  bool estop_active_{false};
  bool motor_fault_{false};
  bool have_docking_{false};
  bool docking_aligned_{false};
  bool docked_{false};
  bool prev_docked_{false};
  bool auto_charge_when_docked_{true};
  bool auto_charging_{false};
  rclcpp::Time last_docking_time_;
  std::string safety_reason_{"waiting for state"};

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<amr_interfaces::msg::IoState>::SharedPtr io_sub_;
  rclcpp::Subscription<amr_interfaces::msg::MotorState>::SharedPtr motor_sub_;
  rclcpp::Subscription<amr_interfaces::msg::SafetyState>::SharedPtr safety_sub_;
  rclcpp::Subscription<amr_interfaces::msg::DockingState>::SharedPtr docking_sub_;
  rclcpp::Publisher<amr_interfaces::msg::RobotState>::SharedPtr robot_state_pub_;
  rclcpp::Service<amr_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_fault_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater updater_;
};
}  // namespace amr_system_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_system_manager::SystemManagerNode>());
  rclcpp::shutdown();
  return 0;
}
