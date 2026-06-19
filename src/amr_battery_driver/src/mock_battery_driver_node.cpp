#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "amr_interfaces/srv/set_battery_percentage.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace amr_battery_driver
{
namespace
{
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

double clamp01(const double value)
{
  return std::clamp(value, 0.0, 1.0);
}
}  // namespace

class MockBatteryDriverNode final : public rclcpp::Node
{
public:
  MockBatteryDriverNode()
  : Node("mock_battery_driver"),
    updater_(this)
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "battery_link");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 2.0);
    nominal_voltage_v_ = declare_parameter<double>("nominal_voltage_v", 24.0);
    empty_voltage_v_ = declare_parameter<double>("empty_voltage_v", 20.0);
    low_voltage_v_ = declare_parameter<double>("low_voltage_v", 22.0);
    critical_voltage_v_ = declare_parameter<double>("critical_voltage_v", 21.0);
    discharge_rate_vps_ = declare_parameter<double>("discharge_rate_vps", 0.002);
    current_a_ = declare_parameter<double>("current_a", -3.5);
    temperature_c_ = declare_parameter<double>("temperature_c", 32.0);
    hardware_id_ = declare_parameter<std::string>("hardware_id", "mock_bms");

    if (publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be positive");
    }
    if (empty_voltage_v_ >= nominal_voltage_v_) {
      throw std::runtime_error("empty_voltage_v must be lower than nominal_voltage_v");
    }
    if (critical_voltage_v_ >= low_voltage_v_) {
      throw std::runtime_error("critical_voltage_v must be lower than low_voltage_v");
    }

    voltage_v_ = declare_parameter<double>("initial_voltage_v", nominal_voltage_v_);
    voltage_v_ = std::clamp(voltage_v_, empty_voltage_v_, nominal_voltage_v_);

    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("battery_state", rclcpp::QoS(10));
    set_percentage_srv_ = create_service<amr_interfaces::srv::SetBatteryPercentage>(
      "set_battery_percentage",
      std::bind(
        &MockBatteryDriverNode::onSetBatteryPercentage,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    updater_.setHardwareID(hardware_id_);
    updater_.add("battery", this, &MockBatteryDriverNode::produceDiagnostics);

    last_update_time_ = now();
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = create_wall_timer(period, std::bind(&MockBatteryDriverNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "Mock battery driver started: %.2f V nominal, %.2f Hz publish rate",
      nominal_voltage_v_,
      publish_rate_hz_);
  }

private:
  void onTimer()
  {
    const rclcpp::Time stamp = now();
    const double dt = std::max(0.0, (stamp - last_update_time_).seconds());
    last_update_time_ = stamp;

    voltage_v_ = std::max(empty_voltage_v_, voltage_v_ - discharge_rate_vps_ * dt);

    auto msg = sensor_msgs::msg::BatteryState();
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.voltage = static_cast<float>(voltage_v_);
    msg.current = static_cast<float>(current_a_);
    msg.charge = std::numeric_limits<float>::quiet_NaN();
    msg.capacity = std::numeric_limits<float>::quiet_NaN();
    msg.design_capacity = std::numeric_limits<float>::quiet_NaN();
    msg.percentage = static_cast<float>(
      clamp01((voltage_v_ - empty_voltage_v_) / (nominal_voltage_v_ - empty_voltage_v_)));
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    msg.power_supply_health = voltage_v_ <= critical_voltage_v_
      ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD
      : sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    msg.power_supply_technology =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    msg.present = true;
    msg.location = "robot_base";
    msg.serial_number = hardware_id_;
    msg.temperature = static_cast<float>(temperature_c_);

    battery_pub_->publish(msg);
    updater_.force_update();
  }

  void onSetBatteryPercentage(
    const std::shared_ptr<amr_interfaces::srv::SetBatteryPercentage::Request> request,
    std::shared_ptr<amr_interfaces::srv::SetBatteryPercentage::Response> response)
  {
    if (request->percentage < 0.0 || request->percentage > 1.0) {
      response->success = false;
      response->message = "percentage must be in the range [0.0, 1.0]";
      return;
    }

    voltage_v_ = empty_voltage_v_ + request->percentage * (nominal_voltage_v_ - empty_voltage_v_);
    response->success = true;
    response->message = "battery percentage set to " + std::to_string(request->percentage);
    RCLCPP_WARN(
      get_logger(),
      "FAE scenario: battery percentage forced to %.1f%%",
      request->percentage * 100.0);
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    if (voltage_v_ <= critical_voltage_v_) {
      status.summary(DiagnosticStatus::ERROR, "battery voltage is critical");
    } else if (voltage_v_ <= low_voltage_v_) {
      status.summary(DiagnosticStatus::WARN, "battery voltage is low");
    } else {
      status.summary(DiagnosticStatus::OK, "battery is healthy");
    }

    status.add("voltage_v", voltage_v_);
    status.add("current_a", current_a_);
    status.add("temperature_c", temperature_c_);
    status.add("percentage", clamp01((voltage_v_ - empty_voltage_v_) / (nominal_voltage_v_ - empty_voltage_v_)));
    status.add("nominal_voltage_v", nominal_voltage_v_);
    status.add("low_voltage_v", low_voltage_v_);
    status.add("critical_voltage_v", critical_voltage_v_);
    status.add("driver_type", "mock");
  }

  std::string frame_id_;
  std::string hardware_id_;
  double publish_rate_hz_{2.0};
  double nominal_voltage_v_{24.0};
  double empty_voltage_v_{20.0};
  double low_voltage_v_{22.0};
  double critical_voltage_v_{21.0};
  double discharge_rate_vps_{0.002};
  double current_a_{-3.5};
  double temperature_c_{32.0};
  double voltage_v_{24.0};
  rclcpp::Time last_update_time_;

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Service<amr_interfaces::srv::SetBatteryPercentage>::SharedPtr set_percentage_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater updater_;
};
}  // namespace amr_battery_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_battery_driver::MockBatteryDriverNode>());
  rclcpp::shutdown();
  return 0;
}
