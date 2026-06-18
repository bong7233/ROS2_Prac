#include "amr_operator_ui/ros_worker.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>

#include <QStringList>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace amr_operator_ui
{
namespace
{
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using SetMode = amr_interfaces::srv::SetMode;
using Trigger = std_srvs::srv::Trigger;

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

QString rosString(const std::string & text)
{
  return QString::fromStdString(text);
}
}  // namespace

RosWorker::RosWorker(QObject * parent)
: QObject(parent)
{
}

RosWorker::~RosWorker()
{
  stop();
}

bool RosWorker::start()
{
  if (running_.load()) {
    return true;
  }

  try {
    node_ = std::make_shared<rclcpp::Node>("amr_operator_ui");
    createRosInterfaces();

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);

    running_ = true;
    spin_thread_ = std::thread(
      [this]() {
        try {
          executor_->spin();
        } catch (const std::exception & error) {
          emit rosLogMessage(QStringLiteral("ROS executor stopped: ") + error.what());
        }
      });

    emit rosLogMessage(QStringLiteral("Connected to ROS graph as /amr_operator_ui"));
    return true;
  } catch (const std::exception & error) {
    emit rosLogMessage(QStringLiteral("Failed to start ROS worker: ") + error.what());
    return false;
  }
}

void RosWorker::stop()
{
  if (!running_.exchange(false)) {
    return;
  }

  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  if (executor_ && node_) {
    executor_->remove_node(node_);
  }

  reset_fault_client_.reset();
  set_mode_client_.reset();
  diagnostics_sub_.reset();
  robot_state_sub_.reset();
  safety_sub_.reset();
  motor_sub_.reset();
  io_sub_.reset();
  battery_sub_.reset();
  odom_sub_.reset();
  cmd_vel_pub_.reset();
  executor_.reset();
  node_.reset();
}

void RosWorker::createRosInterfaces()
{
  const auto qos = rclcpp::QoS(10);
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom", qos, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { onOdom(msg); });
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "battery_state", qos,
    [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) { onBatteryState(msg); });
  io_sub_ = node_->create_subscription<amr_interfaces::msg::IoState>(
    "io_state", qos,
    [this](const amr_interfaces::msg::IoState::SharedPtr msg) { onIoState(msg); });
  motor_sub_ = node_->create_subscription<amr_interfaces::msg::MotorState>(
    "motor_state", qos,
    [this](const amr_interfaces::msg::MotorState::SharedPtr msg) { onMotorState(msg); });
  safety_sub_ = node_->create_subscription<amr_interfaces::msg::SafetyState>(
    "safety_state", qos,
    [this](const amr_interfaces::msg::SafetyState::SharedPtr msg) { onSafetyState(msg); });
  robot_state_sub_ = node_->create_subscription<amr_interfaces::msg::RobotState>(
    "robot_state", qos,
    [this](const amr_interfaces::msg::RobotState::SharedPtr msg) { onRobotState(msg); });
  diagnostics_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "diagnostics", qos,
    [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) { onDiagnostics(msg); });

  set_mode_client_ = node_->create_client<SetMode>("set_mode");
  reset_fault_client_ = node_->create_client<Trigger>("reset_fault");
}

void RosWorker::publishVelocity(const double linear_mps, const double angular_radps)
{
  if (!cmd_vel_pub_) {
    emit serviceResult(
      QStringLiteral("cmd_vel"), false, QStringLiteral("ROS publisher is not ready"));
    return;
  }

  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = linear_mps;
  msg.angular.z = angular_radps;
  cmd_vel_pub_->publish(msg);
}

void RosWorker::requestManualMode()
{
  if (!set_mode_client_ || !set_mode_client_->service_is_ready()) {
    emit serviceResult(
      QStringLiteral("set_mode"), false, QStringLiteral("/set_mode service is not available"));
    return;
  }

  auto request = std::make_shared<SetMode::Request>();
  request->mode = amr_interfaces::msg::RobotState::MODE_MANUAL;
  set_mode_client_->async_send_request(
    request,
    [this](rclcpp::Client<SetMode>::SharedFuture future) {
      const auto response = future.get();
      emit serviceResult(
        QStringLiteral("set_mode"),
        response->success,
        rosString(response->message));
    });
}

void RosWorker::requestResetFault()
{
  if (!reset_fault_client_ || !reset_fault_client_->service_is_ready()) {
    emit serviceResult(
      QStringLiteral("reset_fault"),
      false,
      QStringLiteral("/reset_fault service is not available"));
    return;
  }

  auto request = std::make_shared<Trigger::Request>();
  reset_fault_client_->async_send_request(
    request,
    [this](rclcpp::Client<Trigger>::SharedFuture future) {
      const auto response = future.get();
      emit serviceResult(
        QStringLiteral("reset_fault"),
        response->success,
        rosString(response->message));
    });
}

void RosWorker::emitState()
{
  RobotUiState copy;
  {
    const std::lock_guard<std::mutex> lock(state_mutex_);
    copy = state_;
  }
  emit stateUpdated(copy);
}

void RosWorker::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  {
    const std::lock_guard<std::mutex> lock(state_mutex_);
    state_.have_odom = true;
    state_.x_m = msg->pose.pose.position.x;
    state_.y_m = msg->pose.pose.position.y;
    state_.yaw_rad = yawFromQuaternion(msg->pose.pose.orientation);
  }
  emitState();
}

void RosWorker::onBatteryState(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  {
    const std::lock_guard<std::mutex> lock(state_mutex_);
    state_.have_battery = true;
    if (std::isfinite(msg->percentage)) {
      state_.battery_percentage = msg->percentage;
    }
    if (std::isfinite(msg->voltage)) {
      state_.voltage_v = msg->voltage;
    }
    if (std::isfinite(msg->current)) {
      state_.current_a = msg->current;
    }
  }
  emitState();
}

void RosWorker::onIoState(const amr_interfaces::msg::IoState::SharedPtr msg)
{
  {
    const std::lock_guard<std::mutex> lock(state_mutex_);
    state_.have_io = true;
    state_.io_communication_ok = msg->communication_ok;
    state_.estop_active = msg->estop_active;
    state_.protective_stop_active = msg->protective_stop_active;
    state_.input_count = static_cast<int>(msg->inputs.size());
    state_.active_input_count = static_cast<int>(std::count(msg->inputs.begin(), msg->inputs.end(), true));
    state_.output_count = static_cast<int>(msg->outputs.size());
    state_.active_output_count = static_cast<int>(std::count(msg->outputs.begin(), msg->outputs.end(), true));
  }
  emitState();
}

void RosWorker::onMotorState(const amr_interfaces::msg::MotorState::SharedPtr msg)
{
  {
    const std::lock_guard<std::mutex> lock(state_mutex_);
    state_.have_motor = true;
    state_.motor_communication_ok = msg->communication_ok;
    state_.motor_enabled = msg->drive_enabled;
    state_.motor_fault = msg->fault_active;
    state_.motor_fault_code = msg->fault_code;
    state_.left_wheel_velocity_radps = msg->left_wheel_velocity_radps;
    state_.right_wheel_velocity_radps = msg->right_wheel_velocity_radps;
  }
  emitState();
}

void RosWorker::onSafetyState(const amr_interfaces::msg::SafetyState::SharedPtr msg)
{
  {
    const std::lock_guard<std::mutex> lock(state_mutex_);
    state_.have_safety = true;
    state_.command_allowed = msg->command_allowed;
    state_.estop_active = msg->estop_active;
    state_.protective_stop_active = msg->protective_stop_active;
    state_.motor_fault = msg->motor_fault;
    state_.battery_critical = msg->battery_critical;
    state_.communication_fault = msg->communication_fault;
    state_.safety_reason = rosString(msg->active_reason);
  }
  emitState();
}

void RosWorker::onRobotState(const amr_interfaces::msg::RobotState::SharedPtr msg)
{
  {
    const std::lock_guard<std::mutex> lock(state_mutex_);
    state_.have_robot_state = true;
    state_.mode = msg->mode;
    state_.fault_active = msg->fault_active;
    state_.estop_active = msg->estop_active;
    state_.battery_low = msg->battery_low;
    state_.battery_critical = msg->battery_critical;
    state_.motor_fault = msg->motor_fault;
    state_.communication_fault = msg->communication_fault;
    state_.robot_message = rosString(msg->message);
  }
  emitState();
}

void RosWorker::onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  QStringList active_items;
  int worst_level = DiagnosticStatus::OK;

  for (const auto & status : msg->status) {
    worst_level = std::max(worst_level, static_cast<int>(status.level));
    if (status.level != DiagnosticStatus::OK) {
      active_items << QStringLiteral("%1: %2")
                        .arg(rosString(status.name), rosString(status.message));
    }
  }

  {
    const std::lock_guard<std::mutex> lock(state_mutex_);
    state_.have_diagnostics = true;
    state_.diagnostics_level = worst_level;
    state_.diagnostics_summary =
      active_items.isEmpty() ? QStringLiteral("all diagnostics OK") : active_items.join(QStringLiteral(" | "));
  }
  emitState();
}
}  // namespace amr_operator_ui
