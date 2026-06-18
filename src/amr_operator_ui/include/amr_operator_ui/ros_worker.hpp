#ifndef AMR_OPERATOR_UI__ROS_WORKER_HPP_
#define AMR_OPERATOR_UI__ROS_WORKER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include <QObject>
#include <QString>

#include "amr_interfaces/msg/io_state.hpp"
#include "amr_interfaces/msg/motor_state.hpp"
#include "amr_interfaces/msg/robot_state.hpp"
#include "amr_interfaces/msg/safety_state.hpp"
#include "amr_interfaces/srv/set_mode.hpp"
#include "amr_operator_ui/robot_state_cache.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace amr_operator_ui
{
class RosWorker final : public QObject
{
  Q_OBJECT

public:
  explicit RosWorker(QObject * parent = nullptr);
  ~RosWorker() override;

  bool start();
  void stop();

public slots:
  void publishVelocity(double linear_mps, double angular_radps);
  void requestManualMode();
  void requestResetFault();

signals:
  void stateUpdated(const amr_operator_ui::RobotUiState & state);
  void rosLogMessage(const QString & message);
  void serviceResult(const QString & service_name, bool success, const QString & message);

private:
  void createRosInterfaces();
  void emitState();

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onBatteryState(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void onIoState(const amr_interfaces::msg::IoState::SharedPtr msg);
  void onMotorState(const amr_interfaces::msg::MotorState::SharedPtr msg);
  void onSafetyState(const amr_interfaces::msg::SafetyState::SharedPtr msg);
  void onRobotState(const amr_interfaces::msg::RobotState::SharedPtr msg);
  void onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic_bool running_{false};

  std::mutex state_mutex_;
  RobotUiState state_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<amr_interfaces::msg::IoState>::SharedPtr io_sub_;
  rclcpp::Subscription<amr_interfaces::msg::MotorState>::SharedPtr motor_sub_;
  rclcpp::Subscription<amr_interfaces::msg::SafetyState>::SharedPtr safety_sub_;
  rclcpp::Subscription<amr_interfaces::msg::RobotState>::SharedPtr robot_state_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::Client<amr_interfaces::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_fault_client_;
};
}  // namespace amr_operator_ui

#endif  // AMR_OPERATOR_UI__ROS_WORKER_HPP_
