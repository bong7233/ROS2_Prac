#ifndef AMR_OPERATOR_UI__ROBOT_STATE_CACHE_HPP_
#define AMR_OPERATOR_UI__ROBOT_STATE_CACHE_HPP_

#include <cstdint>

#include <QMetaType>
#include <QString>

#include "amr_interfaces/msg/robot_state.hpp"

namespace amr_operator_ui
{
struct RobotUiState
{
  bool have_odom{false};
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};

  bool have_battery{false};
  double battery_percentage{0.0};
  double voltage_v{0.0};
  double current_a{0.0};

  bool have_io{false};
  bool io_communication_ok{false};
  bool estop_active{false};
  bool protective_stop_active{false};
  int input_count{0};
  int active_input_count{0};
  int output_count{0};
  int active_output_count{0};

  bool have_motor{false};
  bool motor_communication_ok{false};
  bool motor_enabled{false};
  bool motor_fault{false};
  uint32_t motor_fault_code{0};
  double left_wheel_velocity_radps{0.0};
  double right_wheel_velocity_radps{0.0};

  bool have_safety{false};
  bool command_allowed{false};
  QString safety_reason{"waiting for safety state"};

  bool have_robot_state{false};
  uint8_t mode{amr_interfaces::msg::RobotState::MODE_BOOT};
  bool fault_active{false};
  bool battery_low{false};
  bool battery_critical{false};
  bool communication_fault{false};
  QString robot_message{"waiting for robot state"};

  bool have_diagnostics{false};
  int diagnostics_level{3};
  QString diagnostics_summary{"waiting for diagnostics"};
};

inline QString modeName(const uint8_t mode)
{
  using RobotState = amr_interfaces::msg::RobotState;
  switch (mode) {
    case RobotState::MODE_BOOT:
      return QStringLiteral("BOOT");
    case RobotState::MODE_INIT:
      return QStringLiteral("INIT");
    case RobotState::MODE_MANUAL:
      return QStringLiteral("MANUAL");
    case RobotState::MODE_AUTO_READY:
      return QStringLiteral("AUTO_READY");
    case RobotState::MODE_AUTO_RUNNING:
      return QStringLiteral("AUTO_RUNNING");
    case RobotState::MODE_PAUSED:
      return QStringLiteral("PAUSED");
    case RobotState::MODE_CHARGING:
      return QStringLiteral("CHARGING");
    case RobotState::MODE_FAULT:
      return QStringLiteral("FAULT");
    case RobotState::MODE_ESTOP:
      return QStringLiteral("ESTOP");
    default:
      return QStringLiteral("UNKNOWN");
  }
}

inline QString diagnosticsLevelName(const int level)
{
  switch (level) {
    case 0:
      return QStringLiteral("OK");
    case 1:
      return QStringLiteral("WARN");
    case 2:
      return QStringLiteral("ERROR");
    case 3:
      return QStringLiteral("STALE");
    default:
      return QStringLiteral("UNKNOWN");
  }
}
}  // namespace amr_operator_ui

Q_DECLARE_METATYPE(amr_operator_ui::RobotUiState)

#endif  // AMR_OPERATOR_UI__ROBOT_STATE_CACHE_HPP_
