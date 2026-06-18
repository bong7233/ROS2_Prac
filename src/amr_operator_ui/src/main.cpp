#include <QApplication>

#include "amr_operator_ui/main_window.hpp"
#include "amr_operator_ui/robot_state_cache.hpp"
#include "amr_operator_ui/ros_worker.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  qRegisterMetaType<amr_operator_ui::RobotUiState>("amr_operator_ui::RobotUiState");

  amr_operator_ui::RosWorker worker;
  amr_operator_ui::MainWindow window(&worker);

  if (!worker.start()) {
    rclcpp::shutdown();
    return 1;
  }

  window.show();
  const int result = app.exec();

  worker.stop();
  rclcpp::shutdown();
  return result;
}
