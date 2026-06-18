#ifndef AMR_OPERATOR_UI__MAIN_WINDOW_HPP_
#define AMR_OPERATOR_UI__MAIN_WINDOW_HPP_

#include <QLabel>
#include <QMainWindow>
#include <QProgressBar>
#include <QPushButton>
#include <QStackedWidget>

#include "amr_operator_ui/robot_scene_widget.hpp"
#include "amr_operator_ui/robot_state_cache.hpp"
#include "amr_operator_ui/ros_worker.hpp"

namespace amr_operator_ui
{
class MainWindow final : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(RosWorker * worker, QWidget * parent = nullptr);

private slots:
  void onRobotSelected();
  void onStateUpdated(const amr_operator_ui::RobotUiState & state);
  void onServiceResult(const QString & service_name, bool success, const QString & message);

private:
  void buildUi();
  QWidget * buildPlaceholderPage();
  QWidget * buildDetailPage();
  QPushButton * makeJogButton(const QString & label, double linear_mps, double angular_radps);
  void updateStatusBadge();

  RosWorker * worker_;
  RobotUiState state_;

  RobotSceneWidget * scene_{nullptr};
  QStackedWidget * side_stack_{nullptr};
  QLabel * status_badge_{nullptr};
  QLabel * mode_label_{nullptr};
  QLabel * robot_message_label_{nullptr};
  QLabel * safety_label_{nullptr};
  QLabel * fault_label_{nullptr};
  QLabel * odom_label_{nullptr};
  QLabel * motor_label_{nullptr};
  QLabel * io_label_{nullptr};
  QLabel * diagnostics_label_{nullptr};
  QLabel * battery_value_label_{nullptr};
  QLabel * voltage_label_{nullptr};
  QProgressBar * battery_bar_{nullptr};
};
}  // namespace amr_operator_ui

#endif  // AMR_OPERATOR_UI__MAIN_WINDOW_HPP_
