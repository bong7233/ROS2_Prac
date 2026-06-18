#include "amr_operator_ui/main_window.hpp"

#include <algorithm>
#include <cmath>

#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QSizePolicy>
#include <QStatusBar>
#include <QVBoxLayout>

namespace amr_operator_ui
{
namespace
{
QString boolText(const bool value)
{
  return value ? QStringLiteral("true") : QStringLiteral("false");
}
}  // namespace

MainWindow::MainWindow(RosWorker * worker, QWidget * parent)
: QMainWindow(parent),
  worker_(worker)
{
  buildUi();

  connect(scene_, &RobotSceneWidget::robotSelected, this, &MainWindow::onRobotSelected);
  connect(worker_, &RosWorker::stateUpdated, scene_, &RobotSceneWidget::setRobotState);
  connect(worker_, &RosWorker::stateUpdated, this, &MainWindow::onStateUpdated);
  connect(worker_, &RosWorker::serviceResult, this, &MainWindow::onServiceResult);
  connect(
    worker_,
    &RosWorker::rosLogMessage,
    this,
    [this](const QString & message) { statusBar()->showMessage(message, 6000); });
}

void MainWindow::buildUi()
{
  setWindowTitle(QStringLiteral("AMR Operator Console"));
  resize(1180, 720);

  auto * central = new QWidget(this);
  auto * root_layout = new QHBoxLayout(central);
  root_layout->setContentsMargins(12, 12, 12, 12);
  root_layout->setSpacing(12);

  scene_ = new RobotSceneWidget(central);
  scene_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  root_layout->addWidget(scene_, 1);

  side_stack_ = new QStackedWidget(central);
  side_stack_->setFixedWidth(370);
  side_stack_->addWidget(buildPlaceholderPage());
  side_stack_->addWidget(buildDetailPage());
  root_layout->addWidget(side_stack_);

  setCentralWidget(central);
  statusBar()->showMessage(QStringLiteral("Waiting for ROS data"));
}

QWidget * MainWindow::buildPlaceholderPage()
{
  auto * page = new QWidget(this);
  auto * layout = new QVBoxLayout(page);
  layout->setContentsMargins(18, 18, 18, 18);
  layout->addStretch();

  auto * title = new QLabel(QStringLiteral("Select the robot"), page);
  title->setAlignment(Qt::AlignCenter);
  title->setStyleSheet(QStringLiteral("font-size: 20px; font-weight: 700; color: #1f2937;"));

  auto * text = new QLabel(
    QStringLiteral("Click the robot in the workspace to open the operator panel."),
    page);
  text->setAlignment(Qt::AlignCenter);
  text->setWordWrap(true);
  text->setStyleSheet(QStringLiteral("color: #526070;"));

  layout->addWidget(title);
  layout->addSpacing(10);
  layout->addWidget(text);
  layout->addStretch();
  return page;
}

QWidget * MainWindow::buildDetailPage()
{
  auto * page = new QWidget(this);
  auto * layout = new QVBoxLayout(page);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(10);

  status_badge_ = new QLabel(QStringLiteral("WAITING"), page);
  status_badge_->setAlignment(Qt::AlignCenter);
  status_badge_->setMinimumHeight(42);
  layout->addWidget(status_badge_);

  auto * robot_group = new QGroupBox(QStringLiteral("Robot"), page);
  auto * robot_form = new QFormLayout(robot_group);
  mode_label_ = new QLabel(QStringLiteral("-"), robot_group);
  robot_message_label_ = new QLabel(QStringLiteral("-"), robot_group);
  robot_message_label_->setWordWrap(true);
  odom_label_ = new QLabel(QStringLiteral("-"), robot_group);
  robot_form->addRow(QStringLiteral("Mode"), mode_label_);
  robot_form->addRow(QStringLiteral("Message"), robot_message_label_);
  robot_form->addRow(QStringLiteral("Odom"), odom_label_);
  layout->addWidget(robot_group);

  auto * safety_group = new QGroupBox(QStringLiteral("Safety"), page);
  auto * safety_form = new QFormLayout(safety_group);
  safety_label_ = new QLabel(QStringLiteral("-"), safety_group);
  safety_label_->setWordWrap(true);
  fault_label_ = new QLabel(QStringLiteral("-"), safety_group);
  diagnostics_label_ = new QLabel(QStringLiteral("-"), safety_group);
  diagnostics_label_->setWordWrap(true);
  safety_form->addRow(QStringLiteral("Gate"), safety_label_);
  safety_form->addRow(QStringLiteral("Faults"), fault_label_);
  safety_form->addRow(QStringLiteral("Diagnostics"), diagnostics_label_);
  layout->addWidget(safety_group);

  auto * power_group = new QGroupBox(QStringLiteral("Power"), page);
  auto * power_layout = new QVBoxLayout(power_group);
  battery_bar_ = new QProgressBar(power_group);
  battery_bar_->setRange(0, 100);
  battery_value_label_ = new QLabel(QStringLiteral("-"), power_group);
  voltage_label_ = new QLabel(QStringLiteral("-"), power_group);
  power_layout->addWidget(battery_bar_);
  power_layout->addWidget(battery_value_label_);
  power_layout->addWidget(voltage_label_);
  layout->addWidget(power_group);

  auto * device_group = new QGroupBox(QStringLiteral("Devices"), page);
  auto * device_form = new QFormLayout(device_group);
  motor_label_ = new QLabel(QStringLiteral("-"), device_group);
  motor_label_->setWordWrap(true);
  io_label_ = new QLabel(QStringLiteral("-"), device_group);
  io_label_->setWordWrap(true);
  device_form->addRow(QStringLiteral("Motor"), motor_label_);
  device_form->addRow(QStringLiteral("IO"), io_label_);
  layout->addWidget(device_group);

  auto * jog_group = new QGroupBox(QStringLiteral("Manual Jog"), page);
  auto * jog_layout = new QGridLayout(jog_group);
  jog_layout->addWidget(makeJogButton(QStringLiteral("Forward"), 0.20, 0.0), 0, 1);
  jog_layout->addWidget(makeJogButton(QStringLiteral("Left"), 0.0, 0.45), 1, 0);
  auto * stop_button = new QPushButton(QStringLiteral("Stop"), jog_group);
  connect(stop_button, &QPushButton::clicked, worker_, [this]() { worker_->publishVelocity(0.0, 0.0); });
  jog_layout->addWidget(stop_button, 1, 1);
  jog_layout->addWidget(makeJogButton(QStringLiteral("Right"), 0.0, -0.45), 1, 2);
  jog_layout->addWidget(makeJogButton(QStringLiteral("Back"), -0.15, 0.0), 2, 1);
  layout->addWidget(jog_group);

  auto * command_row = new QHBoxLayout();
  auto * manual_button = new QPushButton(QStringLiteral("Set Manual"), page);
  auto * reset_button = new QPushButton(QStringLiteral("Reset Fault"), page);
  connect(manual_button, &QPushButton::clicked, worker_, &RosWorker::requestManualMode);
  connect(reset_button, &QPushButton::clicked, worker_, &RosWorker::requestResetFault);
  command_row->addWidget(manual_button);
  command_row->addWidget(reset_button);
  layout->addLayout(command_row);

  layout->addStretch();
  updateStatusBadge();
  return page;
}

QPushButton * MainWindow::makeJogButton(
  const QString & label,
  const double linear_mps,
  const double angular_radps)
{
  auto * button = new QPushButton(label, this);
  button->setMinimumHeight(38);
  connect(
    button,
    &QPushButton::pressed,
    worker_,
    [this, linear_mps, angular_radps]() { worker_->publishVelocity(linear_mps, angular_radps); });
  connect(button, &QPushButton::released, worker_, [this]() { worker_->publishVelocity(0.0, 0.0); });
  return button;
}

void MainWindow::onRobotSelected()
{
  side_stack_->setCurrentIndex(1);
  statusBar()->showMessage(QStringLiteral("Robot selected"), 3000);
}

void MainWindow::onStateUpdated(const RobotUiState & state)
{
  state_ = state;

  mode_label_->setText(modeName(state_.mode));
  robot_message_label_->setText(state_.robot_message);

  if (state_.have_odom) {
    odom_label_->setText(
      QStringLiteral("x=%1 m, y=%2 m, yaw=%3 rad")
        .arg(state_.x_m, 0, 'f', 2)
        .arg(state_.y_m, 0, 'f', 2)
        .arg(state_.yaw_rad, 0, 'f', 2));
  } else {
    odom_label_->setText(QStringLiteral("waiting for /odom"));
  }

  const int battery_percent =
    std::clamp(static_cast<int>(std::round(state_.battery_percentage * 100.0)), 0, 100);
  battery_bar_->setValue(battery_percent);
  battery_value_label_->setText(
    QStringLiteral("%1%  low=%2  critical=%3")
      .arg(battery_percent)
      .arg(boolText(state_.battery_low), boolText(state_.battery_critical)));
  voltage_label_->setText(
    QStringLiteral("%1 V, %2 A")
      .arg(state_.voltage_v, 0, 'f', 1)
      .arg(state_.current_a, 0, 'f', 1));

  safety_label_->setText(
    QStringLiteral("allowed=%1, reason=%2")
      .arg(boolText(state_.command_allowed), state_.safety_reason));
  fault_label_->setText(
    QStringLiteral("fault=%1, estop=%2, motor=%3, comm=%4")
      .arg(
        boolText(state_.fault_active),
        boolText(state_.estop_active),
        boolText(state_.motor_fault),
        boolText(state_.communication_fault)));

  motor_label_->setText(
    QStringLiteral("enabled=%1, comm=%2, fault_code=%3, L=%4 rad/s, R=%5 rad/s")
      .arg(
        boolText(state_.motor_enabled),
        boolText(state_.motor_communication_ok),
        QString::number(state_.motor_fault_code),
        QString::number(state_.left_wheel_velocity_radps, 'f', 2),
        QString::number(state_.right_wheel_velocity_radps, 'f', 2)));

  io_label_->setText(
    QStringLiteral("comm=%1, inputs=%2/%3, outputs=%4/%5, protective_stop=%6")
      .arg(
        boolText(state_.io_communication_ok),
        QString::number(state_.active_input_count),
        QString::number(state_.input_count),
        QString::number(state_.active_output_count),
        QString::number(state_.output_count),
        boolText(state_.protective_stop_active)));

  diagnostics_label_->setText(
    QStringLiteral("%1 - %2")
      .arg(diagnosticsLevelName(state_.diagnostics_level), state_.diagnostics_summary));

  updateStatusBadge();
}

void MainWindow::onServiceResult(
  const QString & service_name,
  const bool success,
  const QString & message)
{
  const QString prefix = success ? QStringLiteral("OK") : QStringLiteral("FAILED");
  statusBar()->showMessage(
    QStringLiteral("%1 %2: %3").arg(prefix, service_name, message),
    7000);
}

void MainWindow::updateStatusBadge()
{
  if (!status_badge_) {
    return;
  }

  QString background = QStringLiteral("#dbeafe");
  QString foreground = QStringLiteral("#1e3a8a");
  QString text = modeName(state_.mode);

  if (!state_.have_robot_state) {
    background = QStringLiteral("#e5e7eb");
    foreground = QStringLiteral("#374151");
    text = QStringLiteral("WAITING");
  } else if (state_.estop_active) {
    background = QStringLiteral("#fee2e2");
    foreground = QStringLiteral("#991b1b");
    text = QStringLiteral("ESTOP");
  } else if (state_.fault_active || state_.motor_fault || state_.battery_critical) {
    background = QStringLiteral("#ffedd5");
    foreground = QStringLiteral("#9a3412");
    text = QStringLiteral("FAULT");
  } else if (!state_.command_allowed && state_.have_safety) {
    background = QStringLiteral("#fef3c7");
    foreground = QStringLiteral("#92400e");
    text = QStringLiteral("COMMAND BLOCKED");
  }

  status_badge_->setText(text);
  status_badge_->setStyleSheet(
    QStringLiteral(
      "QLabel { background: %1; color: %2; border: 1px solid %2; "
      "border-radius: 6px; font-size: 18px; font-weight: 700; }")
      .arg(background, foreground));
}
}  // namespace amr_operator_ui
