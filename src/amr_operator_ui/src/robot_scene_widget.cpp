#include "amr_operator_ui/robot_scene_widget.hpp"

#include <algorithm>
#include <cmath>

#include <QLineF>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QPolygonF>
#include <QtMath>

namespace amr_operator_ui
{
RobotSceneWidget::RobotSceneWidget(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(620, 480);
  setMouseTracking(true);
}

void RobotSceneWidget::setRobotState(const RobotUiState & state)
{
  state_ = state;

  if (state_.have_odom) {
    const QPointF point(state_.x_m, state_.y_m);
    if (trail_.empty() || QLineF(trail_.back(), point).length() > 0.03) {
      trail_.push_back(point);
    }
    if (trail_.size() > 220) {
      trail_.erase(trail_.begin(), trail_.begin() + static_cast<long>(trail_.size() - 220));
    }
  }

  update();
}

void RobotSceneWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), QColor("#f6f8fa"));

  const auto grid_color = QColor("#d8dee7");
  painter.setPen(QPen(grid_color, 1));
  const int grid_px = 55;
  for (int x = width() / 2 % grid_px; x < width(); x += grid_px) {
    painter.drawLine(x, 0, x, height());
  }
  for (int y = height() / 2 % grid_px; y < height(); y += grid_px) {
    painter.drawLine(0, y, width(), y);
  }

  painter.setPen(QPen(QColor("#aab4c3"), 2));
  painter.drawLine(worldToView(-100.0, 0.0), worldToView(100.0, 0.0));
  painter.drawLine(worldToView(0.0, -100.0), worldToView(0.0, 100.0));

  painter.setPen(QPen(QColor("#506070"), 1));
  painter.drawText(18, 28, QStringLiteral("AMR workspace"));

  if (trail_.size() > 1) {
    QPainterPath path;
    const QPointF first = worldToView(trail_.front().x(), trail_.front().y());
    path.moveTo(first);
    for (size_t i = 1; i < trail_.size(); ++i) {
      path.lineTo(worldToView(trail_[i].x(), trail_[i].y()));
    }
    painter.setPen(QPen(QColor("#58708f"), 2));
    painter.drawPath(path);
  }

  if (!state_.have_odom) {
    painter.setPen(QPen(QColor("#506070"), 1));
    painter.drawText(rect(), Qt::AlignCenter, QStringLiteral("Waiting for /odom"));
    return;
  }

  const QPointF center = worldToView(state_.x_m, state_.y_m);
  if (selected_) {
    painter.setPen(QPen(QColor("#1f6feb"), 3));
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(center, 34.0, 34.0);
  }

  painter.save();
  painter.translate(center);
  painter.rotate(qRadiansToDegrees(state_.yaw_rad));

  QPolygonF body;
  body << QPointF(31.0, 0.0) << QPointF(-22.0, -18.0) << QPointF(-14.0, 0.0)
       << QPointF(-22.0, 18.0);

  painter.setPen(QPen(QColor("#172033"), 2));
  painter.setBrush(robotColor());
  painter.drawPolygon(body);
  painter.setBrush(QColor("#ffffff"));
  painter.drawEllipse(QPointF(10.0, 0.0), 4.5, 4.5);
  painter.restore();

  painter.setPen(QPen(QColor("#2f3b4c"), 1));
  painter.drawText(
    center + QPointF(38.0, -20.0),
    QStringLiteral("%1  x=%2 y=%3")
      .arg(modeName(state_.mode))
      .arg(state_.x_m, 0, 'f', 2)
      .arg(state_.y_m, 0, 'f', 2));
}

void RobotSceneWidget::mousePressEvent(QMouseEvent * event)
{
  if (!state_.have_odom) {
    return;
  }

  const QPointF center = worldToView(state_.x_m, state_.y_m);
  if (QLineF(event->position(), center).length() <= 38.0) {
    selected_ = true;
    emit robotSelected();
    update();
  }
}

QPointF RobotSceneWidget::worldToView(const double x_m, const double y_m) const
{
  constexpr double scale_px_per_m = 55.0;
  return QPointF(
    width() * 0.5 + x_m * scale_px_per_m,
    height() * 0.5 - y_m * scale_px_per_m);
}

QColor RobotSceneWidget::robotColor() const
{
  if (state_.estop_active) {
    return QColor("#df3b3b");
  }
  if (state_.fault_active || state_.motor_fault || state_.battery_critical) {
    return QColor("#f0883e");
  }
  if (!state_.command_allowed && state_.have_safety) {
    return QColor("#d29922");
  }
  return QColor("#2da44e");
}
}  // namespace amr_operator_ui
