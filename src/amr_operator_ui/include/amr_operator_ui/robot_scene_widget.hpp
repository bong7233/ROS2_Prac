#ifndef AMR_OPERATOR_UI__ROBOT_SCENE_WIDGET_HPP_
#define AMR_OPERATOR_UI__ROBOT_SCENE_WIDGET_HPP_

#include <vector>

#include <QColor>
#include <QPointF>
#include <QWidget>

#include "amr_operator_ui/robot_state_cache.hpp"

namespace amr_operator_ui
{
class RobotSceneWidget final : public QWidget
{
  Q_OBJECT

public:
  explicit RobotSceneWidget(QWidget * parent = nullptr);

public slots:
  void setRobotState(const amr_operator_ui::RobotUiState & state);

signals:
  void robotSelected();

protected:
  void paintEvent(QPaintEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;

private:
  QPointF worldToView(double x_m, double y_m) const;
  QColor robotColor() const;

  RobotUiState state_;
  bool selected_{false};
  std::vector<QPointF> trail_;
};
}  // namespace amr_operator_ui

#endif  // AMR_OPERATOR_UI__ROBOT_SCENE_WIDGET_HPP_
