// *****************************************************************************
//
// Copyright (c) 2017, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <mapviz_plugins/nav2_plugin.h>

// C++ standard libraries
#include <cstdio>
#include <vector>

// QT libraries
#include <QDateTime>
#include <QDialog>
#include <QGLWidget>
#include <QMouseEvent>
#include <QPainter>
#include <QPalette>
#include <QStaticText>
#include <QDebug>
#include <QSettings>
#include <fstream>

// ROS libraries
#include <rclcpp/rclcpp.hpp>
#include <swri_transform_util/frames.h>

// Declare plugin
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::Nav2Plugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{

Nav2Plugin::Nav2Plugin()
: MapvizPlugin(),
  ui_(),
  config_widget_(new QWidget()),
  map_canvas_(NULL),
  is_mouse_down_(false),
  monitoring_action_state_(false)
{
  ui_.setupUi(config_widget_);

  // Set background white
  QPalette p(config_widget_->palette());
  p.setColor(QPalette::Background, Qt::white);
  config_widget_->setPalette(p);
  // Set status text red

  ui_.status->setText("OK");
  QPalette p3(ui_.status->palette());
  p3.setColor(QPalette::Text, Qt::green);
  ui_.status->setPalette(p3);

  QObject::connect(ui_.pushButtonGoalPose, SIGNAL(toggled(bool)), this, SLOT(SelectingGoalToggle));
  QObject::connect(ui_.pushButtonAbort,SIGNAL(pressed()),this,SLOT(Abort()));
}

Nav2Plugin::~Nav2Plugin()
{
  if (map_canvas_) {
    map_canvas_->removeEventFilter(this);
  }
}

void Nav2Plugin::PrintError(const std::string& message)
{
  PrintErrorHelper(ui_.status,message);
}

void Nav2Plugin::PrintInfo(const std::string& message)
{
  PrintInfoHelper(ui_.status,message);
}

void Nav2Plugin::PrintWarning(const std::string& message)
{
  PrintWarningHelper(ui_.status,message);
}

QWidget* Nav2Plugin::GetConfigWidget(QWidget* parent)
{
  config_widget_->setParent(parent);
  return config_widget_;
}

bool Nav2Plugin::Initialize(QGLWidget* canvas)
{
  map_canvas_ = static_cast<mapviz::MapCanvas*>(canvas);
  map_canvas_->installEventFilter(this);
  startTimer(1000);

  if (!nav_client_) {
    nav_client_ = rclcpp_action::create_client<NavClientT>(
      node_,
      "navigate_to_pose");

    navigation_feedback_sub_ =
      node_->create_subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>(
      "navigate_to_pose/_action/feedback",
      rclcpp::SystemDefaultsQoS(),
      [this](const nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage::SharedPtr msg) {
        // TODO: Show feedback data in status
      });

    navigation_goal_status_sub_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      "navigate_to_pose/_action/status",
      rclcpp::SystemDefaultsQoS(),
      [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        GoalStatusUpdate(msg->status_list.back());
      });
  }

  initialized_ = true;

  return true;
}

bool Nav2Plugin::eventFilter(QObject* object,QEvent* event)
{
  switch (event->type()) {
    case QEvent::MouseButtonPress:
      return handleMousePress(static_cast<QMouseEvent*>(event));
    case QEvent::MouseButtonRelease:
      return handleMouseRelease(static_cast<QMouseEvent*>(event));
    case QEvent::MouseMove:
      return handleMouseMove(static_cast<QMouseEvent*>(event));
    default:
      return false;
  }
}

void Nav2Plugin::timerEvent(QTimerEvent*)
{
  connected_to_nav_ = nav_client_->action_server_is_ready();
  ui_.pushButtonAbort->setEnabled(connected_to_nav_);
  ui_.pushButtonGoalPose->setEnabled(connected_to_nav_);
  if (!connected_to_nav_) {
    PrintError("[Nav2] server not connected");
  } else if (!monitoring_action_state_) {
    PrintWarning("Ready to send command");
  }
}

void Nav2Plugin::GoalStatusUpdate(action_msgs::msg::GoalStatus status)
{
  auto state = status.status;

  switch (state) {
    case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
      PrintInfo("Goal Accepted");
      break;

    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      PrintError("Goal Aborted");
      monitoring_action_state_ = false;
      break;

    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      PrintWarningHelper(ui_.status,"Goal Canceled");
      monitoring_action_state_ = false;
      break;

    case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
      PrintInfoHelper(ui_.status,"Goal Executing");
      break;

    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      PrintInfoHelper(ui_.status,"Goal Succeeded");
      monitoring_action_state_ = false;
      break;

    default:
      PrintErrorHelper(ui_.status,"Unknown Error");
      monitoring_action_state_ = false;
      break;
  }
}


bool Nav2Plugin::handleMousePress(QMouseEvent* event)
{

  bool goal_checked = ui_.pushButtonGoalPose->isChecked();
  if (!goal_checked) {
    return false;
  }

  if (event->button() == Qt::LeftButton) {
    is_mouse_down_ = true;
    arrow_angle_ = 0;
#if QT_VERSION >= 0x050000
    arrow_tail_position_ = map_canvas_->MapGlCoordToFixedFrame(event->localPos());
#else
    arrow_tail_position_ = map_canvas_->MapGlCoordToFixedFrame(event->posF());
#endif
    return true;
  }
  return false;
}

bool Nav2Plugin::handleMouseMove(QMouseEvent* event)
{
  if (is_mouse_down_) {
#if QT_VERSION >= 0x050000
    QPointF head_pos = map_canvas_->MapGlCoordToFixedFrame(event->localPos());
#else
    QPointF head_pos = map_canvas_->MapGlCoordToFixedFrame(event->posF());
#endif
    arrow_angle_ = atan2(
      head_pos.y() - arrow_tail_position_.y(),
      head_pos.x() - arrow_tail_position_.x());
  }
  return false;
}

bool Nav2Plugin::handleMouseRelease(QMouseEvent* event)
{
  if (!is_mouse_down_) {
    return false;
  }

  is_mouse_down_ = false;

  bool goal_checked = ui_.pushButtonGoalPose->isChecked();
  if (!goal_checked) {
    return false;
  }

  if (!connected_to_nav_) {
    return false;
  }

  nav_client_->async_cancel_all_goals();

  NavClientT::Goal nav_goal_msg_;
  geometry_msgs::msg::PoseStamped& pose = nav_goal_msg_.pose;

  pose.header.frame_id = target_frame_;
  pose.header.stamp = node_->now();

  pose.pose.position.x = arrow_tail_position_.x();
  pose.pose.position.y = arrow_tail_position_.y();
  pose.pose.position.z = 0.0;

  tf2::Quaternion quat;
  quat.setRPY(0,0,arrow_angle_);
  pose.pose.orientation = tf2::toMsg(quat);

  nav_future_goal_handle_ = nav_client_->async_send_goal(nav_goal_msg_);

  ui_.pushButtonGoalPose->setChecked(false);
  monitoring_action_state_ = true;

  return true;
}

void Nav2Plugin::Draw(double x,double y,double scale)
{
  std::array<QPointF,7> arrow_points;
  arrow_points[0] = QPointF(10,0);
  arrow_points[1] = QPointF(6,-2.5);
  arrow_points[2] = QPointF(6.5,-1);
  arrow_points[3] = QPointF(0,-1);
  arrow_points[4] = QPointF(0,1);
  arrow_points[5] = QPointF(6.5,1);
  arrow_points[6] = QPointF(6,2.5);

  if (is_mouse_down_) {
    QPointF transformed_points[7];
    for (size_t i = 0; i < 7; i++) {
      tf2::Vector3 point(arrow_points[i].x(),arrow_points[i].y(),0);
      point *= scale * 10;
      tf2::Quaternion quat;
      quat.setRPY(0,0,arrow_angle_);
      point = tf2::quatRotate(quat,point);
      transformed_points[i] = QPointF(
        point.x() + arrow_tail_position_.x(),
        point.y() + arrow_tail_position_.y());
    }
    glColor3f(0.1,0.9,0.1);
    glLineWidth(2);
    glBegin(GL_TRIANGLE_FAN);
    for (const QPointF& point: transformed_points) {
      glVertex2d(point.x(),point.y());
    }
    glEnd();

    glColor3f(0.0,0.6,0.0);
    glBegin(GL_LINE_LOOP);
    for (const QPointF& point: transformed_points) {
      glVertex2d(point.x(),point.y());
    }
    glEnd();
  }
}


void Nav2Plugin::LoadConfig(const YAML::Node& node,const std::string& path)
{

}

void Nav2Plugin::SaveConfig(YAML::Emitter& emitter,const std::string& path)
{

}

void Nav2Plugin::SelectingGoalToggle(bool checked)
{
  if (checked) {
    QPixmap cursor_pixmap = QPixmap(":/images/green-arrow.png");
    QApplication::setOverrideCursor(QCursor(cursor_pixmap));
  }
  if (!checked) {
    QApplication::restoreOverrideCursor();
  }

}

void Nav2Plugin::Abort()
{
  RCLCPP_INFO(node_->get_logger(),"Aborting all goals!");
  nav_client_->async_cancel_all_goals();
}

}
