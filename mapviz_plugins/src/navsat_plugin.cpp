// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-58058A
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Steve Dellenback <sdellenback@swri.org> (210) 522-3914
//
// This code was developed as part of an internal research project fully funded
// by Southwest Research Institute®.
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#include <mapviz_plugins/navsat_plugin.h>

// QT libraries
#include <QDialog>
#include <QGLWidget>
#include <QPalette>

#include <opencv2/core/core.hpp>

// ROS libraries
#include <swri_transform_util/transform_util.h>

#include <mapviz/select_topic_dialog.h>

// Declare plugin
#include <pluginlib/class_list_macros.hpp>

// C++ standard libraries
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

PLUGINLIB_EXPORT_CLASS(mapviz_plugins::NavSatPlugin, mapviz::MapvizPlugin)

namespace mapviz_plugins
{
  NavSatPlugin::NavSatPlugin()
  : PointDrawingPlugin()
  , ui_()
  , config_widget_(new QWidget())
  , has_message_(false)
  {
    ui_.setupUi(config_widget_);

    ui_.color->setColor(Qt::green);

    // Set background white
    QPalette p(config_widget_->palette());
    p.setColor(QPalette::Background, Qt::white);
    config_widget_->setPalette(p);

    // Set status text red
    QPalette p3(ui_.status->palette());
    p3.setColor(QPalette::Text, Qt::red);
    ui_.status->setPalette(p3);

    QObject::connect(ui_.selecttopic, SIGNAL(clicked()), this,
                     SLOT(SelectTopic()));
    QObject::connect(ui_.topic, SIGNAL(editingFinished()), this,
                     SLOT(TopicEdited()));
    QObject::connect(ui_.positiontolerance, SIGNAL(valueChanged(double)), this,
                     SLOT(PositionToleranceChanged(double)));
    QObject::connect(ui_.buffersize, SIGNAL(valueChanged(int)), this,
                     SLOT(BufferSizeChanged(int)));
    QObject::connect(ui_.drawstyle, SIGNAL(activated(QString)), this,
                     SLOT(SetDrawStyle(QString)));
    QObject::connect(ui_.color, SIGNAL(colorEdited(const QColor&)), this,
                     SLOT(SetColor(const QColor&)));
    QObject::connect(ui_.show_covariance, SIGNAL(toggled(bool)), this,
                     SLOT(CovariancedToggled(bool)));
    QObject::connect(ui_.show_all_covariances, SIGNAL(toggled(bool)), this,
                     SLOT(ShowAllCovariancesToggled(bool)));
    QObject::connect(ui_.buttonResetBuffer, SIGNAL(pressed()), this,
                     SLOT(ClearPoints()));
  }

  void NavSatPlugin::SelectTopic()
  {
    std::string topic =
        mapviz::SelectTopicDialog::selectTopic(node_, "sensor_msgs/msg/NavSatFix");

    if (!topic.empty())
    {
      ui_.topic->setText(QString::fromStdString(topic));
      TopicEdited();
    }
  }

  void NavSatPlugin::TopicEdited()
  {
    std::string topic = ui_.topic->text().trimmed().toStdString();
    if (topic != topic_)
    {
      initialized_ = false;
      ClearPoints();
      has_message_ = false;
      PrintWarning("No messages received.");

      navsat_sub_.reset();
      topic_ = topic;
      if (!topic.empty())
      {
        navsat_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
            topic_,
            rclcpp::SensorDataQoS(rclcpp::KeepLast(1)),
            std::bind(&NavSatPlugin::NavSatFixCallback, this, std::placeholders::_1));

        RCLCPP_INFO(node_->get_logger(), "Subscribing to %s", topic_.c_str());
      }
    }
  }

  void NavSatPlugin::NavSatFixCallback(
      const sensor_msgs::msg::NavSatFix::ConstSharedPtr navsat)
  {
    if (!tf_manager_->LocalXyUtil()->Initialized())
    {
      return;
    }
    if (!has_message_)
    {
      initialized_ = true;
      has_message_ = true;
    }

    StampedPoint stamped_point;
    stamped_point.stamp = navsat->header.stamp;

    double x;
    double y;
    tf_manager_->LocalXyUtil()->ToLocalXy(navsat->latitude, navsat->longitude, x, y);

    stamped_point.point = tf2::Vector3(x, y, navsat->altitude);
    stamped_point.orientation.setRPY(0, 0, 0);
    stamped_point.source_frame = tf_manager_->LocalXyUtil()->Frame();

    if ( ui_.show_covariance->isChecked() )
    {
      tf2::Matrix3x3 tf_cov =
          swri_transform_util::Get3x3Cov(navsat->position_covariance);

      if (tf_cov[0][0] < 100000 && tf_cov[1][1] < 100000)
      {
        cv::Mat cov_matrix_3d(3, 3, CV_32FC1);
        for (int32_t r = 0; r < 3; r++)
        {
          for (int32_t c = 0; c < 3; c++)
          {
            cov_matrix_3d.at<float>(r, c) = tf_cov[r][c];
          }
        }

        cv::Mat cov_matrix_2d = swri_image_util::ProjectEllipsoid(cov_matrix_3d);

        if (!cov_matrix_2d.empty())
        {
          stamped_point.cov_points = swri_image_util::GetEllipsePoints(
              cov_matrix_2d, stamped_point.point, 3, 32);

          stamped_point.transformed_cov_points = stamped_point.cov_points;
        } else {
          RCLCPP_ERROR(node_->get_logger(), "Failed to project x, y, z covariance to xy-plane.");
        }
      }
    }

    pushPoint( std::move(stamped_point) );
  }

  void NavSatPlugin::PrintError(const std::string& message)
  {
    PrintErrorHelper(ui_.status, message);
  }

  void NavSatPlugin::PrintInfo(const std::string& message)
  {
    PrintInfoHelper(ui_.status, message);
  }

  void NavSatPlugin::PrintWarning(const std::string& message)
  {
    PrintWarningHelper(ui_.status, message);
  }

  QWidget* NavSatPlugin::GetConfigWidget(QWidget* parent)
  {
    config_widget_->setParent(parent);

    return config_widget_;
  }

  bool NavSatPlugin::Initialize(QGLWidget* canvas)
  {
    canvas_ = canvas;
    SetColor(ui_.color->color());
    return true;
  }

  void NavSatPlugin::Draw(double x, double y, double scale)
  {
    if (ui_.show_covariance->isChecked())
    {
      DrawCovariance();
    }
    if (DrawPoints(scale))
    {
      PrintInfo("OK");
    }
  }

  void NavSatPlugin::LoadConfig(const YAML::Node& node, const std::string& path)
  {
    if (node["topic"])
    {
      std::string topic = node["topic"].as<std::string>();
      ui_.topic->setText(topic.c_str());
    }

    if (node["color"])
    {
      std::string color = node["color"].as<std::string>();
      QColor qcolor(color.c_str());
      SetColor(qcolor);
      ui_.color->setColor(qcolor);
    }

    if (node["draw_style"])
    {
      std::string draw_style = node["draw_style"].as<std::string>();

      if (draw_style == "lines")
      {
        ui_.drawstyle->setCurrentIndex(0);
        SetDrawStyle( LINES );
      } else if (draw_style == "points") {
        ui_.drawstyle->setCurrentIndex(1);
        SetDrawStyle( POINTS );
      }
    }

    if (node["position_tolerance"])
    {
      auto position_tolerance = node["position_tolerance"].as<double>();
      ui_.positiontolerance->setValue(position_tolerance);
      PositionToleranceChanged(position_tolerance);
    }

    if (node["buffer_size"])
    {
      auto buffer_size = node["buffer_size"].as<int>();
      ui_.buffersize->setValue(buffer_size);
      BufferSizeChanged(buffer_size);
    }

    if (node["show_covariance"])
    {
      bool show_covariance = node["show_covariance"].as<bool>();
      ui_.show_covariance->setChecked(show_covariance);
      CovariancedToggled(show_covariance);
    }

    if (node["show_all_covariances"])
    {
      bool show_all_covariances = node["show_all_covariances"].as<bool>();
      ui_.show_all_covariances->setChecked(show_all_covariances);
      ShowAllCovariancesToggled(show_all_covariances);
    }

    TopicEdited();
  }

  void NavSatPlugin::SaveConfig(YAML::Emitter& emitter, const std::string& path)
  {
    std::string topic = ui_.topic->text().toStdString();
    emitter << YAML::Key << "topic" << YAML::Value << topic;

    std::string color = ui_.color->color().name().toStdString();
    emitter << YAML::Key << "color" << YAML::Value << color;

    std::string draw_style = ui_.drawstyle->currentText().toStdString();
    emitter << YAML::Key << "draw_style" << YAML::Value << draw_style;

    emitter << YAML::Key << "position_tolerance" <<
               YAML::Value << positionTolerance();

    emitter << YAML::Key << "buffer_size" << YAML::Value << bufferSize();

    bool show_covariance = ui_.show_covariance->isChecked();
    emitter << YAML::Key << "show_covariance" << YAML::Value << show_covariance;

    bool show_all_covariances = ui_.show_all_covariances->isChecked();
    emitter << YAML::Key << "show_all_covariances" << YAML::Value << show_all_covariances;
  }
}   // namespace mapviz_plugins
