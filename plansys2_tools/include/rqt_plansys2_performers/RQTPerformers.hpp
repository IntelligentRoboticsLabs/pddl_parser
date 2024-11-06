// Copyright 2022 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef RQT_PLANSYS2_PERFORMERS__RQTPERFORMERS_HPP_
#define RQT_PLANSYS2_PERFORMERS__RQTPERFORMERS_HPP_

#include <ui_rqt_plansys2_performers.h>

#include <QAction>
#include <QImage>
#include <QList>
#include <QString>
#include <QSet>
#include <QSize>
#include <QWidget>

#include <map>
#include <memory>
#include <string>

#include "rqt_gui_cpp/plugin.h"

#include "rqt_plansys2_performers/PerformersTree.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "plansys2_msgs/msg/action_performer_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rqt_plansys2_performers
{


class RQTPerformers
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  RQTPerformers();

  virtual void initPlugin(qt_gui_cpp::PluginContext & context);
  virtual void shutdownPlugin();
  virtual void saveSettings(
    qt_gui_cpp::Settings & plugin_settings, qt_gui_cpp::Settings & instance_settings) const;
  virtual void restoreSettings(
    const qt_gui_cpp::Settings & plugin_settings, const qt_gui_cpp::Settings & instance_settings);

protected slots:
  void spin_loop();

protected:
  Ui::RqtPlansys2Performers ui_;
  QWidget * widget_;

private:
  QTimer * controller_spin_timer_;
  PerformersTree * performers_tree_;

  std::map<std::string, plansys2_msgs::msg::ActionPerformerStatus::UniquePtr> performers_info_;
  rclcpp::Subscription<plansys2_msgs::msg::ActionPerformerStatus>::SharedPtr performers_sub_;
  bool need_update_;

  std::shared_ptr<plansys2::ProblemExpertClient> problem_;

  void performers_callback(plansys2_msgs::msg::ActionPerformerStatus::UniquePtr msg);
  std::optional<QTreeWidgetItem *> get_performer_line(const std::string & performer_name);
  void update_performer_row(
    QTreeWidgetItem * row, const plansys2_msgs::msg::ActionPerformerStatus & performer);
  void add_new_row(const plansys2_msgs::msg::ActionPerformerStatus & performer);
};

}  // namespace rqt_plansys2_performers

#endif  // RQT_PLANSYS2_PERFORMERS__RQTPERFORMERS_HPP_
