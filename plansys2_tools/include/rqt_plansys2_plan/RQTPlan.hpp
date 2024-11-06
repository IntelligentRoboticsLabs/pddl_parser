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


#ifndef RQT_PLANSYS2_PLAN__RQTPLAN_HPP_
#define RQT_PLANSYS2_PLAN__RQTPLAN_HPP_

#include <ui_rqt_plansys2_plan.h>

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

#include "rqt_plansys2_plan/PlanTree.hpp"

#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/action_execution.hpp"

#include "rclcpp/rclcpp.hpp"

namespace rqt_plansys2_plan
{

class RQTPlan
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  RQTPlan();

  virtual void initPlugin(qt_gui_cpp::PluginContext & context);
  virtual void shutdownPlugin();
  virtual void saveSettings(
    qt_gui_cpp::Settings & plugin_settings, qt_gui_cpp::Settings & instance_settings) const;
  virtual void restoreSettings(
    const qt_gui_cpp::Settings & plugin_settings, const qt_gui_cpp::Settings & instance_settings);

protected slots:
  void spin_loop();

protected:
  Ui::RqtPlansys2Plan ui_;
  QWidget * widget_;

private:
  QTimer * controller_spin_timer_;
  PlanTree * plan_tree_;

  std::map<std::string, plansys2_msgs::msg::ActionExecutionInfo::UniquePtr> plan_info_;
  plansys2_msgs::msg::Plan::UniquePtr plan_;

  rclcpp::Subscription<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr action_execution_info_;
  rclcpp::Subscription<plansys2_msgs::msg::Plan>::SharedPtr executing_plan_;
  bool need_update_plan_;
  bool need_update_info_;

  void execution_info_callback(plansys2_msgs::msg::ActionExecutionInfo::UniquePtr msg);
  void plan_callback(plansys2_msgs::msg::Plan::UniquePtr msg);

  std::optional<QTreeWidgetItem *> get_plan_item_row(const std::string & action_full_name);
  void fill_row_info(
    QTreeWidgetItem * row, const plansys2_msgs::msg::ActionExecutionInfo & info);
};

}  // namespace rqt_plansys2_plan

#endif  // RQT_PLANSYS2_PLAN__RQTPLAN_HPP_
