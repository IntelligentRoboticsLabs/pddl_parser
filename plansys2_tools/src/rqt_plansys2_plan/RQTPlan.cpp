// Copyright 2021 Intelligent Robotics Lab
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

#include <unistd.h>

#include <QDebug>
#include <QTime>
#include <QTimer>
#include <QPushButton>
#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>
#include <QCheckBox>
#include <algorithm>
#include <memory>
#include <utility>
#include <string>

#include <pluginlib/class_list_macros.hpp>


#include "rqt_plansys2_plan/RQTPlan.hpp"
#include "rqt_plansys2_plan/PlanTree.hpp"


namespace rqt_plansys2_plan
{

using std::placeholders::_1;

RQTPlan::RQTPlan()
: rqt_gui_cpp::Plugin(),
  widget_(0)
{
  setObjectName("RQTPlan");
}

void RQTPlan::initPlugin(qt_gui_cpp::PluginContext & context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(
      widget_->windowTitle() + " (" +
      QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  plan_tree_ = new PlanTree();
  ui_.gridLayout->addWidget(plan_tree_);

  plan_tree_->setColumnCount(5);
  plan_tree_->setHeaderLabels(
    {"Plan Item", "Status", "Completion", "Message Status", "Execution Time / Predicted"});

  controller_spin_timer_ = new QTimer(this);
  connect(controller_spin_timer_, SIGNAL(timeout()), this, SLOT(spin_loop()));
  controller_spin_timer_->start(100);

  action_execution_info_ = node_->create_subscription<plansys2_msgs::msg::ActionExecutionInfo>(
    "action_execution_info", 100,
    std::bind(&RQTPlan::execution_info_callback, this, _1));

  executing_plan_ = node_->create_subscription<plansys2_msgs::msg::Plan>(
    "executing_plan", rclcpp::QoS(100).transient_local(),
    std::bind(&RQTPlan::plan_callback, this, _1));

  need_update_plan_ = false;
  need_update_info_ = false;
}

void RQTPlan::shutdownPlugin()
{
}

void
RQTPlan::execution_info_callback(plansys2_msgs::msg::ActionExecutionInfo::UniquePtr msg)
{
  if (!need_update_plan_) {
    plan_info_[msg->action_full_name] = std::move(msg);
    need_update_info_ = true;
  }
}

void
RQTPlan::plan_callback(plansys2_msgs::msg::Plan::UniquePtr msg)
{
  if (plan_ == nullptr || *plan_ != *msg) {
    new_plan_ = std::move(msg);
    need_update_plan_ = true;
  }
}

void
RQTPlan::saveSettings(
  qt_gui_cpp::Settings & plugin_settings, qt_gui_cpp::Settings & instance_settings) const
{
  (void)plugin_settings;
  (void)instance_settings;
}

std::optional<QTreeWidgetItem *>
RQTPlan::get_plan_item_row(const std::string & action_full_name)
{
  for (int i = 0; i < plan_tree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem * item = plan_tree_->topLevelItem(i);
    if (item->text(0) == QString(action_full_name.c_str())) {
      return item;
    }
  }

  return {};
}

void
RQTPlan::fill_row_info(
  QTreeWidgetItem * row, const plansys2_msgs::msg::ActionExecutionInfo & info)
{
  switch (info.status) {
    case plansys2_msgs::msg::ActionExecutionInfo::NOT_EXECUTED:
      row->setText(1, QString("NOT_EXECUTED"));
      row->setText(
        4,
        "0.0 / " + QString::number(rclcpp::Duration(info.duration).seconds(), 'f', 5));
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::EXECUTING:
      row->setText(1, QString("EXECUTING"));
      row->setText(2, QString::number(info.completion));
      row->setText(3, QString(info.message_status.c_str()));
      row->setText(
        4, QString::number(
          (rclcpp::Time(info.status_stamp) - rclcpp::Time(info.start_stamp)).seconds(), 'f', 5) +
        " / " + QString::number(rclcpp::Duration(info.duration).seconds(), 'f', 5));
      for (int i = 0; i < 5; i++) {
        row->setBackground(i, Qt::darkGreen);
      }
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::FAILED:
      row->setText(1, QString("FAILED"));
      row->setText(2, QString::number(info.completion));
      row->setText(3, QString(info.message_status.c_str()));
      row->setText(
        4, QString::number(
          (rclcpp::Time(info.status_stamp) - rclcpp::Time(info.start_stamp)).seconds(), 'f', 5) +
        " / " + QString::number(rclcpp::Duration(info.duration).seconds(), 'f', 5));
      for (int i = 0; i < 5; i++) {
        row->setBackground(i, Qt::red);
      }
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED:
      row->setText(1, QString("SUCCEEDED"));
      row->setText(2, QString::number(1.0));
      row->setText(3, QString(info.message_status.c_str()));
      row->setText(
        4, QString::number(
          (rclcpp::Time(info.status_stamp) - rclcpp::Time(info.start_stamp)).seconds(), 'f', 5) +
        " / " + QString::number(rclcpp::Duration(info.duration).seconds(), 'f', 5));
      for (int i = 0; i < 5; i++) {
        row->setBackground(i, Qt::green);
      }
      break;
    case plansys2_msgs::msg::ActionExecutionInfo::CANCELLED:
      row->setText(1, QString("CANCELLED"));
      row->setText(2, QString::number(info.completion));
      row->setText(3, QString(info.message_status.c_str()));
      row->setText(
        4, QString::number(
          (rclcpp::Time(info.status_stamp) - rclcpp::Time(info.start_stamp)).seconds(), 'f', 5) +
        " / " + QString::number(rclcpp::Duration(info.duration).seconds(), 'f', 5));
      for (int i = 0; i < 5; i++) {
        row->setBackground(i, Qt::lightGray);
      }
      break;
  }
}

void
RQTPlan::spin_loop()
{
  if (!need_update_info_ && !need_update_plan_) {
    return;
  }

  if (need_update_plan_) {
    plan_ = std::move(new_plan_);

    plan_info_.clear();
    for (const auto & item : plan_->items) {
      std::string full_name = item.action + ":" +
        std::to_string(static_cast<int>(item.time * 1000));
      plan_info_[full_name] = nullptr;
    }

    need_update_info_ = true;
    need_update_plan_ = false;

    plan_tree_->clearAllItems();

    for (const auto & item : plan_->items) {
      auto plan_item = new QTreeWidgetItem();
      std::string full_name = item.action + ":" +
        std::to_string(static_cast<int>(item.time * 1000));
      plan_item->setText(0, QString(full_name.c_str()));
      plan_tree_->addTopLevelItem(plan_item);
    }
  }

  if (need_update_info_) {
    need_update_info_ = false;

    for (const auto & item : plan_->items) {
      std::string full_name = item.action + ":" +
        std::to_string(static_cast<int>(item.time * 1000));

      auto item_row = get_plan_item_row(full_name);
      if (item_row.has_value()) {
        if (plan_info_.find(full_name) != plan_info_.end() && plan_info_[full_name] != nullptr) {
          fill_row_info(item_row.value(), *(plan_info_[full_name]));
        }
      }
    }
  }
  plan_tree_->parentWidget()->repaint();
}

void
RQTPlan::restoreSettings(
  const qt_gui_cpp::Settings & plugin_settings, const qt_gui_cpp::Settings & instance_settings)
{
  (void)plugin_settings;
  (void)instance_settings;
}


}  // namespace rqt_plansys2_plan

PLUGINLIB_EXPORT_CLASS(rqt_plansys2_plan::RQTPlan, rqt_gui_cpp::Plugin)
