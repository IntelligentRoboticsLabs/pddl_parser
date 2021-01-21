// Copyright 2019 Intelligent Robotics Lab
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

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "plansys2_bt_actions/BTAction.hpp"

namespace plansys2
{

BTAction::BTAction(
  const std::string & action,
  const std::chrono::nanoseconds & rate)
: ActionExecutorClient(action, rate)
{
  declare_parameter("bt_xml_file");
  declare_parameter("plugins");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BTAction::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  get_parameter("action_name", action_);
  get_parameter("bt_xml_file", bt_xml_file_);

  RCLCPP_INFO_STREAM(get_logger(), "action_name: [" << action_ << "]");
  RCLCPP_INFO_STREAM(get_logger(), "bt_xml_file: [" << bt_xml_file_ << "]");

  auto plugin_lib_names = get_parameter("plugins").as_string_array();
  for (auto plugin : plugin_lib_names) {
    RCLCPP_INFO_STREAM(get_logger(), "plugin: [" << plugin << "]");
  }

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  for (auto plugin : plugin_lib_names) {
    factory.registerFromPlugin(loader.getOSName(plugin));
  }

  auto node = rclcpp::Node::make_shared(std::string(get_name()) + "_aux");
  blackboard_ = BT::Blackboard::create();
  blackboard_->set("node", node);

  tree_ = factory.createTreeFromFile(bt_xml_file_, blackboard_);

  return ActionExecutorClient::on_configure(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BTAction::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  for (int i = 0; i < get_arguments().size(); i++) {
    std::string argname = "arg" + std::to_string(i);
    blackboard_->set(argname, get_arguments()[i]);
  }

  return ActionExecutorClient::on_activate(previous_state);
}

void
BTAction::do_work()
{
  auto result = tree_.rootNode()->executeTick();

  switch (result) {
    case BT::NodeStatus::SUCCESS:
      finish(true, 1.0, "Action completed");
      break;
    case BT::NodeStatus::RUNNING:
      send_feedback(0.0, "Action running");
      break;
    case BT::NodeStatus::FAILURE:
      finish(false, 1.0, "Action failed");
      break;
  }
}

}  // namespace plansys2
