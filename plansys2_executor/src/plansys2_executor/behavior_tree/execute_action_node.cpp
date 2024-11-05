// Copyright 2020 Intelligent Robotics Lab
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

#include <string>
#include <map>
#include <memory>

#include "plansys2_executor/behavior_tree/execute_action_node.hpp"

namespace plansys2
{

ExecuteAction::ExecuteAction(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: ActionNodeBase(xml_tag_name, conf)
{
  action_map_ =
    config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
    "action_map");
  node_ = config().blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");
}

void
ExecuteAction::halt()
{
  std::string action;
  getInput("action", action);

  size_t delim = action.find(":");
  auto action_expr = action.substr(0, delim);

  if (action_map_ == nullptr || (*action_map_)[action].action_executor == nullptr) {
    return;
  }

  if ((*action_map_)[action].action_executor->get_status() == BT::NodeStatus::RUNNING) {
    (*action_map_)[action].action_executor->cancel();
  }
}

BT::NodeStatus
ExecuteAction::tick()
{
  std::string action;
  getInput("action", action);

  auto node = config().blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");

  size_t delim = action.find(":");
  auto action_expr = action.substr(0, delim);

  if ((*action_map_)[action].action_executor == nullptr) {
    (*action_map_)[action].action_executor = ActionExecutor::make_shared(action_expr, node_);
  }

  auto retval = (*action_map_)[action].action_executor->tick(node_->now());

  if (retval == BT::NodeStatus::FAILURE) {
    (*action_map_)[action].execution_error_info = "Error executing the action";

    RCLCPP_ERROR_STREAM(
      node->get_logger(),
      "[" << action << "]" << (*action_map_)[action].execution_error_info);
  }

  return retval;
}

}  // namespace plansys2
