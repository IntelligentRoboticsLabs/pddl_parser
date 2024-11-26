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

#ifndef PLANSYS2_EXECUTOR__BEHAVIOR_TREE__WAIT_ACTION_NODE_HPP_
#define PLANSYS2_EXECUTOR__BEHAVIOR_TREE__WAIT_ACTION_NODE_HPP_

#include <map>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/behavior_tree/execute_action_node.hpp"

namespace plansys2
{

class WaitAction : public BT::ActionNodeBase
{
public:
  WaitAction(const std::string & xml_tag_name, const BT::NodeConfig & conf);

  void halt() {}
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("action", "Action to be executed"),
      });
  }

private:
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__BEHAVIOR_TREE__WAIT_ACTION_NODE_HPP_
