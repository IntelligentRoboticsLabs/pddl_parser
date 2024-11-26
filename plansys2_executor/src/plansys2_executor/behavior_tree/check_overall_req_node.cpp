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

#include "plansys2_executor/behavior_tree/check_overall_req_node.hpp"

#include <map>
#include <memory>
#include <string>
#include <tuple>

#include "plansys2_msgs/msg/action.hpp"

namespace plansys2
{

CheckOverAllReq::CheckOverAllReq(const std::string & xml_tag_name, const BT::NodeConfig & conf)
: ActionNodeBase(xml_tag_name, conf)
{
  action_map_ =
    config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
    "action_map");

  problem_client_ =
    config().blackboard->get<std::shared_ptr<plansys2::ProblemExpertClient>>("problem_client");
}

BT::NodeStatus CheckOverAllReq::tick()
{
  std::string action;
  getInput("action", action);

  auto node = config().blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");

  auto reqs = (*action_map_)[action].action_info.get_overall_requirements();

  if (!check(reqs, problem_client_)) {
    (*action_map_)[action].execution_error_info = "Error checking over all requirements";

    RCLCPP_ERROR_STREAM(
      node->get_logger(), "[" << action << "]" << (*action_map_)[action].execution_error_info
                              << ": " << parser::pddl::toString(reqs));

    return BT::NodeStatus::FAILURE;
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace plansys2
