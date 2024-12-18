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
#include <tuple>

#include "plansys2_executor/behavior_tree/wait_atstart_req_node.hpp"
#include "plansys2_msgs/msg/tree.hpp"

namespace plansys2
{

WaitAtStartReq::WaitAtStartReq(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: ActionNodeBase(xml_tag_name, conf)
{
  action_map_ =
    config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
    "action_map");

  problem_client_ =
    config().blackboard->get<std::shared_ptr<plansys2::ProblemExpertClient>>(
    "problem_client");
}

BT::NodeStatus
WaitAtStartReq::tick()
{
  std::string action;
  getInput("action", action);

  if (action_map_ != nullptr && (*action_map_)[action].action_executor != nullptr &&
    (*action_map_)[action].action_executor->get_internal_status() == ActionExecutor::RUNNING)
  {
    return BT::NodeStatus::SUCCESS;
  }

  auto node = config().blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");

  auto reqs_as = (*action_map_)[action].action_info.get_at_start_requirements();
  auto reqs_oa = (*action_map_)[action].action_info.get_overall_requirements();

  bool check_as = check(reqs_as, problem_client_);
  if (!check_as) {
    (*action_map_)[action].execution_error_info = "Error checking at start reqs";

    RCLCPP_ERROR_STREAM(
      node->get_logger(),
      "[" << action << "]" << (*action_map_)[action].execution_error_info << ": " <<
        parser::pddl::toString(reqs_as));

    return BT::NodeStatus::RUNNING;
  }

  bool check_oa = check(reqs_oa, problem_client_);
  if (!check_oa) {
    (*action_map_)[action].execution_error_info = "Error checking over all reqs";

    RCLCPP_ERROR_STREAM(
      node->get_logger(),
      "[" << action << "]" << (*action_map_)[action].execution_error_info << ": " <<
        parser::pddl::toString(reqs_oa));

    return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace plansys2
