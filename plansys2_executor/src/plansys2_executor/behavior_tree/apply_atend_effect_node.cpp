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

#include "plansys2_executor/behavior_tree/apply_atend_effect_node.hpp"

#include <map>
#include <memory>
#include <string>

#include "plansys2_msgs/msg/tree.hpp"

namespace plansys2
{

ApplyAtEndEffect::ApplyAtEndEffect(const std::string & xml_tag_name, const BT::NodeConfig & conf)
: ActionNodeBase(xml_tag_name, conf)
{
  action_map_ =
    config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
    "action_map");

  problem_client_ =
    config().blackboard->get<std::shared_ptr<plansys2::ProblemExpertClient>>("problem_client");
}

BT::NodeStatus ApplyAtEndEffect::tick()
{
  std::string action;
  getInput("action", action);

  auto effect = (*action_map_)[action].action_info.get_at_end_effects();

  if (!(*action_map_)[action].at_end_effects_applied) {
    (*action_map_)[action].at_end_effects_applied = true;
    apply(effect, problem_client_, 0);
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace plansys2
