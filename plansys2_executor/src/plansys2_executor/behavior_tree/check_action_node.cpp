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

#include "plansys2_executor/behavior_tree/check_action_node.hpp"

#include <map>
#include <memory>
#include <sstream>
#include <string>

namespace plansys2
{

CheckAction::CheckAction(const std::string & xml_tag_name, const BT::NodeConfig & conf)
: ActionNodeBase(xml_tag_name, conf)
{
  action_map_ =
    config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
    "action_map");
  action_graph_ = config().blackboard->get<plansys2::bt_builder::Graph::Ptr>("action_graph");
  node_ = config().blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");
}

BT::NodeStatus CheckAction::tick()
{
  std::string xml_action;
  getInput("action", xml_action);

  auto p1 = xml_action.find(':') + 1;      // child time
  auto p2 = xml_action.find(' ', p1) + 1;  // child type
  auto p3 = xml_action.find(' ', p2) + 1;  // parent expression
  auto p4 = xml_action.find(':', p3) + 1;  // parent time
  auto p5 = xml_action.find(' ', p4) + 1;  // parent type

  auto child_id = xml_action.substr(0, p2 - 1);
  auto child_type = xml_action.substr(p2, p3 - p2 - 1);
  auto parent_id = xml_action.substr(p3, p5 - p3 - 1);
  auto parent_type = xml_action.substr(p5);

  if ((*action_map_).find(parent_id) == (*action_map_).end()) {
    return BT::NodeStatus::RUNNING;  // Not started yet
  }

  if ((*action_map_)[parent_id].action_executor == nullptr) {
    return BT::NodeStatus::RUNNING;
  }

  if (
    (parent_type == "START" && (*action_map_)[parent_id].at_start_effects_applied) ||
    (parent_type == "END") && (*action_map_)[parent_id].at_end_effects_applied &&
    (*action_map_)[parent_id].action_executor->is_finished())
  {
    if ((parent_id == child_id) && parent_type == "START" && child_type == "END") {
      return BT::NodeStatus::SUCCESS;
    }

    auto parent_time = (*action_map_)[parent_id].at_end_effects_applied_time;
    if (parent_type == "START") {
      parent_time = (*action_map_)[parent_id].at_start_effects_applied_time;
    }
    auto current_time = node_->now();
    auto dt = current_time.seconds() - parent_time.seconds();

    double lower = 0.0;
    double upper = std::numeric_limits<double>::infinity();
    if (action_graph_) {
      plansys2::bt_builder::Node::Ptr child_node = get_node(child_id, child_type);
      plansys2::bt_builder::Node::Ptr parent_node = get_node(parent_id, parent_type);

      auto in = std::find_if(
        child_node->input_arcs.begin(), child_node->input_arcs.end(),
        [&](std::tuple<plansys2::bt_builder::Node::Ptr, double, double> arc) {
          return std::get<0>(arc) == parent_node;
        });

      lower = std::get<1>(*in);
      upper = std::get<2>(*in);
    }

    if (dt >= lower && dt < upper) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  } else {
    return BT::NodeStatus::RUNNING;
  }
}

plansys2::bt_builder::Node::Ptr CheckAction::get_node(
  const std::string & node_id, const std::string & node_type)
{
  auto it = std::find_if(
    action_graph_->nodes.begin(), action_graph_->nodes.end(),
    [&](plansys2::bt_builder::Node::Ptr n) {
      auto n_id = bt_builder::BTBuilder::to_action_id(n->action, 3);
      auto n_type = bt_builder::BTBuilder::to_string(n->action.type);
      return n_id == node_id && n_type == node_type;
    });
  return *it;
}

}  // namespace plansys2
