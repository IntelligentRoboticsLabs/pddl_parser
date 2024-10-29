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
#include <memory>
#include <vector>
#include <set>
#include <algorithm>
#include <list>
#include <tuple>
#include <map>
#include <utility>
#include <iostream>
#include <sstream>

#include "plansys2_executor/bt_builder_plugins/simple_bt_builder.hpp"

#include "plansys2_problem_expert/Utils.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

SimpleBTBuilder::SimpleBTBuilder()
{
  domain_client_ = std::make_shared<plansys2::DomainExpertClient>();
  problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();
}

void
SimpleBTBuilder::initialize(
  const std::string & bt_action_1,
  const std::string & bt_action_2,
  int precision)
{
  if (bt_action_1 != "") {
    bt_action_ = bt_action_1;
  } else {
    bt_action_ =
      R""""(<Sequence name="ACTION_ID">
WAIT_PREV_ACTIONS
  <ApplyAtStartEffect action="ACTION_ID"/>
  <ReactiveSequence name="ACTION_ID">
    <CheckOverAllReq action="ACTION_ID"/>
    <ExecuteAction action="ACTION_ID"/>
  </ReactiveSequence>
  <CheckAtEndReq action="ACTION_ID"/>
  <ApplyAtEndEffect action="ACTION_ID"/>
</Sequence>
)"""";
  }
}

bool
SimpleBTBuilder::is_action_executable(
  const ActionStamped & action,
  std::unordered_set<plansys2::Instance> & instances,
  std::unordered_set<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions) const
{
  if (action.action.is_action()) {
    return check(action.action.get_overall_requirements(), instances, predicates, functions);
  }

  return check(action.action.get_at_start_requirements(), instances, predicates, functions) &&
         check(action.action.get_at_end_requirements(), instances, predicates, functions) &&
         check(action.action.get_overall_requirements(), instances, predicates, functions);
}

ActionNode::Ptr
SimpleBTBuilder::get_node_satisfy(
  const plansys2_msgs::msg::Tree & requirement,
  const ActionNode::Ptr & node,
  const ActionNode::Ptr & current)
{
  if (node == current) {
    return nullptr;
  }

  ActionNode::Ptr ret = nullptr;

  // Get the state prior to applying the effects
  auto instances = node->instances;
  auto predicates = node->predicates;
  auto functions = node->functions;

  // Is the requirement satisfied before applying the effects?
  bool satisfied_before = check(requirement, instances, predicates, functions);

  // Apply the effects
  if (node->action.action.is_durative_action()) {
    apply(node->action.action.get_at_start_effects(), instances, predicates, functions);
  }
  apply(node->action.action.get_at_end_effects(), instances, predicates, functions);

  // Is the requirement satisfied after applying the effects?
  bool satisfied_after = check(requirement, instances, predicates, functions);

  if (satisfied_after && !satisfied_before) {
    ret = node;
  }

  // Traverse the rest of the graph.
  for (const auto & arc : node->out_arcs) {
    auto node_ret = get_node_satisfy(requirement, arc, current);

    if (node_ret != nullptr) {
      ret = node_ret;
    }
  }

  return ret;
}

void
SimpleBTBuilder::get_node_contradict(
  const ActionNode::Ptr & node,
  const ActionNode::Ptr & current,
  std::list<ActionNode::Ptr> & contradictions)
{
  if (node == current) {
    return;
  }

  // Get the state prior to applying the effects
  auto predicates = node->predicates;
  auto functions = node->functions;
  auto instances = node->instances;

  // Are all of the requirements satisfied?
  if (is_action_executable(current->action, instances, predicates, functions)) {
    // Apply the effects
    if (current->action.action.is_durative_action()) {
      apply(current->action.action.get_at_start_effects(), instances, predicates, functions);
    }

    // Look for a contradiction
    if (!is_action_executable(node->action, instances, predicates, functions)) {
      contradictions.push_back(node);
    }
  }

  // Traverse the rest of the graph
  for (const auto & arc : node->out_arcs) {
    get_node_contradict(arc, current, contradictions);
  }
}

bool
SimpleBTBuilder::is_parallelizable(
  const plansys2::ActionStamped & action,
  const std::unordered_set<plansys2::Instance> & instances,
  const std::unordered_set<plansys2::Predicate> & predicates,
  const std::vector<plansys2::Function> & functions,
  const std::list<ActionNode::Ptr> & nodes) const
{
  // Apply the "at start" effects of the new action.
  auto insts = instances;
  auto preds = predicates;
  auto funcs = functions;
  if (action.action.is_durative_action()) {
    apply(action.action.get_at_start_effects(), insts, preds, funcs);
  }

  // Check the requirements of the actions in the input set.
  for (const auto & other : nodes) {
    if (!is_action_executable(other->action, insts, preds, funcs)) {
      return false;
    }
  }

  // Apply the effects of the actions in the input set one at a time.
  for (const auto & other : nodes) {
    // Apply the "at start" effects of the action.
    insts = instances;
    preds = predicates;
    funcs = functions;

    if (other->action.action.is_durative_action()) {
      apply(other->action.action.get_at_start_effects(), insts, preds, funcs);
    }

    // Check the requirements of the new action.
    if (!is_action_executable(action, insts, preds, funcs)) {
      return false;
    }
  }

  return true;
}

ActionNode::Ptr
SimpleBTBuilder::get_node_satisfy(
  const plansys2_msgs::msg::Tree & requirement,
  const ActionGraph::Ptr & graph,
  const ActionNode::Ptr & current)
{
  ActionNode::Ptr ret;
  for (const auto & root : graph->roots) {
    auto node_satisfy = get_node_satisfy(requirement, root, current);
    if (node_satisfy != nullptr) {
      ret = node_satisfy;
    }
  }

  return ret;
}

std::list<ActionNode::Ptr>
SimpleBTBuilder::get_node_contradict(
  const ActionGraph::Ptr & graph,
  const ActionNode::Ptr & current)
{
  std::list<ActionNode::Ptr> ret;

  for (const auto & root : graph->roots) {
    get_node_contradict(root, current, ret);
  }

  return ret;
}

std::list<ActionNode::Ptr>
SimpleBTBuilder::get_roots(
  std::vector<plansys2::ActionStamped> & action_sequence,
  std::unordered_set<plansys2::Instance> & instances,
  std::unordered_set<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  int & node_counter)
{
  std::list<ActionNode::Ptr> ret;

  auto it = action_sequence.begin();
  while (it != action_sequence.end()) {
    const auto & action = *it;
    if (is_action_executable(action, instances, predicates, functions) &&
      is_parallelizable(action, instances, predicates, functions, ret))
    {
      auto new_root = ActionNode::make_shared();
      new_root->action = action;
      new_root->node_num = node_counter++;
      new_root->level_num = 0;
      new_root->instances = instances;
      new_root->predicates = predicates;
      new_root->functions = functions;

      ret.push_back(new_root);
      it = action_sequence.erase(it);
    } else {
      break;
    }
  }

  return ret;
}

void
SimpleBTBuilder::remove_existing_requirements(
  std::vector<plansys2_msgs::msg::Tree> & requirements,
  std::unordered_set<plansys2::Instance> & instances,
  std::unordered_set<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions) const
{
  auto it = requirements.begin();
  while (it != requirements.end()) {
    if (check(*it, instances, predicates, functions)) {
      it = requirements.erase(it);
    } else {
      ++it;
    }
  }
}

void
SimpleBTBuilder::prune_backwards(ActionNode::Ptr new_node, ActionNode::Ptr node_satisfy)
{
  // Repeat prune to the roots
  for (auto & in : node_satisfy->in_arcs) {
    prune_backwards(new_node, in);
  }

  auto it = node_satisfy->out_arcs.begin();
  while (it != node_satisfy->out_arcs.end()) {
    if (*it == new_node) {
      new_node->in_arcs.remove(node_satisfy);
      it = node_satisfy->out_arcs.erase(it);
    } else {
      ++it;
    }
  }
}

void
SimpleBTBuilder::prune_forward(ActionNode::Ptr current, std::list<ActionNode::Ptr> & used_nodes)
{
  auto it = current->out_arcs.begin();
  while (it != current->out_arcs.end()) {
    if (std::find(used_nodes.begin(), used_nodes.end(), *it) != used_nodes.end()) {
      it = current->out_arcs.erase(it);
    } else {
      prune_forward(*it, used_nodes);
      used_nodes.push_back(*it);

      ++it;
    }
  }
}

void
SimpleBTBuilder::get_state(
  const ActionNode::Ptr & node,
  std::list<ActionNode::Ptr> & used_nodes,
  std::unordered_set<plansys2::Instance> & instances,
  std::unordered_set<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions) const
{
  // Traverse graph to the root
  for (auto & in : node->in_arcs) {
    if (std::find(used_nodes.begin(), used_nodes.end(), in) == used_nodes.end()) {
      get_state(in, used_nodes, instances, predicates, functions);
      if (in->action.action.is_durative_action()) {
        apply(in->action.action.get_at_start_effects(), instances, predicates, functions);
      }
      apply(in->action.action.get_at_end_effects(), instances, predicates, functions);
      used_nodes.push_back(in);
    }
  }
}

std::vector<plansys2_msgs::msg::Tree> SimpleBTBuilder::check_requirements(
  const std::vector<plansys2_msgs::msg::Tree>& requirements,
  std::shared_ptr<plansys2::ActionGraph>& graph,
  std::shared_ptr<plansys2::ActionNode>& new_node
)
{
  std::vector<plansys2_msgs::msg::Tree> non_satisfied_requirements;
  for (const auto& requirement : requirements) {
    if (requirement.nodes[0].node_type == plansys2_msgs::msg::Node::AND ||
        requirement.nodes[0].node_type == plansys2_msgs::msg::Node::EXISTS)
    {
      auto result = check_requirements(parser::pddl::getSubtrees(requirement), graph, new_node);
      non_satisfied_requirements.insert(
        std::end(non_satisfied_requirements), std::begin(result),
        std::end(result));
    } else {
      if (!check_requirement(requirement, graph, new_node)) {
        non_satisfied_requirements.emplace_back(std::move(requirement));
      }
    }
  }
  return std::move(non_satisfied_requirements);
}

bool SimpleBTBuilder::check_requirement(
  const plansys2_msgs::msg::Tree& requirement,
  std::shared_ptr<plansys2::ActionGraph>& graph,
  std::shared_ptr<plansys2::ActionNode>& new_node
)
{
  auto parent = get_node_satisfy(requirement, graph, new_node);
  if (parent != nullptr) {
    prune_backwards(new_node, parent);

    // Create the connections to the parent node
    if (std::find(new_node->in_arcs.begin(), new_node->in_arcs.end(), parent) ==
      new_node->in_arcs.end())
    {
      new_node->in_arcs.push_back(parent);
    }
    if (std::find(parent->out_arcs.begin(), parent->out_arcs.end(), new_node) ==
      parent->out_arcs.end())
    {
      parent->out_arcs.push_back(new_node);
    }
    return true;
  }
  return false;
}

ActionGraph::Ptr
SimpleBTBuilder::get_graph(const plansys2_msgs::msg::Plan & current_plan)
{
  int node_counter = 0;
  int level_counter = 0;
  auto graph = ActionGraph::make_shared();

  auto action_sequence = get_plan_actions(current_plan);
  auto instances = problem_client_->getInstances();
  auto predicates = problem_client_->getPredicates();
  auto functions = problem_client_->getFunctions();

  // Get root actions that can be run in parallel
  graph->roots = get_roots(action_sequence, instances, predicates, functions, node_counter);

  // Build the rest of the graph
  while (!action_sequence.empty()) {
    auto new_node = ActionNode::make_shared();
    new_node->action = *action_sequence.begin();
    new_node->node_num = node_counter++;
    float time = new_node->action.time;

    auto level = graph->levels.find(time);
    if (level == graph->levels.end()) {
      level_counter++;
      std::list<ActionNode::Ptr> new_level;
      new_level.push_back(new_node);
      graph->levels.insert({time, new_level});
    } else {
      level->second.push_back(new_node);
    }
    new_node->level_num = level_counter;

    auto over_all_requirements = parser::pddl::getSubtrees(
      new_node->action.action.get_overall_requirements());
    auto at_start_requirements = parser::pddl::getSubtrees(
      new_node->action.action.get_at_start_requirements());
    auto at_end_requirements = parser::pddl::getSubtrees(
      new_node->action.action.get_at_end_requirements());

    std::vector<plansys2_msgs::msg::Tree> requirements;
    requirements.insert(
      std::end(requirements), std::begin(at_start_requirements),
      std::end(at_start_requirements));
    requirements.insert(
      std::end(requirements), std::begin(over_all_requirements),
      std::end(over_all_requirements));
    requirements.insert(
      std::end(requirements), std::begin(at_end_requirements),
      std::end(at_end_requirements));

    // Look for satisfying nodes
    // A satisfying node is a node with an effect that satisfies a requirement of the new node
    requirements = check_requirements(requirements, graph, new_node);

    // Look for contradicting parallel actions
    // A1 and A2 cannot run in parallel if the effects of A1 contradict the requirements of A2
    auto contradictions = get_node_contradict(graph, new_node);
    for (const auto parent : contradictions) {
      prune_backwards(new_node, parent);

      // Create the connections to the parent node
      if (std::find(new_node->in_arcs.begin(), new_node->in_arcs.end(), parent) ==
        new_node->in_arcs.end())
      {
        new_node->in_arcs.push_back(parent);
      }
      if (std::find(parent->out_arcs.begin(), parent->out_arcs.end(), new_node) ==
        parent->out_arcs.end())
      {
        parent->out_arcs.push_back(new_node);
      }
    }

    // Compute the state up to the new node
    // The effects of the new node are not applied
    std::list<ActionNode::Ptr> used_nodes;
    instances = problem_client_->getInstances();
    predicates = problem_client_->getPredicates();
    functions = problem_client_->getFunctions();
    get_state(new_node, used_nodes, instances, predicates, functions);
    new_node->instances = instances;
    new_node->predicates = predicates;
    new_node->functions = functions;

    // Check any requirements that do not have satisfying nodes.
    // These should be satisfied by the initial state.
    remove_existing_requirements(requirements, instances, predicates, functions);
    for (const auto & req : requirements) {
      std::cerr << "[ERROR] requirement not met: [" <<
        parser::pddl::toString(req) << "]" << std::endl;
    }

    // Return and empyt graph to fail the service call
    if (!requirements.empty()) {
      return nullptr;
    }

    action_sequence.erase(action_sequence.begin());
  }

  return graph;
}

std::string
SimpleBTBuilder::get_tree(const plansys2_msgs::msg::Plan & current_plan)
{
  graph_ = get_graph(current_plan);

  // If graph was not generated, return an empty string.
  // This can be used to fails the serveice call
  if (!graph_) {
    return "";
  }

  std::list<ActionNode::Ptr> used_actions;
  for (auto & root : graph_->roots) {
    prune_forward(root, used_actions);
  }

  std::list<std::string> used_nodes;

  if (graph_->roots.size() > 1) {
    bt_ = std::string("<root BTCPP_format=\"4\" main_tree_to_execute=\"MainTree\">\n") +
      t(1) + "<BehaviorTree ID=\"MainTree\">\n" +
      t(2) + "<Parallel success_count=\"" + std::to_string(graph_->roots.size()) +
      "\" failure_count=\"1\">\n";

    for (const auto & node : graph_->roots) {
      bt_ = bt_ + get_flow_tree(node, used_nodes, 3);
    }

    bt_ = bt_ + t(2) + "</Parallel>\n" +
      t(1) + "</BehaviorTree>\n</root>\n";
  } else {
    bt_ = std::string("<root BTCPP_format=\"4\" main_tree_to_execute=\"MainTree\">\n") +
      t(1) + "<BehaviorTree ID=\"MainTree\">\n";

    bt_ = bt_ + get_flow_tree(*graph_->roots.begin(), used_nodes, 2);

    bt_ = bt_ + t(1) + "</BehaviorTree>\n</root>\n";
  }

  return bt_;
}

std::string
SimpleBTBuilder::get_dotgraph(
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map,
  bool enable_legend,
  bool enable_print_graph)
{
  if (enable_print_graph) {
    print_graph(graph_);
  }

  // create xdot graph
  std::stringstream ss;
  ss.setf(std::ios::fixed);
  ss.precision(2);
  ss << "digraph plan {\n";

  int tab_level = 1;
  // dotgraph formatting options
  ss << t(tab_level);
  ss << "node[shape=box];\n";
  ss << t(tab_level);
  ss << "rankdir=TB;\n";

  // define all the levels and nodes
  ss << t(tab_level);
  ss << "subgraph cluster_0 {\n";

  double duration = 0.0;
  for (auto & node : graph_->roots) {
    duration += node->action.duration;
  }

  tab_level = 2;
  ss << t(tab_level);
  ss << "label = \"Start: 0.0 s\nDuration: " << duration << " s\";\n";
  ss << t(tab_level);
  ss << "style = rounded;\n";
  ss << t(tab_level);
  ss << "color = yellow3;\n";
  ss << t(tab_level);
  ss << "bgcolor = lemonchiffon;\n";
  ss << t(tab_level);
  ss << "labeljust = l;\n";

  tab_level = 3;
  for (auto & node : graph_->roots) {
    ss << get_node_dotgraph(node, action_map, tab_level);
  }
  tab_level = 2;

  ss << t(tab_level);
  ss << "}\n";

  int max_level = 0;
  int max_node = 0;
  for (auto & level : graph_->levels) {
    if (!level.second.empty()) {
      ss << t(tab_level);
      ss << "subgraph cluster_" << level.second.front()->level_num << " {\n";
      max_level = std::max(max_level, level.second.front()->level_num);

      tab_level = 2;
      ss << t(tab_level);
      ss << "label = \"Start: " << level.second.front()->action.time << " s\n";
      ss << "Duration: " << level.second.front()->action.duration << " s\";\n";
      ss << t(tab_level);
      ss << "style = rounded;\n";
      ss << t(tab_level);
      ss << "color = yellow3;\n";
      ss << t(tab_level);
      ss << "bgcolor = lemonchiffon;\n";
      ss << t(tab_level);
      ss << "labeljust = l;\n";

      tab_level = 3;
      for (auto & node : level.second) {
        max_node = std::max(max_node, node->node_num);
        ss << get_node_dotgraph(node, action_map, tab_level);
      }
      tab_level = 2;

      ss << t(tab_level);
      ss << "}\n";
    }
  }

  // define the edges
  std::set<std::string> edges;
  for (const auto & graph_root : graph_->roots) {
    get_flow_dotgraph(graph_root, edges);
  }

  tab_level = 1;
  for (const auto & edge : edges) {
    ss << t(tab_level) << edge;
  }

  if (enable_legend) {
    max_level++;
    max_node++;
    addDotGraphLegend(ss, tab_level, max_level, max_node);
  }

  ss << "}";

  return ss.str();
}

std::string
SimpleBTBuilder::get_flow_tree(
  ActionNode::Ptr node,
  std::list<std::string> & used_nodes,
  int level)
{
  std::string ret;
  int l = level;

  const std::string action_id = "(" + node->action.action.get_action_string() + "):" +
    std::to_string(static_cast<int>(node->action.time * 1000));

  if (std::find(used_nodes.begin(), used_nodes.end(), action_id) != used_nodes.end()) {
    return t(l) + "<WaitAction action=\"" + action_id + "\"/>\n";
  }

  used_nodes.push_back(action_id);

  if (node->out_arcs.size() == 0) {
    ret = ret + execution_block(node, l);
  } else if (node->out_arcs.size() == 1) {
    ret = ret + t(l) + "<Sequence name=\"" + action_id + "\">\n";
    ret = ret + execution_block(node, l + 1);

    for (const auto & child_node : node->out_arcs) {
      ret = ret + get_flow_tree(child_node, used_nodes, l + 1);
    }

    ret = ret + t(l) + "</Sequence>\n";
  } else {
    ret = ret + t(l) + "<Sequence name=\"" + action_id + "\">\n";
    ret = ret + execution_block(node, l + 1);

    ret = ret + t(l + 1) +
      "<Parallel success_count=\"" + std::to_string(node->out_arcs.size()) +
      "\" failure_count=\"1\">\n";

    for (const auto & child_node : node->out_arcs) {
      ret = ret + get_flow_tree(child_node, used_nodes, l + 2);
    }

    ret = ret + t(l + 1) + "</Parallel>\n";
    ret = ret + t(l) + "</Sequence>\n";
  }

  return ret;
}

void
SimpleBTBuilder::get_flow_dotgraph(
  ActionNode::Ptr node,
  std::set<std::string> & edges)
{
  for (const auto & arc : node->out_arcs) {
    std::string edge = std::to_string(node->node_num) + "->" + std::to_string(arc->node_num) +
      ";\n";
    edges.insert(edge);
    get_flow_dotgraph(arc, edges);
  }
}

std::string
SimpleBTBuilder::get_node_dotgraph(
  ActionNode::Ptr node, std::shared_ptr<std::map<std::string,
  ActionExecutionInfo>> action_map, int level)
{
  std::stringstream ss;
  ss << t(level);
  ss << node->node_num << " [label=\"" << node->action.action.get_action_string() <<
    "\"";
  ss << "labeljust=c,style=filled";

  auto status = get_action_status(node->action, action_map);
  switch (status) {
    case ActionExecutor::RUNNING:
      ss << ",color=blue,fillcolor=skyblue";
      break;
    case ActionExecutor::SUCCESS:
      ss << ",color=green4,fillcolor=seagreen2";
      break;
    case ActionExecutor::FAILURE:
      ss << ",color=red,fillcolor=pink";
      break;
    case ActionExecutor::CANCELLED:
      ss << ",color=red,fillcolor=pink";
      break;
    case ActionExecutor::IDLE:
    case ActionExecutor::DEALING:
    default:
      ss << ",color=yellow3,fillcolor=lightgoldenrod1";
      break;
  }
  ss << "];\n";
  return ss.str();
}

ActionExecutor::Status SimpleBTBuilder::get_action_status(
  ActionStamped action_stamped,
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map)
{
  auto index = "(" + action_stamped.action.get_action_string() + "):" +
    std::to_string(static_cast<int>(action_stamped.time * 1000));
  if ((*action_map)[index].action_executor) {
    return (*action_map)[index].action_executor->get_internal_status();
  } else {
    return ActionExecutor::IDLE;
  }
}

void SimpleBTBuilder::addDotGraphLegend(
  std::stringstream & ss, int tab_level, int level_counter,
  int node_counter)
{
  int legend_counter = level_counter;
  int legend_node_counter = node_counter;
  ss << t(tab_level);
  ss << "subgraph cluster_" << legend_counter++ << " {\n";
  tab_level++;
  ss << t(tab_level);
  ss << "label = \"Legend\";\n";

  ss << t(tab_level);
  ss << "subgraph cluster_" << legend_counter++ << " {\n";
  tab_level++;
  ss << t(tab_level);
  ss << "label = \"Plan Action Start (sec): X.X s\n Duration (sec): X.X s\";\n";
  ss << t(tab_level);
  ss << "style = rounded;\n";
  ss << t(tab_level);
  ss << "color = yellow3;\n";
  ss << t(tab_level);
  ss << "bgcolor = lemonchiffon;\n";
  ss << t(tab_level);
  ss << "labeljust = l;\n";
  ss << t(tab_level);
  ss << legend_node_counter++ <<
    " [label=\n\"Finished action\n\",labeljust=c,style=filled,color=green4,fillcolor=seagreen2];\n";
  ss << t(tab_level);
  ss << legend_node_counter++ <<
    " [label=\n\"Failed action\n\",labeljust=c,style=filled,color=red,fillcolor=pink];\n";
  ss << t(tab_level);
  ss << legend_node_counter++ <<
    " [label=\n\"Current action\n\",labeljust=c,style=filled,color=blue,fillcolor=skyblue];\n";
  ss << t(tab_level);
  ss << legend_node_counter++ << " [label=\n\"Future action\n\",labeljust=c,style=filled," <<
    "color=yellow3,fillcolor=lightgoldenrod1];\n";
  tab_level--;
  ss << t(tab_level);
  ss << "}\n";

  ss << t(tab_level);
  for (int i = node_counter; i < legend_node_counter; i++) {
    if (i > node_counter) {
      ss << "->";
    }
    ss << i;
  }
  ss << " [style=invis];\n";

  tab_level--;
  ss << t(tab_level);
  ss << "}\n";
}

std::string
SimpleBTBuilder::t(int level)
{
  std::string ret;
  for (int i = 0; i < level; i++) {
    ret = ret + "  ";
  }
  return ret;
}

void replace(std::string & str, const std::string & from, const std::string & to)
{
  size_t start_pos = std::string::npos;
  while ((start_pos = str.find(from)) != std::string::npos) {
    str.replace(start_pos, from.length(), to);
  }
}

std::string
SimpleBTBuilder::execution_block(const ActionNode::Ptr & node, int l)
{
  const auto & action = node->action;
  std::string ret;
  std::string ret_aux = bt_action_;

  const std::string action_id = "(" + node->action.action.get_action_string() + "):" +
    std::to_string(static_cast<int>(action.time * 1000));

  std::string wait_actions;
  for (const auto & previous_node : node->in_arcs) {
    const std::string parent_action_id = "(" +
      previous_node->action.action.get_action_string() + "):" +
      std::to_string(static_cast<int>( previous_node->action.time * 1000));
    wait_actions = wait_actions + t(1) + "<WaitAction action=\"" + parent_action_id + "\"/>";

    if (previous_node != *node->in_arcs.rbegin()) {
      wait_actions = wait_actions + "\n";
    }
  }

  replace(ret_aux, "ACTION_ID", action_id);
  replace(ret_aux, "WAIT_PREV_ACTIONS", wait_actions);

  std::istringstream f(ret_aux);
  std::string line;
  while (std::getline(f, line)) {
    if (line != "") {
      ret = ret + t(l) + line + "\n";
    }
  }
  return ret;
}

std::vector<ActionStamped>
SimpleBTBuilder::get_plan_actions(const plansys2_msgs::msg::Plan & plan)
{
  std::vector<ActionStamped> ret;

  for (auto & item : plan.items) {
    ActionStamped action_stamped;

    action_stamped.time = item.time;
    action_stamped.duration = item.duration;
    auto actions = domain_client_->getActions();
    if (std::find(actions.begin(), actions.end(), get_action_name(item.action)) != actions.end()) {
      action_stamped.action.action =
        domain_client_->getAction(
        get_action_name(item.action), get_action_params(item.action));
    } else {
      action_stamped.action.action =
        domain_client_->getDurativeAction(
        get_action_name(item.action), get_action_params(item.action));
    }

    ret.push_back(action_stamped);
  }

  return ret;
}

void
SimpleBTBuilder::print_node(
  const plansys2::ActionNode::Ptr & node,
  int level,
  std::set<plansys2::ActionNode::Ptr> & used_nodes) const
{
  std::cerr << std::string(level, '\t') << "[" << node->action.time << "] ";
  std::cerr << node->action.action.get_action_name() << " ";
  for (const auto & param : node->action.action.get_action_params()) {
    std::cerr << param.name << " ";
  }
  std::cerr << " in arcs " << node->in_arcs.size() << "  ";
  std::cerr << " out arcs " << node->out_arcs.size() << std::endl;

  for (const auto & out : node->out_arcs) {
    print_node(out, level + 1, used_nodes);
  }
}

void
SimpleBTBuilder::print_graph(const plansys2::ActionGraph::Ptr & graph) const
{
  std::set<plansys2::ActionNode::Ptr> used_nodes;
  for (const auto & root : graph->roots) {
    print_node(root, 0, used_nodes);
  }
}

void
SimpleBTBuilder::print_node_csv(const plansys2::ActionNode::Ptr & node, uint32_t root_num) const
{
  std::string out_str = std::to_string(root_num) + ", " +
    std::to_string(node->node_num) + ", " +
    std::to_string(node->level_num) + ", " +
    node->action.action.get_action_string();
  for (const auto & arc : node->out_arcs) {
    out_str = out_str + ", " + arc->action.action.get_action_string();
  }
  std::cerr << out_str << std::endl;
  for (const auto & out : node->out_arcs) {
    print_node_csv(out, root_num);
  }
}

void
SimpleBTBuilder::print_graph_csv(const plansys2::ActionGraph::Ptr & graph) const
{
  uint32_t root_num = 0;
  for (const auto & root : graph->roots) {
    print_node_csv(root, root_num);
    root_num++;
  }
}

void
SimpleBTBuilder::get_node_tabular(
  const plansys2::ActionNode::Ptr & node,
  uint32_t root_num,
  std::vector<std::tuple<uint32_t, uint32_t, uint32_t, std::string>> & graph) const
{
  graph.push_back(
    std::make_tuple(
      root_num, node->node_num, node->level_num,
      node->action.action.get_action_string()));
  for (const auto & out : node->out_arcs) {
    get_node_tabular(out, root_num, graph);
  }
}

std::vector<std::tuple<uint32_t, uint32_t, uint32_t, std::string>>
SimpleBTBuilder::get_graph_tabular(const plansys2::ActionGraph::Ptr & graph) const
{
  std::vector<std::tuple<uint32_t, uint32_t, uint32_t, std::string>> graph_tabular;
  uint32_t root_num = 0;
  for (const auto & root : graph->roots) {
    get_node_tabular(root, root_num, graph_tabular);
    root_num++;
  }
  return graph_tabular;
}

}  // namespace plansys2
