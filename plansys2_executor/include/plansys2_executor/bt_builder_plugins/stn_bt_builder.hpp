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

#ifndef PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__STN_BT_BUILDER_HPP_
#define PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__STN_BT_BUILDER_HPP_

#include <eigen3/Eigen/Dense>

#include <list>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <unordered_set>
#include <vector>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

namespace plansys2
{
namespace bt_builder
{

struct StateVec
{
  std::unordered_set<plansys2::Instance> instances;
  std::unordered_set<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
};

class STNBTBuilder : public BTBuilder
{
public:
  STNBTBuilder();
  void initialize(
    const std::string & bt_action_1 = "",
    const std::string & bt_action_2 = "",
    int precision = 3);

  std::string get_tree(const plansys2_msgs::msg::Plan & current_plan);
  Graph::Ptr get_graph() {return stn_;}
  bool propagate(const Graph::Ptr stn);
  std::string get_dotgraph(
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map,
    bool enable_legend = false,
    bool enable_print_graph = false);

protected:
  Graph::Ptr build_stn(const plansys2_msgs::msg::Plan & plan) const;
  std::string build_bt(const Graph::Ptr stn) const;

  Graph::Ptr init_graph(const plansys2_msgs::msg::Plan & plan) const;
  std::vector<ActionStamped> get_plan_actions(const plansys2_msgs::msg::Plan & plan) const;

  std::set<int> get_happenings(const plansys2_msgs::msg::Plan & plan) const;
  std::set<int>::iterator get_happening(int time, const std::set<int> & happenings) const;
  std::set<int>::iterator get_previous(int time, const std::set<int> & happenings) const;

  std::multimap<int, ActionStamped> get_simple_plan(const plansys2_msgs::msg::Plan & plan) const;

  std::map<int, StateVec> get_states(
    const std::set<int> & happenings,
    const std::multimap<int, ActionStamped> & plan) const;
  plansys2_msgs::msg::Tree from_state(
    const std::unordered_set<plansys2::Predicate> & preds,
    const std::vector<plansys2::Function> & funcs) const;

  std::vector<Node::Ptr> get_nodes(
    const ActionStamped & action,
    const Graph::Ptr graph) const;

  bool is_match(
    const Node::Ptr node,
    const ActionStamped & action) const;

  std::vector<std::pair<int, ActionStamped>> get_parents(
    const std::pair<int, ActionStamped> & action,
    const std::multimap<int, ActionStamped> & plan,
    const std::set<int> & happenings,
    const std::map<int, StateVec> & states) const;

  std::vector<std::pair<int, ActionStamped>> get_satisfy(
    const std::pair<int, ActionStamped> & action,
    const std::multimap<int, ActionStamped> & plan,
    const std::set<int> & happenings,
    const std::map<int, StateVec> & states) const;

  std::vector<std::pair<int, ActionStamped>> get_threat(
    const std::pair<int, ActionStamped> & action,
    const std::multimap<int, ActionStamped> & plan,
    const std::set<int> & happenings,
    const std::map<int, StateVec> & states) const;

  bool can_apply(
    const std::pair<int, ActionStamped> & action,
    const std::multimap<int, ActionStamped> & plan,
    const int & time,
    StateVec & state) const;

  StateVec get_diff(const StateVec & X_1, const StateVec & X_2) const;
  StateVec get_intersection(const StateVec & X_1, const StateVec & X_2) const;

  plansys2_msgs::msg::Tree get_conditions(const ActionStamped & action) const;
  plansys2_msgs::msg::Tree get_effects(const ActionStamped & action) const;

  void prune_paths(Node::Ptr current, Node::Ptr previous) const;
  bool check_paths(Node::Ptr current, Node::Ptr previous) const;

  Eigen::MatrixXd get_distance_matrix(const Graph::Ptr stn) const;
  void floyd_warshall(Eigen::MatrixXd & dist) const;

  std::string get_flow(
    const Node::Ptr node,
    const Node::Ptr prev_node,
    std::set<Node::Ptr> & used,
    const int & level) const;

  std::string start_execution_block(
    const Node::Ptr node,
    const int & l) const;
  std::string end_execution_block(
    const Node::Ptr node,
    const int & l) const;

  void get_flow_dotgraph(
    Node::Ptr node,
    std::set<std::string> & edges);
  std::string get_node_dotgraph(
    Node::Ptr node,
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map);
  ActionExecutor::Status get_action_status(
    ActionStamped action,
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map);
  std::string add_dot_graph_legend(
    int level_counter,
    int node_counter);
  void print_graph(const plansys2::Graph::Ptr graph) const;
  void print_node(const Node::Ptr node, int level) const;
  void print_arcs(const plansys2::Graph::Ptr graph) const;

  void replace(std::string & str, const std::string & from, const std::string & to) const;

  bool is_end(
    const std::tuple<Node::Ptr, double, double> & edge,
    const ActionStamped & action) const;

  std::string t(const int & level) const;

  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;

  Graph::Ptr stn_;
  std::string bt_start_action_;
  std::string bt_end_action_;
  int action_time_precision_;
};

}  // namespace bt_builder
}  // namespace plansys2


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(plansys2::bt_builder::STNBTBuilder, plansys2::bt_builder::BTBuilder)

#endif  // PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__STN_BT_BUILDER_HPP_
