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

#ifndef PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__SIMPLE_BT_BUILDER_HPP_
#define PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__SIMPLE_BT_BUILDER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <set>
#include <list>
#include <map>
#include <utility>
#include <tuple>
#include <unordered_set>

#include "std_msgs/msg/empty.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_core/Types.hpp"
#include "plansys2_msgs/msg/durative_action.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace std {
  template <typename T1, typename T2>
  struct hash<std::pair<T1, T2>> {
    std::size_t operator()(const std::pair<T1, T2> & p) const {
      std::size_t seed = 0;
      hash_combine(seed, p.first);
      hash_combine(seed, p.second);
      return seed;
    }

  private:
    // Hash combine function
    template <typename T>
    inline void hash_combine(std::size_t & seed, const T & value) const {
      seed ^= std::hash<T>{}(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
  };
} // namespace std

namespace plansys2
{
namespace bt_builder
{

struct ActionNode
{
  using Ptr = std::shared_ptr<plansys2::bt_builder::ActionNode>;
  static Ptr make_shared() {return std::make_shared<plansys2::bt_builder::ActionNode>();}

  plansys2::bt_builder::ActionStamped action;
  int node_num;
  int level_num;

  plansys2::State state;

  std::list<plansys2::bt_builder::ActionNode::Ptr> in_arcs;
  std::list<plansys2::bt_builder::ActionNode::Ptr> out_arcs;
};

struct ActionGraph
{
  using Ptr = std::shared_ptr<plansys2::bt_builder::ActionGraph>;
  static Ptr make_shared() {return std::make_shared<plansys2::bt_builder::ActionGraph>();}

  std::list<plansys2::bt_builder::ActionNode::Ptr> roots;
  std::map<float, std::list<plansys2::bt_builder::ActionNode::Ptr>> levels;
};

class SimpleBTBuilder : public BTBuilder
{
public:
  SimpleBTBuilder();
  void initialize(
    const std::string & bt_action_1 = "",
    const std::string & bt_action_2 = "",
    int precision = 3);

  std::string get_tree(const plansys2_msgs::msg::Plan & current_plan);
  plansys2::bt_builder::Graph::Ptr get_graph() {return nullptr;}
  bool propagate(plansys2::bt_builder::Graph::Ptr) {return true;}
  std::string get_dotgraph(
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map,
    bool enable_legend = false,
    bool enable_print_graph = false);

protected:
  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;

  plansys2::bt_builder::ActionGraph::Ptr graph_;
  std::string bt_;
  std::string bt_action_;

  std::unordered_map<std::pair<std::string, plansys2::State>, plansys2::State> action_state_cache;

  plansys2::bt_builder::ActionGraph::Ptr get_graph(const plansys2_msgs::msg::Plan & current_plan);

  std::vector<plansys2::bt_builder::ActionStamped> get_plan_actions(const plansys2_msgs::msg::Plan & plan);
  void prune_backwards(plansys2::bt_builder::ActionNode::Ptr new_node, plansys2::bt_builder::ActionNode::Ptr node_satisfy);
  void prune_forward(plansys2::bt_builder::ActionNode::Ptr current, std::list<plansys2::bt_builder::ActionNode::Ptr> & used_nodes);
  void get_state(
    const plansys2::bt_builder::ActionNode::Ptr & node,
    std::list<plansys2::bt_builder::ActionNode::Ptr> & used_nodes,
    plansys2::State & state);

  std::vector<plansys2_msgs::msg::Tree> check_requirements(
    const std::vector<plansys2_msgs::msg::Tree>& requirements,
    std::shared_ptr<plansys2::bt_builder::ActionGraph>& graph,
    std::shared_ptr<plansys2::bt_builder::ActionNode>& new_node
  );
  bool check_requirement(
    const plansys2_msgs::msg::Tree& requirement,
    std::shared_ptr<plansys2::bt_builder::ActionGraph>& graph,
    std::shared_ptr<plansys2::bt_builder::ActionNode>& new_node
  );

  bool is_action_executable(
    const plansys2::bt_builder::ActionStamped & action,
    const plansys2::State & state);
  std::list<plansys2::bt_builder::ActionNode::Ptr> get_roots(
    std::vector<plansys2::bt_builder::ActionStamped> & action_sequence,
    const plansys2::State & state,
    int & node_counter);
  plansys2::bt_builder::ActionNode::Ptr get_node_satisfy(
    const plansys2_msgs::msg::Tree & requirement,
    const plansys2::bt_builder::ActionGraph::Ptr & graph,
    const plansys2::bt_builder::ActionNode::Ptr & current);
  plansys2::bt_builder::ActionNode::Ptr get_node_satisfy(
    const plansys2_msgs::msg::Tree & requirement,
    const plansys2::bt_builder::ActionNode::Ptr & node,
    const plansys2::bt_builder::ActionNode::Ptr & current);
  std::list<plansys2::bt_builder::ActionNode::Ptr> get_node_contradict(
    const plansys2::bt_builder::ActionGraph::Ptr & graph,
    const plansys2::bt_builder::ActionNode::Ptr & current);
  void get_node_contradict(
    const plansys2::bt_builder::ActionNode::Ptr & node,
    const plansys2::bt_builder::ActionNode::Ptr & current,
    std::list<plansys2::bt_builder::ActionNode::Ptr> & parents);
  void remove_existing_requirements(
    std::vector<plansys2_msgs::msg::Tree> & requirements,
    const plansys2::State & state);
  bool is_parallelizable(
    const plansys2::bt_builder::ActionStamped & action,
    const plansys2::State & state,
    const std::list<plansys2::bt_builder::ActionNode::Ptr> & ret);
  void apply_action_to_state(
    const plansys2::bt_builder::ActionStamped & action,
    plansys2::State & state);

  std::string get_flow_tree(
    plansys2::bt_builder::ActionNode::Ptr node,
    std::list<std::string> & used_nodes,
    int level = 0);
  void get_flow_dotgraph(plansys2::bt_builder::ActionNode::Ptr node, std::set<std::string> & edges);
  std::string get_node_dotgraph(
    plansys2::bt_builder::ActionNode::Ptr node, std::shared_ptr<std::map<std::string,
    ActionExecutionInfo>> action_map, int level = 0);
  ActionExecutor::Status get_action_status(
    plansys2::bt_builder::ActionStamped action,
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map);
  void addDotGraphLegend(
    std::stringstream & ss, int tab_level, int level_counter,
    int node_counter);

  std::string t(int level);

  std::string execution_block(const plansys2::bt_builder::ActionNode::Ptr & node, int l);
  void print_node(
    const plansys2::bt_builder::ActionNode::Ptr & node,
    int level,
    std::set<plansys2::bt_builder::ActionNode::Ptr> & used_nodes) const;

  void print_graph(const plansys2::bt_builder::ActionGraph::Ptr & graph) const;

  void print_node_csv(const plansys2::bt_builder::ActionNode::Ptr & node, uint32_t root_num) const;
  void print_graph_csv(const plansys2::bt_builder::ActionGraph::Ptr & graph) const;

  void get_node_tabular(
    const plansys2::bt_builder::ActionNode::Ptr & node,
    uint32_t root_num,
    std::vector<std::tuple<uint32_t, uint32_t, uint32_t, std::string>> & graph) const;
  std::vector<std::tuple<uint32_t, uint32_t, uint32_t, std::string>> get_graph_tabular(
    const plansys2::bt_builder::ActionGraph::Ptr & graph) const;
};

}  // namespace bt_builder
}  // namespace plansys2

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(plansys2::bt_builder::SimpleBTBuilder, plansys2::bt_builder::BTBuilder)

#endif  // PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__SIMPLE_BT_BUILDER_HPP_
