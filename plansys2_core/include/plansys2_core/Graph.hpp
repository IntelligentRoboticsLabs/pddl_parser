// Copyright 2024 Intelligent Robotics Lab
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

#ifndef PLANSYS2_CORE__GRAPH_HPP_
#define PLANSYS2_CORE__GRAPH_HPP_

#include <iostream>
#include <deque>

#include "plansys2_core/Types.hpp"
#include "plansys2_core/Action.hpp"
#include "plansys2_msgs/msg/derived.hpp"
#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/tree.hpp"

namespace plansys2
{

class NodeVariant {
public:
  using NodeType = std::variant<plansys2::Predicate, plansys2::Function, plansys2::Derived, plansys2::ActionVariant>;

  template <typename NodeT>
  NodeVariant(NodeT node) : node_(std::make_shared<NodeType>(node)) {}

  size_t hash() const {
    return std::visit([](auto&& arg) { return std::hash<std::decay_t<decltype(arg)>>{}(arg); }, *node_);
  }

  bool operator==(const NodeVariant& other) const {
    return *node_ == *other.node_;
  }

  const NodeType& getNode() const { return *node_; }

  plansys2::Derived getDerivedNode() const {return std::get<plansys2::Derived>(*node_);}

  std::string getNodeName() const
  {
    std::string node_name;
    if (std::holds_alternative<plansys2::Predicate>(*node_)) {
      node_name = std::get<plansys2::Predicate>(*node_).name;
    } else if (std::holds_alternative<plansys2::Function>(*node_)) {
      node_name = std::get<plansys2::Function>(*node_).name;
    } else if (std::holds_alternative<plansys2::Derived>(*node_)) {
      node_name = std::get<plansys2::Derived>(*node_).predicate.name;
    } else if (std::holds_alternative<plansys2::ActionVariant>(*node_)) {
      node_name =  std::get<plansys2::ActionVariant>(*node_).get_action_name();
    }

    return node_name;
  }

  bool isPredicate() const
  {
    return std::holds_alternative<plansys2::Predicate>(*node_);
  }

  bool isFunction() const
  {
    return std::holds_alternative<plansys2::Function>(*node_);
  }

  bool isDerived() const
  {
    return std::holds_alternative<plansys2::Derived>(*node_);
  }

  plansys2::Predicate &getPredicate() const
  {
    return std::get<plansys2::Predicate>(*node_);
  }

  plansys2::Function &getFunction() const
  {
    return std::get<plansys2::Function>(*node_);
  }

  auto &getDerivedPreconditions() const
  {
    return std::get<plansys2::Derived>(*node_).preconditions;
  }

  auto &getDerivedPredicate() const
  {
    return std::get<plansys2::Derived>(*node_).predicate;
  }

  std::string getNodeType() const
  {
    if (std::holds_alternative<plansys2::Predicate>(*node_)) {
      return "predicate";
    } else if (std::holds_alternative<plansys2::Function>(*node_)) {
      return "function";
    } else if (std::holds_alternative<plansys2::Derived>(*node_)) {
      return "derived";
    }
    return "";
  }

private:
    std::shared_ptr<NodeType> node_;
};

inline bool operator==(const NodeVariant& lhs, const plansys2_msgs::msg::Node& rhs) {
  if (lhs.isPredicate())
  {
    return lhs.getPredicate() == static_cast<plansys2::Predicate>(rhs);
  } else if (lhs.isFunction())
  {
    return lhs.getFunction() == static_cast<plansys2::Function>(rhs);
  }
  return false;
};

}  // namespace plansys2

namespace std {
template<>
struct hash<plansys2::NodeVariant> {
  std::size_t operator()(const plansys2::NodeVariant& nv) const noexcept {
    return nv.hash();
  }
};
}  // namespace std

namespace plansys2
{
class Graph {
public:
  Graph()
  : edge_count_(0) {}

  void addEdge(const NodeVariant& u, const NodeVariant& v) {
    adj_list_[u].insert(v);
    in_nodes_[v].insert(u);
    ++edge_count_;
  }

  const auto &getNodeOutEdges(const NodeVariant& node) {return adj_list_[node];}
  const auto &getNodeInEdges(const NodeVariant& node) {return in_nodes_[node];}

  template <typename Func>
  void depthFirstTraverse(
    const NodeVariant& node,
    std::unordered_set<NodeVariant>& visited,
    Func&& func,
    bool check_dependencies = false) const
  {
    if (visited.find(node) != visited.end()){
      return;
    }

    auto it_in_nodes = in_nodes_.find(node);
    if (check_dependencies && it_in_nodes != in_nodes_.end())
    {
      for (const auto& in_node : it_in_nodes->second)
      {
        if (visited.find(in_node) == visited.end()) {
          return;
        }
      }
    }
    std::forward<Func>(func)(node);
    visited.insert(node);

    auto it = adj_list_.find(node);
    if (it == adj_list_.end())
    {
      return;
    }

    for (const auto& neighbor : it->second) {
      if (visited.find(neighbor) == visited.end()) {
        depthFirstTraverse(neighbor, visited, std::forward<Func>(func), check_dependencies);
      }
    }
  }

  template <typename Func>
  void depthFirstTraverse(
    const NodeVariant& start,
    Func&& func,
    bool check_dependencies = false) const
  {
    std::unordered_set<NodeVariant> visited;
    depthFirstTraverse(start, visited, std::forward<Func>(func), check_dependencies);
  }

  template <typename Func>
  void depthFirstTraverseAll(
    Func&& func,
    bool check_dependencies = true,
    std::vector<plansys2_msgs::msg::Node> root_nodes_only = {}
  ) const
  {
    std::unordered_set<NodeVariant> visited;
    for (const auto& [key, _] : adj_list_)
    {
      if (root_nodes_only.empty() ||
        std::find_if(root_nodes_only.begin(), root_nodes_only.end(),
          [&key](auto& r){return key == r;}) != root_nodes_only.end())
      {
        depthFirstTraverse(key, visited, std::forward<Func>(func), check_dependencies);
      }
    }
  }

  template <typename Func>
  void backtrackTraverse(
    const NodeVariant& start,
    Func&& func) const
  {
    std::unordered_set<NodeVariant> visited;
    backtrackTraverse(start, visited, std::forward<Func>(func));
  }

  template <typename Func>
  void backtrackTraverse(
    const NodeVariant& node,
    std::unordered_set<NodeVariant>& visited,
    Func&& func) const
  {
    // If already visited, skip this node to avoid infinite loops
    if (visited.find(node) != visited.end()) {
      return;
    }

    // Apply the provided function to the current node
    std::forward<Func>(func)(node);
    visited.insert(node);

    // Find predecessors (in-nodes)
    auto it_in_nodes = in_nodes_.find(node);
    if (it_in_nodes == in_nodes_.end()) {
      return; // No predecessors to explore
    }

    // Recursively visit all predecessors
    for (const auto& predecessor : it_in_nodes->second) {
      if (visited.find(predecessor) == visited.end()) {
        backtrackTraverse(predecessor, visited, std::forward<Func>(func));
      }
    }
  }

  auto getEdgeNumber() const { return edge_count_; }

  void clear() {adj_list_.clear();}

  bool operator==(const Graph &graph) const
  {
    return this->adj_list_ == graph.adj_list_;
  }

  using NodeEdgesMap = std::unordered_map<NodeVariant, std::unordered_set<NodeVariant>>;

private:
  NodeEdgesMap adj_list_;
  NodeEdgesMap in_nodes_;

  size_t edge_count_;

  friend struct std::hash<Graph>;
};

class DerivedGraph: public plansys2::Graph {
public:
  DerivedGraph()
  : Graph() {}

  DerivedGraph(const std::vector<plansys2_msgs::msg::Derived> &derived_predicates)
  : Graph()
  {
    derived_predicates_= convertVectorToUnorderedSet<
      plansys2::Derived, plansys2_msgs::msg::Derived>(derived_predicates);
    for (const auto& derived : derived_predicates)
    {
      addEdgeFromPreconditions(static_cast<plansys2::Derived>(derived), derived.preconditions);
    }
  }

  DerivedGraph(const std::vector<plansys2::Derived> &derived_predicates)
  : Graph()
  {
    derived_predicates_= convertVectorToUnorderedSet<
      plansys2::Derived, plansys2::Derived>(derived_predicates);
    for (const auto& derived : derived_predicates)
    {
      addEdgeFromPreconditions(derived, derived.preconditions);
    }
  }

  DerivedGraph(const std::unordered_set<plansys2::Derived> &derived_predicates)
  : Graph(), derived_predicates_(derived_predicates)
  {
    for (const auto& derived : derived_predicates)
    {
      addEdgeFromPreconditions(derived, derived.preconditions);
    }
  }

  auto &getDerivedPredicates() const { return derived_predicates_; }

  std::vector<plansys2::Derived> getDerivedPredicatesDepthFirst(
    std::vector<plansys2_msgs::msg::Node> root_nodes_only = {}) const
  {
    std::vector<plansys2::Derived> all_nodes;
    depthFirstTraverseAll(
      [&all_nodes](const plansys2::NodeVariant& node) {
        if (node.isDerived()) {
          all_nodes.push_back(node.getDerivedNode());
        }
      },
      true,
      root_nodes_only
    );
    return all_nodes;
  }

  std::deque<plansys2::Derived> getDerivedPredicatesFromActions(
    const std::vector<plansys2::ActionVariant>& actions) const
  {
    std::deque<plansys2::Derived> derived_predicates;
    std::unordered_set<plansys2::NodeVariant> visited;
    auto func = [&derived_predicates](const plansys2::NodeVariant& node) {
      if (node.isDerived()) {
        derived_predicates.push_front(node.getDerivedNode());
      }
    };
    for (const auto& action: actions) {
      backtrackTraverse(action, visited, func);
    }
    return derived_predicates;
  }

  plansys2::DerivedGraph pruneGraphToActions(const std::vector<plansys2::ActionVariant>& actions)
  {
    auto new_graph = DerivedGraph();
    auto func_new_graph = [this, &new_graph](const plansys2::NodeVariant& node) {
      for (const auto& parent_node: this->getNodeInEdges(node)) {
        new_graph.addEdge(parent_node, node);
      }
    };

    std::unordered_set<NodeVariant> visited;
    for (const auto& action: actions)
    {
      backtrackTraverse(action, visited, func_new_graph);
    }
    return new_graph;
  }

  void addEdgeFromPreconditions(const NodeVariant& node, const plansys2_msgs::msg::Tree& tree)
  {
    for (const auto& tree_node : tree.nodes){
      switch (tree_node.node_type) {
        case plansys2_msgs::msg::Node::PREDICATE:
        {
          bool is_derived_predicate = false;
          for (const auto& d: derived_predicates_) {
            if (parser::pddl::checkNodeEquality(tree_node, d.predicate)){
              addEdge(static_cast<const plansys2::Derived&>(d), node);
              is_derived_predicate = true;
            }
          }
          if (!is_derived_predicate) {
            addEdge(static_cast<const plansys2::Predicate&>(tree_node), node);
          }
          break;
        }
        case plansys2_msgs::msg::Node::FUNCTION:
          addEdge(
            static_cast<const plansys2::Function&>(tree_node), node);
          break;
        default:
          break;
      }
    }
  }

  void appendAction(const plansys2::ActionVariant& action)
  {
    addEdgeFromPreconditions(action, action.get_overall_requirements());
    if (action.is_durative_action()){
      addEdgeFromPreconditions(action, action.get_at_start_requirements());
      addEdgeFromPreconditions(action, action.get_at_end_requirements());
    }
  }

  void addEdge(const NodeVariant& u, const NodeVariant& v){
    if(u.isDerived())
    {
      derived_predicates_.insert(u.getDerivedNode());
    }
    if (v.isDerived()) {
      derived_predicates_.insert(v.getDerivedNode());
    }
    Graph::addEdge(u, v);
  }

private:
  std::unordered_set<plansys2::Derived> derived_predicates_;

};

}  // namespace plansys2

namespace std {
template<>
struct hash<plansys2::Graph> {
  std::size_t operator()(const plansys2::Graph& graph) const noexcept {
    std::size_t seed = 0;
    for (const auto& [key, neighbors] : graph.adj_list_) {
      hash_combine(seed, key);
      for (const auto& neighbor : neighbors) {
        hash_combine(seed, neighbor);
      }
    }
    return seed;
  }
};

template<>
struct hash<plansys2::DerivedGraph> {
  std::size_t operator()(const plansys2::DerivedGraph& graph) const noexcept {
    return std::hash<plansys2::Graph>{}(graph);
  }
};
}  // namespace std

#endif  // PLANSYS2_CORE__GRAPH_HPP_
