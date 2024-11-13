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

#include "plansys2_core/Types.hpp"
#include "plansys2_msgs/msg/derived.hpp"
#include "plansys2_msgs/msg/node.hpp"

namespace plansys2
{

class NodeVariant {
public:
  using NodeType = std::variant<plansys2::Predicate, plansys2::Function, plansys2::Derived>;

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
    bool check_dependencies = true) const
  {
    std::unordered_set<NodeVariant> visited;
    for (const auto& [key, _] : adj_list_)
    {
      depthFirstTraverse(key, visited, std::forward<Func>(func), check_dependencies);
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
  : Graph(), derived_predicates_(derived_predicates)
  {
    for (const auto& derived : derived_predicates)
    {
      for (const auto& node : derived.preconditions.nodes)
      {
        switch (node.node_type) {
          case plansys2_msgs::msg::Node::PREDICATE:
          {
            bool is_derived_predicate = false;
            for (const auto& d: derived_predicates) {
              if (parser::pddl::checkNodeEquality(node, d.predicate)){
                addEdge(d, derived);
                is_derived_predicate = true;
              }
            }
            if (!is_derived_predicate) {
              addEdge(static_cast<const plansys2::Predicate&>(node), derived);
            }
            break;
          }
          case plansys2_msgs::msg::Node::FUNCTION:
            addEdge(
              static_cast<const plansys2::Function&>(node), derived);
            break;
          default:
            break;
        }
      }
    }
  }

  auto &getDerivedPredicates() const { return derived_predicates_; }

  std::vector<plansys2::Derived> getDerivedPredicatesDepthFirst() const
  {
    std::vector<plansys2::Derived> all_nodes;
    depthFirstTraverseAll(
      [&all_nodes](const plansys2::NodeVariant& node) {
        if (node.isDerived()) {
          all_nodes.push_back(node.getDerivedNode());
        }
      },
      true
    );
    return all_nodes;
  }

private:
  std::vector<plansys2_msgs::msg::Derived> derived_predicates_;

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
