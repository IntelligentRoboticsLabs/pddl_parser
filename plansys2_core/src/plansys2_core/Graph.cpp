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

#include "plansys2_core/Graph.hpp"

namespace plansys2
{

  DerivedGraph::DerivedGraph(const std::vector<plansys2_msgs::msg::Derived> &derived_predicates)
  : Graph()
  {
    derived_predicates_= convertVectorToUnorderedSet<
      plansys2::Derived, plansys2_msgs::msg::Derived>(derived_predicates);
    for (const auto& derived : derived_predicates)
    {
      addEdgeFromPreconditions(static_cast<plansys2::Derived>(derived), derived.preconditions);
    }
  }

  DerivedGraph::DerivedGraph(const std::vector<plansys2::Derived> &derived_predicates)
  : Graph()
  {
    derived_predicates_= convertVectorToUnorderedSet<
      plansys2::Derived, plansys2::Derived>(derived_predicates);
    for (const auto& derived : derived_predicates)
    {
      addEdgeFromPreconditions(derived, derived.preconditions);
    }
  }

  DerivedGraph::DerivedGraph(const std::unordered_set<plansys2::Derived> &derived_predicates)
  : Graph(), derived_predicates_(derived_predicates)
  {
    for (const auto& derived : derived_predicates)
    {
      addEdgeFromPreconditions(derived, derived.preconditions);
    }
  }

  std::vector<plansys2::Derived> DerivedGraph::getDerivedPredicatesDepthFirst(
    std::vector<plansys2_msgs::msg::Node> root_nodes_only) const
  {
    std::vector<plansys2::Derived> all_nodes;
    depthFirstTraverseAll(
      [&all_nodes](const plansys2::NodeVariant& node) {
        if (node.isDerived()) {
          all_nodes.push_back(node.getDerivedNode());
        }
      },
      root_nodes_only.empty(),
      root_nodes_only
    );
    return all_nodes;
  }

  std::deque<plansys2::Derived> DerivedGraph::getDerivedPredicatesFromActions(
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

  plansys2::DerivedGraph DerivedGraph::pruneGraphToActions(const std::vector<plansys2::ActionVariant>& actions)
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

  void DerivedGraph::addEdgeFromPreconditions(const NodeVariant& node, const plansys2_msgs::msg::Tree& tree)
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

  void DerivedGraph::appendActions(const std::vector<plansys2::ActionVariant>& actions)
  {
    for (const auto& action: actions)
    {
      appendAction(action);
    }
  }

  void DerivedGraph::appendAction(const plansys2::ActionVariant& action)
  {
    addEdgeFromPreconditions(action, action.get_overall_requirements());
    if (action.is_durative_action()){
      addEdgeFromPreconditions(action, action.get_at_start_requirements());
      addEdgeFromPreconditions(action, action.get_at_end_requirements());
    }
  }

  void DerivedGraph::addEdge(const NodeVariant& u, const NodeVariant& v){
    if(u.isDerived())
    {
      derived_predicates_.insert(u.getDerivedNode());
    }
    if (v.isDerived()) {
      derived_predicates_.insert(v.getDerivedNode());
    }
    Graph::addEdge(u, v);
  }
}  // namespace plansys2
