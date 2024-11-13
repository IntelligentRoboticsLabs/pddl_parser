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

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "plansys2_core/Graph.hpp"
#include "plansys2_pddl_parser/Utils.hpp"


TEST(graph_test, graph)
{
  plansys2::Predicate predA = parser::pddl::fromStringPredicate("(predicateA ?a)");
  plansys2::Predicate predB = parser::pddl::fromStringPredicate("(predicateB ?b)");

  plansys2::Derived inferredA;
  inferredA.predicate = parser::pddl::fromStringPredicate("(inferredA ?a)");
  inferredA.preconditions = parser::pddl::fromString("(and (predicateA ?a))");

  plansys2::Derived inferredB;
  inferredB.predicate = parser::pddl::fromStringPredicate("(inferredB ?b)");
  inferredB.preconditions = parser::pddl::fromString("(and (predicateB ?b))");

  plansys2::Derived inferredAA;
  inferredAA.predicate = parser::pddl::fromStringPredicate("(inferredAA ?a)");
  inferredAA.preconditions = parser::pddl::fromString("(and (inferredA ?a))");

  plansys2::Derived inferredBB;
  inferredBB.predicate = parser::pddl::fromStringPredicate("(inferredBB ?b)");
  inferredBB.preconditions = parser::pddl::fromString("(and (inferredB ?b))");

  plansys2::Derived inferredAB;
  inferredAB.predicate = parser::pddl::fromStringPredicate("(inferredAB ?a ?b)");
  inferredAB.preconditions = parser::pddl::fromString("(and (inferredA ?a)(inferredB ?b))");

  plansys2::Graph graph;
  graph.addEdge(predA, inferredA);

  graph.addEdge(inferredA, inferredAA);
  graph.addEdge(inferredA, inferredAB);

  graph.addEdge(predB, inferredB);
  graph.addEdge(inferredB, inferredBB);
  graph.addEdge(inferredB, inferredAB);

  ASSERT_EQ(graph.getEdgeNumber(), 6);

  std::vector<std::string> predA_children;
  std::function<void(const plansys2::NodeVariant&)> func =
    [&predA_children](const plansys2::NodeVariant& node) {
      predA_children.push_back(node.getNodeName());
    };

  graph.depthFirstTraverse(predA, func);

  ASSERT_EQ(predA_children.size(), 4);
  ASSERT_EQ(predA_children[0], "predicateA");
  ASSERT_EQ(predA_children[1], "inferredA");
  ASSERT_EQ(predA_children[2], "inferredAB");
  ASSERT_EQ(predA_children[3], "inferredAA");

  std::vector<std::string> predB_children;
  std::function<void(const plansys2::NodeVariant&)> funcB =
    [&predB_children](const plansys2::NodeVariant& node) {
      predB_children.push_back(node.getNodeName());
    };
  graph.depthFirstTraverse(predB, funcB);

  ASSERT_EQ(predB_children.size(), 4);
  ASSERT_EQ(predB_children[0], "predicateB");
  ASSERT_EQ(predB_children[1], "inferredB");
  ASSERT_EQ(predB_children[2], "inferredAB");
  ASSERT_EQ(predB_children[3], "inferredBB");

  std::unordered_set<plansys2::NodeVariant> visited;
  std::vector<std::string> pred_children_deps;
  std::function<void(const plansys2::NodeVariant&)> func_deps =
    [&pred_children_deps](const plansys2::NodeVariant& node) {
      pred_children_deps.push_back(node.getNodeName());
    };

  graph.depthFirstTraverse(predA, visited, func_deps, true);
  ASSERT_EQ(pred_children_deps.size(), 3);
  ASSERT_EQ(pred_children_deps[0], "predicateA");
  ASSERT_EQ(pred_children_deps[1], "inferredA");
  ASSERT_EQ(pred_children_deps[2], "inferredAA");

  graph.depthFirstTraverse(predB, visited, func_deps, true);
  ASSERT_EQ(pred_children_deps.size(), 7);
  ASSERT_EQ(pred_children_deps[0], "predicateA");
  ASSERT_EQ(pred_children_deps[1], "inferredA");
  ASSERT_EQ(pred_children_deps[2], "inferredAA");
  ASSERT_EQ(pred_children_deps[3], "predicateB");
  ASSERT_EQ(pred_children_deps[4], "inferredB");
  ASSERT_EQ(pred_children_deps[5], "inferredAB");
  ASSERT_EQ(pred_children_deps[6], "inferredBB");

  std::vector<std::string> all_nodes;
  auto func_all = [&all_nodes](const plansys2::NodeVariant& node) {
    all_nodes.push_back(node.getNodeName());
  };
  graph.depthFirstTraverseAll(
    func_all,
    true
  );

  ASSERT_EQ(all_nodes.size(), 7);
  ASSERT_EQ(all_nodes[0], "predicateB");
  ASSERT_EQ(all_nodes[1], "inferredB");
  ASSERT_EQ(all_nodes[2], "inferredBB");
  ASSERT_EQ(all_nodes[3], "predicateA");
  ASSERT_EQ(all_nodes[4], "inferredA");
  ASSERT_EQ(all_nodes[5], "inferredAB");
  ASSERT_EQ(all_nodes[6], "inferredAA");
}

TEST(graph_test, graph_derived_constructor)
{
  plansys2::Predicate predA = parser::pddl::fromStringPredicate("(predicateA ?a)");
  plansys2::Predicate predB = parser::pddl::fromStringPredicate("(predicateB ?b)");

  std::vector<plansys2_msgs::msg::Derived> derived_predicates;
  plansys2_msgs::msg::Derived inferredA;
  inferredA.predicate = parser::pddl::fromStringPredicate("(inferredA ?a)");
  inferredA.predicate.parameters[0].type = "typea";
  inferredA.preconditions = parser::pddl::fromString("(and (predicateA ?a))");
  derived_predicates.push_back(inferredA);

  plansys2_msgs::msg::Derived inferredB;
  inferredB.predicate = parser::pddl::fromStringPredicate("(inferredB ?b)");
  inferredB.preconditions = parser::pddl::fromString("(and (predicateB ?b))");
  derived_predicates.push_back(inferredB);

  plansys2_msgs::msg::Derived inferredAA;
  inferredAA.predicate = parser::pddl::fromStringPredicate("(inferredAA ?aa)");
  inferredAA.preconditions = parser::pddl::fromString("(and (inferredA ?aa))");
  derived_predicates.push_back(inferredAA);

  plansys2_msgs::msg::Derived inferredAA2;
  inferredAA2.predicate = parser::pddl::fromStringPredicate("(inferredAA ?a)");
  inferredAA2.preconditions = parser::pddl::fromString("(and (predicateA ?a))");
  derived_predicates.push_back(inferredAA2);

  plansys2_msgs::msg::Derived inferredBB;
  inferredBB.predicate = parser::pddl::fromStringPredicate("(inferredBB ?b)");
  inferredBB.preconditions = parser::pddl::fromString("(and (inferredB ?b))");
  derived_predicates.push_back(inferredBB);

  plansys2_msgs::msg::Derived inferredAB;
  inferredAB.predicate = parser::pddl::fromStringPredicate("(inferredAB ?a ?b)");
  inferredAB.preconditions = parser::pddl::fromString("(and (inferredA ?a)(inferredB ?b))");
  derived_predicates.push_back(inferredAB);

  plansys2::DerivedGraph graph(derived_predicates);

  ASSERT_EQ(graph.getEdgeNumber(), 7);

  std::vector<std::pair<std::string,std::string>> predA_children;
  std::function<void(const plansys2::NodeVariant&)> func =
    [&predA_children](auto& node) {
      predA_children.push_back({node.getNodeName(), node.getNodeType()});
    };

  graph.depthFirstTraverse(predA, func);

  ASSERT_EQ(predA_children.size(), 5);
  ASSERT_EQ(predA_children[0].first, "predicateA");
  ASSERT_EQ(predA_children[0].second, "predicate");
  ASSERT_EQ(predA_children[1].first, "inferredAA");
  ASSERT_EQ(predA_children[1].second, "derived");
  ASSERT_EQ(predA_children[2].first, "inferredA");
  ASSERT_EQ(predA_children[2].second, "derived");
  ASSERT_EQ(predA_children[3].first, "inferredAB");
  ASSERT_EQ(predA_children[3].second, "derived");
  ASSERT_EQ(predA_children[4].first, "inferredAA");
  ASSERT_EQ(predA_children[4].second, "derived");

  std::vector<std::string> predB_children;
  std::function<void(const plansys2::NodeVariant&)> funcB =
    [&predB_children](const plansys2::NodeVariant& node) {
      predB_children.push_back(node.getNodeName());
    };
  graph.depthFirstTraverse(predB, funcB);

  ASSERT_EQ(predB_children.size(), 4);
  ASSERT_EQ(predB_children[0], "predicateB");
  ASSERT_EQ(predB_children[1], "inferredB");
  ASSERT_EQ(predB_children[2], "inferredAB");
  ASSERT_EQ(predB_children[3], "inferredBB");

  auto all_nodes = graph.getDerivedPredicatesDepthFirst();
  ASSERT_EQ(all_nodes.size(), 6);
  ASSERT_EQ(all_nodes[0].predicate.name, "inferredB");
  ASSERT_EQ(all_nodes[1].predicate.name, "inferredBB");
  ASSERT_EQ(all_nodes[2].predicate.name, "inferredAA");
  ASSERT_EQ(all_nodes[3].predicate.name, "inferredA");
  ASSERT_EQ(all_nodes[4].predicate.name, "inferredAB");
  ASSERT_EQ(all_nodes[5].predicate.name, "inferredAA");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
