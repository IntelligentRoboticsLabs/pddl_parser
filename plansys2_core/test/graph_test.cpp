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

  auto nodes_root_predAB = graph.getDerivedPredicatesDepthFirst({predA, predB});
  ASSERT_EQ(nodes_root_predAB.size(), 6);
  ASSERT_EQ(nodes_root_predAB[0].predicate.name, "inferredB");
  ASSERT_EQ(nodes_root_predAB[1].predicate.name, "inferredBB");
  ASSERT_EQ(nodes_root_predAB[2].predicate.name, "inferredAA");
  ASSERT_EQ(nodes_root_predAB[3].predicate.name, "inferredA");
  ASSERT_EQ(nodes_root_predAB[4].predicate.name, "inferredAB");
  ASSERT_EQ(nodes_root_predAB[5].predicate.name, "inferredAA");

  auto nodes_root_predA = graph.getDerivedPredicatesDepthFirst({predA});
  ASSERT_EQ(nodes_root_predA.size(), 3);
  ASSERT_EQ(nodes_root_predA[0].predicate.name, "inferredAA");
  ASSERT_EQ(nodes_root_predA[1].predicate.name, "inferredA");
  ASSERT_EQ(nodes_root_predA[2].predicate.name, "inferredAA");

  auto nodes_root_predB = graph.getDerivedPredicatesDepthFirst({predB});
  ASSERT_EQ(nodes_root_predB.size(), 2);
  ASSERT_EQ(nodes_root_predB[0].predicate.name, "inferredB");
  ASSERT_EQ(nodes_root_predB[1].predicate.name, "inferredBB");

  plansys2::Predicate predA_unified = parser::pddl::fromStringPredicate("(predicateA instance)");
  auto nodes_root_predA_unified = graph.getDerivedPredicatesDepthFirst({predA_unified});
  ASSERT_EQ(nodes_root_predA_unified.size(), 3);
  ASSERT_EQ(nodes_root_predA_unified[0].predicate.name, "inferredAA");
  ASSERT_EQ(nodes_root_predA_unified[1].predicate.name, "inferredA");
  ASSERT_EQ(nodes_root_predA_unified[2].predicate.name, "inferredAA");
}

TEST(graph_test, graph_derived_action)
{
  plansys2::Predicate predA = parser::pddl::fromStringPredicate("(predicateA ?a)");
  plansys2::Predicate predB = parser::pddl::fromStringPredicate("(predicateB ?b)");

  std::vector<plansys2::Derived> derived_predicates;
  plansys2::Derived inferredA;
  inferredA.predicate = parser::pddl::fromStringPredicate("(inferredA ?a)");
  inferredA.predicate.parameters[0].type = "typea";
  inferredA.preconditions = parser::pddl::fromString("(and (predicateA ?a))");
  derived_predicates.push_back(inferredA);

  plansys2::Derived inferredB;
  inferredB.predicate = parser::pddl::fromStringPredicate("(inferredB ?b)");
  inferredB.preconditions = parser::pddl::fromString("(and (predicateB ?b))");
  derived_predicates.push_back(inferredB);

  plansys2::Derived inferredAA;
  inferredAA.predicate = parser::pddl::fromStringPredicate("(inferredAA ?aa)");
  inferredAA.preconditions = parser::pddl::fromString("(and (inferredA ?aa))");
  derived_predicates.push_back(inferredAA);

  plansys2::Derived inferredAA2;
  inferredAA2.predicate = parser::pddl::fromStringPredicate("(inferredAA ?a)");
  inferredAA2.preconditions = parser::pddl::fromString("(and (predicateA ?a))");
  derived_predicates.push_back(inferredAA2);

  plansys2::Derived inferredBB;
  inferredBB.predicate = parser::pddl::fromStringPredicate("(inferredBB ?b)");
  inferredBB.preconditions = parser::pddl::fromString("(and (inferredB ?b))");
  derived_predicates.push_back(inferredBB);

  plansys2::Derived inferredAB;
  inferredAB.predicate = parser::pddl::fromStringPredicate("(inferredAB ?a ?b)");
  inferredAB.preconditions = parser::pddl::fromString("(and (inferredA ?a)(inferredB ?b))");
  derived_predicates.push_back(inferredAB);

  plansys2::DerivedGraph graph(derived_predicates);

  plansys2::Action actionA;
  actionA.name = "actionA";
  actionA.parameters.push_back(parser::pddl::fromStringParam("a"));

  plansys2_msgs::msg::Tree actionA_preconditions;
  parser::pddl::fromString(
    actionA_preconditions,
    "(and (inferredAA a))");
  actionA.preconditions = actionA_preconditions;

  plansys2::Action actionB;
  actionB.name = "actionB";
  actionB.parameters.push_back(parser::pddl::fromStringParam("b"));

  plansys2_msgs::msg::Tree actionB_preconditions;
  parser::pddl::fromString(
    actionB_preconditions,
    "(and (inferredBB b))");
  actionB.preconditions = actionB_preconditions;

  plansys2::Action actionAB;
  actionAB.name = "actionAB";
  actionAB.parameters.push_back(parser::pddl::fromStringParam("a"));
  actionAB.parameters.push_back(parser::pddl::fromStringParam("b"));

  plansys2_msgs::msg::Tree actionAB_preconditions;
  parser::pddl::fromString(
    actionAB_preconditions,
    "(and (inferredAB a b))");
  actionAB.preconditions = actionAB_preconditions;

  graph.appendAction(actionA);
  graph.appendAction(actionB);
  graph.appendAction(actionAB);

  std::vector<std::string> all_nodes;
  auto func_all = [&all_nodes](const plansys2::NodeVariant& node) {
    all_nodes.push_back(node.getNodeName());
  };
  graph.depthFirstTraverseAll(
    func_all,
    true
  );

  ASSERT_EQ(all_nodes.size(), 11);
  ASSERT_EQ(all_nodes[0], "predicateB");
  ASSERT_EQ(all_nodes[1], "inferredB");
  ASSERT_EQ(all_nodes[2], "inferredBB");
  ASSERT_EQ(all_nodes[3], "actionB");
  ASSERT_EQ(all_nodes[4], "predicateA");
  ASSERT_EQ(all_nodes[5], "inferredAA");
  ASSERT_EQ(all_nodes[6], "inferredA");
  ASSERT_EQ(all_nodes[7], "inferredAB");
  ASSERT_EQ(all_nodes[8], "actionAB");
  ASSERT_EQ(all_nodes[9], "inferredAA");
  ASSERT_EQ(all_nodes[10], "actionA");

  std::vector<std::string> actionAB_parent_nodes;
  auto func_actionAB_parents = [&actionAB_parent_nodes](const plansys2::NodeVariant& node) {
    actionAB_parent_nodes.push_back(node.getNodeName());
  };
  graph.backtrackTraverse(
    actionAB,
    func_actionAB_parents
  );

  ASSERT_EQ(actionAB_parent_nodes.size(), 6);
  ASSERT_EQ(actionAB_parent_nodes[0], "actionAB");
  ASSERT_EQ(actionAB_parent_nodes[1], "inferredAB");
  ASSERT_EQ(actionAB_parent_nodes[2], "inferredB");
  ASSERT_EQ(actionAB_parent_nodes[3], "predicateB");
  ASSERT_EQ(actionAB_parent_nodes[4], "inferredA");
  ASSERT_EQ(actionAB_parent_nodes[5], "predicateA");

  std::vector<std::string> actionA_parent_nodes;
  auto func_actionA_parents = [&actionA_parent_nodes](const plansys2::NodeVariant& node) {
    actionA_parent_nodes.push_back(node.getNodeName());
  };
  graph.backtrackTraverse(
    actionA,
    func_actionA_parents
  );

  ASSERT_EQ(actionA_parent_nodes.size(), 5);
  ASSERT_EQ(actionA_parent_nodes[0], "actionA");
  ASSERT_EQ(actionA_parent_nodes[1], "inferredAA");
  ASSERT_EQ(actionA_parent_nodes[2], "inferredA");
  ASSERT_EQ(actionA_parent_nodes[3], "predicateA");
  ASSERT_EQ(actionA_parent_nodes[4], "inferredAA");

  std::vector<std::string> actionB_parent_nodes;
  auto func_actionB_parents = [&actionB_parent_nodes](const plansys2::NodeVariant& node) {
    actionB_parent_nodes.push_back(node.getNodeName());
  };
  graph.backtrackTraverse(
    actionB,
    func_actionB_parents
  );

  ASSERT_EQ(actionB_parent_nodes.size(), 4);
  ASSERT_EQ(actionB_parent_nodes[0], "actionB");
  ASSERT_EQ(actionB_parent_nodes[1], "inferredBB");
  ASSERT_EQ(actionB_parent_nodes[2], "inferredB");
  ASSERT_EQ(actionB_parent_nodes[3], "predicateB");

  auto actionAB_parent_nodes_2 = graph.getDerivedPredicatesFromActions({actionAB});
  ASSERT_EQ(actionAB_parent_nodes_2.size(), 3);
  ASSERT_EQ(actionAB_parent_nodes_2[0].predicate.name, "inferredA");
  ASSERT_EQ(actionAB_parent_nodes_2[1].predicate.name, "inferredB");
  ASSERT_EQ(actionAB_parent_nodes_2[2].predicate.name, "inferredAB");

  auto graph_actionA = graph.pruneGraphToActions({actionA});

  all_nodes.clear();
  graph_actionA.depthFirstTraverseAll(
    func_all,
    true
  );

  ASSERT_EQ(all_nodes.size(), 5);
  ASSERT_EQ(all_nodes[0], "predicateA");
  ASSERT_EQ(all_nodes[1], "inferredAA");
  ASSERT_EQ(all_nodes[2], "inferredA");
  ASSERT_EQ(all_nodes[3], "inferredAA");
  ASSERT_EQ(all_nodes[4], "actionA");

  actionA_parent_nodes.clear();
  graph_actionA.backtrackTraverse(
    actionA,
    func_actionA_parents
  );

  ASSERT_EQ(actionA_parent_nodes.size(), 5);
  ASSERT_EQ(actionA_parent_nodes[0], "actionA");
  ASSERT_EQ(actionA_parent_nodes[1], "inferredAA");
  ASSERT_EQ(actionA_parent_nodes[2], "predicateA");
  ASSERT_EQ(actionA_parent_nodes[3], "inferredAA");
  ASSERT_EQ(actionA_parent_nodes[4], "inferredA");

  auto derived_predicates_A = graph_actionA.getDerivedPredicatesDepthFirst();
  ASSERT_EQ(derived_predicates_A.size(), 3);
  ASSERT_EQ(derived_predicates_A[0].predicate.name, "inferredAA");
  ASSERT_EQ(derived_predicates_A[1].predicate.name, "inferredA");
  ASSERT_EQ(derived_predicates_A[2].predicate.name, "inferredAA");

  auto graph_actionB = graph.pruneGraphToActions({actionB});

  all_nodes.clear();
  graph_actionB.depthFirstTraverseAll(
    func_all,
    true
  );

  ASSERT_EQ(all_nodes.size(), 4);
  ASSERT_EQ(all_nodes[0], "predicateB");
  ASSERT_EQ(all_nodes[1], "inferredB");
  ASSERT_EQ(all_nodes[2], "inferredBB");
  ASSERT_EQ(all_nodes[3], "actionB");

  actionB_parent_nodes.clear();
  graph_actionB.backtrackTraverse(
    actionB,
    func_actionB_parents
  );

  ASSERT_EQ(actionB_parent_nodes.size(), 4);
  ASSERT_EQ(actionB_parent_nodes[0], "actionB");
  ASSERT_EQ(actionB_parent_nodes[1], "inferredBB");
  ASSERT_EQ(actionB_parent_nodes[2], "inferredB");
  ASSERT_EQ(actionB_parent_nodes[3], "predicateB");

  auto derived_predicates_B = graph_actionB.getDerivedPredicatesDepthFirst();
  ASSERT_EQ(derived_predicates_B.size(), 2);
  ASSERT_EQ(derived_predicates_B[0].predicate.name, "inferredB");
  ASSERT_EQ(derived_predicates_B[1].predicate.name, "inferredBB");

  auto graph_actionAB = graph.pruneGraphToActions({actionAB});

  all_nodes.clear();
  graph_actionAB.depthFirstTraverseAll(
    func_all,
    true
  );

  ASSERT_EQ(all_nodes.size(), 6);
  ASSERT_EQ(all_nodes[0], "predicateA");
  ASSERT_EQ(all_nodes[1], "inferredA");
  ASSERT_EQ(all_nodes[2], "predicateB");
  ASSERT_EQ(all_nodes[3], "inferredB");
  ASSERT_EQ(all_nodes[4], "inferredAB");
  ASSERT_EQ(all_nodes[5], "actionAB");

  actionAB_parent_nodes.clear();
  graph_actionAB.backtrackTraverse(
    actionAB,
    func_actionAB_parents
  );

  ASSERT_EQ(actionAB_parent_nodes.size(), 6);
  ASSERT_EQ(actionAB_parent_nodes[0], "actionAB");
  ASSERT_EQ(actionAB_parent_nodes[1], "inferredAB");
  ASSERT_EQ(actionAB_parent_nodes[2], "inferredA");
  ASSERT_EQ(actionAB_parent_nodes[3], "predicateA");
  ASSERT_EQ(actionAB_parent_nodes[4], "inferredB");
  ASSERT_EQ(actionAB_parent_nodes[5], "predicateB");

  auto derived_predicatesAB = graph_actionAB.getDerivedPredicatesDepthFirst();
  ASSERT_EQ(derived_predicatesAB.size(), 3);
  ASSERT_EQ(derived_predicatesAB[0].predicate.name, "inferredA");
  ASSERT_EQ(derived_predicatesAB[1].predicate.name, "inferredB");
  ASSERT_EQ(derived_predicatesAB[2].predicate.name, "inferredAB");

  auto graph_actionA_AB = graph.pruneGraphToActions({actionA, actionAB});

  all_nodes.clear();
  graph_actionA_AB.depthFirstTraverseAll(
    func_all,
    true
  );

  ASSERT_EQ(all_nodes.size(), 9);
  ASSERT_EQ(all_nodes[0], "predicateB");
  ASSERT_EQ(all_nodes[1], "inferredB");
  ASSERT_EQ(all_nodes[2], "predicateA");
  ASSERT_EQ(all_nodes[3], "inferredAA");
  ASSERT_EQ(all_nodes[4], "inferredA");
  ASSERT_EQ(all_nodes[5], "inferredAB");
  ASSERT_EQ(all_nodes[6], "actionAB");
  ASSERT_EQ(all_nodes[7], "inferredAA");
  ASSERT_EQ(all_nodes[8], "actionA");

  actionA_parent_nodes.clear();
  graph_actionA_AB.backtrackTraverse(
    actionA,
    func_actionA_parents
  );

  ASSERT_EQ(actionA_parent_nodes.size(), 5);
  ASSERT_EQ(actionA_parent_nodes[0], "actionA");
  ASSERT_EQ(actionA_parent_nodes[1], "inferredAA");
  ASSERT_EQ(actionA_parent_nodes[2], "predicateA");
  ASSERT_EQ(actionA_parent_nodes[3], "inferredAA");
  ASSERT_EQ(actionA_parent_nodes[4], "inferredA");

  actionAB_parent_nodes.clear();
  graph_actionA_AB.backtrackTraverse(
    actionAB,
    func_actionAB_parents
  );

  ASSERT_EQ(actionAB_parent_nodes.size(), 6);
  ASSERT_EQ(actionAB_parent_nodes[0], "actionAB");
  ASSERT_EQ(actionAB_parent_nodes[1], "inferredAB");
  ASSERT_EQ(actionAB_parent_nodes[2], "inferredA");
  ASSERT_EQ(actionAB_parent_nodes[3], "predicateA");
  ASSERT_EQ(actionAB_parent_nodes[4], "inferredB");
  ASSERT_EQ(actionAB_parent_nodes[5], "predicateB");

  auto derived_predicates_A_AB = graph_actionA_AB.getDerivedPredicatesDepthFirst();
  ASSERT_EQ(derived_predicates_A_AB.size(), 5);
  ASSERT_EQ(derived_predicates_A_AB[0].predicate.name, "inferredB");
  ASSERT_EQ(derived_predicates_A_AB[1].predicate.name, "inferredAA");
  ASSERT_EQ(derived_predicates_A_AB[2].predicate.name, "inferredA");
  ASSERT_EQ(derived_predicates_A_AB[3].predicate.name, "inferredAB");
  ASSERT_EQ(derived_predicates_A_AB[4].predicate.name, "inferredAA");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
