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

#include "plansys2_core/State.hpp"

#include "gtest/gtest.h"
#include "plansys2_core/Graph.hpp"

TEST(state_test, state)
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
  inferredAB.predicate = parser::pddl::fromStringPredicate("(inferredAB ?p)");
  inferredAB.preconditions = parser::pddl::fromString("(and (inferredA ?p)(inferredB ?p))");
  derived_predicates.push_back(inferredAB);

  plansys2::DerivedGraph graph(derived_predicates);

  plansys2::Instance instanceA = parser::pddl::fromStringParam("(instanceA)");
  plansys2::Instance instanceB = parser::pddl::fromStringParam("(instanceB)");
  plansys2::Instance instanceAB = parser::pddl::fromStringParam("(instanceAB)");
  std::unordered_set<plansys2::Instance> instances;
  instances.insert(instanceA);
  instances.insert(instanceB);
  instances.insert(instanceAB);

  plansys2::Predicate predA_instanceA = parser::pddl::fromStringPredicate("(predicateA instanceA)");
  plansys2::Predicate predA_instanceAB =
    parser::pddl::fromStringPredicate("(predicateA instanceAB)");
  plansys2::Predicate predB_instanceB = parser::pddl::fromStringPredicate("(predicateB instanceB)");
  plansys2::Predicate predB_instanceAB =
    parser::pddl::fromStringPredicate("(predicateB instanceAB)");
  std::unordered_set<plansys2::Predicate> predicates;
  predicates.insert(predA_instanceA);
  predicates.insert(predA_instanceAB);
  predicates.insert(predB_instanceB);
  predicates.insert(predB_instanceAB);

  plansys2::Predicate inferredA_instanceA =
    parser::pddl::fromStringPredicate("(inferredA instanceA)");
  plansys2::Predicate inferredAA_instanceA =
    parser::pddl::fromStringPredicate("(inferredAA instanceA)");
  plansys2::Predicate inferredAA2_instanceA =
    parser::pddl::fromStringPredicate("(inferredAA instanceA)");

  plansys2::Predicate inferredA_instanceAB =
    parser::pddl::fromStringPredicate("(inferredA instanceAB)");
  plansys2::Predicate inferredAA_instanceAB =
    parser::pddl::fromStringPredicate("(inferredAA instanceAB)");
  plansys2::Predicate inferredAA2_instanceAB =
    parser::pddl::fromStringPredicate("(inferredAA instanceAB)");

  plansys2::Predicate inferredB_instanceB =
    parser::pddl::fromStringPredicate("(inferredB instanceB)");
  plansys2::Predicate inferredBB_instanceB =
    parser::pddl::fromStringPredicate("(inferredBB instanceB)");

  plansys2::Predicate inferredB_instanceAB =
    parser::pddl::fromStringPredicate("(inferredB instanceAB)");
  plansys2::Predicate inferredBB_instanceAB =
    parser::pddl::fromStringPredicate("(inferredBB instanceAB)");

  plansys2::Predicate inferredAB_instanceAB =
    parser::pddl::fromStringPredicate("(inferredAB instanceAB)");

  std::unordered_set<plansys2::Function> functions;
  plansys2::State state(instances, functions, predicates);

  ASSERT_EQ(state.getInstancesSize(), 3);
  ASSERT_EQ(state.getPredicatesSize(), 4);
  ASSERT_EQ(state.getFunctionsSize(), 0);
  ASSERT_EQ(state.getInferredPredicatesSize(), 4);

  state.addInferredPredicate(inferredA_instanceA);
  state.addInferredPredicate(inferredAA_instanceA);
  state.addInferredPredicate(inferredAA2_instanceA);

  state.addInferredPredicate(inferredA_instanceAB);
  state.addInferredPredicate(inferredAA_instanceAB);
  state.addInferredPredicate(inferredAA2_instanceAB);

  state.addInferredPredicate(inferredB_instanceB);
  state.addInferredPredicate(inferredBB_instanceB);

  state.addInferredPredicate(inferredB_instanceAB);
  state.addInferredPredicate(inferredBB_instanceAB);

  state.addInferredPredicate(inferredAB_instanceAB);

  ASSERT_EQ(state.getInferredPredicatesSize(), 13);
  ASSERT_EQ(state.getNumberInferredFromDerived(inferredA), 2);
  ASSERT_EQ(state.getNumberInferredFromDerived(inferredB), 2);
  ASSERT_EQ(state.getNumberInferredFromDerived(inferredAA), 2);
  ASSERT_EQ(state.getNumberInferredFromDerived(inferredBB), 2);
  ASSERT_EQ(state.getNumberInferredFromDerived(inferredAB), 1);

  std::unordered_set<plansys2::Predicate> inferred_predicates;
  inferred_predicates.insert(inferredA_instanceA);
  inferred_predicates.insert(inferredAA_instanceA);
  inferred_predicates.insert(inferredAA2_instanceA);

  inferred_predicates.insert(inferredA_instanceAB);
  inferred_predicates.insert(inferredAA_instanceAB);
  inferred_predicates.insert(inferredAA2_instanceAB);

  inferred_predicates.insert(inferredB_instanceB);
  inferred_predicates.insert(inferredBB_instanceB);

  inferred_predicates.insert(inferredB_instanceAB);
  inferred_predicates.insert(inferredBB_instanceAB);

  inferred_predicates.insert(inferredAB_instanceAB);

  plansys2::State state_2(instances, functions, predicates, inferred_predicates, graph);

  ASSERT_EQ(state_2.getInferredPredicatesSize(), 13);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredA), 2);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredB), 2);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredAA), 2);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredBB), 2);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredAB), 1);

  state_2.ungroundDerivedPredicate(inferredA);
  ASSERT_EQ(state_2.getInferredPredicatesSize(), 8);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredA), 0);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredAA), 0);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredAB), 0);

  state_2.ungroundDerivedPredicate(inferredA);
  ASSERT_EQ(state_2.getInferredPredicatesSize(), 8);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredA), 0);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredAA), 0);
  ASSERT_EQ(state_2.getNumberInferredFromDerived(inferredAB), 0);

  plansys2::State state_3(instances, functions, predicates, inferred_predicates, graph);
  state_3.ungroundDerivedPredicate(inferredAB);
  ASSERT_EQ(state_3.getInferredPredicatesSize(), 12);
  ASSERT_EQ(state_3.getNumberInferredFromDerived(inferredAB), 0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
