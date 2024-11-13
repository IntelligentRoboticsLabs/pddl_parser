// Copyright 2019 Intelligent Robotics Lab
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
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "plansys2_domain_expert/DomainExpert.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_msgs/msg/knowledge.hpp"
#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_problem_expert/ProblemExpert.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/Utils.hpp"

#include "gtest/gtest.h"

TEST(utils, evaluate_and)
{
  plansys2::State state;

  std::string expression = "(and (patrolled wp1) (patrolled wp2))";
  plansys2_msgs::msg::Tree goal;
  parser::pddl::fromString(goal, expression);

  ASSERT_EQ(
    plansys2::evaluate(goal, state),
    std::make_tuple(true, false, 0, std::vector<std::map<std::string, std::string>>{}));

  state.addPredicate(parser::pddl::fromStringPredicate("(patrolled wp1)"));

  ASSERT_EQ(
    plansys2::evaluate(goal, state),
    std::make_tuple(true, false, 0, std::vector<std::map<std::string, std::string>>{}));

  state.clearPredicates();
  state.addPredicate(parser::pddl::fromStringPredicate("(patrolled wp2)"));

  ASSERT_EQ(
    plansys2::evaluate(goal, state),
    std::make_tuple(true, false, 0, std::vector<std::map<std::string, std::string>>{}));

  state.addPredicate(parser::pddl::fromStringPredicate("(patrolled wp1)"));

  ASSERT_EQ(
    plansys2::evaluate(goal, state),
    std::make_tuple(true, true, 0, std::vector<std::map<std::string, std::string>>{}));
}

TEST(utils, evaluate_or)
{
  plansys2::State state;

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(or (patrolled wp1) (patrolled wp2))", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, false, 0, std::vector<std::map<std::string, std::string>>{}));

  state.addPredicate(parser::pddl::fromStringPredicate("(patrolled wp1)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, true, 0, std::vector<std::map<std::string, std::string>>{}));

  state.clearPredicates();
  state.addPredicate(parser::pddl::fromStringPredicate("(patrolled wp2)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, true, 0, std::vector<std::map<std::string, std::string>>{}));

  state.addPredicate(parser::pddl::fromStringPredicate("(patrolled wp1)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, true, 0, std::vector<std::map<std::string, std::string>>{}));
}

TEST(utils, evaluate_not)
{
  plansys2::State state;

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(not (patrolled wp1))", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, true, 0, std::vector<std::map<std::string, std::string>>{}));

  state.addPredicate(parser::pddl::fromStringPredicate("(patrolled wp1)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, false, 0, std::vector<std::map<std::string, std::string>>{}));

  plansys2_msgs::msg::Tree test_tree2;
  parser::pddl::fromString(test_tree2, "(not (= wp1 wp2))");

  ASSERT_EQ(
    plansys2::evaluate(test_tree2, state),
    std::make_tuple(true, true, 0, std::vector<std::map<std::string, std::string>>{}));

  plansys2_msgs::msg::Tree test_tree3;
  parser::pddl::fromString(test_tree3, "(not (= wp1 wp1))");

  ASSERT_EQ(
    plansys2::evaluate(test_tree3, state),
    std::make_tuple(true, false, 0, std::vector<std::map<std::string, std::string>>{}));
}

TEST(utils, evaluate_predicate_use_state)
{
  plansys2::State state;

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "(patrolled wp1)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, false, 0, std::vector<std::map<std::string, std::string>>{}));

  ASSERT_EQ(
    plansys2::evaluate(
      test_tree, state, 0, true),
    std::make_tuple(true, true, 0, std::vector<std::map<std::string, std::string>>{}));

  ASSERT_TRUE(plansys2::apply(test_tree, state));
  ASSERT_EQ(state.getPredicatesSize(), 1);
  ASSERT_TRUE(state.hasPredicate(parser::pddl::fromStringPredicate("(patrolled wp1)")));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, true, 0, std::vector<std::map<std::string, std::string>>{}));

  ASSERT_EQ(
    plansys2::evaluate(
      test_tree, state, 0, true),
    std::make_tuple(true, false, 0, std::vector<std::map<std::string, std::string>>{}));

  ASSERT_TRUE(plansys2::apply(test_tree, state, 0, true));
  ASSERT_EQ(state.getPredicatesSize(), 0);
}

TEST(utils, evaluate_predicate_client)
{
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {
        exe.spin_some();
      }
    });

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("bedroom", "room")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("kitchen", "room")));

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(is_teleporter_destination bedroom)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_FALSE(plansys2::check(test_tree, problem_client));
  ASSERT_TRUE(plansys2::apply(test_tree, problem_client));
  ASSERT_TRUE(plansys2::check(test_tree, problem_client));

  ASSERT_TRUE(
    plansys2::apply(
      test_tree, problem_client, 0, true));
  ASSERT_FALSE(plansys2::check(test_tree, problem_client));

  finish = true;
  t.join();
}

TEST(utils, evaluate_function_use_state)
{
  plansys2::State state;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(distance wp1 wp2)", false, plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(false, false, 0.0, std::vector<std::map<std::string, std::string>>{}));

  state.addFunction(parser::pddl::fromStringFunction("(= (distance wp1 wp2) 1.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, false, 1.0, std::vector<std::map<std::string, std::string>>{}));
}

TEST(utils, evaluate_expression_ge)
{
  plansys2::State state;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "(>= (vx) 3.0)", false, plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(false, false, 0, std::vector<std::map<std::string, std::string>>{}));

  state.addFunction(parser::pddl::fromStringFunction("(= (vx) 2.9999)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, false, 0, std::vector<std::map<std::string, std::string>>{}));

  state.removeFunction(parser::pddl::fromStringFunction("(= (vx) 2.9999)"));
  state.addFunction(parser::pddl::fromStringFunction("(= (vx) 4.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, true, 0, std::vector<std::map<std::string, std::string>>{}));

  state.removeFunction(parser::pddl::fromStringFunction("(= (vx) 4.0)"));
  state.addFunction(parser::pddl::fromStringFunction("(= (vx) 3.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, true, 0, std::vector<std::map<std::string, std::string>>{}));
}

TEST(utils, evaluate_expression_gt)
{
  plansys2::State state;

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(> (distance wp1 wp2) 3.0)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(false, false, 0, std::vector<std::map<std::string, std::string>>{}));

  state.addFunction(parser::pddl::fromStringFunction("(= (distance wp1 wp2) 3.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, false, 0, std::vector<std::map<std::string, std::string>>{}));

  state.removeFunction(parser::pddl::fromStringFunction("(= (distance wp1 wp2) 3.0)"));
  state.addFunction(parser::pddl::fromStringFunction("(= (distance wp1 wp2) 3.00001)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, true, 0, std::vector<std::map<std::string, std::string>>{}));
}

TEST(utils, evaluate_expression_le)
{
  plansys2::State state;

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(<= (vx) -3.0)", false, plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(false, false, 0, std::vector<std::map<std::string, std::string>>{}));

  state.addFunction(parser::pddl::fromStringFunction("(= (vx) -2.9999)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, false, 0, std::vector<std::map<std::string, std::string>>{}));

  state.removeFunction(parser::pddl::fromStringFunction("(= (vx) -2.9999)"));
  state.addFunction(parser::pddl::fromStringFunction("(= (vx) -4.0)"));


  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, true, 0, std::vector<std::map<std::string, std::string>>{}));

  state.removeFunction(parser::pddl::fromStringFunction("(= (vx) -4.0)"));
  state.addFunction(parser::pddl::fromStringFunction("(= (vx) -3.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, true, 0, std::vector<std::map<std::string, std::string>>{}));
}

TEST(utils, evaluate_expression_lt)
{
  plansys2::State state;

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(< (distance wp1 wp2) -3.0)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(false, false, 0, std::vector<std::map<std::string, std::string>>{}));

  state.addFunction(parser::pddl::fromStringFunction("(= (distance wp1 wp2) -3.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, false, 0, std::vector<std::map<std::string, std::string>>{}));

  state.removeFunction(parser::pddl::fromStringFunction("(= (distance wp1 wp2) -3.0)"));
  state.addFunction(parser::pddl::fromStringFunction("(= (distance wp1 wp2) -3.00001)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, true, 0, std::vector<std::map<std::string, std::string>>{}));
}

TEST(utils, evaluate_expression_multiply)
{
  plansys2::State state;

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "(* (vx) 3.0)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(false, false, 0, std::vector<std::map<std::string, std::string>>{}));

  state.addFunction(parser::pddl::fromStringFunction("(= (vx) 3.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, false, 9.0, std::vector<std::map<std::string, std::string>>{}));

  state.removeFunction(parser::pddl::fromStringFunction("(= (vx) 3.0)"));
  state.addFunction(parser::pddl::fromStringFunction("(= (vx) -0.001)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, false, -0.003, std::vector<std::map<std::string, std::string>>{}));
}

TEST(utils, evaluate_expression_divide)
{
  plansys2::State state;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "(/ (vx) 3.0)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(false, false, 0, std::vector<std::map<std::string, std::string>>{}));

  state.addFunction(parser::pddl::fromStringFunction("(= (vx) 3.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, false, 1.0, std::vector<std::map<std::string, std::string>>{}));

  state.removeFunction(parser::pddl::fromStringFunction("(= (vx) 3.0)"));
  state.addFunction(parser::pddl::fromStringFunction("(= (vx) -9.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, false, -3.0, std::vector<std::map<std::string, std::string>>{}));

  test_tree.nodes.clear();
  parser::pddl::fromString(test_tree, "(/ (vx) 0)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(false, false, 0, std::vector<std::map<std::string, std::string>>{}));
}

TEST(utils, evaluate_expression_add)
{
  plansys2::State state;

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "(+ (vx) 3.0)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(false, false, 0, std::vector<std::map<std::string, std::string>>{}));

  state.addFunction(parser::pddl::fromStringFunction("(= (vx) 3.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, false, 6.0, std::vector<std::map<std::string, std::string>>{}));

  state.removeFunction(parser::pddl::fromStringFunction("(= (vx) 3.0)"));
  state.addFunction(parser::pddl::fromStringFunction("(= (vx) -0.001)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, false, 2.999, std::vector<std::map<std::string, std::string>>{}));
}

TEST(utils, evaluate_expression_subtract)
{
  plansys2::State state;

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "(- (vx) 3.0)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(false, false, 0, std::vector<std::map<std::string, std::string>>{}));

  state.addFunction(parser::pddl::fromStringFunction("(= (vx) 2.5)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, false, -0.5, std::vector<std::map<std::string, std::string>>{}));

  state.removeFunction(parser::pddl::fromStringFunction("(= (vx) 2.5)"));
  state.addFunction(parser::pddl::fromStringFunction("(= (vx) -0.001)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, false, -3.001, std::vector<std::map<std::string, std::string>>{}));
}

TEST(utils, evaluate_expression_invalid)
{
  plansys2::State state;

  // Unknown expression type
  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "(> (vx) 0)", false, plansys2_msgs::msg::Node::AND);
  test_tree.nodes[0].expression_type = plansys2_msgs::msg::Node::UNKNOWN;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(false, false, 0, std::vector<std::map<std::string, std::string>>{}));
}

TEST(utils, evaluate_expression_invalid_client)
{
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {
        exe.spin_some();
      }
    });

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("leia", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("Jack", "person")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("bedroom", "room")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("kitchen", "room")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("m1", "message")));

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(> (room_distance bedroom kitchen) 0)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client),
    std::make_tuple(false, false, 0, std::vector<std::map<std::string, std::string>>{}));

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  test_tree.nodes.clear();
  parser::pddl::fromString(
    test_tree, "(> 0 (room_distance bedroom kitchen))", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client),
    std::make_tuple(false, false, 0, std::vector<std::map<std::string, std::string>>{}));

  finish = true;
  t.join();
}

// TEST(utils, evaluate_function_mod)
// {
//   std::vector<plansys2_msgs::msg::Node> predicates_msg;
//   std::vector<plansys2_msgs::msg::Node> functions_msg;
//
//   auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
//   auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
//
//   plansys2_msgs::msg::Tree test_tree;
//   parser::pddl::fromString(
//     test_tree, "(assign (vx) 3.0)", false, plansys2_msgs::msg::Node::EXPRESSION);
//
//   parser::pddl::getPredicates(predicates_msg, test_tree);
//   parser::pddl::getFunctions(functions_msg, test_tree);
//
//   // std::unordered_set<plansys2::Instance> instances;
//   auto predicates =
//     plansys2::convertVectorToUnorderedSet<plansys2::Predicate, plansys2_msgs::msg::Node>(
//     predicates_msg);
//   auto functions =
//     plansys2::convertVectorToUnorderedSet<plansys2::Function, plansys2_msgs::msg::Node>(functions_msg);
//
//   std::unordered_set<plansys2::Instance> instances;
//   plansys2::State state(instances, functions, predicates);
//
//   ASSERT_EQ(
//     plansys2::evaluate(test_tree, state),
//     std::make_tuple(true, false, 3.0, std::vector<std::map<std::string, std::string>>{}));
//   ASSERT_EQ(functions.find(parser::pddl::fromStringFunction("(vx)"))->value, 0.0);
//
//   ASSERT_TRUE(plansys2::apply(test_tree, state));
//   // ASSERT_EQ(
//   //   plansys2::evaluate(test_tree, state),
//   //   std::make_tuple(true, false, 3.0, std::vector<std::map<std::string, std::string>>{}));
//   // ASSERT_EQ(functions[0].value, 3.0);
//   ASSERT_EQ(state.getFunction(parser::pddl::fromStringFunction("(vx)"))->value, 3.0);
//
//   test_tree.nodes.clear();
//   parser::pddl::fromString(
//     test_tree, "(increase (vx) 3.0)", false, plansys2_msgs::msg::Node::EXPRESSION);
//
//   ASSERT_TRUE(plansys2::apply(test_tree, state));
//   ASSERT_EQ(state.getFunction(parser::pddl::fromStringFunction("(vx)"))->value, 6.0);
//
//   test_tree.nodes.clear();
//   parser::pddl::fromString(
//     test_tree, "(decrease (vx) 3.0)", false, plansys2_msgs::msg::Node::EXPRESSION);
//
//   ASSERT_TRUE(plansys2::apply(test_tree, state));
//   // ASSERT_EQ(functions[0].value, 3.0);
//   ASSERT_EQ(state.getFunction(parser::pddl::fromStringFunction("(vx)"))->value, 3.0);
//
//   test_tree.nodes.clear();
//   parser::pddl::fromString(
//     test_tree, "(scale-up (vx) 3.0)", false, plansys2_msgs::msg::Node::EXPRESSION);
//
//   ASSERT_TRUE(plansys2::apply(test_tree, state));
//   // ASSERT_EQ(functions[0].value, 9.0);
//   ASSERT_EQ(state.getFunction(parser::pddl::fromStringFunction("(vx)"))->value, 9.0);
//
//   test_tree.nodes.clear();
//   parser::pddl::fromString(
//     test_tree, "(scale-down (vx) 3.0)", false, plansys2_msgs::msg::Node::EXPRESSION);
//
//   ASSERT_TRUE(plansys2::apply(test_tree, state));
//   ASSERT_EQ(state.getFunction(parser::pddl::fromStringFunction("(vx)"))->value, 3.0);
//
//   // divide by zero
//   test_tree.nodes.clear();
//   parser::pddl::fromString(
//     test_tree, "(scale-down (vx) 0.0)", false, plansys2_msgs::msg::Node::EXPRESSION);
//
//   ASSERT_TRUE(plansys2::apply(test_tree, state));
//   ASSERT_EQ(state.getFunction(parser::pddl::fromStringFunction("(vx)"))->value, 3.0);
// }
//
// TEST(utils, evaluate_function_mod_client)
// {
//   auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
//   auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
//   auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
//   auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
//
//   std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
//
//   domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
//   problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
//
//   domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
//   problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
//
//   domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
//   problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
//
//   rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);
//
//   exe.add_node(domain_node->get_node_base_interface());
//   exe.add_node(problem_node->get_node_base_interface());
//
//   bool finish = false;
//   std::thread t([&]() {
//       while (!finish) {
//         exe.spin_some();
//       }
//     });
//
//   ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("bedroom", "room")));
//   ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("kitchen", "room")));
//   ASSERT_TRUE(
//     problem_client->addFunction(plansys2::Function("(= (room_distance bedroom kitchen) 1.0)")));
//
//   {
//     rclcpp::Rate rate(10);
//     auto start = test_node->now();
//     while ((test_node->now() - start).seconds() < 0.5) {
//       rate.sleep();
//     }
//   }
//
//   plansys2_msgs::msg::Tree test_tree;
//   parser::pddl::fromString(
//     test_tree, "(assign (room_distance bedroom kitchen) 0)", false,
//     plansys2_msgs::msg::Node::EXPRESSION);
//
//   ASSERT_TRUE(plansys2::apply(test_tree, problem_client));
//   std::optional<plansys2_msgs::msg::Node> func =
//     problem_client->getFunction("(room_distance bedroom kitchen)");
//   ASSERT_TRUE(func.has_value());
//   ASSERT_EQ(func.value().value, 0.0);
//   // ASSERT_EQ(func.find(parser::pddl::fromStringFunction("(room_distance bedroom kitchen)"))->value, 0.0);
//
//   test_tree.nodes.clear();
//   parser::pddl::fromString(
//     test_tree, "(increase (room_distance bedroom kitchen) 10.0)", false,
//     plansys2_msgs::msg::Node::EXPRESSION);
//
//   ASSERT_TRUE(plansys2::apply(test_tree, problem_client));
//   func = problem_client->getFunction("(room_distance bedroom kitchen)");
//   ASSERT_TRUE(func.has_value());
//   // ASSERT_EQ(func.value().value, 10.0);
//   // ASSERT_EQ(func.find(parser::pddl::fromStringFunction("(room_distance bedroom kitchen)"))->value, 10.0);
//
//   finish = true;
//   t.join();
// }
//
// TEST(utils, evaluate_function_mod_invalid)
// {
//   plansys2::State state;
//
//   // Unknown function modifier type
//   plansys2_msgs::msg::Tree test_tree;
//   parser::pddl::fromString(
//     test_tree, "(assign (vx) 3.0)", false, plansys2_msgs::msg::Node::EXPRESSION);
//   test_tree.nodes[0].node_type = plansys2_msgs::msg::Node::UNKNOWN;
//
//   ASSERT_EQ(
//     plansys2::evaluate(test_tree, state),
//     std::make_tuple(false, false, 0, std::vector<std::map<std::string, std::string>>{}));
// }
//
// TEST(utils, evaluate_function_mod_invalid_client)
// {
//   auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
//   auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
//   auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
//   auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
//
//   std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
//
//   domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
//   problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
//
//   domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
//   problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
//
//   domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
//   problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
//
//   rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);
//
//   exe.add_node(domain_node->get_node_base_interface());
//   exe.add_node(problem_node->get_node_base_interface());
//
//   bool finish = false;
//   std::thread t([&]() {
//       while (!finish) {
//         exe.spin_some();
//       }
//     });
//
//   plansys2_msgs::msg::Tree test_tree;
//   parser::pddl::fromString(
//     test_tree, "(assign (room_distance bedroom kitchen) 0)", false,
//     plansys2_msgs::msg::Node::EXPRESSION);
//
//   ASSERT_EQ(
//     plansys2::evaluate(test_tree, problem_client),
//     std::make_tuple(false, false, 0, std::vector<std::map<std::string, std::string>>{}));
//
//   {
//     rclcpp::Rate rate(10);
//     auto start = test_node->now();
//     while ((test_node->now() - start).seconds() < 0.5) {
//       rate.sleep();
//     }
//   }
//
//   test_tree.nodes.clear();
//   parser::pddl::fromString(
//     test_tree, "(assign 0 (room_distance bedroom kitchen))", false,
//     plansys2_msgs::msg::Node::EXPRESSION);
//
//   ASSERT_EQ(
//     plansys2::evaluate(test_tree, problem_client),
//     std::make_tuple(false, false, 0, std::vector<std::map<std::string, std::string>>{}));
//
//   finish = true;
//   t.join();
// }

TEST(utils, evaluate_number)
{
  plansys2::State state;

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "3.0", false, plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, true, 3.0, std::vector<std::map<std::string, std::string>>{}));
}

TEST(utils, evaluate_invalid)
{
  plansys2::State state;

  plansys2_msgs::msg::Tree test_tree;
  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(true, true, 0, std::vector<std::map<std::string, std::string>>{}));

  parser::pddl::fromString(test_tree, "(patrolled wp1)", false, plansys2_msgs::msg::Node::AND);
  test_tree.nodes.front().node_type = plansys2_msgs::msg::Node::UNKNOWN;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, state),
    std::make_tuple(false, false, 0, std::vector<std::map<std::string, std::string>>{}));
}

TEST(utils, evaluate_exists)
{
  plansys2::State state;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");

  std::string expression = "(exists (?1 ?2) (and (robot_at rob1 ?1)(connected ?1 ?2)))";
  plansys2_msgs::msg::Tree goal;
  parser::pddl::fromString(goal, expression);

  ASSERT_EQ(
    plansys2::evaluate(goal, state),
    std::make_tuple(true, false, 0, std::vector<std::map<std::string, std::string>>{}));

  state.addPredicate(parser::pddl::fromStringPredicate("(robot_at rob1 bedroom)"));

  ASSERT_EQ(
    plansys2::evaluate(goal, state),
    std::make_tuple(true, false, 0, std::vector<std::map<std::string, std::string>>{}));

  state.addPredicate(parser::pddl::fromStringPredicate("(connected bedroom kitchen)"));

  std::vector<std::map<std::string, std::string>> expected_result = {
    {{"?1", "bedroom"}, {"?2", "kitchen"}}};
  ASSERT_EQ(
    plansys2::evaluate(goal, state),
    std::make_tuple(true, true, 0, expected_result));
}

TEST(utils, evaluate_exists_client)
{
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");

  problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_exists.pddl"});
  problem_node->set_parameter({"problem_file", pkgpath + "/pddl/problem_simple_exists.pddl"});
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {
        exe.spin_some();
      }
    });

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  std::string expression = "(exists (?1 ?2) (and (robot_at rob1 ?1)(connected ?1 ?2)))";
  plansys2_msgs::msg::Tree goal;
  parser::pddl::fromString(goal, expression);

  ASSERT_EQ(
    plansys2::evaluate(goal, problem_client),
    std::make_tuple(true, false, 0, std::vector<std::map<std::string, std::string>>{}));

  ASSERT_TRUE(
    problem_client->addPredicate(parser::pddl::fromStringPredicate("(robot_at rob1 bedroom)")));

  ASSERT_EQ(
    plansys2::evaluate(goal, problem_client),
    std::make_tuple(true, false, 0, std::vector<std::map<std::string, std::string>>{}));

  ASSERT_TRUE(
    problem_client->addPredicate(parser::pddl::fromStringPredicate("(connected bedroom kitchen)")));

  std::vector<std::map<std::string, std::string>> expected_result = {
    {{"?1", "bedroom"}, {"?2", "kitchen"}}};
  ASSERT_EQ(
    plansys2::evaluate(goal, problem_client),
    std::make_tuple(true, true, 0, expected_result));

  finish = true;
  t.join();
}

TEST(utils, get_subtrees)
{
  std::vector<uint32_t> empty_expected;

  plansys2_msgs::msg::Tree invalid_goal;
  ASSERT_EQ(parser::pddl::getSubtreeIds(invalid_goal), empty_expected);

  parser::pddl::fromString(invalid_goal, "(or (patrolled wp1) (patrolled wp2))");
  ASSERT_EQ(parser::pddl::getSubtreeIds(invalid_goal), empty_expected);

  std::vector<uint32_t> expected;
  expected.push_back(1);
  expected.push_back(2);

  plansys2_msgs::msg::Tree goal;
  parser::pddl::fromString(goal, "(and (patrolled wp1) (patrolled wp2))");
  auto actual = parser::pddl::getSubtreeIds(goal);
  ASSERT_EQ(actual.size(), expected.size());
  for (size_t i = 0; i < expected.size(); i++) {
    ASSERT_EQ(actual[i], expected[i]);
  }
}

TEST(utils, get_action_from_string)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);

  exe.add_node(domain_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {
        exe.spin_some();
      }
    });

  std::string invalid_action_str = "(invalid r2d2 kitchen bedroom)";
  ASSERT_EQ(
    domain_client->getAction(
      plansys2::get_action_name(invalid_action_str),
      plansys2::get_action_params(invalid_action_str)),
    nullptr);

  std::string action_str = "(teleport r2d2 kitchen bedroom)";

  std::shared_ptr<plansys2_msgs::msg::Action> expected =
    std::make_shared<plansys2_msgs::msg::Action>();
  expected->name = "teleport";

  expected->parameters.push_back(parser::pddl::fromStringParam("r2d2"));
  expected->parameters.push_back(parser::pddl::fromStringParam("kitchen"));
  expected->parameters.push_back(parser::pddl::fromStringParam("bedroom"));

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree,
    "(and "
    "(robot_at r2d2 kitchen)(is_teleporter_enabled kitchen)(is_teleporter_destination bedroom))");
  expected->preconditions = test_tree;

  test_tree.nodes.clear();
  parser::pddl::fromString(test_tree, "(and (not(robot_at r2d2 kitchen))(robot_at r2d2 bedroom))");
  expected->effects = test_tree;
  std::shared_ptr<plansys2_msgs::msg::Action> actual = domain_client->getAction(
    plansys2::get_action_name(action_str), plansys2::get_action_params(action_str));

  ASSERT_EQ(parser::pddl::nameActionsToString(actual), parser::pddl::nameActionsToString(expected));
  ASSERT_EQ(
    parser::pddl::toString(actual->preconditions), parser::pddl::toString(expected->preconditions));
  ASSERT_EQ(parser::pddl::toString(actual->effects), parser::pddl::toString(expected->effects));

  std::string durative_action_str = "(move r2d2 kitchen bedroom)";

  std::shared_ptr<plansys2_msgs::msg::DurativeAction> durative_expected =
    std::make_shared<plansys2_msgs::msg::DurativeAction>();

  durative_expected->name = "move";
  durative_expected->parameters.push_back(parser::pddl::fromStringParam("r2d2"));
  durative_expected->parameters.push_back(parser::pddl::fromStringParam("kitchen"));
  durative_expected->parameters.push_back(parser::pddl::fromStringParam("bedroom"));

  test_tree.nodes.clear();
  parser::pddl::fromString(test_tree, "(and (robot_at r2d2 kitchen))");
  durative_expected->at_start_requirements = test_tree;

  test_tree.nodes.clear();
  parser::pddl::fromString(test_tree, "(and (not(robot_at r2d2 kitchen)))");
  durative_expected->at_start_effects = test_tree;

  test_tree.nodes.clear();
  parser::pddl::fromString(test_tree, "(and (robot_at r2d2 bedroom))");
  durative_expected->at_end_effects = test_tree;

  std::shared_ptr<plansys2_msgs::msg::DurativeAction> durative_actual =
    domain_client->getDurativeAction(
    plansys2::get_action_name(durative_action_str),
    plansys2::get_action_params(durative_action_str));

  ASSERT_EQ(
    parser::pddl::nameActionsToString(durative_actual),
    parser::pddl::nameActionsToString(durative_expected));
  ASSERT_EQ(
    parser::pddl::toString(durative_actual->at_start_requirements),
    parser::pddl::toString(durative_expected->at_start_requirements));
  ASSERT_EQ(
    parser::pddl::toString(durative_actual->over_all_requirements),
    parser::pddl::toString(durative_expected->over_all_requirements));
  ASSERT_EQ(
    parser::pddl::toString(durative_actual->at_end_requirements),
    parser::pddl::toString(durative_expected->at_end_requirements));
  ASSERT_EQ(
    parser::pddl::toString(durative_actual->at_start_effects),
    parser::pddl::toString(durative_expected->at_start_effects));
  ASSERT_EQ(
    parser::pddl::toString(durative_actual->at_end_effects),
    parser::pddl::toString(durative_expected->at_end_effects));

  std::string overall_action_str = "(approach leia kitchen Jack)";

  std::shared_ptr<plansys2_msgs::msg::DurativeAction> overall_expected =
    std::make_shared<plansys2_msgs::msg::DurativeAction>();

  overall_expected->name = "approach";

  overall_expected->parameters.push_back(parser::pddl::fromStringParam("leia"));
  overall_expected->parameters.push_back(parser::pddl::fromStringParam("kitchen"));
  overall_expected->parameters.push_back(parser::pddl::fromStringParam("Jack"));

  parser::pddl::fromString(
    overall_expected->over_all_requirements,
    "(and (robot_at leia kitchen) (person_at Jack kitchen))");
  parser::pddl::fromString(overall_expected->at_end_requirements, "(and (person_at Jack kitchen))");
  parser::pddl::fromString(overall_expected->at_end_effects, "(and (robot_near_person leia Jack))");

  std::shared_ptr<plansys2_msgs::msg::DurativeAction> overall_actual =
    domain_client->getDurativeAction(
    plansys2::get_action_name(overall_action_str),
    plansys2::get_action_params(overall_action_str));

  ASSERT_EQ(
    parser::pddl::toString(overall_actual->at_start_requirements),
    parser::pddl::toString(overall_expected->at_start_requirements));
  ASSERT_EQ(
    parser::pddl::toString(overall_actual->over_all_requirements),
    parser::pddl::toString(overall_expected->over_all_requirements));
  ASSERT_EQ(
    parser::pddl::toString(overall_actual->at_end_requirements),
    parser::pddl::toString(overall_expected->at_end_requirements));
  ASSERT_EQ(
    parser::pddl::toString(overall_actual->at_start_effects),
    parser::pddl::toString(overall_expected->at_start_effects));
  ASSERT_EQ(
    parser::pddl::toString(overall_actual->at_end_effects),
    parser::pddl::toString(overall_expected->at_end_effects));

  finish = true;
  t.join();
}

TEST(utils, get_params)
{
  std::string action_str = "(move r2d2 bedroom)";

  std::vector<std::string> expected;
  expected.push_back("r2d2");
  expected.push_back("bedroom");

  ASSERT_EQ(plansys2::get_action_params(action_str), expected);
}

TEST(utils, get_name)
{
  std::string action_str = "(move r2d2 bedroom)";

  ASSERT_EQ(plansys2::get_action_name(action_str), "move");
}

TEST(utils, unify_predicate)
{
  std::unordered_set<plansys2::Predicate> predicates;
  auto predicate1 = parser::pddl::fromStringPredicate("(predicate a b)");
  auto predicate2 = parser::pddl::fromStringPredicate("(predicate a c)");
  auto predicate3 = parser::pddl::fromStringPredicate("(predicate b c)");
  auto predicate4 = parser::pddl::fromStringPredicate("(predicate d c)");
  auto predicate5 = parser::pddl::fromStringPredicate("(predicate e c)");
  predicates.insert(predicate1);
  predicates.insert(predicate2);
  predicates.insert(predicate3);
  predicates.insert(predicate4);
  predicates.insert(predicate5);

  auto res = plansys2::unifyPredicate(predicate1, predicates);
  ASSERT_EQ(std::get<0>(res), true);
  ASSERT_EQ(std::get<1>(res).size(), 0);

  res = plansys2::unifyPredicate(parser::pddl::fromStringPredicate("(predicate ?x c)"), predicates);
  std::vector<std::map<std::string, std::string>> expected_param_values = {
    {{"?x", "a"}}, {{"?x", "b"}}, {{"?x", "d"}}, {{"?x", "e"}}};
  std::sort(std::get<1>(res).begin(), std::get<1>(res).end());
  std::sort(expected_param_values.begin(), expected_param_values.end());
  ASSERT_EQ(std::get<0>(res), true);
  ASSERT_EQ(std::get<1>(res), expected_param_values);

  res =
    plansys2::unifyPredicate(parser::pddl::fromStringPredicate("(predicate ?x ?y)"), predicates);
  expected_param_values = {
    {{"?x", "a"}, {"?y", "b"}}, {{"?x", "a"}, {"?y", "c"}}, {{"?x", "b"}, {"?y", "c"}},
    {{"?x", "d"}, {"?y", "c"}}, {{"?x", "e"}, {"?y", "c"}},
  };
  std::sort(std::get<1>(res).begin(), std::get<1>(res).end());
  std::sort(expected_param_values.begin(), expected_param_values.end());
  ASSERT_EQ(std::get<0>(res), true);
  ASSERT_EQ(std::get<1>(res), expected_param_values);

  res = plansys2::unifyPredicate(parser::pddl::fromStringPredicate("(predicate ?x f)"), predicates);
  ASSERT_EQ(std::get<0>(res), false);
  ASSERT_EQ(std::get<1>(res).size(), 0);

  res = plansys2::unifyPredicate(parser::pddl::fromStringPredicate("(predicate z f)"), predicates);
  ASSERT_EQ(std::get<0>(res), false);
  ASSERT_EQ(std::get<1>(res).size(), 0);
}

TEST(utils, intersection_parameters)
{
  std::vector<std::map<std::string, std::string>> vector1 = {
    {{"?x", "a"}, {"?y", "a"}},
    {{"?x", "b"}, {"?y", "b"}},
    {{"?x", "c"}, {"?y", "c"}},
    {{"?x", "d"}, {"?y", "d"}},
  };
  std::vector<std::map<std::string, std::string>> vector2 = {
    {{"?x", "a"}},
    {{"?x", "b"}},
    {{"?x", "d"}},
  };
  std::vector<std::map<std::string, std::string>> vector3 = {
    {{"?x", "d"}, {"?y", "c"}},
    {{"?x", "d"}, {"?y", "d"}},
  };
  std::vector<std::map<std::string, std::string>> vector4;
  std::vector<std::map<std::string, std::string>> vector5 = {
    {{"?z", "a"}},
    {{"?z", "b"}},
    {{"?z", "d"}},
  };

  std::vector<std::map<std::string, std::string>> vector6 = {
    {{"?a", "a"}, {"?b", "a"}, {"?z", "a"}},
    {{"?b", "a"}, {"?x", "b"}, {"?y", "b"}, {"?z", "b"}},
    {{"?a", "a"}, {"?b", "a"}, {"?c", "a"}, {"?z", "d"}},
  };

  std::vector<std::map<std::string, std::string>> res =
    plansys2::mergeParamsValuesVector(vector1, vector2);
  std::sort(res.begin(), res.end());
  std::vector<std::map<std::string, std::string>> expected_intersection = {
    {{"?x", "a"}, {"?y", "a"}},
    {{"?x", "b"}, {"?y", "b"}},
    {{"?x", "d"}, {"?y", "d"}},
  };
  ASSERT_EQ(res, expected_intersection);

  res = plansys2::mergeParamsValuesVector(vector1, vector3);
  std::sort(res.begin(), res.end());
  expected_intersection = {
    {{"?x", "d"}, {"?y", "d"}},
  };
  ASSERT_EQ(res, expected_intersection);

  res = plansys2::mergeParamsValuesVector(vector1, vector4);
  ASSERT_EQ(res.size(), 0);

  res = plansys2::mergeParamsValuesVector(vector4, vector1);
  std::sort(res.begin(), res.end());
  std::sort(expected_intersection.begin(), expected_intersection.end());
  ASSERT_EQ(res.size(), 0);

  expected_intersection = {
    {{"?x", "a"}, {"?y", "a"}, {"?z", "a"}}, {{"?x", "a"}, {"?y", "a"}, {"?z", "b"}},
    {{"?x", "a"}, {"?y", "a"}, {"?z", "d"}}, {{"?x", "b"}, {"?y", "b"}, {"?z", "a"}},
    {{"?x", "b"}, {"?y", "b"}, {"?z", "b"}}, {{"?x", "b"}, {"?y", "b"}, {"?z", "d"}},
    {{"?x", "c"}, {"?y", "c"}, {"?z", "a"}}, {{"?x", "c"}, {"?y", "c"}, {"?z", "b"}},
    {{"?x", "c"}, {"?y", "c"}, {"?z", "d"}}, {{"?x", "d"}, {"?y", "d"}, {"?z", "a"}},
    {{"?x", "d"}, {"?y", "d"}, {"?z", "b"}}, {{"?x", "d"}, {"?y", "d"}, {"?z", "d"}},
  };
  res = plansys2::mergeParamsValuesVector(vector1, vector5);
  std::sort(res.begin(), res.end());
  std::sort(expected_intersection.begin(), expected_intersection.end());
  ASSERT_EQ(res, expected_intersection);

  expected_intersection = {
    {{"?x", "a"}, {"?y", "a"}, {"?z", "a"}}, {{"?x", "b"}, {"?y", "b"}, {"?z", "a"}},
    {{"?x", "c"}, {"?y", "c"}, {"?z", "a"}}, {{"?x", "d"}, {"?y", "d"}, {"?z", "a"}},
    {{"?x", "a"}, {"?y", "a"}, {"?z", "b"}}, {{"?x", "b"}, {"?y", "b"}, {"?z", "b"}},
    {{"?x", "c"}, {"?y", "c"}, {"?z", "b"}}, {{"?x", "d"}, {"?y", "d"}, {"?z", "b"}},
    {{"?x", "a"}, {"?y", "a"}, {"?z", "d"}}, {{"?x", "b"}, {"?y", "b"}, {"?z", "d"}},
    {{"?x", "c"}, {"?y", "c"}, {"?z", "d"}}, {{"?x", "d"}, {"?y", "d"}, {"?z", "d"}},
  };
  res = plansys2::mergeParamsValuesVector(vector5, vector1);
  std::sort(res.begin(), res.end());
  std::sort(expected_intersection.begin(), expected_intersection.end());
  ASSERT_EQ(res, expected_intersection);

  expected_intersection = {
    {{"?a", "a"}, {"?b", "a"}, {"?x", "a"}, {"?y", "a"}, {"?z", "a"}},
    {{"?a", "a"}, {"?b", "a"}, {"?c", "a"}, {"?x", "a"}, {"?y", "a"}, {"?z", "d"}},
    {{"?a", "a"}, {"?b", "a"}, {"?x", "b"}, {"?y", "b"}, {"?z", "a"}},
    {{"?b", "a"}, {"?x", "b"}, {"?y", "b"}, {"?z", "b"}},
    {{"?a", "a"}, {"?b", "a"}, {"?c", "a"}, {"?x", "b"}, {"?y", "b"}, {"?z", "d"}},
    {{"?a", "a"}, {"?b", "a"}, {"?x", "c"}, {"?y", "c"}, {"?z", "a"}},
    {{"?a", "a"}, {"?b", "a"}, {"?c", "a"}, {"?x", "c"}, {"?y", "c"}, {"?z", "d"}},
    {{"?a", "a"}, {"?b", "a"}, {"?x", "d"}, {"?y", "d"}, {"?z", "a"}},
    {{"?a", "a"}, {"?b", "a"}, {"?c", "a"}, {"?x", "d"}, {"?y", "d"}, {"?z", "d"}},
  };
  res = plansys2::mergeParamsValuesVector(vector1, vector6);
  std::sort(res.begin(), res.end());
  std::sort(expected_intersection.begin(), expected_intersection.end());
  ASSERT_EQ(res, expected_intersection);
}

TEST(utils, complement_parameters)
{
  auto predicate = parser::pddl::fromStringPredicate("(robot_at ?0 ?1)");
  predicate.parameters[0].type = "robot";
  predicate.parameters[1].type = "room";

  std::unordered_set<plansys2::Instance> instances;
  instances.insert(parser::pddl::fromStringParam("r2d2", "robot"));
  instances.insert(parser::pddl::fromStringParam("leia", "robot"));

  instances.insert(parser::pddl::fromStringParam("kitchen", "room"));
  instances.insert(parser::pddl::fromStringParam("bedroom", "room"));
  instances.insert(parser::pddl::fromStringParam("garage", "room"));

  std::vector<std::map<std::string, std::string>> vector = {
    {{"?0", "r2d2"}, {"?1", "kitchen"}},
    {{"?0", "leia"}, {"?1", "garage"}},
    {{"?0", "leia"}, {"?1", "kitchen"}},
  };

  auto result = complementParamsValuesVector(
    {predicate.parameters[0], predicate.parameters[1]}, vector, instances);

  ASSERT_EQ(result.size(), 3);
  std::vector<std::map<std::string, std::string>> expected = {
    {{"?0", "leia"}, {"?1", "bedroom"}},
    {{"?0", "r2d2"}, {"?1", "garage"}},
    {{"?0", "r2d2"}, {"?1", "bedroom"}},
  };
  ASSERT_EQ(result, expected);

  predicate.parameters[0].name = "?1";
  predicate.parameters[1].name = "?3";
  vector = {
    {{"?1", "r2d2"}, {"?3", "kitchen"}},
    {{"?1", "leia"}, {"?3", "garage"}},
    {{"?1", "leia"}, {"?3", "kitchen"}},
  };

  result = complementParamsValuesVector(
    {predicate.parameters[0], predicate.parameters[1]}, vector, instances);

  ASSERT_EQ(result.size(), 3);
  expected = {
    {{"?1", "leia"}, {"?3", "bedroom"}},
    {{"?1", "r2d2"}, {"?3", "garage"}},
    {{"?1", "r2d2"}, {"?3", "bedroom"}},
  };
  ASSERT_EQ(result, expected);

  vector = {
    {{"?1", "r2d2"}},
  };

  result = complementParamsValuesVector(
    {predicate.parameters[0]}, vector, instances);

  ASSERT_EQ(result.size(), 1);
  expected = {
    {{"?1", "leia"}},
  };
  ASSERT_EQ(result, expected);
}

TEST(utils, apply)
{
  plansys2::State state;

  plansys2_msgs::msg::Tree tree1;
  parser::pddl::fromString(tree1, "(and (patrolled wp1) (patrolled wp2))");

  ASSERT_TRUE(plansys2::apply(tree1, state));
  ASSERT_EQ(state.getPredicatesSize(), 2);
  ASSERT_TRUE(state.hasPredicate(parser::pddl::fromStringPredicate("(patrolled wp1)")));
  ASSERT_TRUE(state.hasPredicate(parser::pddl::fromStringPredicate("(patrolled wp2)")));

  plansys2_msgs::msg::Tree tree2;
  parser::pddl::fromString(tree2, "(and (not (patrolled wp1)))");
  ASSERT_TRUE(plansys2::apply(tree2, state));
  ASSERT_EQ(state.getPredicatesSize(), 1);
  ASSERT_FALSE(state.hasPredicate(parser::pddl::fromStringPredicate("(patrolled wp1)")));
  ASSERT_TRUE(state.hasPredicate(parser::pddl::fromStringPredicate("(patrolled wp2)")));

  plansys2_msgs::msg::Tree tree3;
  parser::pddl::fromString(tree3, "(and (patrolled wp1) (not (patrolled wp2)) (patrolled wp3))");
  ASSERT_TRUE(plansys2::apply(tree3, state));
  ASSERT_EQ(state.getPredicatesSize(), 2);
  ASSERT_TRUE(state.hasPredicate(parser::pddl::fromStringPredicate("(patrolled wp1)")));
  ASSERT_FALSE(state.hasPredicate(parser::pddl::fromStringPredicate("(patrolled wp2)")));
  ASSERT_TRUE(state.hasPredicate(parser::pddl::fromStringPredicate("(patrolled wp3)")));
}

TEST(utils, apply_with_derived)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_derived.pddl");
  std::string domain_str(
    (std::istreambuf_iterator<char>(domain_ifs)), std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  std::ifstream problem_ifs(pkgpath + "/pddl/problem_derived.pddl");
  std::string problem_str(
    (std::istreambuf_iterator<char>(problem_ifs)), std::istreambuf_iterator<char>());
  ASSERT_TRUE(problem_expert.addProblem(problem_str));

  auto state = problem_expert.getState();
  ASSERT_TRUE(state.getInstances().size() > 0);
  ASSERT_TRUE(state.getFunctions().size() > 0);
  ASSERT_TRUE(state.getPredicates().size() > 0);
  ASSERT_TRUE(state.getInferredPredicates().size() > 0);
  ASSERT_TRUE(state.getDerivedPredicates().getEdgeNumber() > 0);

  ASSERT_TRUE(state.getInferredPredicates().find(parser::pddl::fromStringPredicate("(robot_at leia kitchen)")) != state.getInferredPredicates().end());
  ASSERT_TRUE(state.getInferredPredicates().find(parser::pddl::fromStringPredicate("(inferred-robot_at leia kitchen)")) != state.getInferredPredicates().end());
  ASSERT_FALSE(state.getInferredPredicates().find(parser::pddl::fromStringPredicate("(robot_at leia bedroom)")) != state.getInferredPredicates().end());
  ASSERT_FALSE(state.getInferredPredicates().find(parser::pddl::fromStringPredicate("(inferred-robot_at leia bedroom)")) != state.getInferredPredicates().end());

  plansys2_msgs::msg::Tree tree1;
  parser::pddl::fromString(tree1, "(and (not (robot_at leia kitchen)) (robot_at leia bedroom))");

  plansys2::apply(tree1, state);
  ASSERT_FALSE(state.getInferredPredicates().find(parser::pddl::fromStringPredicate("(robot_at leia kitchen)")) != state.getInferredPredicates().end());
  ASSERT_FALSE(state.getInferredPredicates().find(parser::pddl::fromStringPredicate("(inferred-robot_at leia kitchen)")) != state.getInferredPredicates().end());
  ASSERT_TRUE(state.getInferredPredicates().find(parser::pddl::fromStringPredicate("(robot_at leia bedroom)")) != state.getInferredPredicates().end());
  ASSERT_TRUE(state.getInferredPredicates().find(parser::pddl::fromStringPredicate("(inferred-robot_at leia bedroom)")) != state.getInferredPredicates().end());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
