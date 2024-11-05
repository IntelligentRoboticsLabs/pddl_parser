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

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <set>
#include <tuple>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "gtest/gtest.h"

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/tree.hpp"

#include "plansys2_problem_expert/ProblemExpert.hpp"
#include "plansys2_domain_expert/DomainExpert.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/Utils.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

#include "plansys2_msgs/msg/knowledge.hpp"

TEST(utils, evaluate_and)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  std::string expression = "(and (patrolled wp1) (patrolled wp2))";
  plansys2_msgs::msg::Tree goal;
  parser::pddl::fromString(goal, expression);

  ASSERT_EQ(
    plansys2::evaluate(goal, problem_client, predicates, functions, false, true),
    std::make_tuple(true, false, 0));

  predicates.push_back(parser::pddl::fromStringPredicate("(patrolled wp1)"));

  ASSERT_EQ(
    plansys2::evaluate(goal, problem_client, predicates, functions, false, true),
    std::make_tuple(true, false, 0));

  predicates.clear();
  predicates.push_back(parser::pddl::fromStringPredicate("(patrolled wp2)"));

  ASSERT_EQ(
    plansys2::evaluate(goal, problem_client, predicates, functions, false, true),
    std::make_tuple(true, false, 0));

  predicates.push_back(parser::pddl::fromStringPredicate("(patrolled wp1)"));

  ASSERT_EQ(
    plansys2::evaluate(goal, problem_client, predicates, functions, false, true),
    std::make_tuple(true, true, 0));
}

TEST(utils, evaluate_or)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(or (patrolled wp1) (patrolled wp2))", false,
    plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, false, true),
    std::make_tuple(true, false, 0));

  predicates.push_back(parser::pddl::fromStringPredicate("(patrolled wp1)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, false, true),
    std::make_tuple(true, true, 0));

  predicates.clear();
  predicates.push_back(parser::pddl::fromStringPredicate("(patrolled wp2)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, false, true),
    std::make_tuple(true, true, 0));

  predicates.push_back(parser::pddl::fromStringPredicate("(patrolled wp1)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, false, true),
    std::make_tuple(true, true, 0));
}

TEST(utils, evaluate_not)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(not (patrolled wp1))", false,
    plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, false, true),
    std::make_tuple(true, true, 0));

  predicates.push_back(parser::pddl::fromStringPredicate("(patrolled wp1)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, false, true),
    std::make_tuple(true, false, 0));

  plansys2_msgs::msg::Tree test_tree2;
  parser::pddl::fromString(
    test_tree2, "(not (= wp1 wp2))");

  ASSERT_EQ(
    plansys2::evaluate(test_tree2, problem_client, predicates, functions, false, true),
    std::make_tuple(true, true, 0));

  plansys2_msgs::msg::Tree test_tree3;
  parser::pddl::fromString(
    test_tree3, "(not (= wp1 wp1))");

  ASSERT_EQ(
    plansys2::evaluate(test_tree3, problem_client, predicates, functions, false, true),
    std::make_tuple(true, false, 0));
}

TEST(utils, evaluate_predicate_use_state)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "(patrolled wp1)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, false, true),
    std::make_tuple(true, false, 0));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, false, true, 0, true),
    std::make_tuple(true, true, 0));

  ASSERT_TRUE(plansys2::apply(test_tree, predicates, functions));
  ASSERT_EQ(predicates.size(), 1);
  ASSERT_EQ(parser::pddl::toString(*predicates.begin()), "(patrolled wp1)");

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, false, true),
    std::make_tuple(true, true, 0));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, false, true, 0, true),
    std::make_tuple(true, false, 0));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, true, true, 0, true),
    std::make_tuple(true, false, 0));
  ASSERT_TRUE(predicates.empty());
}

TEST(utils, evaluate_predicate_client)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
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

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
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
    test_tree, "(is_teleporter_destination bedroom)", false,
    plansys2_msgs::msg::Node::AND);

  ASSERT_FALSE(plansys2::check(test_tree, problem_client));
  ASSERT_TRUE(plansys2::apply(test_tree, problem_client));
  ASSERT_TRUE(plansys2::check(test_tree, problem_client));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions, true, false, 0, true),
    std::make_tuple(true, false, 0.0));
  ASSERT_FALSE(plansys2::check(test_tree, problem_client));

  finish = true;
  t.join();
}

TEST(utils, evaluate_function_use_state)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(distance wp1 wp2)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0.0));

  functions.push_back(parser::pddl::fromStringFunction("(= (distance wp1 wp2) 1.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, 1.0));
}

TEST(utils, evaluate_expression_ge)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "(>= (vx) 3.0)", false, plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));

  functions.push_back(parser::pddl::fromStringFunction("(= (vx) 2.9999)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, 0));

  functions[0].value = 4.0;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, true, 0));

  functions[0].value = 3.0;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, true, 0));
}

TEST(utils, evaluate_expression_gt)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(> (distance wp1 wp2) 3.0)", false,
    plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));

  functions.push_back(parser::pddl::fromStringFunction("(= (distance wp1 wp2) 3.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, 0));

  functions[0].value = 3.00001;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, true, 0));
}

TEST(utils, evaluate_expression_le)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(<= (vx) -3.0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));

  functions.push_back(parser::pddl::fromStringFunction("(= (vx) -2.9999)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, 0));

  functions[0].value = -4.0;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, true, 0));

  functions[0].value = -3.0;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, true, 0));
}

TEST(utils, evaluate_expression_lt)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(< (distance wp1 wp2) -3.0)", false,
    plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));

  functions.push_back(parser::pddl::fromStringFunction("(= (distance wp1 wp2) -3.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, 0));

  functions[0].value = -3.00001;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, true, 0));
}

TEST(utils, evaluate_expression_multiply)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "(* (vx) 3.0)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));

  functions.push_back(parser::pddl::fromStringFunction("(= (vx) 3.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, 9.0));

  functions[0].value = -0.001;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, -0.003));
}

TEST(utils, evaluate_expression_divide)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "(/ (vx) 3.0)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));

  functions.push_back(parser::pddl::fromStringFunction("(= (vx) 3.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, 1.0));

  functions[0].value = -9.0;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, -3.0));

  // Divide by zero
  test_tree.nodes.clear();
  parser::pddl::fromString(test_tree, "(/ (vx) 0)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));
}

TEST(utils, evaluate_expression_add)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "(+ (vx) 3.0)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));

  functions.push_back(parser::pddl::fromStringFunction("(= (vx) 3.0)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, 6.0));

  functions[0].value = -0.001;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, 2.999));
}

TEST(utils, evaluate_expression_subtract)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "(- (vx) 3.0)", false, plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));

  functions.push_back(parser::pddl::fromStringFunction("(= (vx) 2.5)"));

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, -0.5));

  functions[0].value = -0.001;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, -3.001));
}

TEST(utils, evaluate_expression_invalid)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  // Unknown expression type
  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "(> (vx) 0)", false, plansys2_msgs::msg::Node::AND);
  test_tree.nodes[0].expression_type = plansys2_msgs::msg::Node::UNKNOWN;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));
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

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
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
    test_tree, "(> (room_distance bedroom kitchen) 0)", false,
    plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client),
    std::make_tuple(false, false, 0));

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  test_tree.nodes.clear();
  parser::pddl::fromString(
    test_tree, "(> 0 (room_distance bedroom kitchen))", false,
    plansys2_msgs::msg::Node::AND);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client),
    std::make_tuple(false, false, 0));

  finish = true;
  t.join();
}

TEST(utils, evaluate_function_mod)
{
  std::vector<plansys2_msgs::msg::Node> predicates_msg;
  std::vector<plansys2_msgs::msg::Node> functions_msg;

  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(assign (vx) 3.0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

  parser::pddl::getPredicates(predicates_msg, test_tree);
  parser::pddl::getFunctions(functions_msg, test_tree);

  auto predicates = plansys2::convertVector<plansys2::Predicate, plansys2_msgs::msg::Node>(
    predicates_msg);
  auto functions = plansys2::convertVector<plansys2::Function, plansys2_msgs::msg::Node>(
    functions_msg);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(true, false, 3.0));
  ASSERT_EQ(functions[0].value, 0.0);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions, true),
    std::make_tuple(true, false, 3.0));
  ASSERT_EQ(functions[0].value, 3.0);

  test_tree.nodes.clear();
  parser::pddl::fromString(
    test_tree, "(increase (vx) 3.0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions, true),
    std::make_tuple(true, false, 6.0));
  ASSERT_EQ(functions[0].value, 6.0);

  test_tree.nodes.clear();
  parser::pddl::fromString(
    test_tree, "(decrease (vx) 3.0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions, true),
    std::make_tuple(true, false, 3.0));
  ASSERT_EQ(functions[0].value, 3.0);

  test_tree.nodes.clear();
  parser::pddl::fromString(
    test_tree, "(scale-up (vx) 3.0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions, true),
    std::make_tuple(true, false, 9.0));
  ASSERT_EQ(functions[0].value, 9.0);

  test_tree.nodes.clear();
  parser::pddl::fromString(
    test_tree, "(scale-down (vx) 3.0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions, true),
    std::make_tuple(true, false, 3.0));
  ASSERT_EQ(functions[0].value, 3.0);

  // divide by zero
  test_tree.nodes.clear();
  parser::pddl::fromString(
    test_tree, "(scale-down (vx) 0.0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions, true),
    std::make_tuple(false, false, 0.0));
  ASSERT_EQ(functions[0].value, 3.0);
}

TEST(utils, evaluate_function_mod_client)
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

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("bedroom", "room")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("kitchen", "room")));
  ASSERT_TRUE(
    problem_client->addFunction(
      plansys2::Function(
        "(= (room_distance bedroom kitchen) 1.0)")));

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(assign (room_distance bedroom kitchen) 0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, true),
    std::make_tuple(true, false, 0));
  std::optional<plansys2_msgs::msg::Node> func = problem_client->getFunction(
    "(room_distance bedroom kitchen)");
  ASSERT_TRUE(func.has_value());
  ASSERT_EQ(func.value().value, 0.0);

  test_tree.nodes.clear();
  parser::pddl::fromString(
    test_tree, "(increase (room_distance bedroom kitchen) 10.0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, true),
    std::make_tuple(true, false, 10.0));
  func = problem_client->getFunction("(room_distance bedroom kitchen)");
  ASSERT_TRUE(func.has_value());
  ASSERT_EQ(func.value().value, 10.0);

  finish = true;
  t.join();
}

TEST(utils, evaluate_function_mod_invalid)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  // Unknown function modifier type
  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(assign (vx) 3.0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);
  test_tree.nodes[0].node_type = plansys2_msgs::msg::Node::UNKNOWN;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, predicates, functions),
    std::make_tuple(false, false, 0));
}

TEST(utils, evaluate_function_mod_invalid_client)
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

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(
    test_tree, "(assign (room_distance bedroom kitchen) 0)", false,
    plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client),
    std::make_tuple(false, false, 0));

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  test_tree.nodes.clear();
  parser::pddl::fromString(
    test_tree, "(assign 0 (room_distance bedroom kitchen))", false,
    plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client),
    std::make_tuple(false, false, 0));

  finish = true;
  t.join();
}

TEST(utils, evaluate_number)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  plansys2_msgs::msg::Tree test_tree;
  parser::pddl::fromString(test_tree, "3.0", false, plansys2_msgs::msg::Node::EXPRESSION);

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions),
    std::make_tuple(true, true, 3.0));
}

TEST(utils, evaluate_invalid)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  plansys2_msgs::msg::Tree test_tree;
  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions),
    std::make_tuple(true, true, 0));

  parser::pddl::fromString(test_tree, "(patrolled wp1)", false, plansys2_msgs::msg::Node::AND);
  test_tree.nodes.front().node_type = plansys2_msgs::msg::Node::UNKNOWN;

  ASSERT_EQ(
    plansys2::evaluate(test_tree, problem_client, predicates, functions),
    std::make_tuple(false, false, 0));
}

TEST(utils, evaluate_exists)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  std::string expression = "(exists (?1 ?2) (and (robot_at rob1 ?1)(connected ?1 ?2)))";
  plansys2_msgs::msg::Tree goal;
  parser::pddl::fromString(goal, expression);

  ASSERT_EQ(
    plansys2::evaluate(goal, problem_client, predicates, functions, false, true),
    std::make_tuple(true, false, 0));

  predicates.push_back(parser::pddl::fromStringPredicate("(robot_at rob1 bedroom)"));

  ASSERT_EQ(
    plansys2::evaluate(goal, problem_client, predicates, functions, false, true),
    std::make_tuple(true, false, 0));

  predicates.push_back(parser::pddl::fromStringPredicate("(connected bedroom kitchen)"));

  ASSERT_EQ(
    plansys2::evaluate(goal, problem_client, predicates, functions, false, true),
    std::make_tuple(true, true, 0));
}

TEST(utils, evaluate_exists_client)
{
  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");

  problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_exists.pddl"});
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(problem_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("bedroom", "room")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("kitchen", "room")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("rob1", "robot")));

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
    plansys2::evaluate(goal, problem_client, predicates, functions, false, false),
    std::make_tuple(true, false, 0));

  ASSERT_TRUE(
    problem_client->addPredicate(
      parser::pddl::fromStringPredicate("(robot_at rob1 bedroom)")));

  ASSERT_EQ(
    plansys2::evaluate(goal, problem_client, predicates, functions, false, false),
    std::make_tuple(true, false, 0));

  ASSERT_TRUE(
    problem_client->addPredicate(
      parser::pddl::fromStringPredicate("(connected bedroom kitchen)")));

  ASSERT_EQ(
    plansys2::evaluate(goal, problem_client, predicates, functions, false, false),
    std::make_tuple(true, true, 0));

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

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  std::string invalid_action_str = "(invalid r2d2 kitchen bedroom)";
  ASSERT_EQ(
    domain_client->getAction(
      plansys2::get_action_name(invalid_action_str),
      plansys2::get_action_params(invalid_action_str)), nullptr);

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
  parser::pddl::fromString(
    test_tree,
    "(and (not(robot_at r2d2 kitchen))(robot_at r2d2 bedroom))");
  expected->effects = test_tree;
  std::shared_ptr<plansys2_msgs::msg::Action> actual =
    domain_client->getAction(
    plansys2::get_action_name(action_str),
    plansys2::get_action_params(action_str));

  ASSERT_EQ(parser::pddl::nameActionsToString(actual), parser::pddl::nameActionsToString(expected));
  ASSERT_EQ(
    parser::pddl::toString(actual->preconditions),
    parser::pddl::toString(expected->preconditions));
  ASSERT_EQ(
    parser::pddl::toString(actual->effects),
    parser::pddl::toString(expected->effects));

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
  parser::pddl::fromString(
    overall_expected->at_end_requirements,
    "(and (person_at Jack kitchen))");
  parser::pddl::fromString(
    overall_expected->at_end_effects,
    "(and (robot_near_person leia Jack))");

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

TEST(utils, replace_children_param) {
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
  std::string domain_file = pkgpath + "/pddl/domain_exists.pddl";

  std::ifstream domain_ifs(domain_file);
  std::string domain_str(
    (std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());
  parser::pddl::Domain domain(domain_str);

  auto action = domain.actions.get("action_test");
  plansys2_msgs::msg::Tree tree;
  action->pre->getTree(tree, domain);
  std::map<std::string, std::string> replace;
  replace["?1"] = "bedroom";
  replace["?2"] = "bathroom";
  plansys2_msgs::msg::Tree tree2 = plansys2::replace_children_param(
    tree,
    1,
    replace
  );
  std::string str = parser::pddl::toString(tree2);
  ASSERT_EQ(
    str,
    "(and (exists (bedroom) (and (robot_at ?0 bedroom)(charging_point_at bedroom)))"
    "(and (>  (battery_level ?0) 1.000000)(<  (battery_level ?0) 200.000000)))");

  auto action2 = domain.actions.get("action_test2");
  plansys2_msgs::msg::Tree tree3;
  action2->pre->getTree(tree3, domain);
  plansys2_msgs::msg::Tree tree4 = plansys2::replace_children_param(
    tree3,
    0,
    replace
  );
  std::string str2 = parser::pddl::toString(tree4);
  ASSERT_EQ(
    str2,
    "(exists (bedroom bathroom) (and (robot_at ?0 bedroom)(connected bedroom bathroom)))");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
