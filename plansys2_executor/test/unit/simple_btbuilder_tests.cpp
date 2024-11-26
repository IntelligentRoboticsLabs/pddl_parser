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

#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <regex>
#include <set>
#include <string>
#include <tuple>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_executor/behavior_tree/execute_action_node.hpp"
#include "plansys2_executor/behavior_tree/wait_action_node.hpp"
#include "plansys2_executor/bt_builder_plugins/simple_bt_builder.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/Utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class SimpleBTBuilderTest : public plansys2::bt_builder::SimpleBTBuilder
{
public:
  SimpleBTBuilderTest()
  : SimpleBTBuilder() {}

  std::string get_tree(const plansys2_msgs::msg::Plan & current_plan)
  {
    return plansys2::bt_builder::SimpleBTBuilder::get_tree(current_plan);
  }

  std::vector<plansys2::bt_builder::ActionStamped> get_plan_actions(const plansys2_msgs::msg::Plan & plan)
  {
    return plansys2::bt_builder::SimpleBTBuilder::get_plan_actions(plan);
  }

  bool is_action_executable(
    const plansys2::bt_builder::ActionStamped & action,
    const plansys2::State & state)
  {
    return plansys2::bt_builder::SimpleBTBuilder::is_action_executable(action, state);
  }

  plansys2::bt_builder::ActionGraph::Ptr get_graph(const plansys2_msgs::msg::Plan & current_plan)
  {
    return plansys2::bt_builder::SimpleBTBuilder::get_graph(current_plan);
  }

  std::list<plansys2::bt_builder::ActionNode::Ptr> get_roots(
    std::vector<plansys2::bt_builder::ActionStamped> & action_sequence,
    const plansys2::State & state, int & node_counter)
  {
    return plansys2::bt_builder::SimpleBTBuilder::get_roots(
      action_sequence, state, node_counter);
  }

  plansys2::bt_builder::ActionNode::Ptr get_node_satisfy(
    const plansys2_msgs::msg::Tree & requirement, const plansys2::bt_builder::ActionGraph::Ptr & graph,
    const plansys2::bt_builder::ActionNode::Ptr & current)
  {
    return plansys2::bt_builder::SimpleBTBuilder::get_node_satisfy(requirement, graph, current);
  }

  plansys2::bt_builder::ActionNode::Ptr get_node_satisfy(
    const plansys2_msgs::msg::Tree & requirement, const plansys2::bt_builder::ActionNode::Ptr & node,
    const plansys2::bt_builder::ActionNode::Ptr & current)
  {
    return plansys2::bt_builder::SimpleBTBuilder::get_node_satisfy(requirement, node, current);
  }

  void print_graph(const plansys2::bt_builder::ActionGraph::Ptr & graph) const
  {
    plansys2::bt_builder::SimpleBTBuilder::print_graph(graph);
  }

  void print_graph_csv(const plansys2::bt_builder::ActionGraph::Ptr & graph) const
  {
    plansys2::bt_builder::SimpleBTBuilder::print_graph_csv(graph);
  }

  std::vector<std::tuple<uint32_t, uint32_t, uint32_t, std::string>> get_graph_tabular(
    const plansys2::bt_builder::ActionGraph::Ptr & graph) const
  {
    return plansys2::bt_builder::SimpleBTBuilder::get_graph_tabular(graph);
  }

  void remove_existing_requirements(
    std::vector<plansys2_msgs::msg::Tree> & requirements,
    const plansys2::State & state)
  {
    plansys2::bt_builder::SimpleBTBuilder::remove_existing_requirements(requirements, state);
  }
};

class TestPlannerNode : public rclcpp::Node
{
private:
  rclcpp::Service<plansys2_msgs::srv::ValidateDomain>::SharedPtr
    validate_domain_service_ = create_service<plansys2_msgs::srv::ValidateDomain>(
      "planner/validate_domain",
      std::bind(
        &TestPlannerNode::validate_domain_service_callback,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3));
public:
  TestPlannerNode()
  : Node("test_planner_node") {}

  void validate_domain_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::ValidateDomain::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::ValidateDomain::Response> response)
  {
    response->success = true;
  }
};

TEST(simple_btbuilder_tests, test_plan_1)
{
  auto test_node = rclcpp::Node::make_shared("test_plan_1");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();

  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();

  auto btbuilder = std::make_shared<SimpleBTBuilderTest>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple_2.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple_2.pddl"});

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {
        exe.spin_some();
      }
    });

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("leia", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("entrance", "room")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("kitchen", "room")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("bedroom", "room")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("dinning", "room")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("bathroom", "room")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("chargingroom", "room")));

  std::vector<std::string> predicate_strings = {
    "(connected entrance dinning)", "(connected dinning entrance)",
    "(connected dinning kitchen)", "(connected kitchen dinning)",
    "(connected dinning bedroom)", "(connected bedroom dinning)",
    "(connected bathroom bedroom)", "(connected bedroom bathroom)",
    "(connected chargingroom kitchen)", "(connected kitchen chargingroom)",
    "(charging_point_at chargingroom)", "(battery_low leia)",
    "(robot_at leia entrance)"};

  for (const auto & pred : predicate_strings) {
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }

  ASSERT_TRUE(problem_client->setGoal(plansys2::Goal("(and(robot_at leia bathroom))")));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  auto state = problem_client->getState();

  auto action_sequence = btbuilder->get_plan_actions(plan.value());

  ASSERT_EQ(action_sequence.size(), 6u);

  ASSERT_NEAR(action_sequence[0].time, 0.000, 0.0001);
  ASSERT_EQ(action_sequence[0].action.get_action_name(), "askcharge");
  ASSERT_EQ(action_sequence[0].action.get_action_params()[0].name, "leia");
  ASSERT_EQ(action_sequence[0].action.get_action_params()[1].name, "entrance");
  ASSERT_EQ(action_sequence[0].action.get_action_params()[2].name, "chargingroom");
  ASSERT_EQ(
    parser::pddl::toString(action_sequence[0].action.get_at_start_effects()),
    "(and (not (robot_at leia entrance))(robot_at leia chargingroom))");
  std::vector<plansys2_msgs::msg::Node> action_0_predicates;
  parser::pddl::getPredicates(
    action_0_predicates, action_sequence[0].action.get_at_start_effects());
  ASSERT_EQ(action_0_predicates.size(), 2u);
  ASSERT_EQ(action_0_predicates[0].name, "robot_at");
  ASSERT_EQ(action_0_predicates[0].parameters[0].name, "leia");
  ASSERT_EQ(action_0_predicates[0].parameters[1].name, "entrance");
  ASSERT_TRUE(action_0_predicates[0].negate);

  ASSERT_TRUE(
    plansys2::check(action_sequence[0].action.get_at_start_requirements(), problem_client));

  ASSERT_FALSE(
    plansys2::check(action_sequence[1].action.get_at_start_requirements(), problem_client));
  ASSERT_FALSE(
    plansys2::check(action_sequence[2].action.get_at_start_requirements(), problem_client));
  ASSERT_FALSE(
    plansys2::check(action_sequence[3].action.get_at_start_requirements(), problem_client));
  ASSERT_FALSE(
    plansys2::check(action_sequence[4].action.get_at_start_requirements(), problem_client));
  ASSERT_FALSE(
    plansys2::check(action_sequence[5].action.get_at_start_requirements(), problem_client));

  ASSERT_TRUE(
    btbuilder->is_action_executable(action_sequence[0], state));
  ASSERT_FALSE(
    btbuilder->is_action_executable(action_sequence[1], state));
  ASSERT_FALSE(
    btbuilder->is_action_executable(action_sequence[2], state));
  ASSERT_FALSE(
    btbuilder->is_action_executable(action_sequence[3], state));
  ASSERT_FALSE(
    btbuilder->is_action_executable(action_sequence[4], state));
  ASSERT_FALSE(
    btbuilder->is_action_executable(action_sequence[5], state));

  ASSERT_TRUE(state.hasPredicate(parser::pddl::fromStringPredicate("(robot_at leia entrance)")));
  ASSERT_FALSE(state.hasPredicate(parser::pddl::fromStringPredicate("(robot_at leia chargingroom)")));

  plansys2::apply(
    action_sequence[0].action.get_at_start_effects(), state);
  plansys2::apply(action_sequence[0].action.get_at_end_effects(), state);

  ASSERT_FALSE(state.hasPredicate(parser::pddl::fromStringPredicate("(robot_at leia entrance)")));
  ASSERT_TRUE(state.hasPredicate(parser::pddl::fromStringPredicate("(robot_at leia chargingroom)")));

  ASSERT_TRUE(
    btbuilder->is_action_executable(action_sequence[1], state));
  ASSERT_FALSE(
    btbuilder->is_action_executable(action_sequence[2], state));
  ASSERT_FALSE(
    btbuilder->is_action_executable(action_sequence[3], state));
  ASSERT_FALSE(
    btbuilder->is_action_executable(action_sequence[4], state));
  ASSERT_FALSE(
    btbuilder->is_action_executable(action_sequence[5], state));
  plansys2::apply(
    action_sequence[1].action.get_at_start_effects(), state);
  plansys2::apply(action_sequence[1].action.get_at_end_effects(), state);

  ASSERT_TRUE(
    btbuilder->is_action_executable(action_sequence[2], state));
  ASSERT_FALSE(
    btbuilder->is_action_executable(action_sequence[3], state));
  ASSERT_FALSE(
    btbuilder->is_action_executable(action_sequence[4], state));
  ASSERT_FALSE(
    btbuilder->is_action_executable(action_sequence[5], state));
  plansys2::apply(
    action_sequence[2].action.get_at_start_effects(), state);
  plansys2::apply(action_sequence[2].action.get_at_end_effects(), state);

  ASSERT_TRUE(
    btbuilder->is_action_executable(action_sequence[3], state));
  ASSERT_FALSE(
    btbuilder->is_action_executable(action_sequence[4], state));
  ASSERT_FALSE(
    btbuilder->is_action_executable(action_sequence[5], state));
  plansys2::apply(
    action_sequence[3].action.get_at_start_effects(), state);
  plansys2::apply(action_sequence[3].action.get_at_end_effects(), state);

  ASSERT_TRUE(
    btbuilder->is_action_executable(action_sequence[4], state));
  ASSERT_FALSE(
    btbuilder->is_action_executable(action_sequence[5], state));
  plansys2::apply(
    action_sequence[4].action.get_at_start_effects(), state);
  plansys2::apply(action_sequence[4].action.get_at_end_effects(), state);

  ASSERT_TRUE(
    btbuilder->is_action_executable(action_sequence[5], state));
  plansys2::apply(
    action_sequence[5].action.get_at_start_effects(), state);
  plansys2::apply(action_sequence[5].action.get_at_end_effects(), state);

  ASSERT_TRUE(state.hasPredicate(parser::pddl::fromStringPredicate("(robot_at leia bathroom)")));

  finish = true;
  t.join();
}

TEST(simple_btbuilder_tests, test_plan_2)
{
  auto test_node = rclcpp::Node::make_shared("test_plan_2");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();

  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();

  auto btbuilder = std::make_shared<SimpleBTBuilderTest>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/factory.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/factory.pddl"});

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {
        exe.spin_some();
      }
    });

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("robot1", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("robot2", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("robot3", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wheels_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("steering_wheels_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("body_car_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("assembly_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wheel_1", "piece")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wheel_2", "piece")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wheel_3", "piece")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("body_car_1", "piece")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("body_car_2", "piece")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("body_car_3", "piece")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("steering_wheel_1", "piece")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("steering_wheel_2", "piece")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("steering_wheel_3", "piece")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("car_1", "car")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("car_2", "car")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("car_3", "car")));

  std::vector<std::string> predicate_strings = {
    "(robot_at robot1 assembly_zone)",
    "(robot_at robot2 assembly_zone)",
    "(robot_at robot3 assembly_zone)",
    "(is_assembly_zone assembly_zone)",
    "(robot_available robot1)",
    "(robot_available robot2)",
    "(robot_available robot3)",
    "(piece_at wheel_1 wheels_zone)",
    "(piece_at body_car_1 body_car_zone)",
    "(piece_at steering_wheel_1 steering_wheels_zone)",
    "(piece_is_wheel wheel_1)",
    "(piece_is_body_car body_car_1)",
    "(piece_is_steering_wheel steering_wheel_1)",
    "(piece_at wheel_2 wheels_zone)",
    "(piece_at body_car_2 body_car_zone)",
    "(piece_at steering_wheel_2 steering_wheels_zone)",
    "(piece_is_wheel wheel_2)",
    "(piece_is_body_car body_car_2)",
    "(piece_is_steering_wheel steering_wheel_2)",
    "(piece_at wheel_3 wheels_zone)",
    "(piece_at body_car_3 body_car_zone)",
    "(piece_at steering_wheel_3 steering_wheels_zone)",
    "(piece_is_wheel wheel_3)",
    "(piece_is_body_car body_car_3)",
    "(piece_is_steering_wheel steering_wheel_3)",
    "(piece_not_used wheel_1)",
    "(piece_not_used wheel_2)",
    "(piece_not_used wheel_3)",
    "(piece_not_used body_car_1)",
    "(piece_not_used body_car_2)",
    "(piece_not_used body_car_3)",
    "(piece_not_used steering_wheel_1)",
    "(piece_not_used steering_wheel_2)",
    "(piece_not_used steering_wheel_3)"};

  for (const auto & pred : predicate_strings) {
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }

  ASSERT_TRUE(
    problem_client->setGoal(
      plansys2::Goal("(and(car_assembled car_1)(car_assembled car_2)(car_assembled car_3))")));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  auto state = problem_client->getState();

  auto predicates_plus_one = predicate_strings;
  predicates_plus_one.push_back("(is_assembly_zone body_car_zone)");
  plansys2_msgs::msg::Tree tree = parser::pddl::fromPredicates(predicates_plus_one);

  std::vector<plansys2_msgs::msg::Tree> check_predicates = parser::pddl::getSubtrees(tree);

  btbuilder->remove_existing_requirements(check_predicates, state);

  ASSERT_EQ(check_predicates.size(), 1);
  ASSERT_EQ(parser::pddl::toString(check_predicates.front()), "(is_assembly_zone body_car_zone)");

  ASSERT_EQ(problem_client->getPredicates().size(), state.getPredicatesSize());

  auto action_sequence = btbuilder->get_plan_actions(plan.value());

  ASSERT_EQ(action_sequence.size(), 22u);

  int node_counter = 0;
  auto roots =
    btbuilder->get_roots(action_sequence, state, node_counter);
  ASSERT_EQ(roots.size(), 3u);
  // Apply roots actions
  for (auto & action_node : roots) {
    plansys2::apply(
      action_node->action.action.get_at_start_effects(), state);
    plansys2::apply(
      action_node->action.action.get_at_end_effects(), state);
  }

  ASSERT_TRUE(state.hasPredicate(parser::pddl::fromStringPredicate("(robot_at robot1 steering_wheels_zone)")));
  ASSERT_TRUE(state.hasPredicate(parser::pddl::fromStringPredicate("(robot_at robot2 wheels_zone)")));
  ASSERT_TRUE(state.hasPredicate(parser::pddl::fromStringPredicate("(robot_at robot3 body_car_zone)")));

  tree.nodes.clear();
  parser::pddl::fromString(
    tree, "(robot_at robot3 body_car_zone)", false, plansys2_msgs::msg::Node::AND);
  auto node_satisfy_1 = btbuilder->get_node_satisfy(tree, *roots.begin(), nullptr);
  ASSERT_NE(node_satisfy_1, nullptr);
  ASSERT_EQ(node_satisfy_1->action.action.get_action_name(), "move");
  ASSERT_EQ(node_satisfy_1->action.action.get_action_params().size(), 3u);
  ASSERT_EQ(node_satisfy_1->action.action.get_action_params()[0].name, "robot3");
  ASSERT_EQ(node_satisfy_1->action.action.get_action_params()[1].name, "assembly_zone");
  ASSERT_EQ(node_satisfy_1->action.action.get_action_params()[2].name, "body_car_zone");

  auto it = roots.begin();
  it++;
  ASSERT_EQ(btbuilder->get_node_satisfy(tree, *it, nullptr), nullptr);
  it++;
  ASSERT_EQ(btbuilder->get_node_satisfy(tree, *it, nullptr), nullptr);

  auto graph = btbuilder->get_graph(plan.value());
  ASSERT_NE(graph, nullptr);

  btbuilder->print_graph(graph);

  finish = true;
  t.join();
}

TEST(simple_btbuilder_tests, test_plan_3)
{
  auto test_node = rclcpp::Node::make_shared("test_plan_3");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();

  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();

  auto btbuilder = std::make_shared<SimpleBTBuilderTest>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple_2.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple_2.pddl"});

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {
        exe.spin_some();
      }
    });

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("leia", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("ro1", "room")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("ro2", "room")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("ro3", "room")));

  std::vector<std::string> predicates = {
    "(connected ro1 ro2)", "(connected ro2 ro1)", "(connected ro1 ro3)", "(connected ro3 ro1)",
    "(connected ro2 ro3)", "(connected ro3 ro2)", "(robot_at leia ro1)", "(battery_full leia)"};

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }

  ASSERT_TRUE(
    problem_client->setGoal(
      plansys2::Goal("(and (patrolled ro1) (patrolled ro2) (patrolled ro3))")));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  auto bt = btbuilder->get_tree(plan.value());

  std::cerr << bt << std::endl;

  finish = true;
  t.join();
}

TEST(simple_btbuilder_tests, test_plan_4)
{
  auto test_node = rclcpp::Node::make_shared("test_plan_4");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();

  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();

  auto btbuilder = std::make_shared<SimpleBTBuilderTest>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/cooking_domain.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/cooking_domain.pddl"});

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {
        exe.spin_some();
      }
    });

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("r2d2", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("c3po", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("fridge_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("pantry_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("watertap_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("cooking_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("recharge_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("eggs", "ingredient")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("oil", "ingredient")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("salt", "ingredient")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("omelette", "dish")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("flour", "ingredient")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("sugar", "ingredient")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("cake", "dish")));

  std::vector<std::string> predicates = {
    "(is_cooking_zone cooking_zone)",
    "(is_fridge_zone fridge_zone)",
    "(is_fridge_zone fridge_zone)",
    "(is_watertap_zone watertap_zone)",
    "(is_recharge_zone recharge_zone)",
    "(ingredient_at eggs fridge_zone)",
    "(ingredient_at oil pantry_zone)",
    "(ingredient_at salt pantry_zone)",
    "(ingredient_at flour pantry_zone)",
    "(ingredient_at sugar pantry_zone)",
    "(is_oil oil)",
    "(is_egg eggs)",
    "(is_salt salt)",
    "(is_flour flour)",
    "(is_sugar sugar)",
    "(is_cake cake)",
    "(is_omelette omelette)",
    "(robot_at r2d2 cooking_zone)",
    "(battery_full r2d2)",
    "(robot_at c3po cooking_zone)",
    "(battery_full c3po)"};

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }

  ASSERT_TRUE(
    problem_client->setGoal(plansys2::Goal("(and (dish_prepared cake)(dish_prepared omelette))")));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  btbuilder->print_graph(btbuilder->get_graph(plan.value()));
  auto bt = btbuilder->get_tree(plan.value());

  std::cerr << bt << std::endl;

  // Todo: Test that the BT is correct

  finish = true;
  t.join();
}

TEST(simple_btbuilder_tests, test_plan_5)
{
  auto test_node = rclcpp::Node::make_shared("test_plan_5");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();

  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();

  auto btbuilder = std::make_shared<SimpleBTBuilderTest>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/road_trip_domain.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/road_trip_domain.pddl"});

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {
        exe.spin_some();
      }
    });

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  std::ifstream problem_ifs(pkgpath + "/pddl/road_trip_problem.pddl");
  std::string problem_str(
    (std::istreambuf_iterator<char>(problem_ifs)), std::istreambuf_iterator<char>());
  ASSERT_TRUE(problem_client->addProblem(problem_str));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  auto action_graph = btbuilder->get_graph(plan.value());
  btbuilder->print_graph_csv(action_graph);

  auto tabulated_graph = btbuilder->get_graph_tabular(action_graph);

  std::ifstream expected_graph_ifs(pkgpath + "/test_data/road_trip_graph.csv");

  std::string line, word, action;
  std::string whitespace = " \n\r\t\f\v";
  uint32_t root_num, node_num, level_num;
  std::vector<std::tuple<uint32_t, uint32_t, uint32_t, std::string>> expected_graph;

  while (std::getline(expected_graph_ifs, line)) {
    std::stringstream ss(line);
    unsigned i = 0;
    while (std::getline(ss, word, ',')) {
      if (i == 0) {
        root_num = std::stoul(word);
      }
      if (i == 1) {
        node_num = std::stoul(word);
      }
      if (i == 2) {
        level_num = std::stoul(word);
      }
      if (i == 3) {
        action = word;
        size_t start = action.find_first_not_of(whitespace);
        action = (start == std::string::npos) ? "" : action.substr(start);
        size_t end = action.find_last_not_of(whitespace);
        action = (end == std::string::npos) ? "" : action.substr(0, end + 1);
      }
      i++;
    }
    expected_graph.push_back(std::make_tuple(root_num, node_num, level_num, action));
  }

  ASSERT_EQ(tabulated_graph.size(), expected_graph.size());
  for (size_t i = 0; i < tabulated_graph.size(); i++) {
    ASSERT_EQ(std::get<0>(tabulated_graph[i]), std::get<0>(expected_graph[i]));
    ASSERT_EQ(std::get<1>(tabulated_graph[i]), std::get<1>(expected_graph[i]));
    ASSERT_EQ(std::get<2>(tabulated_graph[i]), std::get<2>(expected_graph[i]));
    ASSERT_EQ(std::get<3>(tabulated_graph[i]), std::get<3>(expected_graph[i]));
  }

  finish = true;
  t.join();
}

TEST(simple_btbuilder_tests, test_plan_6)
{
  auto test_node = rclcpp::Node::make_shared("test_plan_6");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();

  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();

  auto btbuilder = std::make_shared<SimpleBTBuilderTest>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/elevator_domain.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/elevator_domain.pddl"});

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {
        exe.spin_some();
      }
    });

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  std::ifstream problem_ifs(pkgpath + "/pddl/elevator_problem.pddl");
  std::string problem_str(
    (std::istreambuf_iterator<char>(problem_ifs)), std::istreambuf_iterator<char>());
  ASSERT_TRUE(problem_client->addProblem(problem_str));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  auto action_graph = btbuilder->get_graph(plan.value());
  btbuilder->print_graph_csv(action_graph);

  auto tabulated_graph = btbuilder->get_graph_tabular(action_graph);

  std::ifstream expected_graph_ifs(pkgpath + "/test_data/elevator_graph.csv");

  std::string line, word, action;
  std::string whitespace = " \n\r\t\f\v";
  uint32_t root_num, node_num, level_num;
  std::vector<std::tuple<uint32_t, uint32_t, uint32_t, std::string>> expected_graph;

  while (std::getline(expected_graph_ifs, line)) {
    std::stringstream ss(line);
    unsigned i = 0;
    while (std::getline(ss, word, ',')) {
      if (i == 0) {
        root_num = std::stoul(word);
      }
      if (i == 1) {
        node_num = std::stoul(word);
      }
      if (i == 2) {
        level_num = std::stoul(word);
      }
      if (i == 3) {
        action = word;
        size_t start = action.find_first_not_of(whitespace);
        action = (start == std::string::npos) ? "" : action.substr(start);
        size_t end = action.find_last_not_of(whitespace);
        action = (end == std::string::npos) ? "" : action.substr(0, end + 1);
      }
      i++;
    }
    expected_graph.push_back(std::make_tuple(root_num, node_num, level_num, action));
  }

  ASSERT_EQ(tabulated_graph.size(), expected_graph.size());
  for (size_t i = 0; i < tabulated_graph.size(); i++) {
    ASSERT_EQ(std::get<0>(tabulated_graph[i]), std::get<0>(expected_graph[i]));
    ASSERT_EQ(std::get<1>(tabulated_graph[i]), std::get<1>(expected_graph[i]));
    ASSERT_EQ(std::get<2>(tabulated_graph[i]), std::get<2>(expected_graph[i]));
    ASSERT_EQ(std::get<3>(tabulated_graph[i]), std::get<3>(expected_graph[i]));
  }

  finish = true;
  t.join();
}

TEST(simple_btbuilder_tests, test_plan_with_derived_existential)
{
  auto test_node = rclcpp::Node::make_shared("test_plan_1");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<TestPlannerNode>();

  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();

  auto btbuilder = std::make_shared<SimpleBTBuilderTest>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/suave_domain.pddl"});
  domain_node->set_parameter({"validate_using_planner_node", true});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/suave_domain.pddl"});
  problem_node->set_parameter({"problem_file", pkgpath + "/pddl/suave_problem.pddl"});

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {
        exe.spin_some();
      }
    });

  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  plansys2_msgs::msg::Plan plan;

  plansys2_msgs::msg::PlanItem plan_item_0;
  plansys2_msgs::msg::PlanItem plan_item_1;
  plansys2_msgs::msg::PlanItem plan_item_2;
  plansys2_msgs::msg::PlanItem plan_item_3;
  plansys2_msgs::msg::PlanItem plan_item_4;
  plansys2_msgs::msg::PlanItem plan_item_5;
  plansys2_msgs::msg::PlanItem plan_item_6;
  plan_item_0.action = "(start_robot bluerov)";
  plan_item_0.time = 0;
  plan_item_1.action = "(reconfigure f_maintain_motion fd_unground fd_recover_thrusters)";
  plan_item_1.time = 0;
  // plan_item_1.time = 1;
  plan_item_2.action = "(reconfigure f_generate_search_path fd_unground fd_spiral_high)";
  plan_item_2.time = 0;
  // plan_item_2.time = 2;
  plan_item_3.action = "(search_pipeline a_search_pipeline pipeline bluerov fd_spiral_high fd_recover_thrusters)";
  plan_item_3.time = 0;
  // plan_item_3.time = 3;
  plan_item_4.action = "(reconfigure f_follow_pipeline fd_unground fd_follow_pipeline)";
  plan_item_4.time = 0;
  // plan_item_4.time = 4;
  plan_item_5.action = "(reconfigure f_generate_search_path fd_spiral_high fd_unground)";
  plan_item_5.time = 0;
  // plan_item_5.time = 5;
  plan_item_6.action = "(inspect_pipeline a_inspect_pipeline pipeline bluerov fd_follow_pipeline fd_recover_thrusters)";
  // plan_item_6.time = 6;
  plan_item_6.time = 0;

  plan.items.push_back(plan_item_0);
  plan.items.push_back(plan_item_1);
  plan.items.push_back(plan_item_2);
  plan.items.push_back(plan_item_3);
  plan.items.push_back(plan_item_4);
  plan.items.push_back(plan_item_5);
  plan.items.push_back(plan_item_6);

  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  auto action_graph = btbuilder->get_graph(plan);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::cout << "\n get_graph time difference = " << std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count() << "[ms]\n" << std::endl;
  btbuilder->print_graph_csv(action_graph);

  auto tabulated_graph = btbuilder->get_graph_tabular(action_graph);

  std::ifstream expected_graph_ifs(pkgpath + "/test_data/suave_graph.csv");

  std::string line, word, action;
  std::string whitespace = " \n\r\t\f\v";
  uint32_t root_num, node_num, level_num;
  std::vector<std::tuple<uint32_t, uint32_t, uint32_t, std::string>> expected_graph;

  while (std::getline(expected_graph_ifs, line)) {
    std::stringstream ss(line);
    unsigned i = 0;
    while (std::getline(ss, word, ',')) {
      if (i == 0) {
        root_num = std::stoul(word);
      }
      if (i == 1) {
        node_num = std::stoul(word);
      }
      if (i == 2) {
        level_num = std::stoul(word);
      }
      if (i == 3) {
        action = word;
        size_t start = action.find_first_not_of(whitespace);
        action = (start == std::string::npos) ? "" : action.substr(start);
        size_t end = action.find_last_not_of(whitespace);
        action = (end == std::string::npos) ? "" : action.substr(0, end + 1);
      }
      i++;
    }
    expected_graph.push_back(std::make_tuple(root_num, node_num, level_num, action));
  }

  ASSERT_EQ(tabulated_graph.size(), expected_graph.size());
  for (size_t i = 0; i < tabulated_graph.size(); i++) {
    ASSERT_EQ(std::get<0>(tabulated_graph[i]), std::get<0>(expected_graph[i]));
    ASSERT_EQ(std::get<1>(tabulated_graph[i]), std::get<1>(expected_graph[i]));
    ASSERT_EQ(std::get<2>(tabulated_graph[i]), std::get<2>(expected_graph[i]));
    ASSERT_EQ(std::get<3>(tabulated_graph[i]), std::get<3>(expected_graph[i]));
  }

  finish = true;
  t.join();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
