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
#include <regex>
#include <iostream>
#include <memory>
#include <map>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "gtest/gtest.h"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/BTBuilder.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/rclcpp.hpp"


TEST(executiotest_noden_tree, bt_builder_factory)
{
  auto test_node = rclcpp::Node::make_shared("get_action_from_string");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/factory.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/factory.pddl"});

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
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

  std::vector<std::string> predicates = {
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

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }

  ASSERT_TRUE(
    problem_client->setGoal(
      plansys2::Goal(
        "(and (car_assembled car_1) (car_assembled car_2) (car_assembled car_3))")));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  std::shared_ptr<plansys2::BTBuilder> bt_builder;
  pluginlib::ClassLoader<plansys2::BTBuilder> bt_builder_loader("plansys2_executor",
    "plansys2::BTBuilder");
  try {
    bt_builder = bt_builder_loader.createSharedInstance("plansys2::SimpleBTBuilder");
  } catch (pluginlib::PluginlibException & ex) {
    std::cerr << "pluginlib error: " << std::string(ex.what()) << std::endl;
  }

  bt_builder->initialize();
  auto tree_str = bt_builder->get_tree(plan.value());

  std::cout << tree_str << std::endl;
  finish = true;
  t.join();
}


TEST(executiotest_noden_tree, bt_builder_factory_2)
{
  auto test_node = rclcpp::Node::make_shared("get_action_from_string");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/factory.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/factory.pddl"});

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
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

  std::vector<std::string> predicates = {
    "(robot_at robot1 wheels_zone)",
    "(robot_at robot2 body_car_zone)",
    "(robot_at robot3 steering_wheels_zone)",
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

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }

  ASSERT_TRUE(
    problem_client->setGoal(
      plansys2::Goal(
        std::string("(and (car_assembled car_1) (piece_at body_car_2 assembly_zone)") +
        std::string("(piece_at body_car_3 assembly_zone))"))));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  std::shared_ptr<plansys2::BTBuilder> bt_builder;
  pluginlib::ClassLoader<plansys2::BTBuilder> bt_builder_loader("plansys2_executor",
    "plansys2::BTBuilder");
  try {
    bt_builder = bt_builder_loader.createSharedInstance("plansys2::SimpleBTBuilder");
  } catch (pluginlib::PluginlibException & ex) {
    std::cerr << "pluginlib error: " << std::string(ex.what()) << std::endl;
  }

  bt_builder->initialize();
  auto tree_str = bt_builder->get_tree(plan.value());

  finish = true;
  t.join();
}

TEST(executiotest_noden_tree, bt_builder_factory_3)
{
  auto test_node = rclcpp::Node::make_shared("get_action_from_string");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_charging.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_charging.pddl"});

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
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
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wp_control", "waypoint")));
  for (unsigned i = 1; i <= 4; i++) {
    ASSERT_TRUE(
      problem_client->addInstance(
        plansys2::Instance(
          "wp" + std::to_string(
            i), "waypoint")));
  }

  std::vector<std::string> predicates = {
    "(robot_at r2d2 wp_control)",
    "(charger_at wp3)",
    "(connected wp_control wp1)",
    "(connected wp1 wp_control)",
    "(connected wp_control wp2)",
    "(connected wp2 wp_control)",
    "(connected wp_control wp3)",
    "(connected wp3 wp_control)",
    "(connected wp_control wp4)",
    "(connected wp4 wp_control)"};

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }

  std::vector<std::string> functions = {
    "(= (speed r2d2) 3)",
    "(= (max_range r2d2) 75)",
    "(= (state_of_charge r2d2) 99)",
    "(= (distance wp1 wp2) 15)",
    "(= (distance wp1 wp3) 18)",
    "(= (distance wp1 wp4) 20)",
    "(= (distance wp1 wp_control) 23)",
    "(= (distance wp_control wp1) 23)",
    "(= (distance wp4 wp1) 20)",
    "(= (distance wp3 wp1) 18)",
    "(= (distance wp2 wp1) 15)",
    "(= (distance wp2 wp3) 23)",
    "(= (distance wp2 wp4) 18)",
    "(= (distance wp2 wp_control) 15)",
    "(= (distance wp_control wp2) 15)",
    "(= (distance wp4 wp2) 18)",
    "(= (distance wp3 wp2) 23)",
    "(= (distance wp3 wp4) 20)",
    "(= (distance wp3 wp_control) 23)",
    "(= (distance wp_control wp3) 23)",
    "(= (distance wp4 wp3) 20)",
    "(= (distance wp4 wp_control) 20)",
    "(= (distance wp_control wp4) 20)"};

  for (const auto & func : functions) {
    ASSERT_TRUE(problem_client->addFunction(plansys2::Function(func)));
  }

  ASSERT_TRUE(
    problem_client->setGoal(
      plansys2::Goal(
        "(and (patrolled wp1) (patrolled wp2) (patrolled wp3) (patrolled wp4))")));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  std::shared_ptr<plansys2::BTBuilder> bt_builder;
  pluginlib::ClassLoader<plansys2::BTBuilder> bt_builder_loader("plansys2_executor",
    "plansys2::BTBuilder");
  try {
    bt_builder = bt_builder_loader.createSharedInstance("plansys2::SimpleBTBuilder");
  } catch (pluginlib::PluginlibException & ex) {
    std::cerr << "pluginlib error: " << std::string(ex.what()) << std::endl;
  }

  bt_builder->initialize();
  auto tree_str = bt_builder->get_tree(plan.value());

  finish = true;
  t.join();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
