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
#include <iostream>
#include <fstream>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "gtest/gtest.h"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_msgs/msg/param.h"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_planner/PlannerClient.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/rclcpp.hpp"


TEST(planner_expert, generate_plan_good)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_planner");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple.pddl"});

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

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("leia", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("francisco", "person")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("message1", "message")));

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("bedroom", "room")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("kitchen", "room")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("corridor", "room")));

  std::vector<std::string> predicates = {
    "(robot_at leia kitchen)",
    "(person_at francisco bedroom)"};

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }

  ASSERT_TRUE(
    problem_client->setGoal(plansys2::Goal("(and (robot_talk leia message1 francisco))")));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);

  finish = true;
  t.join();
}


TEST(planner_expert, generate_plan_with_domain_constants)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_planner");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple_constants.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_simple_constants.pddl"});

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

  std::ifstream problem_1_ifs(pkgpath + "/pddl/problem_simple_constants_1.pddl");
  std::string problem_1_str((
      std::istreambuf_iterator<char>(problem_1_ifs)),
    std::istreambuf_iterator<char>());

  ASSERT_TRUE(problem_client->addProblem(problem_1_str));
  ASSERT_EQ(problem_client->getProblem(), problem_1_str);

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);


  problem_client->clearKnowledge();

  std::ifstream problem_2_ifs(pkgpath + "/pddl/problem_simple_constants_2.pddl");
  std::string problem_2_str((
      std::istreambuf_iterator<char>(problem_2_ifs)),
    std::istreambuf_iterator<char>());

  ASSERT_TRUE(problem_client->addProblem(problem_2_str));
  ASSERT_NE(problem_client->getProblem(), problem_2_str);
  ASSERT_NE(problem_1_str, problem_2_str);

  plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);


  finish = true;
  t.join();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
