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

#include "plansys2_msgs/msg/knowledge.hpp"

TEST(problem_expert_node, addget_instances)
{
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto test_node_2 = rclcpp::Node::make_shared("test_problem_expert_node_2");
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
  exe.add_node(test_node_2->get_node_base_interface());

  plansys2_msgs::msg::Knowledge last_knowledge_msg;
  int knowledge_msg_counter = 0;
  auto knowledge_sub = test_node_2->create_subscription<plansys2_msgs::msg::Knowledge>(
    "problem_expert/knowledge", rclcpp::QoS(100).transient_local(),
    [&last_knowledge_msg, &knowledge_msg_counter]
    (const plansys2_msgs::msg::Knowledge::SharedPtr msg) {
      last_knowledge_msg = *msg;
      knowledge_msg_counter++;
    });

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("Paco", "person")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("Paco", "person")));
  ASSERT_FALSE(problem_client->addInstance(plansys2::Instance("Paco", "SCIENTIFIC")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("bedroom", "room")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("kitchen", "room")));

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_EQ(knowledge_msg_counter, 4u);
  ASSERT_EQ(last_knowledge_msg.instances.size(), 3u);
  ASSERT_EQ(last_knowledge_msg.instances[0], "Paco");
  ASSERT_EQ(last_knowledge_msg.instances[1], "bedroom");
  ASSERT_EQ(last_knowledge_msg.instances[2], "kitchen");
  ASSERT_EQ(last_knowledge_msg.predicates.size(), 0);
  ASSERT_EQ(last_knowledge_msg.goal, "");

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("r2d2", "robot")));

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_EQ(knowledge_msg_counter, 5u);
  ASSERT_EQ(last_knowledge_msg.instances.size(), 4u);
  ASSERT_EQ(last_knowledge_msg.instances[0], "Paco");
  ASSERT_EQ(last_knowledge_msg.instances[1], "bedroom");
  ASSERT_EQ(last_knowledge_msg.instances[2], "kitchen");
  ASSERT_EQ(last_knowledge_msg.instances[3], "r2d2");
  ASSERT_EQ(last_knowledge_msg.predicates.size(), 0);
  ASSERT_EQ(last_knowledge_msg.goal, "");

  ASSERT_EQ(problem_client->getInstances().size(), 4);
  ASSERT_EQ(problem_client->getInstances()[0].name, "Paco");
  ASSERT_EQ(problem_client->getInstances()[0].type, "person");
  ASSERT_EQ(problem_client->getInstances()[1].name, "bedroom");
  ASSERT_EQ(problem_client->getInstances()[1].type, "room");
  ASSERT_EQ(problem_client->getInstances()[2].name, "kitchen");
  ASSERT_EQ(problem_client->getInstances()[2].type, "room");
  ASSERT_EQ(problem_client->getInstances()[3].name, "r2d2");
  ASSERT_EQ(problem_client->getInstances()[3].type, "robot");

  ASSERT_TRUE(problem_client->removeInstance(plansys2::Instance("Paco", "person")));
  ASSERT_EQ(problem_client->getInstances().size(), 3);
  ASSERT_EQ(problem_client->getInstances()[0].name, "bedroom");
  ASSERT_EQ(problem_client->getInstances()[0].type, "room");
  ASSERT_EQ(problem_client->getInstances()[1].name, "kitchen");
  ASSERT_EQ(problem_client->getInstances()[1].type, "room");
  ASSERT_EQ(problem_client->getInstances()[2].name, "r2d2");
  ASSERT_EQ(problem_client->getInstances()[2].type, "robot");

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_EQ(knowledge_msg_counter, 6u);
  ASSERT_EQ(last_knowledge_msg.instances.size(), 3u);
  ASSERT_EQ(last_knowledge_msg.instances[0], "bedroom");
  ASSERT_EQ(last_knowledge_msg.instances[1], "kitchen");
  ASSERT_EQ(last_knowledge_msg.instances[2], "r2d2");
  ASSERT_EQ(last_knowledge_msg.predicates.size(), 0);
  ASSERT_EQ(last_knowledge_msg.goal, "");

  plansys2_msgs::msg::Node predicate_1;
  predicate_1.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_1.name = "robot_at";
  predicate_1.parameters.push_back(parser::pddl::fromStringParam("r2d2", "robot"));
  predicate_1.parameters.push_back(parser::pddl::fromStringParam("bedroom", "room"));

  ASSERT_TRUE(problem_client->addPredicate(predicate_1));

  plansys2_msgs::msg::Node predicate_2;
  predicate_2.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_2.name = "robot_at";
  predicate_2.parameters.push_back(parser::pddl::fromStringParam("r2d2", "robot"));
  predicate_2.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room"));

  ASSERT_TRUE(problem_client->addPredicate(predicate_2));

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_EQ(knowledge_msg_counter, 8u);
  ASSERT_EQ(last_knowledge_msg.instances.size(), 3u);
  ASSERT_EQ(last_knowledge_msg.instances[0], "bedroom");
  ASSERT_EQ(last_knowledge_msg.instances[1], "kitchen");
  ASSERT_EQ(last_knowledge_msg.instances[2], "r2d2");
  ASSERT_EQ(last_knowledge_msg.predicates.size(), 2u);
  ASSERT_EQ(last_knowledge_msg.predicates[0], "(robot_at r2d2 bedroom)");
  ASSERT_EQ(last_knowledge_msg.predicates[1], "(robot_at r2d2 kitchen)");
  ASSERT_EQ(last_knowledge_msg.goal, "");

  ASSERT_TRUE(problem_client->setGoal(plansys2::Goal("(and (robot_at r2d2 kitchen))")));

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_EQ(knowledge_msg_counter, 9u);
  ASSERT_EQ(last_knowledge_msg.instances[0], "bedroom");
  ASSERT_EQ(last_knowledge_msg.instances[1], "kitchen");
  ASSERT_EQ(last_knowledge_msg.instances[2], "r2d2");
  ASSERT_EQ(last_knowledge_msg.predicates.size(), 2u);
  ASSERT_EQ(last_knowledge_msg.predicates[0], "(robot_at r2d2 bedroom)");
  ASSERT_EQ(last_knowledge_msg.predicates[1], "(robot_at r2d2 kitchen)");
  ASSERT_EQ(last_knowledge_msg.goal, "(and (robot_at r2d2 kitchen))");

  ASSERT_TRUE(problem_client->clearKnowledge());

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->getInstances().empty());
  ASSERT_TRUE(problem_client->getFunctions().empty());
  ASSERT_TRUE(problem_client->getPredicates().empty());

  finish = true;
  t.join();
}

/*
TEST(problem_expert, add_assignments)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("bedroom", "room")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("kitchen", "room_with_teleporter")));

  plansys2_msgs::msg::Node function_1;
  function_1.node_type = plansys2_msgs::msg::Node::FUNCTION;
  function_1.name = "room_distance";
  function_1.parameters.push_back(parser::pddl::fromStringParam("bedroom", "room"));
  function_1.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room_with_teleporter"));
  function_1.value = 1.23;

  ASSERT_EQ(function_1.name, "room_distance");
  ASSERT_EQ(function_1.parameters.size(), 2);
  ASSERT_EQ(function_1.parameters[0].name, "bedroom");
  ASSERT_EQ(function_1.parameters[0].type, "room");
  ASSERT_EQ(function_1.parameters[1].name, "kitchen");
  ASSERT_EQ(function_1.parameters[1].type, "room_with_teleporter");
  ASSERT_EQ(function_1.value, 1.23);

  ASSERT_TRUE(problem_client->addFunction(function_1));

  ASSERT_EQ(
    problem_client->getProblem(),
    "( define ( problem problem_1 )\n"
    "( :domain plansys2 )\n"
    "( :objects\n"
    "\tbedroom - room\n"
    "\tkitchen - room_with_teleporter\n"
    ")\n"
    "( :init\n"
    "\t( = ( room_distance bedroom kitchen ) 1.23 )\n"
    ")\n"
    "( :goal\n"
    "\t( and\n"
    "\t)\n"
    ")\n"
    ")\n");

  plansys2_msgs::msg::Node function_2;
  function_2.node_type = plansys2_msgs::msg::Node::FUNCTION;
  function_2.name = "room_distance";
  function_2.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room_with_teleporter"));
  function_2.parameters.push_back(parser::pddl::fromStringParam("bedroom", "room"));
  function_2.value = 2.34;

  ASSERT_EQ(function_2.name, "room_distance");
  ASSERT_EQ(function_2.parameters.size(), 2);
  ASSERT_EQ(function_2.parameters[0].name, "kitchen");
  ASSERT_EQ(function_2.parameters[0].type, "room_with_teleporter");
  ASSERT_EQ(function_2.parameters[1].name, "bedroom");
  ASSERT_EQ(function_2.parameters[1].type, "room");
  ASSERT_EQ(function_2.value, 2.34);

  ASSERT_TRUE(problem_client->addFunction(function_2));

  ASSERT_EQ(
    problem_client->getProblem(),
    "( define ( problem problem_1 )\n"

    "( :domain plansys2 )\n"
    "( :objects\n"
    "\tbedroom - room\n"
    "\tkitchen - room_with_teleporter\n"
    ")\n"
    "( :init\n"
    "\t( = ( room_distance bedroom kitchen ) 1.23 )\n"
    "\t( = ( room_distance kitchen bedroom ) 2.34 )\n"
    ")\n"
    "( :goal\n"
    "\t( and\n"
    "\t)\n"
    ")\n"
    ")\n");

  function_2.value = 3.45;

  ASSERT_TRUE(problem_client->addFunction(function_2));

  ASSERT_EQ(
    problem_client->getProblem(),
    "( define ( problem problem_1 )\n"
    "( :domain plansys2 )\n"
    "( :objects\n"
    "\tbedroom - room\n"
    "\tkitchen - room_with_teleporter\n"
    ")\n"
    "( :init\n"
    "\t( = ( room_distance bedroom kitchen ) 1.23 )\n"
    "\t( = ( room_distance kitchen bedroom ) 3.45 )\n"
    ")\n"
    "( :goal\n"
    "\t( and\n"
    "\t)\n"
    ")\n"
    ")\n");

  plansys2_msgs::msg::Node function_3;
  function_3.node_type = plansys2_msgs::msg::Node::FUNCTION;
  function_3.name = "room_temperature";
  function_3.parameters.push_back(parser::pddl::fromStringParam("bedroom", "room"));
  function_3.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room_with_teleporter"));
  function_3.value = 2.34;

  ASSERT_FALSE(problem_client->addFunction(function_3));

  ASSERT_FALSE(problem_client->removeFunction(function_3));


  ASSERT_TRUE(problem_client->removeInstance("kitchen"));
}

TEST(problem_expert, addget_predicates)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  plansys2_msgs::msg::Node predicate_1;
  predicate_1.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_1.name = "robot_at";
  predicate_1.parameters.push_back(parser::pddl::fromStringParam("r2d2", "robot"));
  predicate_1.parameters.push_back(parser::pddl::fromStringParam("bedroom", "room"));

  ASSERT_EQ(predicate_1.name, "robot_at");
  ASSERT_EQ(predicate_1.parameters.size(), 2);
  ASSERT_EQ(predicate_1.parameters[0].name, "r2d2");
  ASSERT_EQ(predicate_1.parameters[0].type, "robot");
  ASSERT_EQ(predicate_1.parameters[1].name, "bedroom");
  ASSERT_EQ(predicate_1.parameters[1].type, "room");

  plansys2_msgs::msg::Node predicate_2;
  predicate_2.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_2.name = "robot_at";
  predicate_2.parameters.push_back(parser::pddl::fromStringParam("r2d2", "robot"));
  predicate_2.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room"));

  plansys2_msgs::msg::Node predicate_3;
  predicate_3.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_3.name = "person_at";
  predicate_3.parameters.push_back(parser::pddl::fromStringParam("paco", "person"));
  predicate_3.parameters.push_back(parser::pddl::fromStringParam("bedroom", "room"));

  plansys2_msgs::msg::Node predicate_4;
  predicate_4.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_4.name = "person_at";
  predicate_4.parameters.push_back(parser::pddl::fromStringParam("paco", "person"));
  predicate_4.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room"));

  ASSERT_EQ(predicate_4.name, "person_at");
  ASSERT_EQ(predicate_4.parameters.size(), 2);
  ASSERT_EQ(predicate_4.parameters[0].name, "paco");
  ASSERT_EQ(predicate_4.parameters[0].type, "person");
  ASSERT_EQ(predicate_4.parameters[1].name, "kitchen");
  ASSERT_EQ(predicate_4.parameters[1].type, "room");


  plansys2_msgs::msg::Node predicate_5;
  predicate_5.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_5.name = "person_at";
  predicate_5.parameters.push_back(parser::pddl::fromStringParam("paco", "person"));
  predicate_5.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room"));
  predicate_5.parameters.push_back(parser::pddl::fromStringParam("r2d2", "robot"));
  predicate_5.parameters.push_back(parser::pddl::fromStringParam("bedroom", "room"));

  plansys2_msgs::msg::Node predicate_6;
  predicate_6.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_6.name = "person_at";
  predicate_6.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room"));
  predicate_6.parameters.push_back(parser::pddl::fromStringParam("paco", "person"));

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("paco", "person")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("r2d2", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("bedroom", "room")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("kitchen", "room")));

  std::vector<plansys2_msgs::msg::Node> predicates = problem_client->getPredicates();
  ASSERT_TRUE(predicates.empty());

  ASSERT_TRUE(problem_client->addPredicate(predicate_1));
  predicates = problem_client->getPredicates();
  ASSERT_FALSE(predicates.empty());
  ASSERT_TRUE(problem_client->addPredicate(predicate_1));
  ASSERT_TRUE(problem_client->addPredicate(predicate_2));
  ASSERT_TRUE(problem_client->addPredicate(predicate_3));
  ASSERT_TRUE(problem_client->addPredicate(predicate_4));
  ASSERT_FALSE(problem_client->addPredicate(predicate_5));
  ASSERT_FALSE(problem_client->addPredicate(predicate_6));

  predicates = problem_client->getPredicates();
  ASSERT_EQ(predicates.size(), 4);

  ASSERT_FALSE(problem_client->removePredicate(predicate_5));
  ASSERT_TRUE(problem_client->removePredicate(predicate_4));
  ASSERT_TRUE(problem_client->removePredicate(predicate_4));

  predicates = problem_client->getPredicates();
  ASSERT_EQ(predicates.size(), 3);

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("bathroom", "room_with_teleporter")));

  plansys2_msgs::msg::Node predicate_7;
  predicate_7.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_7.name = "is_teleporter_enabled";
  predicate_7.parameters.push_back(parser::pddl::fromStringParam("bathroom", "room_with_teleporter"));

  ASSERT_EQ(predicate_7.name, "is_teleporter_enabled");
  ASSERT_EQ(predicate_7.parameters.size(), 1);
  ASSERT_EQ(predicate_7.parameters[0].name, "bathroom");
  ASSERT_EQ(predicate_7.parameters[0].type, "room_with_teleporter");

  ASSERT_TRUE(problem_client->addPredicate(predicate_7));

  plansys2_msgs::msg::Node predicate_8;
  predicate_8.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_8.name = "is_teleporter_destination";
  predicate_8.parameters.push_back(parser::pddl::fromStringParam("bathroom", "room_with_teleporter"));

  ASSERT_EQ(predicate_8.name, "is_teleporter_destination");
  ASSERT_EQ(predicate_8.parameters.size(), 1);
  ASSERT_EQ(predicate_8.parameters[0].name, "bathroom");
  ASSERT_EQ(predicate_8.parameters[0].type, "room_with_teleporter");

  ASSERT_TRUE(problem_client->addPredicate(predicate_8));

  ASSERT_TRUE(problem_client->removeInstance("bathroom"));
}

TEST(problem_expert, addget_goals)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("paco", "person")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("r2d2", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("bedroom", "room")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("kitchen", "room")));

  plansys2_msgs::msg::Tree goal;
  parser::pddl::fromString(goal, "(and (robot_at r2d2 bedroom)(person_at paco kitchen))");
  ASSERT_EQ(parser::pddl::toString(goal), "(and (robot_at r2d2 bedroom)(person_at paco kitchen))");

  plansys2_msgs::msg::Tree goal2;
  parser::pddl::fromString(goal2, "(and (robot_at r2d2 bedroom)(not(person_at paco kitchen)))");
  ASSERT_EQ(parser::pddl::toString(goal2), "(and (robot_at r2d2 bedroom)(not (person_at paco kitchen)))");

  ASSERT_TRUE(problem_client->setGoal(goal));
  ASSERT_TRUE(problem_client->setGoal(goal2));

  ASSERT_EQ(
    problem_client->getGoal().toString(),
    "(and (robot_at r2d2 bedroom)(not (person_at paco kitchen)))");

  const plansys2_msgs::msg::Tree & goal3 = problem_client->getGoal();
  ASSERT_EQ(parser::pddl::toString(goal3), "(and (robot_at r2d2 bedroom)(not (person_at paco kitchen)))");

  ASSERT_TRUE(problem_client->clearGoal());
  ASSERT_TRUE(problem_client->clearGoal());

  ASSERT_EQ(problem_client->getGoal().toString(), "");

  ASSERT_TRUE(problem_client->setGoal(plansys2::Goal("(and (or (robot_at r2d2 bedroom)(robot_at r2d2 kitchen))(not(person_at paco kitchen)))")));
}

TEST(problem_expert, get_probem)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  plansys2_msgs::msg::Node predicate_1;
  predicate_1.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_1.name = "robot_at";
  predicate_1.parameters.push_back(parser::pddl::fromStringParam("r2d2", "robot"));
  predicate_1.parameters.push_back(parser::pddl::fromStringParam("bedroom", "room"));

  plansys2_msgs::msg::Node predicate_2;
  predicate_2.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_2.name = "robot_at";
  predicate_2.parameters.push_back(parser::pddl::fromStringParam("r2d2", "robot"));
  predicate_2.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room"));

  plansys2_msgs::msg::Node predicate_3;
  predicate_3.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_3.name = "person_at";
  predicate_3.parameters.push_back(parser::pddl::fromStringParam("paco", "person"));
  predicate_3.parameters.push_back(parser::pddl::fromStringParam("bedroom", "room"));

  plansys2_msgs::msg::Node predicate_4;
  predicate_4.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_4.name = "person_at";
  predicate_4.parameters.push_back(parser::pddl::fromStringParam("paco", "person"));
  predicate_4.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room"));

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("paco", "person")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("r2d2", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("bedroom", "room")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("kitchen", "room")));

  ASSERT_TRUE(problem_client->addPredicate(predicate_1));
  ASSERT_TRUE(problem_client->addPredicate(predicate_2));
  ASSERT_TRUE(problem_client->addPredicate(predicate_3));
  ASSERT_TRUE(problem_client->addPredicate(predicate_4));

  ASSERT_TRUE(problem_client->setGoal(plansys2::Goal("(and (robot_at r2d2 bedroom)(person_at paco kitchen))")));

  ASSERT_EQ(
    problem_client->getProblem(),
    std::string("( define ( problem problem_1 )\n( :domain plansys2 ") +
    std::string(")\n( :objects\n\tpaco - person\n\tr2d2 - robot\n\tbedroom kitchen - room\n)\n") +
    std::string("( :init\n\t( robot_at r2d2 bedroom )\n\t( robot_at r2d2 kitchen )\n\t( ") +
    std::string("person_at paco bedroom )\n\t( person_at paco kitchen )\n)\n( :goal\n\t( ") +
    std::string("and\n\t\t( robot_at r2d2 bedroom )\n\t\t( person_at paco kitchen )\n\t)\n)\n)\n"));
}


TEST(problem_expert, set_goal)
{
  std::string expresion = std::string("(and (patrolled ro1) (patrolled ro2) (patrolled ro3))");
  plansys2_msgs::msg::Tree goal;
  parser::pddl::fromString(goal, expresion);

  ASSERT_EQ(goal.toString(), "(and (patrolled ro1)(patrolled ro2)(patrolled ro3))");
}
*/

TEST(problem_expert_node, addget_goal_is_satisfied)
{
  auto test_node = rclcpp::Node::make_shared("test_problem_expert_node");
  auto test_node_2 = rclcpp::Node::make_shared("test_problem_expert_node_2");
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
  exe.add_node(test_node_2->get_node_base_interface());

  plansys2_msgs::msg::Knowledge last_knowledge_msg;
  int knowledge_msg_counter = 0;
  auto knowledge_sub = test_node_2->create_subscription<plansys2_msgs::msg::Knowledge>(
    "problem_expert/knowledge", rclcpp::QoS(100).transient_local(),
    [&last_knowledge_msg, &knowledge_msg_counter]
    (const plansys2_msgs::msg::Knowledge::SharedPtr msg) {
      last_knowledge_msg = *msg;
      knowledge_msg_counter++;
    });

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

  ASSERT_EQ(knowledge_msg_counter, 5u);
  ASSERT_EQ(last_knowledge_msg.instances.size(), 5u);
  ASSERT_EQ(last_knowledge_msg.instances[0], "leia");
  ASSERT_EQ(last_knowledge_msg.instances[1], "Jack");
  ASSERT_EQ(last_knowledge_msg.instances[2], "bedroom");
  ASSERT_EQ(last_knowledge_msg.instances[3], "kitchen");
  ASSERT_EQ(last_knowledge_msg.instances[4], "m1");
  ASSERT_EQ(last_knowledge_msg.predicates.size(), 0);
  ASSERT_EQ(last_knowledge_msg.goal, "");

  ASSERT_TRUE(
    problem_client->addPredicate(plansys2::Predicate("(robot_at leia kitchen)")));
  ASSERT_TRUE(
    problem_client->addPredicate(plansys2::Predicate("(person_at Jack bedroom)")));

  std::string expression = "(and (robot_talk leia m1 Jack))";
  plansys2_msgs::msg::Tree goal;
  parser::pddl::fromString(goal, expression);
  ASSERT_TRUE(problem_client->setGoal(goal));
  ASSERT_FALSE(problem_client->isGoalSatisfied(goal));

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_EQ(knowledge_msg_counter, 8u);
  ASSERT_EQ(last_knowledge_msg.instances.size(), 5u);
  ASSERT_EQ(last_knowledge_msg.instances[0], "leia");
  ASSERT_EQ(last_knowledge_msg.instances[1], "Jack");
  ASSERT_EQ(last_knowledge_msg.instances[2], "bedroom");
  ASSERT_EQ(last_knowledge_msg.instances[3], "kitchen");
  ASSERT_EQ(last_knowledge_msg.instances[4], "m1");
  ASSERT_EQ(last_knowledge_msg.predicates.size(), 2u);
  ASSERT_EQ(last_knowledge_msg.predicates[0], "(robot_at leia kitchen)");
  ASSERT_EQ(last_knowledge_msg.predicates[1], "(person_at Jack bedroom)");
  ASSERT_EQ(last_knowledge_msg.goal, "(and (robot_talk leia m1 Jack))");

  ASSERT_TRUE(
    problem_client->addPredicate(plansys2::Predicate("(robot_talk leia m1 Jack)")));

  ASSERT_TRUE(problem_client->isGoalSatisfied(goal));

  ASSERT_EQ(knowledge_msg_counter, 9u);
  ASSERT_EQ(last_knowledge_msg.instances.size(), 5u);
  ASSERT_EQ(last_knowledge_msg.instances[0], "leia");
  ASSERT_EQ(last_knowledge_msg.instances[1], "Jack");
  ASSERT_EQ(last_knowledge_msg.instances[2], "bedroom");
  ASSERT_EQ(last_knowledge_msg.instances[3], "kitchen");
  ASSERT_EQ(last_knowledge_msg.instances[4], "m1");
  ASSERT_EQ(last_knowledge_msg.predicates.size(), 3u);
  ASSERT_EQ(last_knowledge_msg.predicates[0], "(robot_at leia kitchen)");
  ASSERT_EQ(last_knowledge_msg.predicates[1], "(person_at Jack bedroom)");
  ASSERT_EQ(last_knowledge_msg.predicates[2], "(robot_talk leia m1 Jack)");
  ASSERT_EQ(last_knowledge_msg.goal, "(and (robot_talk leia m1 Jack))");

  finish = true;
  t.join();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
