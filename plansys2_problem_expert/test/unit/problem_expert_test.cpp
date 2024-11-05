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

#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "gtest/gtest.h"
#include "plansys2_domain_expert/DomainExpert.hpp"
#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_problem_expert/ProblemExpert.hpp"

TEST(problem_expert, addget_instances)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str(
    (std::istreambuf_iterator<char>(domain_ifs)), std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("Paco", "person")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("Paco", "person")));
  ASSERT_FALSE(problem_expert.addInstance(parser::pddl::fromStringParam("Paco", "room")));
  ASSERT_FALSE(problem_expert.addInstance(parser::pddl::fromStringParam("Paco", "SCIENTIFIC")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("r2d2", "robot")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("ur5e", "Robot")));

  auto instances = problem_expert.getInstances();
  ASSERT_EQ(instances.size(), 3);
  ASSERT_TRUE(instances.find(parser::pddl::fromStringParam("Paco", "person")) != instances.end());
  ASSERT_TRUE(instances.find(parser::pddl::fromStringParam("r2d2", "robot")) != instances.end());
  ASSERT_TRUE(instances.find(parser::pddl::fromStringParam("ur5e", "robot")) != instances.end());

  ASSERT_TRUE(problem_expert.removeInstance(parser::pddl::fromStringParam("Paco", "person")));
  instances = problem_expert.getInstances();
  ASSERT_EQ(instances.size(), 2);
  ASSERT_TRUE(instances.find(parser::pddl::fromStringParam("r2d2", "robot")) != instances.end());
  ASSERT_TRUE(instances.find(parser::pddl::fromStringParam("ur5e", "robot")) != instances.end());

  auto paco_instance = problem_expert.getInstance("Paco");
  ASSERT_FALSE(paco_instance);
  auto r2d2_instance = problem_expert.getInstance("r2d2");
  ASSERT_TRUE(r2d2_instance);
  ASSERT_EQ(r2d2_instance.value().name, "r2d2");
  ASSERT_EQ(r2d2_instance.value().type, "robot");
  auto ur5e_instance = problem_expert.getInstance("ur5e");
  ASSERT_TRUE(ur5e_instance);
  ASSERT_EQ(ur5e_instance.value().name, "ur5e");
  ASSERT_EQ(ur5e_instance.value().type, "robot");
}

TEST(problem_expert, add_functions)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str(
    (std::istreambuf_iterator<char>(domain_ifs)), std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("bedroom", "room")));
  ASSERT_TRUE(
    problem_expert.addInstance(parser::pddl::fromStringParam("kitchen", "room_with_teleporter")));

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

  ASSERT_TRUE(problem_expert.addFunction(function_1));

  ASSERT_EQ(
    problem_expert.getProblem(),
    std::string("( define ( problem problem_1 )\n( :domain simple )\n") +
    std::string("( :objects\n\tbedroom - room\n\tkitchen - room_with_teleporter\n)\n") +
    std::string("( :init\n\t( = ( room_distance bedroom kitchen ) 1.2300000000 )\n)\n") +
    std::string("( :goal\n\t( and\n\t))\n)\n"));

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

  ASSERT_TRUE(problem_expert.addFunction(function_2));

  ASSERT_EQ(
    problem_expert.getProblem(),
    std::string("( define ( problem problem_1 )\n( :domain simple )\n") +
    std::string("( :objects\n\tbedroom - room\n\tkitchen - room_with_teleporter\n)\n") +
    std::string("( :init\n\t( = ( room_distance kitchen bedroom ) 2.3400000000 )\n") +
    std::string("\t( = ( room_distance bedroom kitchen ) 1.2300000000 )\n)\n") +
    std::string("( :goal\n\t( and\n\t))\n)\n"));

  function_2.value = 3.45;

  ASSERT_TRUE(problem_expert.addFunction(function_2));

  ASSERT_EQ(
    problem_expert.getProblem(),
    std::string("( define ( problem problem_1 )\n( :domain simple )\n") +
    std::string("( :objects\n\tbedroom - room\n\tkitchen - room_with_teleporter\n)\n") +
    std::string("( :init\n\t( = ( room_distance kitchen bedroom ) 3.4500000000 )\n") +
    std::string("\t( = ( room_distance bedroom kitchen ) 1.2300000000 )\n)\n") +
    std::string("( :goal\n\t( and\n\t))\n)\n"));

  plansys2_msgs::msg::Node function_3;
  function_3.node_type = plansys2_msgs::msg::Node::FUNCTION;
  function_3.name = "room_temperature";
  function_3.parameters.push_back(parser::pddl::fromStringParam("bedroom", "room"));
  function_3.parameters.push_back(parser::pddl::fromStringParam("kitchen", "room_with_teleporter"));
  function_3.value = 2.34;

  ASSERT_FALSE(problem_expert.addFunction(function_3));

  ASSERT_FALSE(problem_expert.removeFunction(function_3));

  ASSERT_TRUE(
    problem_expert.removeInstance(
      parser::pddl::fromStringParam("kitchen", "room_with_teleporter")));
}

TEST(problem_expert, addget_predicates)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str(
    (std::istreambuf_iterator<char>(domain_ifs)), std::istreambuf_iterator<char>());

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

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("paco", "person")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("r2d2", "robot")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("bedroom", "room")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("kitchen", "room")));

  std::unordered_set<plansys2::Predicate> predicates = problem_expert.getPredicates();
  ASSERT_TRUE(predicates.empty());

  ASSERT_TRUE(problem_expert.addPredicate(predicate_1));
  predicates = problem_expert.getPredicates();
  ASSERT_FALSE(predicates.empty());
  ASSERT_TRUE(problem_expert.addPredicate(predicate_1));
  ASSERT_TRUE(problem_expert.addPredicate(predicate_2));
  ASSERT_TRUE(problem_expert.addPredicate(predicate_3));
  ASSERT_TRUE(problem_expert.addPredicate(predicate_4));
  ASSERT_FALSE(problem_expert.addPredicate(predicate_5));
  ASSERT_FALSE(problem_expert.addPredicate(predicate_6));

  predicates = problem_expert.getPredicates();
  ASSERT_EQ(predicates.size(), 4);

  auto pred_2 = problem_expert.getPredicate("(robot_at r2d2 kitchen)");
  ASSERT_TRUE(pred_2);
  ASSERT_EQ(pred_2.value().name, "robot_at");
  ASSERT_EQ(pred_2.value().parameters.size(), 2);
  ASSERT_EQ(pred_2.value().parameters[0].name, "r2d2");
  ASSERT_EQ(pred_2.value().parameters[0].type, "robot");
  ASSERT_EQ(pred_2.value().parameters[1].name, "kitchen");
  ASSERT_EQ(pred_2.value().parameters[1].type, "room");

  ASSERT_FALSE(problem_expert.removePredicate(predicate_5));
  ASSERT_TRUE(problem_expert.removePredicate(predicate_4));
  ASSERT_FALSE(problem_expert.removePredicate(predicate_4));

  predicates = problem_expert.getPredicates();
  ASSERT_EQ(predicates.size(), 3);

  ASSERT_TRUE(
    problem_expert.addInstance(parser::pddl::fromStringParam("bathroom", "room_with_teleporter")));

  plansys2_msgs::msg::Node predicate_7;
  predicate_7.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_7.name = "is_teleporter_enabled";
  predicate_7.parameters.push_back(
    parser::pddl::fromStringParam("bathroom", "room_with_teleporter"));

  ASSERT_EQ(predicate_7.name, "is_teleporter_enabled");
  ASSERT_EQ(predicate_7.parameters.size(), 1);
  ASSERT_EQ(predicate_7.parameters[0].name, "bathroom");
  ASSERT_EQ(predicate_7.parameters[0].type, "room_with_teleporter");

  ASSERT_TRUE(problem_expert.addPredicate(predicate_7));

  plansys2_msgs::msg::Node predicate_8;
  predicate_8.node_type = plansys2_msgs::msg::Node::PREDICATE;
  predicate_8.name = "is_teleporter_destination";
  predicate_8.parameters.push_back(
    parser::pddl::fromStringParam("bathroom", "room_with_teleporter"));

  ASSERT_EQ(predicate_8.name, "is_teleporter_destination");
  ASSERT_EQ(predicate_8.parameters.size(), 1);
  ASSERT_EQ(predicate_8.parameters[0].name, "bathroom");
  ASSERT_EQ(predicate_8.parameters[0].type, "room_with_teleporter");

  ASSERT_TRUE(problem_expert.addPredicate(predicate_8));

  ASSERT_TRUE(
    problem_expert.removeInstance(
      parser::pddl::fromStringParam("bathroom", "room_with_teleporter")));
}

TEST(problem_expert, addget_functions)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_charging.pddl");
  std::string domain_str(
    (std::istreambuf_iterator<char>(domain_ifs)), std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  plansys2_msgs::msg::Node function_1;
  function_1.node_type = plansys2_msgs::msg::Node::FUNCTION;
  function_1.name = "speed";
  function_1.parameters.push_back(parser::pddl::fromStringParam("r2d2", "robot"));
  function_1.value = 3;

  ASSERT_EQ(function_1.name, "speed");
  ASSERT_EQ(function_1.parameters.size(), 1);
  ASSERT_EQ(function_1.parameters[0].name, "r2d2");
  ASSERT_EQ(function_1.parameters[0].type, "robot");
  ASSERT_EQ(function_1.value, 3);

  plansys2_msgs::msg::Node function_2;
  function_2.node_type = plansys2_msgs::msg::Node::FUNCTION;
  function_2.name = "distance";
  function_2.parameters.push_back(parser::pddl::fromStringParam("wp1", "waypoint"));
  function_2.parameters.push_back(parser::pddl::fromStringParam("wp2", "waypoint"));
  function_2.value = 15;

  ASSERT_EQ(function_2.name, "distance");
  ASSERT_EQ(function_2.parameters.size(), 2);
  ASSERT_EQ(function_2.parameters[0].name, "wp1");
  ASSERT_EQ(function_2.parameters[0].type, "waypoint");
  ASSERT_EQ(function_2.parameters[1].name, "wp2");
  ASSERT_EQ(function_2.parameters[1].type, "waypoint");
  ASSERT_EQ(function_2.value, 15);

  plansys2_msgs::msg::Node function_3;
  function_3.node_type = plansys2_msgs::msg::Node::FUNCTION;
  function_3.name = "speed";
  function_3.parameters.push_back(parser::pddl::fromStringParam("r2d2", "robot"));
  function_3.parameters.push_back(parser::pddl::fromStringParam("wp1", "waypoint"));

  plansys2_msgs::msg::Node function_4;
  function_4.node_type = plansys2_msgs::msg::Node::FUNCTION;
  function_4.name = "distance";
  function_4.parameters.push_back(parser::pddl::fromStringParam("r2d2", "robot"));
  function_4.parameters.push_back(parser::pddl::fromStringParam("wp1", "waypoint"));

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("r2d2", "robot")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("wp1", "waypoint")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("wp2", "waypoint")));

  std::unordered_set<plansys2::Function> functions = problem_expert.getFunctions();
  ASSERT_TRUE(functions.empty());

  ASSERT_TRUE(problem_expert.addFunction(function_1));
  functions = problem_expert.getFunctions();
  ASSERT_FALSE(functions.empty());
  ASSERT_TRUE(problem_expert.addFunction(function_1));
  ASSERT_TRUE(problem_expert.addFunction(function_2));
  ASSERT_FALSE(problem_expert.addFunction(function_3));
  ASSERT_FALSE(problem_expert.addFunction(function_4));

  functions = problem_expert.getFunctions();
  ASSERT_EQ(functions.size(), 2);

  auto func_2 = problem_expert.getFunction("(distance wp1 wp2)");
  ASSERT_TRUE(func_2);
  ASSERT_EQ(func_2.value().name, "distance");
  ASSERT_EQ(func_2.value().parameters.size(), 2);
  ASSERT_EQ(func_2.value().parameters[0].name, "wp1");
  ASSERT_EQ(func_2.value().parameters[0].type, "waypoint");
  ASSERT_EQ(func_2.value().parameters[1].name, "wp2");
  ASSERT_EQ(func_2.value().parameters[1].type, "waypoint");
  ASSERT_EQ(func_2.value().value, 15);

  ASSERT_FALSE(problem_expert.removeFunction(function_3));
  ASSERT_TRUE(problem_expert.removeFunction(function_2));

  functions = problem_expert.getFunctions();
  ASSERT_EQ(functions.size(), 1);
}

TEST(problem_expert, addget_goals)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str(
    (std::istreambuf_iterator<char>(domain_ifs)), std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("paco", "person")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("r2d2", "robot")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("bedroom", "room")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("kitchen", "room")));

  plansys2_msgs::msg::Tree goal;
  parser::pddl::fromString(goal, "(and (robot_at r2d2 bedroom)(person_at paco kitchen))");
  ASSERT_EQ(parser::pddl::toString(goal), "(and (robot_at r2d2 bedroom)(person_at paco kitchen))");

  plansys2_msgs::msg::Tree goal2;
  parser::pddl::fromString(goal2, "(and (robot_at r2d2 bedroom)(not(person_at paco kitchen)))");
  ASSERT_EQ(
    parser::pddl::toString(goal2), "(and (robot_at r2d2 bedroom)(not (person_at paco kitchen)))");

  ASSERT_TRUE(problem_expert.setGoal(goal));
  ASSERT_TRUE(problem_expert.setGoal(goal2));

  ASSERT_EQ(
    parser::pddl::toString(problem_expert.getGoal()),
    "(and (robot_at r2d2 bedroom)(not (person_at paco kitchen)))");

  const plansys2_msgs::msg::Tree & goal3 = problem_expert.getGoal();
  ASSERT_EQ(
    parser::pddl::toString(goal3), "(and (robot_at r2d2 bedroom)(not (person_at paco kitchen)))");

  ASSERT_TRUE(problem_expert.clearGoal());
  ASSERT_TRUE(problem_expert.clearGoal());

  ASSERT_EQ(parser::pddl::toString(problem_expert.getGoal()), "");

  plansys2_msgs::msg::Tree goal4;
  parser::pddl::fromString(
    goal4,
    "(and (or (robot_at r2d2 bedroom) (robot_at r2d2 kitchen)) (not (person_at paco kitchen)))");
  ASSERT_TRUE(problem_expert.setGoal(goal4));
}

TEST(problem_expert, empty_goals)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str(
    (std::istreambuf_iterator<char>(domain_ifs)), std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  plansys2_msgs::msg::Tree goal;
  ASSERT_FALSE(problem_expert.isValidGoal(goal));
}

TEST(problem_expert, get_problem)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str(
    (std::istreambuf_iterator<char>(domain_ifs)), std::istreambuf_iterator<char>());

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

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("paco", "person")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("r2d2", "robot")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("bedroom", "room")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("kitchen", "room")));

  ASSERT_TRUE(problem_expert.addPredicate(predicate_1));
  ASSERT_TRUE(problem_expert.addPredicate(predicate_2));
  ASSERT_TRUE(problem_expert.addPredicate(predicate_3));
  ASSERT_TRUE(problem_expert.addPredicate(predicate_4));

  plansys2_msgs::msg::Tree goal;
  parser::pddl::fromString(goal, "(and (robot_at r2d2 bedroom)(person_at paco kitchen))");
  ASSERT_TRUE(problem_expert.setGoal(goal));

  ASSERT_EQ(
    problem_expert.getProblem(),
    std::string("( define ( problem problem_1 )\n( :domain simple )\n") +
    std::string("( :objects\n\tpaco - person\n\tr2d2 - robot\n") +
    std::string("\tkitchen bedroom - room\n)\n") +
    std::string("( :init\n\t( person_at paco kitchen )\n") +
    std::string("\t( robot_at r2d2 kitchen )\n") + std::string("\t( person_at paco bedroom )\n") +
    std::string("\t( robot_at r2d2 bedroom )\n)\n") +
    std::string("( :goal\n\t( and\n\t\t( robot_at r2d2 bedroom )\n\t\t") +
    std::string("( person_at paco kitchen )\n\t))\n)\n"));

  ASSERT_TRUE(problem_expert.clearKnowledge());
  ASSERT_EQ(problem_expert.getPredicates().size(), 0);
  ASSERT_EQ(problem_expert.getFunctions().size(), 0);
  ASSERT_EQ(problem_expert.getInstances().size(), 0);
}

TEST(problem_expert, add_problem)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str(
    (std::istreambuf_iterator<char>(domain_ifs)), std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_FALSE(problem_expert.addProblem(""));

  // Empty domain name
  std::ifstream problem_empty_domain_ifs(pkgpath + "/pddl/problem_empty_domain.pddl");
  std::string problem_empty_domain_str(
    (std::istreambuf_iterator<char>(problem_empty_domain_ifs)), std::istreambuf_iterator<char>());
  ASSERT_FALSE(problem_expert.addProblem(problem_empty_domain_str));

  // Domain doesn't exist
  std::ifstream problem_charging_ifs(pkgpath + "/pddl/problem_charging.pddl");
  std::string problem_charging_str(
    (std::istreambuf_iterator<char>(problem_charging_ifs)), std::istreambuf_iterator<char>());
  ASSERT_FALSE(problem_expert.addProblem(problem_charging_str));

  // missing syntax causes std::runtime_error
  std::ifstream problem_unexpected_syntax_ifs(pkgpath + "/pddl/problem_unexpected_syntax.pddl");
  std::string problem_unexpected_syntax_str(
    (std::istreambuf_iterator<char>(problem_unexpected_syntax_ifs)),
    std::istreambuf_iterator<char>());
  ASSERT_FALSE(problem_expert.addProblem(problem_unexpected_syntax_str));

  std::ifstream problem_ifs(pkgpath + "/pddl/problem_simple_1.pddl");
  std::string problem_str(
    (std::istreambuf_iterator<char>(problem_ifs)), std::istreambuf_iterator<char>());
  ASSERT_TRUE(problem_expert.addProblem(problem_str));

  ASSERT_TRUE(problem_expert.isValidType("robot"));
  ASSERT_TRUE(problem_expert.isValidType("person"));
  ASSERT_TRUE(problem_expert.isValidType("room"));
  ASSERT_TRUE(problem_expert.isValidType("room_with_teleporter"));
  ASSERT_TRUE(problem_expert.isValidType("message"));
  ASSERT_TRUE(problem_expert.isValidType("ROBOT"));
  ASSERT_TRUE(problem_expert.isValidType("Person"));
  ASSERT_TRUE(problem_expert.isValidType("ROOM"));
  ASSERT_TRUE(problem_expert.isValidType("ROOM_with_TELEPORTER"));
  ASSERT_TRUE(problem_expert.isValidType("Message"));

  ASSERT_EQ(problem_expert.getInstances().size(), 6);
  ASSERT_EQ(problem_expert.getPredicates().size(), 2);
  ASSERT_EQ(problem_expert.getFunctions().size(), 1);

  ASSERT_TRUE(problem_expert.existInstance("leia"));
  ASSERT_TRUE(problem_expert.existInstance("jack"));
  ASSERT_TRUE(problem_expert.existInstance("kitchen"));
  ASSERT_TRUE(problem_expert.existInstance("bedroom"));
  ASSERT_TRUE(problem_expert.existInstance("m1"));

  ASSERT_FALSE(problem_expert.existInstance("r2d2"));
  ASSERT_FALSE(problem_expert.existInstance("hallway"));
  ASSERT_FALSE(problem_expert.existInstance("m2"));

  ASSERT_TRUE(
    problem_expert.existPredicate(parser::pddl::fromStringPredicate("(robot_at leia kitchen)")));
  ASSERT_TRUE(
    problem_expert.existPredicate(parser::pddl::fromStringPredicate("(person_at jack bedroom)")));

  ASSERT_EQ(parser::pddl::toString(problem_expert.getGoal()), "(and (robot_talk leia m1 jack))");

  ASSERT_EQ(
    problem_expert.getProblem(),
    std::string("( define ( problem problem_1 )\n") +
    std::string("( :domain simple )\n( :objects\n") + std::string("\talice jack - person\n") +
    std::string("\tm1 - message\n") + std::string("\tleia - robot\n") +
    std::string("\tbedroom kitchen - room\n)\n") +
    std::string("( :init\n\t( person_at jack bedroom )\n") +
    std::string("\t( robot_at leia kitchen )\n") +
    std::string("\t( = ( room_distance kitchen bedroom ) 10.0000000000 )\n)\n") +
    std::string("( :goal\n\t( and\n\t\t( robot_talk leia m1 jack )\n\t))\n)\n"));

  ASSERT_TRUE(problem_expert.clearKnowledge());
  ASSERT_EQ(problem_expert.getPredicates().size(), 0);
  ASSERT_EQ(problem_expert.getFunctions().size(), 0);
  ASSERT_EQ(problem_expert.getInstances().size(), 0);
}

TEST(problem_expert, add_problem_with_constants)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple_constants.pddl");
  std::string domain_str(
    (std::istreambuf_iterator<char>(domain_ifs)), std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  std::ifstream problem_1_ifs(pkgpath + "/pddl/problem_simple_constants_1.pddl");
  std::string problem_1_str(
    (std::istreambuf_iterator<char>(problem_1_ifs)), std::istreambuf_iterator<char>());
  ASSERT_TRUE(problem_expert.addProblem(problem_1_str));

  ASSERT_TRUE(problem_expert.isValidType("robot"));
  ASSERT_TRUE(problem_expert.isValidType("person"));
  ASSERT_TRUE(problem_expert.isValidType("room"));
  ASSERT_TRUE(problem_expert.isValidType("teleporter_room"));
  ASSERT_TRUE(problem_expert.isValidType("message"));

  ASSERT_EQ(problem_expert.getInstances().size(), 7);
  ASSERT_EQ(problem_expert.getPredicates().size(), 2);
  ASSERT_EQ(problem_expert.getFunctions().size(), 0);

  ASSERT_TRUE(problem_expert.existInstance("leia"));
  ASSERT_TRUE(problem_expert.existInstance("lema"));
  ASSERT_TRUE(problem_expert.existInstance("jack"));
  ASSERT_TRUE(problem_expert.existInstance("john"));
  ASSERT_TRUE(problem_expert.existInstance("kitchen"));
  ASSERT_TRUE(problem_expert.existInstance("bedroom"));
  ASSERT_TRUE(problem_expert.existInstance("m1"));

  ASSERT_FALSE(problem_expert.existInstance("r2d2"));
  ASSERT_FALSE(problem_expert.existInstance("hallway"));
  ASSERT_FALSE(problem_expert.existInstance("m2"));

  ASSERT_TRUE(
    problem_expert.existPredicate(parser::pddl::fromStringPredicate("(robot_at leia kitchen)")));
  ASSERT_TRUE(
    problem_expert.existPredicate(parser::pddl::fromStringPredicate("(person_at jack bedroom)")));

  ASSERT_EQ(parser::pddl::toString(problem_expert.getGoal()), "(and (robot_talk leia m1 jack))");

  ASSERT_EQ(
    problem_expert.getProblem(),
    std::string("( define ( problem problem_1 )\n( :domain plansys2 )\n") +
    std::string("( :objects\n\tm1 - message\n\tbedroom kitchen - room\n)\n") +
    std::string("( :init\n\t( person_at jack bedroom )\n") +
    std::string("\t( robot_at leia kitchen )\n)\n") +
    std::string("( :goal\n\t( and\n\t\t( robot_talk leia m1 jack )\n\t))\n)\n"));

  ASSERT_TRUE(problem_expert.clearKnowledge());
  ASSERT_EQ(problem_expert.getPredicates().size(), 0);
  ASSERT_EQ(problem_expert.getFunctions().size(), 0);
  ASSERT_EQ(problem_expert.getInstances().size(), 0);
  ASSERT_EQ(
    problem_expert.getProblem(),
    std::string("( define ( problem problem_1 )\n( :domain plansys2 )\n") +
    std::string("( :objects\n)\n( :init\n)\n( :goal\n\t( and\n\t))\n)\n"));

  std::ifstream problem_2_ifs(pkgpath + "/pddl/problem_simple_constants_2.pddl");
  std::string problem_2_str(
    (std::istreambuf_iterator<char>(problem_2_ifs)), std::istreambuf_iterator<char>());
  ASSERT_TRUE(problem_expert.addProblem(problem_2_str));

  ASSERT_NE(problem_1_str, problem_2_str);
  ASSERT_NE(problem_expert.getProblem(), problem_2_str);
}

TEST(problem_expert, is_goal_satisfied)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_problem_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str(
    (std::istreambuf_iterator<char>(domain_ifs)), std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);
  plansys2::ProblemExpert problem_expert(domain_expert);

  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("leia", "robot")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("Jack", "person")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("kitchen", "room")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("bedroom", "room")));
  ASSERT_TRUE(problem_expert.addInstance(parser::pddl::fromStringParam("m1", "message")));

  ASSERT_TRUE(
    problem_expert.addPredicate(parser::pddl::fromStringPredicate("(robot_at leia kitchen)")));
  ASSERT_TRUE(
    problem_expert.addPredicate(parser::pddl::fromStringPredicate("(person_at Jack bedroom)")));

  std::string expression = "(and (robot_talk leia m1 Jack))";
  plansys2_msgs::msg::Tree goal;
  parser::pddl::fromString(goal, expression);

  ASSERT_EQ(parser::pddl::toString(goal), "(and (robot_talk leia m1 Jack))");
  ASSERT_TRUE(problem_expert.setGoal(goal));
  ASSERT_FALSE(problem_expert.isGoalSatisfied(goal));

  ASSERT_TRUE(
    problem_expert.addPredicate(parser::pddl::fromStringPredicate("(robot_talk leia m1 Jack)")));

  ASSERT_TRUE(problem_expert.isGoalSatisfied(goal));
}

TEST(problem_expert, exist_predicate)
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

  problem_expert.updateInferredPredicates();
  ASSERT_TRUE(
    problem_expert.existPredicate(parser::pddl::fromStringPredicate("(robot_at leia kitchen)")));
  ASSERT_TRUE(
    problem_expert.existInferredPredicate(
      parser::pddl::fromStringPredicate("(inferred-robot_at leia kitchen)")));
  ASSERT_TRUE(
    problem_expert.existPredicate(parser::pddl::fromStringPredicate("(person_at jack bedroom)")));
  ASSERT_TRUE(
    problem_expert.existInferredPredicate(
      parser::pddl::fromStringPredicate("(inferred-person_at jack bedroom)")));
  ASSERT_FALSE(
    problem_expert.existInferredPredicate(
      parser::pddl::fromStringPredicate("(inferred-person_at jack kitchen)")));
  ASSERT_FALSE(
    problem_expert.existInferredPredicate(
      parser::pddl::fromStringPredicate("(inferred-robot_at leia bedroom)")));

  problem_expert.removePredicate(parser::pddl::fromStringPredicate("(robot_at leia kitchen)"));
  problem_expert.removePredicate(parser::pddl::fromStringPredicate("(person_at jack bedroom)"));
  problem_expert.updateInferredPredicates();

  ASSERT_FALSE(
    problem_expert.existInferredPredicate(
      parser::pddl::fromStringPredicate("(inferred-person_at jack bedroom)")));
  ASSERT_FALSE(
    problem_expert.existInferredPredicate(
      parser::pddl::fromStringPredicate("(inferred-robot_at leia kitchen)")));
  ASSERT_FALSE(
    problem_expert.existPredicate(parser::pddl::fromStringPredicate("(person_at jack bedroom)")));
  ASSERT_FALSE(
    problem_expert.existPredicate(parser::pddl::fromStringPredicate("(robot_at leia kitchen)")));
}

TEST(problem_expert, get_predicate_with_derived)
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

  auto predicates = problem_expert.getInferredPredicates();

  ASSERT_TRUE(
    predicates.find(parser::pddl::fromStringPredicate("(inferred-robot_at leia kitchen)")) !=
    predicates.end());

  ASSERT_FALSE(
    predicates.find(parser::pddl::fromStringPredicate("(inferred-exists-another-drone-at-drone_area drone123)")) !=
    predicates.end());

  ASSERT_TRUE(
    predicates.find(parser::pddl::fromStringPredicate("(person_at jack bedroom)")) !=
    predicates.end());

  ASSERT_TRUE(
    predicates.find(parser::pddl::fromStringPredicate("(inferred-person_at jack bedroom)")) !=
    predicates.end());

  ASSERT_TRUE(
    predicates.find(
      parser::pddl::fromStringPredicate(
        "(inferred-party jose jose jose jose jose turtlebot turtlebot livingroom)")) !=
    predicates.end());

  ASSERT_TRUE(
    predicates.find(
      parser::pddl::fromStringPredicate(
        "(inferred-exists-party-in-room livingroom)")) != predicates.end());

  ASSERT_TRUE(
    predicates.find(parser::pddl::fromStringPredicate("(inferred-aerial rob1)")) !=
    predicates.end());

  ASSERT_TRUE(
    predicates.find(parser::pddl::fromStringPredicate("(inferred-aerial rob2)")) !=
    predicates.end());

  ASSERT_TRUE(
    predicates.find(parser::pddl::fromStringPredicate("(inferred-not_aerial leia)")) !=
    predicates.end());

  ASSERT_FALSE(
    predicates.find(parser::pddl::fromStringPredicate("(inferred-person_at jack kitchen)")) !=
    predicates.end());

  ASSERT_FALSE(
    predicates.find(parser::pddl::fromStringPredicate("(inferred-robot_at leia bedroom)")) !=
    predicates.end());

  // ASSERT_TRUE(predicates.find(
  //   parser::pddl::fromStringPredicate(
  // "(inferred-drone_has_battery_level rob1)")) != predicates.end());
  //
  // ASSERT_FALSE(predicates.find(
  //   parser::pddl::fromStringPredicate(
  // "(inferred-drone_has_battery_level r2d2)")) != predicates.end());

  ASSERT_TRUE(
    predicates.find(parser::pddl::fromStringPredicate("(inferred-drone123 drone123)")) !=
    predicates.end());

  ASSERT_TRUE(
    predicates.find(parser::pddl::fromStringPredicate("(inferred-not-drone123 rob1)")) !=
    predicates.end());

  ASSERT_TRUE(
    predicates.find(parser::pddl::fromStringPredicate("(inferred-same-drone rob1 rob1)")) !=
    predicates.end());

  ASSERT_TRUE(
    predicates.find(parser::pddl::fromStringPredicate("(inferred-not-same-drone rob1 drone123)")) !=
    predicates.end());

  ASSERT_TRUE(
    predicates.find(parser::pddl::fromStringPredicate("(inferred-exists-equal-drone rob1)")) !=
    predicates.end());

  ASSERT_TRUE(
    predicates.find(parser::pddl::fromStringPredicate("(inferred-exists-another-drone rob1)")) !=
    predicates.end());

  ASSERT_TRUE(
    problem_expert.removePredicate(parser::pddl::fromStringPredicate("(robot_at leia kitchen)")));
  ASSERT_TRUE(
    problem_expert.removePredicate(parser::pddl::fromStringPredicate("(person_at jack bedroom)")));

  predicates = problem_expert.getInferredPredicates();

  ASSERT_FALSE(
    predicates.find(parser::pddl::fromStringPredicate("(inferred-person_at jack bedroom)")) !=
    predicates.end());
  ASSERT_FALSE(
    predicates.find(parser::pddl::fromStringPredicate("(inferred-robot_at leia kitchen)")) !=
    predicates.end());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
