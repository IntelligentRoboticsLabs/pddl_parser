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

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "gtest/gtest.h"
#include "plansys2_domain_expert/DomainExpert.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

#include "plansys2_msgs/msg/node.hpp"

TEST(domain_expert, get_reduced_string)
{
  std::string my_string("(and)");
  ASSERT_EQ(parser::pddl::getReducedString(my_string), "(and)");

  my_string = "( and)";
  ASSERT_EQ(parser::pddl::getReducedString(my_string), "(and)");

  my_string = "( \tand)";
  ASSERT_EQ(parser::pddl::getReducedString(my_string), "(and)");

  my_string = "( \tand\t)";
  ASSERT_EQ(parser::pddl::getReducedString(my_string), "(and)");

  my_string = "( and\n)";
  ASSERT_EQ(parser::pddl::getReducedString(my_string), "(and)");
  my_string = "( and\n\t)";
  ASSERT_EQ(parser::pddl::getReducedString(my_string), "(and)");
  my_string = "( ( and\n\t ) )";
  ASSERT_EQ(parser::pddl::getReducedString(my_string), "((and))");
}

TEST(domain_expert, exist_domain)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_charging.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);
  ASSERT_TRUE(domain_expert.existDomain("charging"));

  std::ifstream domain_simple_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_simple_str((
      std::istreambuf_iterator<char>(domain_simple_ifs)),
    std::istreambuf_iterator<char>());

  domain_expert.extendDomain(domain_simple_str);
  ASSERT_TRUE(domain_expert.existDomain("plansys2"));
}

TEST(domain_expert, get_domain)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  std::ifstream domain_ifs_p(pkgpath + "/pddl/domain_simple_processed.pddl");
  std::string domain_str_p((
      std::istreambuf_iterator<char>(domain_ifs_p)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);
  ASSERT_EQ(domain_expert.getDomain(), domain_str_p);
}

TEST(domain_expert, get_domain2)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/factory.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  std::ifstream domain_ifs_p(pkgpath + "/pddl/factory_processed.pddl");
  std::string domain_str_p((
      std::istreambuf_iterator<char>(domain_ifs_p)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);
  ASSERT_EQ(domain_expert.getDomain(), domain_str_p);
}

TEST(domain_expert, get_name)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/factory.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  std::string name = domain_expert.getName();
  std::string test_name("factory");

  ASSERT_EQ(name, test_name);
}

TEST(domain_expert, get_domain3)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple_constants.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  std::ifstream domain_ifs_p(pkgpath + "/pddl/domain_simple_constants_processed.pddl");
  std::string domain_str_p((
      std::istreambuf_iterator<char>(domain_ifs_p)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);
  ASSERT_EQ(domain_expert.getDomain(), domain_str_p);
}

TEST(domain_expert, get_types)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  std::vector<std::string> types = domain_expert.getTypes();
  std::vector<std::string> test_types {"object", "person", "message", "robot",
    "room", "teleporter_room"};

  ASSERT_EQ(types, test_types);
}

TEST(domain_expert, get_constants)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple_constants.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  std::vector<std::string> consts1 = domain_expert.getConstants("robot");
  std::vector<std::string> consts2 = domain_expert.getConstants("person");
  std::vector<std::string> test_consts1 {"leia", "lema"};
  std::vector<std::string> test_consts2 {"jack", "john"};

  ASSERT_EQ(consts1, test_consts1);
  ASSERT_EQ(consts2, test_consts2);
}

TEST(domain_expert, get_predicates)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  std::vector<plansys2::Predicate> predicates = domain_expert.getPredicates();
  std::vector<std::string> predicates_types {"person_at", "robot_at", "robot_near_person",
    "robot_talk"};

  ASSERT_EQ(predicates.size(), predicates_types.size());
  for (unsigned i = 0; i < predicates.size(); i++) {
    ASSERT_EQ(predicates[i].name, predicates_types[i]);
  }
}

TEST(domain_expert, get_predicate_params)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  auto params_1 = domain_expert.getPredicate("robot_talk");
  ASSERT_TRUE(params_1);
  ASSERT_EQ(params_1.value().name, "robot_talk");
  ASSERT_EQ(params_1.value().parameters.size(), 3);
  ASSERT_EQ(params_1.value().parameters[0].name, "?robot0");
  ASSERT_EQ(params_1.value().parameters[0].type, "robot");
  ASSERT_EQ(params_1.value().parameters[1].name, "?message1");
  ASSERT_EQ(params_1.value().parameters[1].type, "message");
  ASSERT_EQ(params_1.value().parameters[2].name, "?person2");
  ASSERT_EQ(params_1.value().parameters[2].type, "person");

  auto params_2 = domain_expert.getPredicate("ROBOT_TALK");
  ASSERT_TRUE(params_2);

  auto params_3 = domain_expert.getPredicate("person_at");
  ASSERT_TRUE(params_3);
  ASSERT_EQ(params_3.value().parameters.size(), 2);
  ASSERT_EQ(params_3.value().parameters[0].name, "?person0");
  ASSERT_EQ(params_3.value().parameters[0].type, "person");
  ASSERT_EQ(params_3.value().parameters[1].name, "?room1");
  ASSERT_EQ(params_3.value().parameters[1].type, "room");
}

TEST(domain_expert, get_functions)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_charging.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  std::vector<plansys2::Function> functions = domain_expert.getFunctions();
  std::vector<std::string> functions_types {"speed", "max_range", "state_of_charge",
    "distance"};

  ASSERT_EQ(functions.size(), functions_types.size());
  for (unsigned i = 0; i < functions.size(); i++) {
    ASSERT_EQ(functions[i].name, functions_types[i]);
  }
}

TEST(domain_expert, get_function_params)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_charging.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  auto params_1 = domain_expert.getFunction("speed");
  ASSERT_TRUE(params_1);
  ASSERT_EQ(params_1.value().name, "speed");
  ASSERT_EQ(params_1.value().parameters.size(), 1);
  ASSERT_EQ(params_1.value().parameters[0].name, "?robot0");
  ASSERT_EQ(params_1.value().parameters[0].type, "robot");

  auto params_2 = domain_expert.getFunction("SPEED");
  ASSERT_TRUE(params_2);

  auto params_3 = domain_expert.getFunction("distance");
  ASSERT_TRUE(params_3);
  ASSERT_EQ(params_3.value().parameters.size(), 2);
  ASSERT_EQ(params_3.value().parameters[0].name, "?waypoint0");
  ASSERT_EQ(params_3.value().parameters[0].type, "waypoint");
  ASSERT_EQ(params_3.value().parameters[1].name, "?waypoint1");
  ASSERT_EQ(params_3.value().parameters[1].type, "waypoint");
}

TEST(domain_expert, get_derived_predicates)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple_derived.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  std::vector<plansys2_msgs::msg::Derived> predicates = domain_expert.getDerivedPredicates();
  std::vector<std::string> predicates_types {"inferred-robot_at", "inferred-person_at"};

  ASSERT_EQ(predicates.size(), predicates_types.size());
  for (unsigned i = 0; i < predicates.size(); i++) {
    ASSERT_EQ(predicates[i].predicate.name, predicates_types[i]);
  }
}

TEST(domain_expert, get_derived_predicate_params)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple_derived.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  auto pred_1 = domain_expert.getDerivedPredicate("inferred-robot_at");
  ASSERT_GT(pred_1.size(), 0);
  ASSERT_EQ(pred_1[0].predicate.name, "inferred-robot_at");
  ASSERT_EQ(pred_1[0].predicate.parameters.size(), 2);
  ASSERT_EQ(pred_1[0].predicate.parameters[0].name, "?robot0");
  ASSERT_EQ(pred_1[0].predicate.parameters[0].type, "robot");
  ASSERT_EQ(pred_1[0].predicate.parameters[1].name, "?room1");
  ASSERT_EQ(pred_1[0].predicate.parameters[1].type, "room");
  ASSERT_EQ(
    parser::pddl::toString(pred_1[0].preconditions), "(and (robot_at ?0 ?1))");
}

TEST(domain_expert, get_actions)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  auto actions = domain_expert.getActions();
  ASSERT_EQ(actions.size(), 1);
  ASSERT_EQ(actions[0], "move_person");

  auto durative_actions = domain_expert.getDurativeActions();
  ASSERT_EQ(durative_actions.size(), 3);
  ASSERT_EQ(durative_actions[0], "move");
  ASSERT_EQ(durative_actions[1], "talk");
  ASSERT_EQ(durative_actions[2], "approach");
}


TEST(domain_expert, get_action_params)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  domain_expert.getAction("move");
  auto no_action = domain_expert.getAction("noexist");
  ASSERT_FALSE(no_action);

  auto move_action = domain_expert.getDurativeAction("move");
  auto at_start = move_action->at_start_requirements;
  ASSERT_EQ(parser::pddl::toString(at_start), "(and (robot_at ?0 ?1))");

  ASSERT_TRUE(move_action);
  ASSERT_EQ(move_action->name, "move");
  ASSERT_EQ(move_action->parameters.size(), 3);
  ASSERT_EQ(move_action->parameters[0].name, "?0");
  ASSERT_EQ(move_action->parameters[0].type, "robot");
  ASSERT_EQ(move_action->parameters[1].name, "?1");
  ASSERT_EQ(move_action->parameters[1].type, "room");
  ASSERT_EQ(move_action->parameters[2].name, "?2");
  ASSERT_EQ(move_action->parameters[2].type, "room");

  ASSERT_FALSE(parser::pddl::empty(at_start));
  ASSERT_TRUE(parser::pddl::empty(move_action->over_all_requirements));
  ASSERT_TRUE(parser::pddl::empty(move_action->at_end_requirements));

  ASSERT_EQ(
    parser::pddl::toString(move_action->at_start_requirements),
    "(and (robot_at ?0 ?1))");
  ASSERT_EQ(
    parser::pddl::toString(move_action->at_start_effects),
    "(and (not (robot_at ?0 ?1)))");
  ASSERT_EQ(
    parser::pddl::toString(move_action->at_end_effects),
    "(and (robot_at ?0 ?2))");
}

TEST(domain_expert, multidomain_get_types)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<plansys2::DomainExpert>(domain_str);

  std::ifstream domain_ext_ifs(pkgpath + "/pddl/domain_simple_ext.pddl");
  std::string domain_ext_str((
      std::istreambuf_iterator<char>(domain_ext_ifs)),
    std::istreambuf_iterator<char>());

  domain_expert->extendDomain(domain_ext_str);

  std::vector<std::string> types = domain_expert->getTypes();
  std::vector<std::string> test_types {"object", "person", "message", "robot", "room",
    "teleporter_room", "pickable_object"};

  ASSERT_EQ(types, test_types);

  std::vector<plansys2::Predicate> predicates = domain_expert->getPredicates();
  std::vector<std::string> test_predicates {"object_at_robot", "object_at_room", "person_at",
    "robot_at", "robot_near_person", "robot_talk"};

  ASSERT_EQ(predicates.size(), test_predicates.size());
  for (unsigned i = 0; i < predicates.size(); i++) {
    ASSERT_EQ(predicates[i].name, test_predicates[i]);
  }

  std::vector<std::string> actions = domain_expert->getActions();
  std::vector<std::string> test_actions {"move_person"};

  ASSERT_EQ(actions, test_actions);

  std::vector<std::string> dactions = domain_expert->getDurativeActions();
  std::vector<std::string> test_dactions {"move", "talk", "approach", "pick_object",
    "place_object"};

  ASSERT_EQ(dactions, test_dactions);
}

TEST(domain_expert, sub_types)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_domain_expert");
  std::ifstream domain_ifs(pkgpath + "/pddl/domain_simple.pddl");
  std::string domain_str((
      std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

  plansys2::DomainExpert domain_expert(domain_str);

  // Parameter subtypes with a durative-action
  plansys2_msgs::msg::DurativeAction::SharedPtr durative_action =
    domain_expert.getDurativeAction("move");
  if (durative_action) {
    if (durative_action->parameters.size() == 3) {
      ASSERT_EQ(durative_action->parameters[1].type, "room");

      std::vector<std::string> subtypes {"teleporter_room"};
      ASSERT_EQ(durative_action->parameters[1].sub_types, subtypes);
    } else {
      FAIL() << "The `move` durative-action is expected to have 2 parameters";
    }
  } else {
    FAIL() << "No `move` durative-action found in the domain";
  }

  // Parameter subtypes with a predicate
  std::optional<plansys2::Predicate> predicate = domain_expert.getPredicate("robot_at");
  if (predicate.has_value()) {
    if (predicate.value().parameters.size() == 2) {
      ASSERT_EQ(predicate.value().parameters[1].type, "room");

      std::vector<std::string> subtypes {"teleporter_room"};
      ASSERT_EQ(predicate.value().parameters[1].sub_types, subtypes);
    } else {
      FAIL() << "The `robot_at` predicate is expected to have 3 parameters";
    }
  } else {
    FAIL() << "No `robot_at` predicate found in the domain";
  }

  // Parameter subtypes with as action
  plansys2_msgs::msg::Action::SharedPtr action =
    domain_expert.getAction("move_person");
  if (action) {
    if (action->parameters.size() == 3) {
      ASSERT_EQ(action->parameters[1].type, "room");

      std::vector<std::string> subtypes {"teleporter_room"};
      ASSERT_EQ(action->parameters[1].sub_types, subtypes);
    } else {
      FAIL() << "The `move_person` action is expected to have 3 parameters";
    }
  } else {
    FAIL() << "No `move_person` action found in the domain";
  }

  // Parameter subtypes with as function
  std::optional<plansys2::Function> function =
    domain_expert.getFunction("teleportation_time");
  if (function.has_value()) {
    if (function.value().parameters.size() == 2) {
      ASSERT_EQ(function.value().parameters[0].type, "teleporter_room");
      ASSERT_EQ(function.value().parameters[1].type, "room");

      std::vector<std::string> emptySubtypes {};
      ASSERT_EQ(function.value().parameters[0].sub_types, emptySubtypes);

      std::vector<std::string> subtypes {"teleporter_room"};
      ASSERT_EQ(function.value().parameters[1].sub_types, subtypes);
    } else {
      FAIL() << "The `teleportation_time` function is expected to have 2 parameters";
    }
  } else {
    FAIL() << "No `teleportation_time` function found in the domain";
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
