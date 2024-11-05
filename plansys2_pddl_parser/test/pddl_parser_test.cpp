// Copyright 2022 Marco Roveri - University of Trento
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

#include <algorithm>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "gtest/gtest.h"
#include "plansys2_pddl_parser/Instance.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class PDDLParserTestCase : public ::testing::Test
{
protected:
  static void SetUpTestCase() {rclcpp::init(0, nullptr);}
};

TEST(PDDLParserTestCase, pddl_parser)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_pddl_parser");
  std::string domain_file = pkgpath + "/pddl/dom1.pddl";
  std::string instance_file = pkgpath + "/pddl/prob1.pddl";

  std::ifstream domain_ifs(domain_file);
  ASSERT_TRUE(domain_ifs.good());
  std::string domain_str(
    (std::istreambuf_iterator<char>(domain_ifs)), std::istreambuf_iterator<char>());
  ASSERT_NE(domain_str, "");
  std::ifstream instance_ifs(instance_file);
  ASSERT_TRUE(instance_ifs.good());
  std::string instance_str(
    (std::istreambuf_iterator<char>(instance_ifs)), std::istreambuf_iterator<char>());

  ASSERT_NE(instance_str, "");
  // Read domain and instance
  bool okparse = false;
  bool okprint = false;
  try {
    parser::pddl::Domain domain(domain_str);
    parser::pddl::Instance instance(domain, instance_str);
    okparse = true;
    try {
      std::cout << domain << std::endl;
      std::cout << instance << std::endl;
      okprint = true;
    } catch (std::runtime_error e) {
      std::cerr << e.what() << std::endl;
    }
  } catch (std::runtime_error e) {
    std::cerr << e.what() << std::endl;
  }
  ASSERT_TRUE(okparse);
  ASSERT_TRUE(okprint);
}

TEST(PDDLParserTestCase, exists_get_tree)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_pddl_parser");
  std::string domain_file = pkgpath + "/pddl/dom1.pddl";

  std::ifstream domain_ifs(domain_file);
  std::string domain_str(
    (std::istreambuf_iterator<char>(domain_ifs)), std::istreambuf_iterator<char>());
  parser::pddl::Domain domain(domain_str);

  auto action = domain.actions.get("action_test4");
  plansys2_msgs::msg::Tree tree;
  action->pre->getTree(tree, domain);
  std::string str = parser::pddl::toString(tree);

  ASSERT_EQ(
    str,
    "(and (exists (?1) (and (robot_at ?0 ?1)(charging_point_at ?1)))(and (>  (battery_level ?0) "
    "1.000000)(<  (battery_level ?0) 200.000000)))");

  plansys2_msgs::msg::Tree tree2;
  std::vector<std::string> replace = {"rob1"};
  action->pre->getTree(tree2, domain, replace);
  std::string str2 = parser::pddl::toString(tree2);
  ASSERT_EQ(
    str2,
    "(and (exists (?1) (and (robot_at rob1 ?1)(charging_point_at ?1)))(and (>  (battery_level "
    "rob1) 1.000000)(<  (battery_level rob1) 200.000000)))");

  auto action2 = domain.actions.get("action_test5");
  plansys2_msgs::msg::Tree tree3;
  action2->pre->getTree(tree3, domain);
  std::string str3 = parser::pddl::toString(tree3);
  ASSERT_EQ(str3, "(exists (?1 ?2) (and (robot_at ?0 ?1)(connected ?1 ?2)))");
}

TEST(PDDLParserTestCase, check_node_equality)
{
  auto predicate1 = parser::pddl::fromStringPredicate("(predicate a b)");
  auto predicate2 = parser::pddl::fromStringPredicate("(predicate a b)");
  auto predicate3 = parser::pddl::fromStringPredicate("(predicate ?x b)");
  auto predicate4 = parser::pddl::fromStringPredicate("(predicate a ?y)");
  auto predicate5 = parser::pddl::fromStringPredicate("(predicate a c)");
  auto predicate6 = parser::pddl::fromStringPredicate("(predicate ?x ?y)");

  ASSERT_TRUE(parser::pddl::checkNodeEquality(predicate1, predicate2));
  ASSERT_TRUE(parser::pddl::checkNodeEquality(predicate1, predicate3));
  ASSERT_TRUE(parser::pddl::checkNodeEquality(predicate1, predicate4));
  ASSERT_FALSE(parser::pddl::checkNodeEquality(predicate1, predicate5));
  ASSERT_TRUE(parser::pddl::checkNodeEquality(predicate1, predicate6));
  ASSERT_TRUE(parser::pddl::checkNodeEquality(predicate6, predicate1));
}
