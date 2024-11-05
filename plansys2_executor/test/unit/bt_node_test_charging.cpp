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
#include <fstream>
#include <map>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/BTBuilder.hpp"

#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_problem_expert/Utils.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/blackboard.h"

#include "plansys2_executor/behavior_tree/execute_action_node.hpp"
#include "plansys2_executor/behavior_tree/wait_atstart_req_node.hpp"
#include "plansys2_executor/behavior_tree/apply_atstart_effect_node.hpp"
#include "plansys2_executor/behavior_tree/apply_atend_effect_node.hpp"

#include "lifecycle_msgs/msg/state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "gtest/gtest.h"

TEST(problem_expert, wait_atstart_req_test)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto test_lc_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_lc_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_charging.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_charging.pddl"});

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });


  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

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

  auto action_map = std::make_shared<std::map<std::string, plansys2::ActionExecutionInfo>>();
  (*action_map)["(move robot1 wp1 wp2):5"] = plansys2::ActionExecutionInfo();
  (*action_map)["(move robot1 wp1 wp2):5"].action_info =
    domain_client->getDurativeAction(
    plansys2::get_action_name("(move robot1 wp1 wp2)"),
    plansys2::get_action_params("(move robot1 wp1 wp2)"));

  ASSERT_NE(
    (*action_map)["(move robot1 wp1 wp2):5"].action_info.action.index(),
    std::variant_npos);

  std::string bt_xml_tree =
    R"(
    <root BTCPP_format="4" main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
          <WaitAtStartReq action="(move robot1 wp1 wp2):5"/>
       </Sequence>
      </BehaviorTree>
    </root>
  )";

  auto blackboard = BT::Blackboard::create();

  blackboard->set("action_map", action_map);
  blackboard->set("node", test_lc_node);
  blackboard->set("problem_client", problem_client);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<plansys2::ExecuteAction>("ExecuteAction");
  factory.registerNodeType<plansys2::WaitAtStartReq>("WaitAtStartReq");

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("robot1", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wp1", "waypoint")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wp2", "waypoint")));

  std::vector<std::string> predicates = {
    "(robot_at robot1 wp1)",
    "(charger_at wp2)",
    "(connected wp1 wp2)"};

  std::vector<std::string> functions = {
    "(= (speed robot1) 3)",
    "(= (max_range robot1) 75)",
    "(= (state_of_charge robot1) 99)",
    "(= (distance wp1 wp2) 15)",
    "(= (distance wp2 wp1) 15)"};

  try {
    auto tree = factory.createTreeFromText(bt_xml_tree, blackboard);

    auto status = BT::NodeStatus::RUNNING;
    status = tree.tickOnce();
    ASSERT_EQ(status, BT::NodeStatus::RUNNING);
    status = tree.tickOnce();
    ASSERT_EQ(status, BT::NodeStatus::RUNNING);
    status = tree.tickOnce();
    ASSERT_EQ(status, BT::NodeStatus::RUNNING);

    for (const auto & pred : predicates) {
      ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
    }

    for (const auto & func : functions) {
      ASSERT_TRUE(problem_client->addFunction(plansys2::Function(func)));
    }

    status = tree.tickOnce();
    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  } catch (std::exception & e) {
    std::cerr << e.what() << std::endl;
  }

  finish = true;
  t.join();
}

TEST(problem_expert, apply_atstart_effect_test)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto test_lc_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_lc_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_charging.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_charging.pddl"});

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });


  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

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

  auto action_map = std::make_shared<std::map<std::string, plansys2::ActionExecutionInfo>>();
  (*action_map)["(move robot1 wp1 wp2):5"] = plansys2::ActionExecutionInfo();
  (*action_map)["(move robot1 wp1 wp2):5"].action_info =
    domain_client->getDurativeAction(
    plansys2::get_action_name("(move robot1 wp1 wp2)"),
    plansys2::get_action_params("(move robot1 wp1 wp2)"));

  ASSERT_NE(
    (*action_map)["(move robot1 wp1 wp2):5"].action_info.action.index(),
    std::variant_npos);

  std::string bt_xml_tree =
    R"(
    <root BTCPP_format="4" main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
          <ApplyAtStartEffect action="(move robot1 wp1 wp2):5"/>
       </Sequence>
      </BehaviorTree>
    </root>
  )";

  auto blackboard = BT::Blackboard::create();

  blackboard->set("action_map", action_map);
  blackboard->set("node", test_lc_node);
  blackboard->set("problem_client", problem_client);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<plansys2::ExecuteAction>("ExecuteAction");
  factory.registerNodeType<plansys2::ApplyAtStartEffect>("ApplyAtStartEffect");

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("robot1", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wp1", "waypoint")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wp2", "waypoint")));

  try {
    std::vector<std::string> predicates = {
      "(robot_at robot1 wp1)",
      "(charger_at wp2)",
      "(connected wp1 wp2)"};

    for (const auto & pred : predicates) {
      ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
    }

    std::vector<std::string> functions = {
      "(= (speed robot1) 3)",
      "(= (max_range robot1) 75)",
      "(= (state_of_charge robot1) 99)",
      "(= (distance wp1 wp2) 15)",
      "(= (distance wp2 wp1) 15)"};

    for (const auto & func : functions) {
      ASSERT_TRUE(problem_client->addFunction(plansys2::Function(func)));
    }

    auto tree = factory.createTreeFromText(bt_xml_tree, blackboard);

    auto status = BT::NodeStatus::RUNNING;
    status = tree.tickOnce();
    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);

    {
      rclcpp::Rate rate(10);
      auto start = test_node->now();
      while ((test_node->now() - start).seconds() < 0.5) {
        rate.sleep();
      }
    }
    ASSERT_FALSE(problem_client->existPredicate(plansys2::Predicate("(robot_at robot1 wp1)")));
  } catch (std::exception & e) {
    std::cerr << e.what() << std::endl;
  }

  finish = true;
  t.join();
}

TEST(problem_expert, apply_atend_effect_test)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto test_lc_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_lc_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_charging.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_charging.pddl"});

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });


  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

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

  auto action_map = std::make_shared<std::map<std::string, plansys2::ActionExecutionInfo>>();
  (*action_map)["(move robot1 wp1 wp2):5"] = plansys2::ActionExecutionInfo();
  (*action_map)["(move robot1 wp1 wp2):5"].action_info =
    domain_client->getDurativeAction(
    plansys2::get_action_name("(move robot1 wp1 wp2)"),
    plansys2::get_action_params("(move robot1 wp1 wp2)"));

  ASSERT_NE(
    (*action_map)["(move robot1 wp1 wp2):5"].action_info.action.index(),
    std::variant_npos);

  std::string bt_xml_tree =
    R"(
    <root BTCPP_format="4" main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
          <ApplyAtEndEffect action="(move robot1 wp1 wp2):5"/>
       </Sequence>
      </BehaviorTree>
    </root>
  )";

  auto blackboard = BT::Blackboard::create();

  blackboard->set("action_map", action_map);
  blackboard->set("node", test_lc_node);
  blackboard->set("problem_client", problem_client);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<plansys2::ExecuteAction>("ExecuteAction");
  factory.registerNodeType<plansys2::ApplyAtEndEffect>("ApplyAtEndEffect");

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("robot1", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wp1", "waypoint")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wp2", "waypoint")));

  try {
    std::vector<std::string> predicates = {
      "(robot_at robot1 wp1)",
      "(charger_at wp2)",
      "(connected wp1 wp2)"};

    for (const auto & pred : predicates) {
      ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
    }

    std::vector<std::string> functions = {
      "(= (speed robot1) 3)",
      "(= (max_range robot1) 75)",
      "(= (state_of_charge robot1) 99)",
      "(= (distance wp1 wp2) 15)",
      "(= (distance wp2 wp1) 15)"};

    for (const auto & func : functions) {
      ASSERT_TRUE(problem_client->addFunction(plansys2::Function(func)));
    }

    auto tree = factory.createTreeFromText(bt_xml_tree, blackboard);

    auto status = BT::NodeStatus::RUNNING;
    status = tree.tickOnce();
    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);

    {
      rclcpp::Rate rate(10);
      auto start = test_node->now();
      while ((test_node->now() - start).seconds() < 0.5) {
        rate.sleep();
      }
    }

    ASSERT_TRUE(problem_client->existPredicate(plansys2::Predicate("(robot_at robot1 wp2)")));
  } catch (std::exception & e) {
    std::cerr << e.what() << std::endl;
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
