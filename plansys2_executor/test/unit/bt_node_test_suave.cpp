// Copyright 2024 Intelligent Robotics Lab
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
#include <map>
#include <memory>
#include <regex>
#include <string>
#include <vector>

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
#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_executor/behavior_tree/apply_atend_effect_node.hpp"
#include "plansys2_executor/behavior_tree/apply_atstart_effect_node.hpp"
#include "plansys2_executor/behavior_tree/check_overall_req_node.hpp"
#include "plansys2_executor/behavior_tree/check_atend_req_node.hpp"
#include "plansys2_executor/behavior_tree/execute_action_node.hpp"
#include "plansys2_executor/behavior_tree/wait_atstart_req_node.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/Utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


TEST(bt_node_test_suave, suave_bt_execution_test)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto test_lc_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_lc_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/suave_domain.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/suave_domain.pddl"});
  problem_node->set_parameter({"problem_file", pkgpath + "/pddl/suave_problem.pddl"});

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 8);

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {
        exe.spin_some();
      }
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
  (*action_map)["(start_robot bluerov):0"] = plansys2::ActionExecutionInfo();
  (*action_map)["(start_robot bluerov):0"].action_info = domain_client->getAction(
    plansys2::get_action_name("(start_robot bluerov)"),
    plansys2::get_action_params("(start_robot bluerov)"));

  (*action_map)["(reconfigure f_maintain_motion fd_unground fd_recover_thrusters):0"] = plansys2::ActionExecutionInfo();
  (*action_map)["(reconfigure f_maintain_motion fd_unground fd_recover_thrusters):0"].action_info = domain_client->getAction(
    plansys2::get_action_name("(reconfigure f_maintain_motion fd_unground fd_recover_thrusters)"),
    plansys2::get_action_params("(reconfigure f_maintain_motion fd_unground fd_recover_thrusters)"));

  (*action_map)["(reconfigure f_generate_search_path fd_unground fd_spiral_high):0"] = plansys2::ActionExecutionInfo();
  (*action_map)["(reconfigure f_generate_search_path fd_unground fd_spiral_high):0"].action_info = domain_client->getAction(
    plansys2::get_action_name("(reconfigure f_generate_search_path fd_unground fd_spiral_high)"),
    plansys2::get_action_params("(reconfigure f_generate_search_path fd_unground fd_spiral_high)"));

  (*action_map)["(search_pipeline a_search_pipeline pipeline bluerov fd_spiral_high fd_recover_thrusters):0"] = plansys2::ActionExecutionInfo();
  (*action_map)["(search_pipeline a_search_pipeline pipeline bluerov fd_spiral_high fd_recover_thrusters):0"].action_info = domain_client->getAction(
    plansys2::get_action_name("(search_pipeline a_search_pipeline pipeline bluerov fd_spiral_high fd_recover_thrusters)"),
    plansys2::get_action_params("(search_pipeline a_search_pipeline pipeline bluerov fd_spiral_high fd_recover_thrusters)"));

  (*action_map)["(reconfigure f_follow_pipeline fd_unground fd_follow_pipeline):0"] = plansys2::ActionExecutionInfo();
  (*action_map)["(reconfigure f_follow_pipeline fd_unground fd_follow_pipeline):0"].action_info = domain_client->getAction(
    plansys2::get_action_name("(reconfigure f_follow_pipeline fd_unground fd_follow_pipeline)"),
    plansys2::get_action_params("(reconfigure f_follow_pipeline fd_unground fd_follow_pipeline)"));

  (*action_map)["(reconfigure f_generate_search_path fd_spiral_high fd_unground):0"] = plansys2::ActionExecutionInfo();
  (*action_map)["(reconfigure f_generate_search_path fd_spiral_high fd_unground):0"].action_info = domain_client->getAction(
    plansys2::get_action_name("(reconfigure f_generate_search_path fd_spiral_high fd_unground)"),
    plansys2::get_action_params("(reconfigure f_generate_search_path fd_spiral_high fd_unground)"));

  (*action_map)["(inspect_pipeline a_inspect_pipeline pipeline bluerov fd_follow_pipeline fd_recover_thrusters):0"] = plansys2::ActionExecutionInfo();
  (*action_map)["(inspect_pipeline a_inspect_pipeline pipeline bluerov fd_follow_pipeline fd_recover_thrusters):0"].action_info = domain_client->getAction(
    plansys2::get_action_name("(inspect_pipeline a_inspect_pipeline pipeline bluerov fd_follow_pipeline fd_recover_thrusters)"),
    plansys2::get_action_params("(inspect_pipeline a_inspect_pipeline pipeline bluerov fd_follow_pipeline fd_recover_thrusters)"));

  ASSERT_FALSE((*action_map)["(start_robot bluerov):0"].action_info.is_empty());
  ASSERT_FALSE((*action_map)["(reconfigure f_maintain_motion fd_unground fd_recover_thrusters):0"].action_info.is_empty());
  ASSERT_FALSE((*action_map)["(reconfigure f_generate_search_path fd_unground fd_spiral_high):0"].action_info.is_empty());
  ASSERT_FALSE((*action_map)["(search_pipeline a_search_pipeline pipeline bluerov fd_spiral_high fd_recover_thrusters):0"].action_info.is_empty());
  ASSERT_FALSE((*action_map)["(reconfigure f_follow_pipeline fd_unground fd_follow_pipeline):0"].action_info.is_empty());
  ASSERT_FALSE((*action_map)["(reconfigure f_generate_search_path fd_spiral_high fd_unground):0"].action_info.is_empty());
  ASSERT_FALSE((*action_map)["(inspect_pipeline a_inspect_pipeline pipeline bluerov fd_follow_pipeline fd_recover_thrusters):0"].action_info.is_empty());

  std::string bt_xml_tree =
    R"(
    <root BTCPP_format="4" main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
          <ApplyAtEndEffect action="(start_robot bluerov):0"/>
          <ApplyAtEndEffect action="(reconfigure f_maintain_motion fd_unground fd_recover_thrusters):0"/>
          <ApplyAtEndEffect action="(reconfigure f_generate_search_path fd_unground fd_spiral_high):0"/>
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
  factory.registerNodeType<plansys2::ApplyAtStartEffect>("ApplyAtStartEffect");
  factory.registerNodeType<plansys2::CheckOverAllReq>("CheckOverAllReq");
  factory.registerNodeType<plansys2::CheckAtEndReq>("CheckAtEndReq");

  try {
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

    auto state = problem_client->getState();
    state.addActionsAndPruneDerived({(*action_map)["(search_pipeline a_search_pipeline pipeline bluerov fd_spiral_high fd_recover_thrusters):0"].action_info});
    plansys2::solveDerivedPredicates(state);

    ASSERT_TRUE(state.hasPredicate(plansys2::Predicate("(robot_started bluerov)")));
    ASSERT_FALSE(state.hasPredicate(plansys2::Predicate("(system_in_mode f_maintain_motion fd_unground)")));
    ASSERT_TRUE(state.hasPredicate(plansys2::Predicate("(system_in_mode f_maintain_motion fd_recover_thrusters)")));
    ASSERT_FALSE(state.hasPredicate(plansys2::Predicate("(system_in_mode f_generate_search_path fd_unground)")));
    ASSERT_TRUE(state.hasPredicate(plansys2::Predicate("(system_in_mode f_generate_search_path fd_spiral_high)")));

    ASSERT_TRUE(state.hasInferredPredicate(plansys2::Predicate("(robot_started bluerov)")));
    ASSERT_FALSE(state.hasInferredPredicate(plansys2::Predicate("(system_in_mode f_maintain_motion fd_unground)")));
    ASSERT_TRUE(state.hasInferredPredicate(plansys2::Predicate("(system_in_mode f_maintain_motion fd_recover_thrusters)")));
    ASSERT_FALSE(state.hasInferredPredicate(plansys2::Predicate("(system_in_mode f_generate_search_path fd_unground)")));
    ASSERT_TRUE(state.hasInferredPredicate(plansys2::Predicate("(system_in_mode f_generate_search_path fd_spiral_high)")));

    ASSERT_FALSE(state.hasInferredPredicate(plansys2::Predicate("(inferred-Fd_realisability fd_recover_thrusters false_boolean)")));
    ASSERT_FALSE(state.hasInferredPredicate(plansys2::Predicate("(inferred-Fd_realisability fd_spiral_high false_boolean)")));
    ASSERT_FALSE(state.hasInferredPredicate(plansys2::Predicate("(inferred-f_activated f_follow_pipeline)")));

  } catch (std::exception & e) {
    std::cerr << e.what() << std::endl;
  }

  bt_xml_tree =
    R"(
    <root BTCPP_format="4" main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
          <ApplyAtStartEffect action="(search_pipeline a_search_pipeline pipeline bluerov fd_spiral_high fd_recover_thrusters):0"/>
          <CheckOverAllReq action="(search_pipeline a_search_pipeline pipeline bluerov fd_spiral_high fd_recover_thrusters):0"/>
          <CheckAtEndReq action="(search_pipeline a_search_pipeline pipeline bluerov fd_spiral_high fd_recover_thrusters):0"/>
          <ApplyAtEndEffect action="(search_pipeline a_search_pipeline pipeline bluerov fd_spiral_high fd_recover_thrusters):0"/>
        </Sequence>
      </BehaviorTree>
    </root>
  )";

  try {
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

    auto state = problem_client->getState();
    state.addActionsAndPruneDerived({(*action_map)["(search_pipeline a_search_pipeline pipeline bluerov fd_spiral_high fd_recover_thrusters):0"].action_info});
    plansys2::solveDerivedPredicates(state);

    ASSERT_TRUE(state.hasPredicate(plansys2::Predicate("(robot_started bluerov)")));
    ASSERT_FALSE(state.hasInferredPredicate(plansys2::Predicate("(system_in_mode f_maintain_motion fd_unground)")));
    ASSERT_TRUE(state.hasInferredPredicate(plansys2::Predicate("(system_in_mode f_maintain_motion fd_recover_thrusters)")));
    ASSERT_FALSE(state.hasInferredPredicate(plansys2::Predicate("(system_in_mode f_generate_search_path fd_unground)")));
    ASSERT_TRUE(state.hasInferredPredicate(plansys2::Predicate("(system_in_mode f_generate_search_path fd_spiral_high)")));

    ASSERT_FALSE(state.hasInferredPredicate(plansys2::Predicate("(inferred-Fd_realisability fd_recover_thrusters false_boolean)")));
    ASSERT_FALSE(state.hasInferredPredicate(plansys2::Predicate("(inferred-Fd_realisability fd_spiral_high false_boolean)")));
    ASSERT_FALSE(state.hasInferredPredicate(plansys2::Predicate("(inferred-f_activated f_follow_pipeline)")));

    ASSERT_TRUE(state.hasInferredPredicate(plansys2::Predicate("(pipeline_found pipeline)")));

  } catch (std::exception & e) {
    std::cerr << e.what() << std::endl;
  }


  bt_xml_tree =
    R"(
    <root BTCPP_format="4" main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
        <ApplyAtEndEffect action="(reconfigure f_follow_pipeline fd_unground fd_follow_pipeline):0"/>
        <ApplyAtEndEffect action="(reconfigure f_generate_search_path fd_spiral_high fd_unground):0"/>
        </Sequence>
      </BehaviorTree>
    </root>
  )";

  try {
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

    auto state = problem_client->getState();
    state.addActionsAndPruneDerived({(*action_map)["(inspect_pipeline a_inspect_pipeline pipeline bluerov fd_follow_pipeline fd_recover_thrusters):0"].action_info});
    plansys2::solveDerivedPredicates(state);

    ASSERT_TRUE(state.hasPredicate(plansys2::Predicate("(robot_started bluerov)")));
    ASSERT_TRUE(state.hasInferredPredicate(plansys2::Predicate("(system_in_mode f_generate_search_path fd_unground)")));
    ASSERT_FALSE(state.hasInferredPredicate(plansys2::Predicate("(system_in_mode f_maintain_motion fd_unground)")));
    ASSERT_TRUE(state.hasInferredPredicate(plansys2::Predicate("(system_in_mode f_maintain_motion fd_recover_thrusters)")));
    ASSERT_TRUE(state.hasInferredPredicate(plansys2::Predicate("(system_in_mode f_follow_pipeline fd_follow_pipeline)")));
    ASSERT_FALSE(state.hasInferredPredicate(plansys2::Predicate("(system_in_mode f_follow_pipeline fd_unground)")));

    ASSERT_FALSE(state.hasInferredPredicate(plansys2::Predicate("(inferred-Fd_realisability fd_recover_thrusters false_boolean)")));
    ASSERT_FALSE(state.hasInferredPredicate(plansys2::Predicate("(inferred-Fd_realisability fd_follow_pipeline false_boolean)")));
    ASSERT_FALSE(state.hasInferredPredicate(plansys2::Predicate("(inferred-f_activated f_generate_search_path)")));

    ASSERT_TRUE(state.hasInferredPredicate(plansys2::Predicate("(pipeline_found pipeline)")));

  } catch (std::exception & e) {
    std::cerr << e.what() << std::endl;
  }

  bt_xml_tree =
    R"(
    <root BTCPP_format="4" main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
          <ApplyAtStartEffect action="(inspect_pipeline a_inspect_pipeline pipeline bluerov fd_follow_pipeline fd_recover_thrusters):0"/>
          <CheckOverAllReq action="(inspect_pipeline a_inspect_pipeline pipeline bluerov fd_follow_pipeline fd_recover_thrusters):0"/>
          <CheckAtEndReq action="(inspect_pipeline a_inspect_pipeline pipeline bluerov fd_follow_pipeline fd_recover_thrusters):0"/>
          <ApplyAtEndEffect action="(inspect_pipeline a_inspect_pipeline pipeline bluerov fd_follow_pipeline fd_recover_thrusters):0"/>
        </Sequence>
      </BehaviorTree>
    </root>
  )";

  try {
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

    auto state = problem_client->getState();
    state.addActionsAndPruneDerived({(*action_map)["(inspect_pipeline a_inspect_pipeline pipeline bluerov fd_follow_pipeline fd_recover_thrusters):0"].action_info});
    plansys2::solveDerivedPredicates(state);

    ASSERT_TRUE(state.hasPredicate(plansys2::Predicate("(robot_started bluerov)")));
    ASSERT_TRUE(state.hasInferredPredicate(plansys2::Predicate("(system_in_mode f_generate_search_path fd_unground)")));
    ASSERT_FALSE(state.hasInferredPredicate(plansys2::Predicate("(system_in_mode f_maintain_motion fd_unground)")));
    ASSERT_TRUE(state.hasInferredPredicate(plansys2::Predicate("(system_in_mode f_maintain_motion fd_recover_thrusters)")));
    ASSERT_TRUE(state.hasInferredPredicate(plansys2::Predicate("(system_in_mode f_follow_pipeline fd_follow_pipeline)")));
    ASSERT_FALSE(state.hasInferredPredicate(plansys2::Predicate("(system_in_mode f_follow_pipeline fd_unground)")));

    ASSERT_FALSE(state.hasInferredPredicate(plansys2::Predicate("(inferred-Fd_realisability fd_recover_thrusters false_boolean)")));
    ASSERT_FALSE(state.hasInferredPredicate(plansys2::Predicate("(inferred-Fd_realisability fd_follow_pipeline false_boolean)")));
    ASSERT_FALSE(state.hasInferredPredicate(plansys2::Predicate("(inferred-f_activated f_generate_search_path)")));

    ASSERT_TRUE(state.hasInferredPredicate(plansys2::Predicate("(pipeline_found pipeline)")));
    ASSERT_TRUE(state.hasInferredPredicate(plansys2::Predicate("(pipeline_inspected pipeline)")));

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
