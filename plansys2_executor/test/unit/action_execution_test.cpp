// Copyright 2021 Intelligent Robotics Lab
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
#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_problem_expert/Utils.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/blackboard.h"

#include "plansys2_executor/behavior_tree/execute_action_node.hpp"
#include "plansys2_executor/behavior_tree/wait_action_node.hpp"
#include "plansys2_executor/behavior_tree/wait_atstart_req_node.hpp"
#include "plansys2_executor/behavior_tree/check_overall_req_node.hpp"
#include "plansys2_executor/behavior_tree/check_atend_req_node.hpp"
#include "plansys2_executor/behavior_tree/apply_atstart_effect_node.hpp"
#include "plansys2_executor/behavior_tree/apply_atend_effect_node.hpp"

#include "lifecycle_msgs/msg/state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "gtest/gtest.h"


using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  using Ptr = std::shared_ptr<MoveAction>;
  static Ptr make_shared(const std::string & node_name, const std::chrono::nanoseconds & rate)
  {
    return std::make_shared<MoveAction>(node_name, rate);
  }


  MoveAction(const std::string & id, const std::chrono::nanoseconds & rate)
  : ActionExecutorClient(id, rate)
  {
    executions_ = 0;
    cycles_ = 0;
  }

  CallbackReturnT
  on_activate(const rclcpp_lifecycle::State & state)
  {
    std::cerr << "MoveAction::on_activate" << std::endl;
    counter_ = 0;

    return ActionExecutorClient::on_activate(state);
  }

  void do_work() override
  {
    RCLCPP_INFO_STREAM(get_logger(), "Executing [" << action_managed_ << "]");
    for (const auto & arg : current_arguments_) {
      RCLCPP_INFO_STREAM(get_logger(), "\t[" << arg << "]");
    }

    cycles_++;

    if (counter_++ > 3) {
      finish(true, 1.0, "completed");
      executions_++;
    } else {
      send_feedback(counter_ * 0.0, "running");
    }
  }

  int counter_;
  int executions_;
  int cycles_;
};

TEST(action_execution, protocol_basic)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto test_lf_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_lf_node");
  auto move_action_node = std::make_shared<MoveAction>("move_action", 1s);
  auto move_action_executor = plansys2::ActionExecutor::make_shared(
    "(move r2d2 steering_wheels_zone assembly_zone)", test_lf_node);

  ASSERT_EQ(move_action_executor->get_action_name(), "move");
  ASSERT_EQ(move_action_executor->get_action_params().size(), 3u);
  ASSERT_EQ(move_action_executor->get_action_params()[0], "r2d2");
  ASSERT_EQ(move_action_executor->get_action_params()[2], "assembly_zone");

  move_action_node->set_parameter({"action_name", "move"});

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(test_node);
  exe.add_node(test_lf_node->get_node_base_interface());
  exe.add_node(move_action_node->get_node_base_interface());

  std::vector<plansys2_msgs::msg::ActionExecution> action_execution_msgs;

  auto action_hub_sub = test_node->create_subscription<plansys2_msgs::msg::ActionExecution>(
    "/actions_hub", rclcpp::QoS(100).reliable(),
    [&action_execution_msgs](const plansys2_msgs::msg::ActionExecution::SharedPtr msg) {
      action_execution_msgs.push_back(*msg);
    });

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  ASSERT_EQ(move_action_executor->get_internal_status(), plansys2::ActionExecutor::Status::IDLE);
  ASSERT_EQ(
    move_action_node->get_internal_status().state,
    plansys2_msgs::msg::ActionPerformerStatus::NOT_READY);

  test_lf_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  move_action_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_EQ(
    move_action_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  ASSERT_EQ(
    move_action_node->get_internal_status().state,
    plansys2_msgs::msg::ActionPerformerStatus::READY);
  ASSERT_TRUE(action_execution_msgs.empty());

  {
    std::vector<BT::NodeStatus> tick_status_log;
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      tick_status_log.push_back(move_action_executor->tick(test_node->now()));
      rate.sleep();
    }
    ASSERT_EQ(tick_status_log[0], BT::NodeStatus::RUNNING);
  }

  ASSERT_EQ(
    move_action_node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_EQ(move_action_executor->get_internal_status(), plansys2::ActionExecutor::Status::RUNNING);
  ASSERT_EQ(
    move_action_node->get_internal_status().state,
    plansys2_msgs::msg::ActionPerformerStatus::RUNNING);

  ASSERT_EQ(action_execution_msgs.size(), 3u);
  ASSERT_EQ(action_execution_msgs[0].type, plansys2_msgs::msg::ActionExecution::REQUEST);
  ASSERT_EQ(action_execution_msgs[1].type, plansys2_msgs::msg::ActionExecution::RESPONSE);
  ASSERT_EQ(action_execution_msgs[2].type, plansys2_msgs::msg::ActionExecution::CONFIRM);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 5) {
      move_action_executor->tick(test_node->now());
      rate.sleep();
    }
  }

  ASSERT_EQ(move_action_executor->get_internal_status(), plansys2::ActionExecutor::Status::SUCCESS);
  ASSERT_EQ(
    move_action_node->get_internal_status().state,
    plansys2_msgs::msg::ActionPerformerStatus::READY);


  ASSERT_EQ(action_execution_msgs.size(), 8u);
  ASSERT_EQ(action_execution_msgs[3].type, plansys2_msgs::msg::ActionExecution::FEEDBACK);
  ASSERT_EQ(action_execution_msgs[4].type, plansys2_msgs::msg::ActionExecution::FEEDBACK);
  ASSERT_EQ(action_execution_msgs[5].type, plansys2_msgs::msg::ActionExecution::FEEDBACK);
  ASSERT_EQ(action_execution_msgs[6].type, plansys2_msgs::msg::ActionExecution::FEEDBACK);
  ASSERT_EQ(action_execution_msgs[7].type, plansys2_msgs::msg::ActionExecution::FINISH);


  ASSERT_EQ(move_action_executor->get_internal_status(), plansys2::ActionExecutor::Status::SUCCESS);
  ASSERT_EQ(
    move_action_node->get_internal_status().state,
    plansys2_msgs::msg::ActionPerformerStatus::READY);

  finish = true;
  t.join();
}

TEST(action_execution, protocol_cancelation)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto test_lf_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_lf_node");
  auto move_action_node = std::make_shared<MoveAction>("move_action", 1s);
  auto move_action_executor = plansys2::ActionExecutor::make_shared(
    "(move r2d2 steering_wheels_zone assembly_zone)", test_lf_node);

  ASSERT_EQ(move_action_executor->get_action_name(), "move");
  ASSERT_EQ(move_action_executor->get_action_params().size(), 3u);
  ASSERT_EQ(move_action_executor->get_action_params()[0], "r2d2");
  ASSERT_EQ(move_action_executor->get_action_params()[2], "assembly_zone");

  move_action_node->set_parameter({"action_name", "move"});

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(test_node);
  exe.add_node(test_lf_node->get_node_base_interface());
  exe.add_node(move_action_node->get_node_base_interface());

  std::vector<plansys2_msgs::msg::ActionExecution> action_execution_msgs;

  auto action_hub_sub = test_node->create_subscription<plansys2_msgs::msg::ActionExecution>(
    "/actions_hub", rclcpp::QoS(100).reliable(),
    [&action_execution_msgs](const plansys2_msgs::msg::ActionExecution::SharedPtr msg) {
      action_execution_msgs.push_back(*msg);
    });

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  ASSERT_EQ(move_action_executor->get_internal_status(), plansys2::ActionExecutor::Status::IDLE);
  ASSERT_EQ(
    move_action_node->get_internal_status().state,
    plansys2_msgs::msg::ActionPerformerStatus::NOT_READY);

  test_lf_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  move_action_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_EQ(
    move_action_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  ASSERT_EQ(
    move_action_node->get_internal_status().state,
    plansys2_msgs::msg::ActionPerformerStatus::READY);
  ASSERT_TRUE(action_execution_msgs.empty());

  {
    std::vector<BT::NodeStatus> tick_status_log;
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      tick_status_log.push_back(move_action_executor->tick(test_node->now()));
      rate.sleep();
    }
    ASSERT_EQ(tick_status_log[0], BT::NodeStatus::RUNNING);
  }

  ASSERT_EQ(
    move_action_node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_EQ(
    move_action_executor->get_internal_status(), plansys2::ActionExecutor::Status::RUNNING);
  ASSERT_EQ(
    move_action_node->get_internal_status().state,
    plansys2_msgs::msg::ActionPerformerStatus::RUNNING);

  ASSERT_EQ(action_execution_msgs.size(), 3u);
  ASSERT_EQ(action_execution_msgs[0].type, plansys2_msgs::msg::ActionExecution::REQUEST);
  ASSERT_EQ(action_execution_msgs[1].type, plansys2_msgs::msg::ActionExecution::RESPONSE);
  ASSERT_EQ(action_execution_msgs[2].type, plansys2_msgs::msg::ActionExecution::CONFIRM);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 2) {
      move_action_executor->tick(test_node->now());
      rate.sleep();
    }
  }

  move_action_executor->cancel();

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 2) {
      move_action_executor->tick(test_node->now());
      rate.sleep();
    }
  }

  ASSERT_EQ(
    move_action_executor->get_internal_status(),
    plansys2::ActionExecutor::Status::CANCELLED);
  ASSERT_EQ(
    move_action_node->get_internal_status().state,
    plansys2_msgs::msg::ActionPerformerStatus::READY);


  ASSERT_EQ(action_execution_msgs.size(), 6u);
  ASSERT_EQ(action_execution_msgs[3].type, plansys2_msgs::msg::ActionExecution::FEEDBACK);
  ASSERT_EQ(action_execution_msgs[4].type, plansys2_msgs::msg::ActionExecution::FEEDBACK);
  ASSERT_EQ(action_execution_msgs[5].type, plansys2_msgs::msg::ActionExecution::CANCEL);

  finish = true;
  t.join();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
