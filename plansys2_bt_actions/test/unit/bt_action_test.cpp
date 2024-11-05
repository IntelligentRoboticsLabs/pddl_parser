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

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"

#include "../behavior_tree/OpenGripper.hpp"
#include "../behavior_tree/CloseGripper.hpp"
#include "../behavior_tree/Move.hpp"
#include "../behavior_tree/FailureNodes.hpp"

#include "plansys2_executor/ActionExecutor.hpp"

#include "test_msgs/action/fibonacci.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "plansys2_bt_actions/BTAction.hpp"

#include "gtest/gtest.h"

using namespace std::placeholders;
using namespace std::chrono_literals;

class MoveServer : public rclcpp::Node
{
  using Fibonacci = test_msgs::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

public:
  MoveServer()
  : Node("move_server") {}

  void start_server()
  {
    move_action_server_ = rclcpp_action::create_server<Fibonacci>(
      shared_from_this(),
      "move",
      std::bind(&MoveServer::handle_goal, this, _1, _2),
      std::bind(&MoveServer::handle_cancel, this, _1),
      std::bind(&MoveServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr move_action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_DEBUG(get_logger(), "handle_goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    cancelled_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_DEBUG(get_logger(), "handle_accepted");
    std::thread{std::bind(&MoveServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    auto feedback = std::make_shared<Fibonacci::Feedback>();

    int counter = 0;
    rclcpp::Rate rate(10);
    while (rclcpp::ok() && counter++ < 50 && !cancelled_) {  // 5 secs
      RCLCPP_DEBUG(get_logger(), "execute");
      goal_handle->publish_feedback(feedback);
      rate.sleep();
    }

    if (!cancelled_) {
      auto result = std::make_shared<Fibonacci::Result>();

      result->sequence.push_back(4);
      goal_handle->succeed(result);
    }
  }

  bool cancelled_ {false};
};

TEST(bt_actions, load_plugins)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("load_plugins_node");
  auto move_server_node = std::make_shared<MoveServer>();
  move_server_node->start_server();

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {
        rclcpp::spin_some(move_server_node);
        rclcpp::spin_some(node->get_node_base_interface());
      }
    });

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("plansys2_close_gripper_bt_node"));
  factory.registerFromPlugin(loader.getOSName("plansys2_open_gripper_bt_node"));
  factory.registerFromPlugin(loader.getOSName("plansys2_move_bt_test_node"));

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_bt_actions");
  std::string xml_file = pkgpath + "/test/behavior_tree/transport.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  rclcpp::Rate rate(10);

  int counter = 0;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
    counter++;
    rate.sleep();
  }

  t.join();
}

TEST(bt_actions, on_tick_failure)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto move_server_node = std::make_shared<MoveServer>();
  move_server_node->start_server();

  bool finished = false;
  std::thread t([&]() {
      while (!finished) {
        rclcpp::spin_some(move_server_node);
        rclcpp::spin_some(node->get_node_base_interface());
      }
    });


  BT::NodeConfig config;
  BT::assignDefaultRemapping<plansys2_bt_tests::OnTickFail>(config);
  auto bb = BT::Blackboard::create();
  bb->set("node", node);
  config.blackboard = bb;

  plansys2_bt_tests::OnTickFail failure_node("OnTickFail", "move", config);

  rclcpp::Rate rate(10);

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    status = failure_node.executeTick();
    rate.sleep();
  }

  ASSERT_TRUE(failure_node.on_tick_run);
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);

  finished = true;
  t.join();
}

TEST(bt_actions, on_feedback_failure)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto move_server_node = std::make_shared<MoveServer>();
  move_server_node->start_server();

  bool finished = false;
  std::thread t([&]() {
      while (!finished) {
        rclcpp::spin_some(move_server_node);
        rclcpp::spin_some(node->get_node_base_interface());
      }
    });

  BT::NodeConfig config;
  BT::assignDefaultRemapping<plansys2_bt_tests::OnFeedbackFail>(config);
  auto bb = BT::Blackboard::create();
  bb->set("node", node);
  config.blackboard = bb;

  plansys2_bt_tests::OnFeedbackFail failure_node("OnFeedbackFail",
    "move",
    config);

  rclcpp::Rate rate(10);

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    status = failure_node.executeTick();
    rate.sleep();
  }

  ASSERT_TRUE(failure_node.on_feedback_run);
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);

  finished = true;
  t.join();
}

TEST(bt_actions, bt_action)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_bt_actions");
  std::string xml_file = pkgpath + "/test/behavior_tree/assemble.xml";

  std::vector<std::string> plugins = {
    "plansys2_close_gripper_bt_node", "plansys2_open_gripper_bt_node"};

  auto bt_action = std::make_shared<plansys2::BTAction>("assemble", 100ms);

  auto lc_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto action_client = plansys2::ActionExecutor::make_shared("(assemble r2d2 z p1 p2 p3)", lc_node);

  bt_action->set_parameter(rclcpp::Parameter("action_name", "assemble"));
  bt_action->set_parameter(rclcpp::Parameter("bt_xml_file", xml_file));
  bt_action->set_parameter(rclcpp::Parameter("plugins", plugins));

  bt_action->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::experimental::executors::EventsExecutor exe;
  exe.add_node(bt_action->get_node_base_interface());
  exe.add_node(lc_node->get_node_base_interface());

  bool finished = false;
  while (rclcpp::ok && !finished) {
    exe.spin_some();

    action_client->tick(lc_node->now());
    finished = action_client->get_status() == BT::NodeStatus::SUCCESS;
  }

  auto start = lc_node->now();
  while ( (lc_node->now() - start).seconds() < 2) {
    exe.spin_some();
  }
}

TEST(bt_actions, cancel_bt_action)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_bt_actions");
  std::string xml_file = pkgpath + "/test/behavior_tree/assemble.xml";

  std::vector<std::string> plugins = {
    "plansys2_close_gripper_bt_node", "plansys2_open_gripper_bt_node"};

  auto bt_action = std::make_shared<plansys2::BTAction>("assemble", 1s);

  auto lc_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto action_client = plansys2::ActionExecutor::make_shared("(assemble r2d2 z p1 p2 p3)", lc_node);

  bt_action->set_parameter(rclcpp::Parameter("action_name", "assemble"));
  bt_action->set_parameter(rclcpp::Parameter("bt_xml_file", xml_file));
  bt_action->set_parameter(rclcpp::Parameter("plugins", plugins));

  bt_action->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::experimental::executors::EventsExecutor exe;
  exe.add_node(bt_action->get_node_base_interface());
  exe.add_node(lc_node->get_node_base_interface());

  std::vector<plansys2_msgs::msg::ActionExecution> action_execution_msgs;

  auto action_hub_sub = lc_node->create_subscription<plansys2_msgs::msg::ActionExecution>(
    "/actions_hub", rclcpp::QoS(100).reliable(),
    [&action_execution_msgs](const plansys2_msgs::msg::ActionExecution::SharedPtr msg) {
      action_execution_msgs.push_back(*msg);
    });

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  ASSERT_EQ(action_client->get_internal_status(), plansys2::ActionExecutor::Status::IDLE);
  ASSERT_EQ(
    bt_action->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  {
    rclcpp::Rate rate(10);
    auto start = lc_node->now();
    while ((lc_node->now() - start).seconds() < 0.5) {
      action_client->tick(lc_node->now());
      rate.sleep();
    }
  }

  ASSERT_EQ(action_client->get_internal_status(), plansys2::ActionExecutor::Status::RUNNING);
  ASSERT_EQ(bt_action->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_EQ(action_execution_msgs.size(), 3u);
  ASSERT_EQ(action_execution_msgs[0].type, plansys2_msgs::msg::ActionExecution::REQUEST);
  ASSERT_EQ(action_execution_msgs[1].type, plansys2_msgs::msg::ActionExecution::RESPONSE);
  ASSERT_EQ(action_execution_msgs[2].type, plansys2_msgs::msg::ActionExecution::CONFIRM);
  {
    rclcpp::Rate rate(10);
    auto start = lc_node->now();
    while ((lc_node->now() - start).seconds() < 1.0) {
      action_client->tick(lc_node->now());
      rate.sleep();
    }
  }

  ASSERT_EQ(action_execution_msgs.size(), 4u);
  action_client->cancel();

  {
    rclcpp::Rate rate(10);
    auto start = lc_node->now();
    while ((lc_node->now() - start).seconds() < 1.0) {
      action_client->tick(lc_node->now());
      rate.sleep();
    }
  }

  ASSERT_EQ(action_execution_msgs.size(), 5u);
  ASSERT_EQ(action_execution_msgs[3].type, plansys2_msgs::msg::ActionExecution::FEEDBACK);
  ASSERT_EQ(action_execution_msgs[4].type, plansys2_msgs::msg::ActionExecution::CANCEL);

  ASSERT_EQ(action_client->get_internal_status(), plansys2::ActionExecutor::Status::CANCELLED);
  ASSERT_EQ(
    bt_action->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  finish = true;
  t.join();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
