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
#include <map>

#include "gtest/gtest.h"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "plansys2_lifecycle_manager/lifecycle_manager.hpp"

using namespace std::chrono_literals;

TEST(lifecycle_manager, lf_client)
{
  auto test_node = rclcpp_lifecycle::LifecycleNode::make_shared("test");
  auto client_node = std::make_shared<plansys2::LifecycleServiceClient>("mng_client", "test");

  auto exe = rclcpp::experimental::executors::EventsExecutor::make_shared();
  exe->add_node(test_node->get_node_base_interface());
  exe->add_node(client_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe->spin_some();}
    });

  client_node->init();

  auto start = test_node->now();
  while ((test_node->now() - start).seconds() < 1.0) {}

  ASSERT_EQ(
    test_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(client_node->get_state(1s), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  ASSERT_FALSE(client_node->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));

  ASSERT_TRUE(client_node->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
  ASSERT_EQ(client_node->get_state(1s), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  ASSERT_TRUE(client_node->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE));
  ASSERT_EQ(client_node->get_state(1s), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_TRUE(client_node->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE));
  ASSERT_EQ(client_node->get_state(1s), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  finish = true;
  t.join();
}

TEST(lifecycle_manager, lf_startup)
{
  auto de_node = rclcpp_lifecycle::LifecycleNode::make_shared("domain_expert");
  auto pe_node = rclcpp_lifecycle::LifecycleNode::make_shared("problem_expert");
  auto pl_node = rclcpp_lifecycle::LifecycleNode::make_shared("planner");
  auto ex_node = rclcpp_lifecycle::LifecycleNode::make_shared("executor");

  ASSERT_EQ(
    de_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(
    pe_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(
    pl_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(
    ex_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  std::map<std::string, std::shared_ptr<plansys2::LifecycleServiceClient>> manager_nodes;
  manager_nodes["domain_expert"] = std::make_shared<plansys2::LifecycleServiceClient>(
    "domain_expert_lc_mngr", "domain_expert");
  manager_nodes["problem_expert"] = std::make_shared<plansys2::LifecycleServiceClient>(
    "domain_expert_lc_mngr", "problem_expert");
  manager_nodes["planner"] = std::make_shared<plansys2::LifecycleServiceClient>(
    "domain_expert_lc_mngr", "planner");
  manager_nodes["executor"] = std::make_shared<plansys2::LifecycleServiceClient>(
    "domain_expert_lc_mngr", "executor");

  rclcpp::experimental::executors::EventsExecutor exe;
  for (auto & manager_node : manager_nodes) {
    manager_node.second->init();
    exe.add_node(manager_node.second);
  }

  exe.add_node(de_node->get_node_base_interface());
  exe.add_node(pe_node->get_node_base_interface());
  exe.add_node(pl_node->get_node_base_interface());
  exe.add_node(ex_node->get_node_base_interface());

  auto start = de_node->now();
  while ((de_node->now() - start).seconds() < 1.0) {}

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  std::shared_future<bool> startup_future = std::async(
    std::launch::async,
    std::bind(plansys2::startup_function, manager_nodes, std::chrono::seconds(3)));

  startup_future.wait();

  ASSERT_EQ(
    de_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_EQ(
    pe_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_EQ(
    pl_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_EQ(
    ex_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  start = de_node->now();
  while ((de_node->now() - start).seconds() < 1.0) {}

  finish = true;
  t.join();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
