// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"

#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_executor/ExecutorNode.hpp"

#include "plansys2_lifecycle_manager/lifecycle_manager.hpp"

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::experimental::executors::EventsExecutor exe;

  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto executor_node = std::make_shared<plansys2::ExecutorNode>();

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(executor_node->get_node_base_interface());

  std::map<std::string, std::shared_ptr<plansys2::LifecycleServiceClient>> manager_nodes;
  manager_nodes["domain_expert"] = std::make_shared<plansys2::LifecycleServiceClient>(
    "domain_expert_lc_mngr", "domain_expert");
  manager_nodes["problem_expert"] = std::make_shared<plansys2::LifecycleServiceClient>(
    "problem_expert_lc_mngr", "problem_expert");
  manager_nodes["planner"] = std::make_shared<plansys2::LifecycleServiceClient>(
    "planner_lc_mngr", "planner");
  manager_nodes["executor"] = std::make_shared<plansys2::LifecycleServiceClient>(
    "executor_lc_mngr", "executor");

  for (auto & manager_node : manager_nodes) {
    manager_node.second->init();
    exe.add_node(manager_node.second);
  }

  std::shared_future<bool> startup_future = std::async(
    std::launch::async,
    std::bind(plansys2::startup_function, manager_nodes, std::chrono::seconds(5)));
  exe.spin_until_future_complete(startup_future);

  if (!startup_future.get()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("plansys2_bringup"),
      "Failed to start plansys2!");
    rclcpp::shutdown();
    return -1;
  }

  exe.spin();
  rclcpp::shutdown();
  return 0;
}
