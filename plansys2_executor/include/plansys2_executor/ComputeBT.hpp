// Copyright 2020 Intelligent Robotics Lab
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

#ifndef PLANSYS2_EXECUTOR__COMPUTEBT_HPP_
#define PLANSYS2_EXECUTOR__COMPUTEBT_HPP_

#include <memory>
#include <string>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "std_srvs/srv/trigger.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace plansys2
{

class ComputeBT : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  ComputeBT();

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

private:
  std::string action_bt_xml_;
  std::string start_action_bt_xml_;
  std::string end_action_bt_xml_;
  pluginlib::ClassLoader<plansys2::bt_builder::BTBuilder> bt_builder_loader_;

  std::shared_ptr<plansys2::DomainExpertNode> domain_node_;
  std::shared_ptr<plansys2::PlannerNode> planner_node_;
  std::shared_ptr<plansys2::ProblemExpertNode> problem_node_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr compute_bt_srv_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr dotgraph_pub_;

  void computeBTCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  std::string getProblem(const std::string & filename) const;
  void savePlan(const plansys2_msgs::msg::Plan & plan, const std::string & filename) const;
  void saveBT(const std::string & bt_xml, const std::string & filename) const;
  void saveDotGraph(const std::string & dotgraph, const std::string & filename) const;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__COMPUTEBT_HPP_
