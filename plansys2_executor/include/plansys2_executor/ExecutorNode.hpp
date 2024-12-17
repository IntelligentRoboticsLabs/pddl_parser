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

#ifndef PLANSYS2_EXECUTOR__EXECUTORNODE_HPP_
#define PLANSYS2_EXECUTOR__EXECUTORNODE_HPP_

#include <memory>
#include <vector>
#include <string>
#include <map>
#include <tuple>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/BTBuilder.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/blackboard.h"

#include "plansys2_msgs/action/execute_plan.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/srv/get_ordered_sub_goals.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace plansys2
{

struct TreeInfo
{
  using Ptr = std::shared_ptr<TreeInfo>;
  BT::Tree tree;
  BT::Blackboard::Ptr blackboard;
  std::shared_ptr<plansys2::BTBuilder> bt_builder;
};

struct PlanRuntineInfo
{
  plansys2_msgs::msg::Plan remaining_plan;
  plansys2_msgs::msg::Plan complete_plan;
  std::vector<plansys2_msgs::msg::Tree> ordered_sub_goals;
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map;
  TreeInfo::Ptr current_tree;
};

class ExecutorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using ExecutePlan = plansys2_msgs::action::ExecutePlan;
  using GoalHandleExecutePlan = rclcpp_action::ServerGoalHandle<ExecutePlan>;
  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  ExecutorNode();

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  void get_ordered_sub_goals_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetOrderedSubGoals::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetOrderedSubGoals::Response> response);

  void get_plan_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetPlan::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetPlan::Response> response);

  void get_remaining_plan_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetPlan::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetPlan::Response> response);

protected:
  bool cancel_plan_requested_;
  bool replan_requested_;

  plansys2_msgs::msg::Plan * remaining_plan_;
  plansys2_msgs::msg::Plan * complete_plan_;
  std::vector<plansys2_msgs::msg::Tree> * ordered_sub_goals_;

  std::string action_bt_xml_;
  std::string start_action_bt_xml_;
  std::string end_action_bt_xml_;
  pluginlib::ClassLoader<plansys2::BTBuilder> bt_builder_loader_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;

  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr
    execution_info_pub_;
  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::Plan>::SharedPtr executing_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::Plan>::SharedPtr remaining_plan_pub_;

  rclcpp_action::Server<ExecutePlan>::SharedPtr execute_plan_action_server_;
  rclcpp::Service<plansys2_msgs::srv::GetOrderedSubGoals>::SharedPtr
    get_ordered_sub_goals_service_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr dotgraph_pub_;

  void get_ordered_subgoals(PlanRuntineInfo & plan_info);

  rclcpp::Service<plansys2_msgs::srv::GetPlan>::SharedPtr get_plan_service_;
  rclcpp::Service<plansys2_msgs::srv::GetPlan>::SharedPtr get_remaining_plan_service_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecutePlan::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecutePlan> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleExecutePlan> goal_handle);
  std::shared_ptr<GoalHandleExecutePlan> current_goal_handle_;

  std::vector<plansys2_msgs::msg::ActionExecutionInfo> get_feedback_info(
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map);

  void print_execution_info(
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> exec_info);

  void update_plan(PlanRuntineInfo & runtime_info);
  bool init_plan_for_execution(PlanRuntineInfo & runtime_info);

  void execute_plan();
  bool get_tree_from_plan(PlanRuntineInfo & plan_info);
  void create_plan_runtime_info(PlanRuntineInfo & plan_info);

  static const int IDLE_STATE = 0;
  static const int EXECUTING_STATE = 1;
  int executor_state_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__EXECUTORNODE_HPP_
