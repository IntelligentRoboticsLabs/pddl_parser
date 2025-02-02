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

#ifndef PLANSYS2_EXECUTOR__EXECUTORCLIENT_HPP_
#define PLANSYS2_EXECUTOR__EXECUTORCLIENT_HPP_

#include <optional>
#include <string>
#include <memory>
#include <list>
#include <vector>

#include "plansys2_msgs/action/execute_plan.hpp"
#include "plansys2_msgs/srv/get_ordered_sub_goals.hpp"
#include "plansys2_msgs/srv/get_plan.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_msgs/msg/tree.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace plansys2
{

class ExecutorClient
{
public:
  using ExecutePlan = plansys2_msgs::action::ExecutePlan;
  using GoalHandleExecutePlan = rclcpp_action::ClientGoalHandle<ExecutePlan>;

  ExecutorClient();
  explicit ExecutorClient(const std::string & node_name);

  bool start_plan_execution(const plansys2_msgs::msg::Plan & plan);
  bool execute_and_check_plan();
  void cancel_plan_execution();
  std::vector<plansys2_msgs::msg::Tree> getOrderedSubGoals();
  std::optional<plansys2_msgs::msg::Plan> get_plan();
  std::optional<plansys2_msgs::msg::Plan> get_remaining_plan();

  ExecutePlan::Feedback getFeedBack() {return feedback_;}
  std::optional<ExecutePlan::Result> getResult();

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp_action::Client<ExecutePlan>::SharedPtr action_client_;
  rclcpp::Client<plansys2_msgs::srv::GetOrderedSubGoals>::SharedPtr
    get_ordered_sub_goals_client_;
  rclcpp::Client<plansys2_msgs::srv::GetPlan>::SharedPtr get_plan_client_;
  rclcpp::Client<plansys2_msgs::srv::GetPlan>::SharedPtr get_remaining_plan_client_;

  ExecutePlan::Feedback feedback_;
  rclcpp_action::ClientGoalHandle<ExecutePlan>::SharedPtr goal_handler_;
  rclcpp_action::ClientGoalHandle<ExecutePlan>::WrappedResult result_;

  bool goal_result_available_{false};

  bool executing_plan_{false};

  void result_callback(const GoalHandleExecutePlan::WrappedResult & result);
  void feedback_callback(
    GoalHandleExecutePlan::SharedPtr goal_handle,
    const std::shared_ptr<const ExecutePlan::Feedback> feedback);

  bool on_new_goal_received(const plansys2_msgs::msg::Plan & plan);
  bool should_cancel_goal();
  void createActionClient();
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__EXECUTORCLIENT_HPP_
