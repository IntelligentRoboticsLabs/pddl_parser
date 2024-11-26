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

#ifndef PLANSYS2_EXECUTOR__ACTIONEXECUTORCLIENT_HPP_
#define PLANSYS2_EXECUTOR__ACTIONEXECUTORCLIENT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_msgs/msg/action_execution.hpp"
#include "plansys2_msgs/msg/action_performer_status.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace plansys2
{

class ActionExecutorClient : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  using Ptr = std::shared_ptr<ActionExecutorClient>;
  static Ptr make_shared(const std::string & node_name, const std::chrono::nanoseconds & rate)
  {
    return std::make_shared<ActionExecutorClient>(node_name, rate);
  }

  ActionExecutorClient(const std::string & node_name, const std::chrono::nanoseconds & rate);

  rclcpp::Time get_start_time() const {return start_time_;}
  plansys2_msgs::msg::ActionPerformerStatus get_internal_status() const {return status_;}

protected:
  virtual void do_work() {}

  const std::vector<std::string> & get_arguments() const {return current_arguments_;}
  const std::string get_action_name() const {return action_managed_;}

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  virtual CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  virtual CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  virtual CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  void action_hub_callback(const plansys2_msgs::msg::ActionExecution::SharedPtr msg);

  bool should_execute(const std::string & action, const std::vector<std::string> & args);
  void send_response(const plansys2_msgs::msg::ActionExecution::SharedPtr msg);
  void send_feedback(float completion, const std::string & status = "");
  void finish(bool success, float completion, const std::string & status = "");

  std::chrono::nanoseconds rate_;
  std::string action_managed_;
  bool commited_;

  std::vector<std::string> current_arguments_;
  std::vector<std::string> specialized_arguments_;

  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::ActionExecution>::SharedPtr
    action_hub_pub_;
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecution>::SharedPtr action_hub_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::ActionPerformerStatus>::SharedPtr
    status_pub_;
  rclcpp::TimerBase::SharedPtr hearbeat_pub_;
  plansys2_msgs::msg::ActionPerformerStatus status_;
  rclcpp::Time start_time_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__ACTIONEXECUTORCLIENT_HPP_
