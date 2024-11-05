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

#ifndef PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTNODE_HPP_
#define PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTNODE_HPP_

#include <memory>

#include "plansys2_problem_expert/ProblemExpert.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_msgs/msg/knowledge.hpp"
#include "plansys2_msgs/srv/affect_node.hpp"
#include "plansys2_msgs/srv/affect_param.hpp"
#include "plansys2_msgs/srv/add_problem.hpp"
#include "plansys2_msgs/srv/add_problem_goal.hpp"
#include "plansys2_msgs/srv/exist_node.hpp"
#include "plansys2_msgs/srv/get_problem.hpp"
#include "plansys2_msgs/srv/get_problem_goal.hpp"
#include "plansys2_msgs/srv/get_problem_instance_details.hpp"
#include "plansys2_msgs/srv/get_problem_instances.hpp"
#include "plansys2_msgs/srv/get_problem_state.hpp"
#include "plansys2_msgs/srv/get_node_details.hpp"
#include "plansys2_msgs/srv/get_states.hpp"
#include "plansys2_msgs/srv/is_problem_goal_satisfied.hpp"
#include "plansys2_msgs/srv/remove_problem_goal.hpp"
#include "plansys2_msgs/srv/clear_problem_knowledge.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace plansys2
{

class ProblemExpertNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  ProblemExpertNode();

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  plansys2_msgs::msg::Knowledge::SharedPtr get_knowledge_as_msg() const;

  void add_problem_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AddProblem::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AddProblem::Response> response);

  void add_problem_goal_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AddProblemGoal::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AddProblemGoal::Response> response);

  void add_problem_instance_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AffectParam::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AffectParam::Response> response);

  void add_problem_predicate_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response);

  void add_problem_function_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response);

  void get_problem_goal_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemGoal::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemGoal::Response> response);

  void get_problem_state_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemState::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemState::Response> response);

  void get_problem_instance_details_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemInstanceDetails::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemInstanceDetails::Response> response);

  void get_problem_instances_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemInstances::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetProblemInstances::Response> response);

  void get_problem_predicate_details_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Response> response);

  void get_problem_predicates_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetStates::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetStates::Response> response);

  void get_problem_inferred_predicates_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetStates::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetStates::Response> response);

  void get_problem_function_details_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Response> response);

  void get_problem_functions_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetStates::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetStates::Response> response);

  void get_problem_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetProblem::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetProblem::Response> response);

  void is_problem_goal_satisfied_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::IsProblemGoalSatisfied::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::IsProblemGoalSatisfied::Response> response);

  void remove_problem_goal_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::RemoveProblemGoal::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::RemoveProblemGoal::Response> response);

  void clear_problem_knowledge_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::ClearProblemKnowledge::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::ClearProblemKnowledge::Response> response);

  void remove_problem_instance_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AffectParam::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AffectParam::Response> response);

  void remove_problem_predicate_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response);

  void remove_problem_function_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response);

  void exist_problem_predicate_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::ExistNode::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::ExistNode::Response> response);

  void exist_problem_function_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::ExistNode::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::ExistNode::Response> response);

  void update_problem_function_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response);

private:
  std::shared_ptr<ProblemExpert> problem_expert_;

  rclcpp::Service<plansys2_msgs::srv::AddProblem>::SharedPtr
    add_problem_service_;
  rclcpp::Service<plansys2_msgs::srv::AddProblemGoal>::SharedPtr
    add_problem_goal_service_;
  rclcpp::Service<plansys2_msgs::srv::AffectParam>::SharedPtr
    add_problem_instance_service_;
  rclcpp::Service<plansys2_msgs::srv::AffectNode>::SharedPtr
    add_problem_predicate_service_;
  rclcpp::Service<plansys2_msgs::srv::AffectNode>::SharedPtr
    add_problem_function_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblemGoal>::SharedPtr
    get_problem_goal_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblemState>::SharedPtr
    get_problem_state_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblemInstanceDetails>::SharedPtr
    get_problem_instance_details_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblemInstances>::SharedPtr
    get_problem_instances_service_;
  rclcpp::Service<plansys2_msgs::srv::GetNodeDetails>::SharedPtr
    get_problem_predicate_details_service_;
  rclcpp::Service<plansys2_msgs::srv::GetStates>::SharedPtr
    get_problem_predicates_service_;
  rclcpp::Service<plansys2_msgs::srv::GetStates>::SharedPtr
    get_problem_inferred_predicates_service_;
  rclcpp::Service<plansys2_msgs::srv::GetNodeDetails>::SharedPtr
    get_problem_function_details_service_;
  rclcpp::Service<plansys2_msgs::srv::GetStates>::SharedPtr
    get_problem_functions_service_;
  rclcpp::Service<plansys2_msgs::srv::GetProblem>::SharedPtr
    get_problem_service_;
  rclcpp::Service<plansys2_msgs::srv::IsProblemGoalSatisfied>::SharedPtr
    is_problem_goal_satisfied_service_;
  rclcpp::Service<plansys2_msgs::srv::RemoveProblemGoal>::SharedPtr
    remove_problem_goal_service_;
  rclcpp::Service<plansys2_msgs::srv::ClearProblemKnowledge>::SharedPtr
    clear_problem_knowledge_service_;
  rclcpp::Service<plansys2_msgs::srv::AffectParam>::SharedPtr
    remove_problem_instance_service_;
  rclcpp::Service<plansys2_msgs::srv::AffectNode>::SharedPtr
    remove_problem_predicate_service_;
  rclcpp::Service<plansys2_msgs::srv::AffectNode>::SharedPtr
    remove_problem_function_service_;
  rclcpp::Service<plansys2_msgs::srv::ExistNode>::SharedPtr
    exist_problem_predicate_service_;
  rclcpp::Service<plansys2_msgs::srv::ExistNode>::SharedPtr
    exist_problem_function_service_;
  rclcpp::Service<plansys2_msgs::srv::AffectNode>::SharedPtr
    update_problem_function_service_;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr update_pub_;
  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::msg::Knowledge>::SharedPtr knowledge_pub_;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTNODE_HPP_
