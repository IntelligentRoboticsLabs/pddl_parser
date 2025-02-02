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

#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

#include "plansys2_pddl_parser/Utils.hpp"

namespace plansys2
{

ProblemExpertClient::ProblemExpertClient()
{
  node_ = rclcpp::Node::make_shared("problem_expert_client");

  add_problem_client_ = node_->create_client<plansys2_msgs::srv::AddProblem>(
    "problem_expert/add_problem");
  add_problem_goal_client_ = node_->create_client<plansys2_msgs::srv::AddProblemGoal>(
    "problem_expert/add_problem_goal");
  add_problem_instance_client_ = node_->create_client<plansys2_msgs::srv::AffectParam>(
    "problem_expert/add_problem_instance");
  add_problem_predicate_client_ = node_->create_client<plansys2_msgs::srv::AffectNode>(
    "problem_expert/add_problem_predicate");
  add_problem_function_client_ = node_->create_client<plansys2_msgs::srv::AffectNode>(
    "problem_expert/add_problem_function");
  get_problem_goal_client_ = node_->create_client<plansys2_msgs::srv::GetProblemGoal>(
    "problem_expert/get_problem_goal");
  get_problem_instance_details_client_ =
    node_->create_client<plansys2_msgs::srv::GetProblemInstanceDetails>(
    "problem_expert/get_problem_instance");
  get_problem_instances_client_ = node_->create_client<plansys2_msgs::srv::GetProblemInstances>(
    "problem_expert/get_problem_instances");
  get_problem_predicate_details_client_ =
    node_->create_client<plansys2_msgs::srv::GetNodeDetails>(
    "problem_expert/get_problem_predicate");
  get_problem_predicates_client_ = node_->create_client<plansys2_msgs::srv::GetStates>(
    "problem_expert/get_problem_predicates");
  get_problem_function_details_client_ =
    node_->create_client<plansys2_msgs::srv::GetNodeDetails>(
    "problem_expert/get_problem_function");
  get_problem_functions_client_ = node_->create_client<plansys2_msgs::srv::GetStates>(
    "problem_expert/get_problem_functions");
  get_problem_client_ = node_->create_client<plansys2_msgs::srv::GetProblem>(
    "problem_expert/get_problem");
  remove_problem_goal_client_ = node_->create_client<plansys2_msgs::srv::RemoveProblemGoal>(
    "problem_expert/remove_problem_goal");
  clear_problem_knowledge_client_ = node_->create_client<plansys2_msgs::srv::ClearProblemKnowledge>(
    "problem_expert/clear_problem_knowledge");
  remove_problem_instance_client_ =
    node_->create_client<plansys2_msgs::srv::AffectParam>(
    "problem_expert/remove_problem_instance");
  remove_problem_predicate_client_ =
    node_->create_client<plansys2_msgs::srv::AffectNode>(
    "problem_expert/remove_problem_predicate");
  remove_problem_function_client_ =
    node_->create_client<plansys2_msgs::srv::AffectNode>(
    "problem_expert/remove_problem_function");
  exist_problem_predicate_client_ =
    node_->create_client<plansys2_msgs::srv::ExistNode>(
    "problem_expert/exist_problem_predicate");
  exist_problem_function_client_ =
    node_->create_client<plansys2_msgs::srv::ExistNode>(
    "problem_expert/exist_problem_function");
  update_problem_function_client_ =
    node_->create_client<plansys2_msgs::srv::AffectNode>(
    "problem_expert/update_problem_function");
  is_problem_goal_satisfied_client_ =
    node_->create_client<plansys2_msgs::srv::IsProblemGoalSatisfied>(
    "problem_expert/is_problem_goal_satisfied");

  problem_sub_ = node_->create_subscription<plansys2_msgs::msg::Problem>(
    "problem_expert/problem",
    rclcpp::QoS(100), [this](plansys2_msgs::msg::Problem::SharedPtr msg) {
      cached_problem_ = msg->problem;
      problem_ts_ = msg->stamp;
    });

  update_problem_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
    "problem_expert/update_notify", 100,
    [this](std_msgs::msg::Empty::SharedPtr msg){
      update_time_ = this->node_->now();
    });

  problem_ts_ = node_->now();
}

std::vector<plansys2::Instance>
ProblemExpertClient::getInstances()
{
  while (!get_problem_instances_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_instances_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetProblemInstances::Request>();

  auto future_result = get_problem_instances_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  auto result = *future_result.get();

  if (result.success) {
    return plansys2::convertVector<plansys2::Instance, plansys2_msgs::msg::Param>(
      result.instances);
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      node_->get_namespace() <<
        get_problem_instances_client_->get_service_name() << ": " <<
        result.error_info);
    return {};
  }
}

bool
ProblemExpertClient::addInstance(const plansys2::Instance & instance)
{
  while (!add_problem_instance_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      add_problem_instance_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::AffectParam::Request>();
  request->param = instance;

  auto future_result = add_problem_instance_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto result = *future_result.get();

  if (result.success) {
    update_time_ = node_->now();
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      add_problem_instance_client_->get_service_name() << ": " <<
        result.error_info);
    return false;
  }
}

bool
ProblemExpertClient::removeInstance(const plansys2::Instance & instance)
{
  while (!remove_problem_instance_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      remove_problem_instance_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::AffectParam::Request>();
  request->param = instance;

  auto future_result = remove_problem_instance_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto result = *future_result.get();

  if (result.success) {
    update_time_ = node_->now();
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      remove_problem_instance_client_->get_service_name() << ": " <<
        result.error_info);
    return false;
  }
}


std::optional<plansys2::Instance>
ProblemExpertClient::getInstance(const std::string & name)
{
  while (!get_problem_instance_details_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_instance_details_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetProblemInstanceDetails::Request>();

  request->instance = name;

  auto future_result = get_problem_instance_details_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  auto result = *future_result.get();

  if (result.success) {
    return result.instance;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_instance_details_client_->get_service_name() << ": " <<
        result.error_info);
    return {};
  }
}

std::vector<plansys2::Predicate>
ProblemExpertClient::getPredicates()
{
  while (!get_problem_predicates_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_predicates_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetStates::Request>();

  auto future_result = get_problem_predicates_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  auto result = *future_result.get();

  if (result.success) {
    return plansys2::convertVector<plansys2::Predicate, plansys2_msgs::msg::Node>(
      result.states);
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_predicates_client_->get_service_name() << ": " <<
        result.error_info);
    return {};
  }
}

bool
ProblemExpertClient::addPredicate(const plansys2::Predicate & predicate)
{
  while (!add_problem_predicate_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      add_problem_predicate_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::AffectNode::Request>();
  request->node = predicate;

  auto future_result = add_problem_predicate_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto result = *future_result.get();

  if (result.success) {
    update_time_ = node_->now();
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      add_problem_predicate_client_->get_service_name() << ": " <<
        result.error_info);
    return false;
  }
}

bool
ProblemExpertClient::removePredicate(const plansys2::Predicate & predicate)
{
  while (!remove_problem_predicate_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      remove_problem_predicate_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::AffectNode::Request>();
  request->node = predicate;

  auto future_result = remove_problem_predicate_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto result = *future_result.get();

  if (result.success) {
    update_time_ = node_->now();
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      remove_problem_predicate_client_->get_service_name() << ": " <<
        result.error_info);
    return false;
  }
}

bool
ProblemExpertClient::existPredicate(const plansys2::Predicate & predicate)
{
  while (!exist_problem_predicate_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      exist_problem_predicate_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::ExistNode::Request>();
  request->node = predicate;

  auto future_result = exist_problem_predicate_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto result = *future_result.get();

  return result.exist;
}

std::optional<plansys2::Predicate>
ProblemExpertClient::getPredicate(const std::string & predicate)
{
  while (!get_problem_predicate_details_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_predicate_details_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetNodeDetails::Request>();

  request->expression = predicate;

  auto future_result = get_problem_predicate_details_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  auto result = *future_result.get();

  if (result.success) {
    return result.node;
  } else {
    RCLCPP_DEBUG_STREAM(
      node_->get_logger(),
      get_problem_predicate_details_client_->get_service_name() << ": " <<
        result.error_info);
    return {};
  }
}

std::vector<plansys2::Function>
ProblemExpertClient::getFunctions()
{
  while (!get_problem_functions_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_functions_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetStates::Request>();

  auto future_result = get_problem_functions_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  auto result = *future_result.get();

  if (result.success) {
    return plansys2::convertVector<plansys2::Function, plansys2_msgs::msg::Node>(
      result.states);
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_functions_client_->get_service_name() << ": " <<
        result.error_info);
    return {};
  }
}

bool
ProblemExpertClient::addFunction(const plansys2::Function & function)
{
  while (!add_problem_function_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      add_problem_function_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::AffectNode::Request>();
  request->node = function;

  auto future_result = add_problem_function_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(
      node_,
      future_result,
      std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto result = *future_result.get();

  if (result.success) {
    update_time_ = node_->now();
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      add_problem_function_client_->get_service_name() << ": " <<
        result.error_info);
    return false;
  }
}

bool
ProblemExpertClient::removeFunction(const plansys2::Function & function)
{
  while (!remove_problem_function_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      remove_problem_function_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::AffectNode::Request>();
  request->node = function;

  auto future_result = remove_problem_function_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto result = *future_result.get();

  if (result.success) {
    update_time_ = node_->now();
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      remove_problem_function_client_->get_service_name() << ": " <<
        result.error_info);
    return false;
  }
}

bool
ProblemExpertClient::existFunction(const plansys2::Function & function)
{
  while (!exist_problem_function_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      exist_problem_function_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::ExistNode::Request>();
  request->node = function;

  auto future_result = exist_problem_function_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto result = *future_result.get();

  return result.exist;
}

bool ProblemExpertClient::updateFunction(const plansys2::Function & function)
{
  while (!update_problem_function_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      update_problem_function_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::AffectNode::Request>();
  request->node = function;

  auto future_result = update_problem_function_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto result = *future_result.get();

  if (result.success) {
    update_time_ = node_->now();
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      update_problem_function_client_->get_service_name() << ": " <<
        result.error_info);
    return false;
  }
}

std::optional<plansys2::Function>
ProblemExpertClient::getFunction(const std::string & function)
{
  while (!get_problem_function_details_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_function_details_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetNodeDetails::Request>();

  request->expression = function;

  auto future_result = get_problem_function_details_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  auto result = *future_result.get();

  if (result.success) {
    return result.node;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_function_details_client_->get_service_name() << ": " <<
        result.error_info);
    return {};
  }
}


plansys2::Goal
ProblemExpertClient::getGoal()
{
  plansys2_msgs::msg::Tree ret;

  while (!get_problem_goal_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_goal_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetProblemGoal::Request>();

  auto future_result = get_problem_goal_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  auto result = *future_result.get();

  if (result.success) {
    return result.tree;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_goal_client_->get_service_name() << ": " <<
        result.error_info);
  }

  return ret;
}

bool
ProblemExpertClient::setGoal(const plansys2::Goal & goal)
{
  while (!add_problem_goal_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      add_problem_goal_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::AddProblemGoal::Request>();
  request->tree = goal;

  auto future_result = add_problem_goal_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto result = *future_result.get();

  if (result.success) {
    update_time_ = node_->now();
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      add_problem_goal_client_->get_service_name() << ": " <<
        result.error_info);
    return false;
  }
}

bool
ProblemExpertClient::isGoalSatisfied(const plansys2::Goal & goal)
{
  while (!is_problem_goal_satisfied_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      is_problem_goal_satisfied_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::IsProblemGoalSatisfied::Request>();
  request->tree = goal;

  auto future_result = is_problem_goal_satisfied_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto result = *future_result.get();

  if (result.success) {
    return result.satisfied;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      is_problem_goal_satisfied_client_->get_service_name() << ": " <<
        result.error_info);
    return false;
  }
}

bool
ProblemExpertClient::clearGoal()
{
  while (!remove_problem_goal_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      remove_problem_goal_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::RemoveProblemGoal::Request>();

  auto future_result = remove_problem_goal_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto result = *future_result.get();

  if (result.success) {
    update_time_ = node_->now();
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      remove_problem_goal_client_->get_service_name() << ": " <<
        result.error_info);
    return false;
  }
}

bool
ProblemExpertClient::clearKnowledge()
{
  while (!clear_problem_knowledge_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      clear_problem_knowledge_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::ClearProblemKnowledge::Request>();

  auto future_result = clear_problem_knowledge_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto result = *future_result.get();

  if (result.success) {
    update_time_ = node_->now();
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      clear_problem_knowledge_client_->get_service_name() << ": " <<
        result.error_info);
    return false;
  }
}

std::tuple<std::string, rclcpp::Time>
ProblemExpertClient::getProblemWithTimestamp(bool use_cache)
{
  return {getProblem(use_cache), problem_ts_};
}

std::string
ProblemExpertClient::getProblem(bool use_cache)
{
  if (use_cache && cached_problem_ != "") {
    return cached_problem_;
  } else {
    return getProblem();
  }
}

std::string
ProblemExpertClient::getProblem()
{
  while (!get_problem_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetProblem::Request>();

  auto future_result = get_problem_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  auto result = *future_result.get();

  if (result.success) {
    cached_problem_ = result.problem;
    problem_ts_ = result.stamp;

    return result.problem;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_problem_client_->get_service_name() << ": " <<
        result.error_info);
    return {};
  }
}

bool
ProblemExpertClient::addProblem(const std::string & problem_str)
{
  while (!add_problem_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      add_problem_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::AddProblem::Request>();
  request->problem = problem_str;

  auto future_result = add_problem_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }

  auto result = *future_result.get();

  if (result.success) {
    update_time_ = node_->now();
    return true;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      add_problem_client_->get_service_name() << ": " <<
        result.error_info);
    return false;
  }
}

}  // namespace plansys2
