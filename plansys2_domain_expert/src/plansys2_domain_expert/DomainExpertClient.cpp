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

#include "plansys2_domain_expert/DomainExpertClient.hpp"

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

namespace plansys2
{

DomainExpertClient::DomainExpertClient()
{
  node_ = rclcpp::Node::make_shared("domain_expert_client");

  get_domain_client_ = node_->create_client<plansys2_msgs::srv::GetDomain>(
    "domain_expert/get_domain");
  get_name_client_ = node_->create_client<plansys2_msgs::srv::GetDomainName>(
    "domain_expert/get_domain_name");
  get_types_client_ = node_->create_client<plansys2_msgs::srv::GetDomainTypes>(
    "domain_expert/get_domain_types");
  get_constants_client_ = node_->create_client<plansys2_msgs::srv::GetDomainConstants>(
    "domain_expert/get_domain_constants");
  get_predicates_client_ = node_->create_client<plansys2_msgs::srv::GetStates>(
    "domain_expert/get_domain_predicates");
  get_functions_client_ = node_->create_client<plansys2_msgs::srv::GetStates>(
    "domain_expert/get_domain_functions");
  get_derived_predicates_client_ =
    node_->create_client<plansys2_msgs::srv::GetDomainDerivedPredicateDetails>(
    "domain_expert/get_domain_derived_predicates");
  get_derived_predicate_details_client_ =
    node_->create_client<plansys2_msgs::srv::GetDomainDerivedPredicateDetails>(
    "domain_expert/get_domain_derived_predicate_details");
  get_actions_client_ = node_->create_client<plansys2_msgs::srv::GetDomainActions>(
    "domain_expert/get_domain_actions");
  get_durative_actions_client_ = node_->create_client<plansys2_msgs::srv::GetDomainActions>(
    "domain_expert/get_domain_durative_actions");
  get_predicate_details_client_ =
    node_->create_client<plansys2_msgs::srv::GetNodeDetails>(
    "domain_expert/get_domain_predicate_details");
  get_function_details_client_ =
    node_->create_client<plansys2_msgs::srv::GetNodeDetails>(
    "domain_expert/get_domain_function_details");
  get_action_details_client_ = node_->create_client<plansys2_msgs::srv::GetDomainActionDetails>(
    "domain_expert/get_domain_action_details");
  get_durative_action_details_client_ =
    node_->create_client<plansys2_msgs::srv::GetDomainDurativeActionDetails>(
    "domain_expert/get_domain_durative_action_details");
}

std::string
DomainExpertClient::getName()
{
  std::string ret;

  while (!get_name_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_INFO_STREAM(
      node_->get_logger(),
      get_name_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainName::Request>();

  auto future_result = get_name_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  auto result = *future_result.get();

  ret = result.name;

  return ret;
}

std::vector<std::string>
DomainExpertClient::getTypes()
{
  std::vector<std::string> ret;

  while (!get_types_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_types_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainTypes::Request>();

  auto future_result = get_types_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  auto result = *future_result.get();

  ret = result.types;

  return ret;
}

std::vector<std::string>
DomainExpertClient::getConstants(const std::string & type)
{
  std::vector<std::string> ret;

  while (!get_constants_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_constants_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainConstants::Request>();

  auto future_result = get_constants_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  auto result = *future_result.get();

  ret = result.constants;

  return ret;
}

std::vector<plansys2::Predicate>
DomainExpertClient::getPredicates()
{
  std::vector<plansys2::Predicate> ret;

  while (!get_predicates_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_predicates_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetStates::Request>();

  auto future_result = get_predicates_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  auto result = *future_result.get();

  ret = plansys2::convertVector<plansys2::Predicate, plansys2_msgs::msg::Node>(
    result.states);

  return ret;
}

std::optional<plansys2::Predicate>
DomainExpertClient::getPredicate(const std::string & predicate)
{
  while (!get_predicate_details_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_predicate_details_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetNodeDetails::Request>();

  request->expression = predicate;

  auto future_result = get_predicate_details_client_->async_send_request(request);

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
      get_predicate_details_client_->get_service_name() << ": " <<
        result.error_info);
    return {};
  }
  return {};
}

std::vector<plansys2::Function>
DomainExpertClient::getFunctions()
{
  std::vector<plansys2::Function> ret;

  while (!get_functions_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_functions_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetStates::Request>();

  auto future_result = get_functions_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  auto result = *future_result.get();

  ret = plansys2::convertVector<plansys2::Function, plansys2_msgs::msg::Node>(
    result.states);

  return ret;
}

std::optional<plansys2::Function>
DomainExpertClient::getFunction(const std::string & function)
{
  while (!get_function_details_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_function_details_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetNodeDetails::Request>();

  request->expression = function;

  auto future_result = get_function_details_client_->async_send_request(request);

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
      get_function_details_client_->get_service_name() << ": " <<
        result.error_info);
    return {};
  }
  return {};
}

std::vector<plansys2_msgs::msg::Derived>
DomainExpertClient::getDerivedPredicates()
{
  std::vector<plansys2_msgs::msg::Derived> ret;

  while (!get_derived_predicates_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_derived_predicates_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainDerivedPredicateDetails::Request>();

  auto future_result = get_derived_predicates_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  return future_result.get()->predicates;
}

plansys2::DerivedGraph
DomainExpertClient::getDerivedPredicatesGraph()
{
  plansys2::DerivedGraph ret;

  while (!get_derived_predicates_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_derived_predicates_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainDerivedPredicateDetails::Request>();

  auto future_result = get_derived_predicates_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  return plansys2::DerivedGraph(future_result.get()->predicates);
}

std::vector<plansys2_msgs::msg::Derived>
DomainExpertClient::getDerivedPredicate(
  const std::string & predicate, const std::vector<std::string> & params)
{
  while (!get_derived_predicate_details_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_derived_predicate_details_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainDerivedPredicateDetails::Request>();

  request->predicate = predicate;

  auto future_result = get_derived_predicate_details_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  auto result = *future_result.get();


  if (result.success) {
    return result.predicates;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_derived_predicate_details_client_->get_service_name() << ": " <<
        result.error_info);
    return {};
  }
}

std::vector<std::string>
DomainExpertClient::getActions()
{
  std::vector<std::string> ret;

  while (!get_actions_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_actions_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainActions::Request>();

  auto future_result = get_actions_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  auto result = *future_result.get();

  for (size_t i = 0; i < result.actions.size(); i++) {
    ret.push_back(result.actions[i]);
  }

  return ret;
}

plansys2_msgs::msg::Action::SharedPtr
DomainExpertClient::getAction(
  const std::string & action,
  const std::vector<std::string> & params)
{
  while (!get_action_details_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_action_details_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainActionDetails::Request>();

  request->action = action;
  request->parameters = params;

  auto future_result = get_action_details_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  auto result = *future_result.get();


  if (result.success) {
    return std::make_shared<plansys2_msgs::msg::Action>(result.action);
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_action_details_client_->get_service_name() << ": " <<
        result.error_info);
    return {};
  }
}

std::vector<std::string>
DomainExpertClient::getDurativeActions()
{
  std::vector<std::string> ret;

  while (!get_durative_actions_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_durative_actions_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainActions::Request>();

  auto future_result = get_durative_actions_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  auto result = *future_result.get();

  for (size_t i = 0; i < result.actions.size(); i++) {
    ret.push_back(result.actions[i]);
  }

  return ret;
}

plansys2_msgs::msg::DurativeAction::SharedPtr
DomainExpertClient::getDurativeAction(
  const std::string & action,
  const std::vector<std::string> & params)
{
  while (!get_durative_action_details_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return nullptr;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_durative_action_details_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomainDurativeActionDetails::Request>();

  request->durative_action = action;
  request->parameters = params;

  auto future_result = get_durative_action_details_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return nullptr;
  }

  auto result = *future_result.get();

  if (result.success) {
    return std::make_shared<plansys2_msgs::msg::DurativeAction>(
      result.durative_action);
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_durative_action_details_client_->get_service_name() << ": " <<
        result.error_info);
    return nullptr;
  }
}

std::string
DomainExpertClient::getDomain()
{
  std::string ret;

  while (!get_domain_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_domain_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::srv::GetDomain::Request>();

  auto future_result = get_domain_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  auto result = *future_result.get();

  ret = result.domain;

  return ret;
}
}  // namespace plansys2
