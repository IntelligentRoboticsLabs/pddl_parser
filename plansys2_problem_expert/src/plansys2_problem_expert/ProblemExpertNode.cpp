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

#include "plansys2_problem_expert/ProblemExpertNode.hpp"

#include <string>
#include <memory>
#include <vector>

#include "plansys2_pddl_parser/Utils.hpp"

std::vector<std::string> tokenize(const std::string & string, const std::string & delim)
{
  std::string::size_type lastPos = 0, pos = string.find_first_of(delim, lastPos);
  std::vector<std::string> tokens;

  while (lastPos != std::string::npos) {
    if (pos != lastPos) {
      tokens.push_back(string.substr(lastPos, pos - lastPos));
    }
    lastPos = pos;
    if (lastPos == std::string::npos || lastPos + 1 == string.length()) {
      break;
    }
    pos = string.find_first_of(delim, ++lastPos);
  }

  return tokens;
}

namespace plansys2
{

ProblemExpertNode::ProblemExpertNode()
: rclcpp_lifecycle::LifecycleNode("problem_expert")
{
  declare_parameter("model_file", "");
  declare_parameter("problem_file", "");

  add_problem_service_ = create_service<plansys2_msgs::srv::AddProblem>(
    "problem_expert/add_problem",
    std::bind(
      &ProblemExpertNode::add_problem_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  add_problem_goal_service_ = create_service<plansys2_msgs::srv::AddProblemGoal>(
    "problem_expert/add_problem_goal",
    std::bind(
      &ProblemExpertNode::add_problem_goal_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  add_problem_instance_service_ = create_service<plansys2_msgs::srv::AffectParam>(
    "problem_expert/add_problem_instance",
    std::bind(
      &ProblemExpertNode::add_problem_instance_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  add_problem_predicate_service_ = create_service<plansys2_msgs::srv::AffectNode>(
    "problem_expert/add_problem_predicate",
    std::bind(
      &ProblemExpertNode::add_problem_predicate_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  add_problem_function_service_ = create_service<plansys2_msgs::srv::AffectNode>(
    "problem_expert/add_problem_function",
    std::bind(
      &ProblemExpertNode::add_problem_function_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_problem_goal_service_ = create_service<plansys2_msgs::srv::GetProblemGoal>(
    "problem_expert/get_problem_goal",
    std::bind(
      &ProblemExpertNode::get_problem_goal_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_problem_instance_details_service_ =
    create_service<plansys2_msgs::srv::GetProblemInstanceDetails>(
    "problem_expert/get_problem_instance",
    std::bind(
      &ProblemExpertNode::get_problem_instance_details_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_problem_instances_service_ = create_service<plansys2_msgs::srv::GetProblemInstances>(
    "problem_expert/get_problem_instances",
    std::bind(
      &ProblemExpertNode::get_problem_instances_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_problem_predicate_details_service_ =
    create_service<plansys2_msgs::srv::GetNodeDetails>(
    "problem_expert/get_problem_predicate", std::bind(
      &ProblemExpertNode::get_problem_predicate_details_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_problem_predicates_service_ = create_service<plansys2_msgs::srv::GetStates>(
    "problem_expert/get_problem_predicates",
    std::bind(
      &ProblemExpertNode::get_problem_predicates_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_problem_function_details_service_ =
    create_service<plansys2_msgs::srv::GetNodeDetails>(
    "problem_expert/get_problem_function", std::bind(
      &ProblemExpertNode::get_problem_function_details_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_problem_functions_service_ = create_service<plansys2_msgs::srv::GetStates>(
    "problem_expert/get_problem_functions",
    std::bind(
      &ProblemExpertNode::get_problem_functions_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_problem_service_ = create_service<plansys2_msgs::srv::GetProblem>(
    "problem_expert/get_problem", std::bind(
      &ProblemExpertNode::get_problem_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  is_problem_goal_satisfied_service_ = create_service<plansys2_msgs::srv::IsProblemGoalSatisfied>(
    "problem_expert/is_problem_goal_satisfied", std::bind(
      &ProblemExpertNode::is_problem_goal_satisfied_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  remove_problem_goal_service_ = create_service<plansys2_msgs::srv::RemoveProblemGoal>(
    "problem_expert/remove_problem_goal",
    std::bind(
      &ProblemExpertNode::remove_problem_goal_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  clear_problem_knowledge_service_ = create_service<plansys2_msgs::srv::ClearProblemKnowledge>(
    "problem_expert/clear_problem_knowledge",
    std::bind(
      &ProblemExpertNode::clear_problem_knowledge_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  remove_problem_instance_service_ = create_service<plansys2_msgs::srv::AffectParam>(
    "problem_expert/remove_problem_instance",
    std::bind(
      &ProblemExpertNode::remove_problem_instance_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  remove_problem_predicate_service_ = create_service<plansys2_msgs::srv::AffectNode>(
    "problem_expert/remove_problem_predicate",
    std::bind(
      &ProblemExpertNode::remove_problem_predicate_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  remove_problem_function_service_ = create_service<plansys2_msgs::srv::AffectNode>(
    "problem_expert/remove_problem_function",
    std::bind(
      &ProblemExpertNode::remove_problem_function_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  exist_problem_predicate_service_ = create_service<plansys2_msgs::srv::ExistNode>(
    "problem_expert/exist_problem_predicate",
    std::bind(
      &ProblemExpertNode::exist_problem_predicate_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  exist_problem_function_service_ = create_service<plansys2_msgs::srv::ExistNode>(
    "problem_expert/exist_problem_function",
    std::bind(
      &ProblemExpertNode::exist_problem_function_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  update_problem_function_service_ = create_service<plansys2_msgs::srv::AffectNode>(
    "problem_expert/update_problem_function",
    std::bind(
      &ProblemExpertNode::update_problem_function_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  problem_pub_ = create_publisher<plansys2_msgs::msg::Problem>(
    "problem_expert/problem",
    rclcpp::QoS(100));

  update_pub_ = create_publisher<std_msgs::msg::Empty>(
    "problem_expert/update_notify",
    rclcpp::QoS(100));

  knowledge_pub_ = create_publisher<plansys2_msgs::msg::Knowledge>(
    "problem_expert/knowledge",
    rclcpp::QoS(100).transient_local());
}


using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
ProblemExpertNode::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Configuring...", get_name());

  // (fmrico) Here we could have a discussion if we should read the domain from file or
  // from the domain_expert service. Then, we should configure first domain_expert node
  auto model_file = get_parameter("model_file").get_value<std::string>();

  auto model_files = tokenize(model_file, ":");

  std::ifstream domain_first_ifs(model_files[0]);
  std::string domain_first_str((
      std::istreambuf_iterator<char>(domain_first_ifs)),
    std::istreambuf_iterator<char>());

  auto domain_expert = std::make_shared<DomainExpert>(domain_first_str);

  for (size_t i = 1; i < model_files.size(); i++) {
    std::ifstream domain_ifs(model_files[i]);
    std::string domain_str((
        std::istreambuf_iterator<char>(domain_ifs)),
      std::istreambuf_iterator<char>());
    domain_expert->extendDomain(domain_str);
  }

  problem_expert_ = std::make_shared<ProblemExpert>(domain_expert);

  auto problem_file = get_parameter("problem_file").get_value<std::string>();
  if (!problem_file.empty()) {
    std::ifstream problem_ifs(problem_file);
    std::string problem_str((
        std::istreambuf_iterator<char>(problem_ifs)),
      std::istreambuf_iterator<char>());
    problem_expert_->addProblem(problem_str);
  }

  RCLCPP_INFO(get_logger(), "[%s] Configured", get_name());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ProblemExpertNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Activating...", get_name());
  update_pub_->on_activate();
  knowledge_pub_->on_activate();
  problem_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "[%s] Activated", get_name());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ProblemExpertNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Deactivating...", get_name());
  update_pub_->on_deactivate();
  knowledge_pub_->on_deactivate();
  problem_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "[%s] Deactivated", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ProblemExpertNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Cleaning up...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Cleaned up", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ProblemExpertNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Shutting down...", get_name());
  RCLCPP_INFO(get_logger(), "[%s] Shutted down", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ProblemExpertNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_ERROR(get_logger(), "[%s] Error transition", get_name());

  return CallbackReturnT::SUCCESS;
}

void
ProblemExpertNode::add_problem_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::AddProblem::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AddProblem::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    RCLCPP_INFO(get_logger(), "Adding problem:\n%s", request->problem.c_str());
    response->success = problem_expert_->addProblem(request->problem);

    if (response->success) {
      update_pub_->publish(std_msgs::msg::Empty());
      knowledge_pub_->publish(*get_knowledge_as_msg());

      plansys2_msgs::msg::Problem problem_msg;
      problem_msg.problem = problem_expert_->getProblem();
      problem_msg.stamp = now();
      problem_pub_->publish(problem_msg);
    } else {
      response->error_info = "Problem not valid";
    }
  }
}

void
ProblemExpertNode::add_problem_goal_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::AddProblemGoal::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AddProblemGoal::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    if (!parser::pddl::empty(request->tree)) {
      response->success = problem_expert_->setGoal(request->tree);
      if (response->success) {
        update_pub_->publish(std_msgs::msg::Empty());
        knowledge_pub_->publish(*get_knowledge_as_msg());

        plansys2_msgs::msg::Problem problem_msg;
        problem_msg.problem = problem_expert_->getProblem();
        problem_msg.stamp = now();
        problem_pub_->publish(problem_msg);
      } else {
        response->error_info = "Goal not valid";
      }
    } else {
      response->success = false;
      response->error_info = "Malformed expression";
    }
  }
}

void
ProblemExpertNode::add_problem_instance_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::AffectParam::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AffectParam::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = problem_expert_->addInstance(request->param);
    if (response->success) {
      update_pub_->publish(std_msgs::msg::Empty());
      knowledge_pub_->publish(*get_knowledge_as_msg());

      plansys2_msgs::msg::Problem problem_msg;
      problem_msg.problem = problem_expert_->getProblem();
      problem_msg.stamp = now();
      problem_pub_->publish(problem_msg);
    } else {
      response->error_info = "Instance not valid";
    }
  }
}

void
ProblemExpertNode::add_problem_predicate_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = problem_expert_->addPredicate(request->node);
    if (response->success) {
      update_pub_->publish(std_msgs::msg::Empty());
      knowledge_pub_->publish(*get_knowledge_as_msg());

      plansys2_msgs::msg::Problem problem_msg;
      problem_msg.problem = problem_expert_->getProblem();
      problem_msg.stamp = now();
      problem_pub_->publish(problem_msg);
    } else {
      response->error_info =
        "Predicate [" + parser::pddl::toString(request->node) + "] not valid";
    }
  }
}

void
ProblemExpertNode::add_problem_function_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = problem_expert_->addFunction(request->node);
    if (response->success) {
      update_pub_->publish(std_msgs::msg::Empty());
      knowledge_pub_->publish(*get_knowledge_as_msg());

      plansys2_msgs::msg::Problem problem_msg;
      problem_msg.problem = problem_expert_->getProblem();
      problem_msg.stamp = now();
      problem_pub_->publish(problem_msg);
    } else {
      response->error_info =
        "Function [" + parser::pddl::toString(request->node) + "] not valid";
    }
  }
}

void
ProblemExpertNode::get_problem_goal_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemGoal::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemGoal::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = true;
    response->tree = problem_expert_->getGoal();
  }
}

void
ProblemExpertNode::get_problem_instance_details_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemInstanceDetails::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemInstanceDetails::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    auto instance = problem_expert_->getInstance(request->instance);
    if (instance) {
      response->success = true;
      response->instance = instance.value();
    } else {
      response->success = false;
      response->error_info = "Instance not found";
    }
  }
}

void
ProblemExpertNode::get_problem_instances_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemInstances::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblemInstances::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = true;
    response->instances = plansys2::convertVector<plansys2_msgs::msg::Param, plansys2::Instance>(
      problem_expert_->getInstances());
  }
}

void
ProblemExpertNode::get_problem_predicate_details_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    auto predicate = problem_expert_->getPredicate(request->expression);
    if (predicate) {
      response->node = predicate.value();
      response->success = true;
    } else {
      response->success = false;
      response->error_info = "Predicate not found";
    }
  }
}

void
ProblemExpertNode::get_problem_predicates_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetStates::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetStates::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = true;
    response->states = plansys2::convertVector<plansys2_msgs::msg::Node, plansys2::Predicate>(
      problem_expert_->getPredicates());
  }
}

void
ProblemExpertNode::get_problem_function_details_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetNodeDetails::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    auto function = problem_expert_->getFunction(request->expression);
    if (function) {
      response->node = function.value();
      response->success = true;
    } else {
      response->success = false;
      response->error_info = "Function not found";
    }
  }
}

void
ProblemExpertNode::get_problem_functions_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetStates::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetStates::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = true;
    response->states = plansys2::convertVector<plansys2_msgs::msg::Node, plansys2::Function>(
      problem_expert_->getFunctions());
  }
}

void
ProblemExpertNode::get_problem_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetProblem::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetProblem::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = true;
    response->stamp = now();
    response->problem = problem_expert_->getProblem();
  }
}

void
ProblemExpertNode::is_problem_goal_satisfied_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::IsProblemGoalSatisfied::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::IsProblemGoalSatisfied::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = true;
    response->satisfied = problem_expert_->isGoalSatisfied(request->tree);
  }
}

void
ProblemExpertNode::remove_problem_goal_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::RemoveProblemGoal::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::RemoveProblemGoal::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = problem_expert_->clearGoal();

    if (response->success) {
      update_pub_->publish(std_msgs::msg::Empty());
      knowledge_pub_->publish(*get_knowledge_as_msg());

      plansys2_msgs::msg::Problem problem_msg;
      problem_msg.problem = problem_expert_->getProblem();
      problem_msg.stamp = now();
      problem_pub_->publish(problem_msg);
    } else {
      response->error_info = "Error clearing goal";
    }
  }
}

void
ProblemExpertNode::clear_problem_knowledge_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::ClearProblemKnowledge::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::ClearProblemKnowledge::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = problem_expert_->clearKnowledge();

    if (response->success) {
      update_pub_->publish(std_msgs::msg::Empty());
      knowledge_pub_->publish(*get_knowledge_as_msg());

      plansys2_msgs::msg::Problem problem_msg;
      problem_msg.problem = problem_expert_->getProblem();
      problem_msg.stamp = now();
      problem_pub_->publish(problem_msg);
    } else {
      response->error_info = "Error clearing knowledge";
    }
  }
}


void
ProblemExpertNode::remove_problem_instance_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::AffectParam::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AffectParam::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = problem_expert_->removeInstance(request->param);
    if (response->success) {
      update_pub_->publish(std_msgs::msg::Empty());
      knowledge_pub_->publish(*get_knowledge_as_msg());

      plansys2_msgs::msg::Problem problem_msg;
      problem_msg.problem = problem_expert_->getProblem();
      problem_msg.stamp = now();
      problem_pub_->publish(problem_msg);

    } else {
      response->error_info = "Error removing instance";
    }
  }
}

void
ProblemExpertNode::remove_problem_predicate_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = problem_expert_->removePredicate(request->node);
    if (response->success) {
      update_pub_->publish(std_msgs::msg::Empty());
      knowledge_pub_->publish(*get_knowledge_as_msg());

      plansys2_msgs::msg::Problem problem_msg;
      problem_msg.problem = problem_expert_->getProblem();
      problem_msg.stamp = now();
      problem_pub_->publish(problem_msg);
    } else {
      response->error_info = "Error removing predicate";
    }
  }
}

void
ProblemExpertNode::remove_problem_function_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = problem_expert_->removeFunction(request->node);
    if (response->success) {
      update_pub_->publish(std_msgs::msg::Empty());
    } else {
      response->error_info = "Error removing function";
    }
  }
}

void
ProblemExpertNode::exist_problem_predicate_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::ExistNode::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::ExistNode::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->exist = false;
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->exist = problem_expert_->existPredicate(request->node);
  }
}

void
ProblemExpertNode::exist_problem_function_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::ExistNode::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::ExistNode::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->exist = false;
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->exist = problem_expert_->existFunction(request->node);
  }
}

void
ProblemExpertNode::update_problem_function_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::AffectNode::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::AffectNode::Response> response)
{
  if (problem_expert_ == nullptr) {
    response->success = false;
    response->error_info = "Requesting service in non-active state";
    RCLCPP_WARN(get_logger(), "Requesting service in non-active state");
  } else {
    response->success = problem_expert_->updateFunction(request->node);
    if (response->success) {
      update_pub_->publish(std_msgs::msg::Empty());
    } else {
      response->error_info = "Function not valid";
    }
  }
}

plansys2_msgs::msg::Knowledge::SharedPtr
ProblemExpertNode::get_knowledge_as_msg() const
{
  auto ret_msgs = std::make_shared<plansys2_msgs::msg::Knowledge>();

  for (const auto & instance : problem_expert_->getInstances()) {
    ret_msgs->instances.push_back(instance.name);
  }

  for (const auto & predicate : problem_expert_->getPredicates()) {
    ret_msgs->predicates.push_back(parser::pddl::toString(predicate));
  }

  for (const auto & function : problem_expert_->getFunctions()) {
    ret_msgs->functions.push_back(parser::pddl::toString(function));
  }

  auto goal = problem_expert_->getGoal();
  ret_msgs->goal = parser::pddl::toString(goal);

  return ret_msgs;
}

}  // namespace plansys2
