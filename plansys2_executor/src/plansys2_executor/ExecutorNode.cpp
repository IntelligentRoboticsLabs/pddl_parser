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

#include <filesystem>

#include <algorithm>
#include <string>
#include <memory>
#include <iostream>
#include <fstream>
#include <map>
#include <set>
#include <vector>

#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/BTBuilder.hpp"
#include "plansys2_problem_expert/Utils.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/blackboard.h"

#include "plansys2_executor/behavior_tree/execute_action_node.hpp"
#include "plansys2_executor/behavior_tree/wait_action_node.hpp"
#include "plansys2_executor/behavior_tree/check_action_node.hpp"
#include "plansys2_executor/behavior_tree/wait_atstart_req_node.hpp"
#include "plansys2_executor/behavior_tree/check_overall_req_node.hpp"
#include "plansys2_executor/behavior_tree/check_atend_req_node.hpp"
#include "plansys2_executor/behavior_tree/check_timeout_node.hpp"
#include "plansys2_executor/behavior_tree/apply_atstart_effect_node.hpp"
#include "plansys2_executor/behavior_tree/apply_atend_effect_node.hpp"

namespace plansys2
{

using ExecutePlan = plansys2_msgs::action::ExecutePlan;
using namespace std::chrono_literals;

ExecutorNode::ExecutorNode()
: rclcpp_lifecycle::LifecycleNode("executor"),
  bt_builder_loader_("plansys2_executor", "plansys2::bt_builder::BTBuilder")
{
  using namespace std::placeholders;

  this->declare_parameter<std::string>("default_action_bt_xml_filename", "");
  this->declare_parameter<std::string>("default_start_action_bt_xml_filename", "");
  this->declare_parameter<std::string>("default_end_action_bt_xml_filename", "");
  this->declare_parameter<std::string>("bt_builder_plugin", "");
  this->declare_parameter<int>("action_time_precision", 3);
  this->declare_parameter<bool>("enable_dotgraph_legend", true);
  this->declare_parameter<bool>("print_graph", false);
  this->declare_parameter("action_timeouts.actions", std::vector<std::string>{});
  // Declaring individual action parameters so they can be queried on the command line
  auto action_timeouts_actions = this->get_parameter("action_timeouts.actions").as_string_array();
  for (auto action : action_timeouts_actions) {
    this->declare_parameter<double>(
      "action_timeouts." + action + ".duration_overrun_percentage",
      0.0);
  }

  execute_plan_action_server_ = rclcpp_action::create_server<ExecutePlan>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "execute_plan",
    std::bind(&ExecutorNode::handle_goal, this, _1, _2),
    std::bind(&ExecutorNode::handle_cancel, this, _1),
    std::bind(&ExecutorNode::handle_accepted, this, _1));

  get_ordered_sub_goals_service_ = create_service<plansys2_msgs::srv::GetOrderedSubGoals>(
    "executor/get_ordered_sub_goals",
    std::bind(
      &ExecutorNode::get_ordered_sub_goals_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  get_plan_service_ = create_service<plansys2_msgs::srv::GetPlan>(
    "executor/get_plan",
    std::bind(
      &ExecutorNode::get_plan_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));
}


using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
ExecutorNode::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Configuring...", get_name());

  auto default_action_bt_xml_filename =
    this->get_parameter("default_action_bt_xml_filename").as_string();
  if (default_action_bt_xml_filename.empty()) {
    default_action_bt_xml_filename =
      ament_index_cpp::get_package_share_directory("plansys2_executor") +
      "/behavior_trees/plansys2_action_bt.xml";
  }

  std::ifstream action_bt_ifs(default_action_bt_xml_filename);
  if (!action_bt_ifs) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error openning [" << default_action_bt_xml_filename << "]");
    return CallbackReturnT::FAILURE;
  }

  action_bt_xml_.assign(
    std::istreambuf_iterator<char>(action_bt_ifs), std::istreambuf_iterator<char>());

  auto default_start_action_bt_xml_filename =
    this->get_parameter("default_start_action_bt_xml_filename").as_string();
  if (default_start_action_bt_xml_filename.empty()) {
    default_start_action_bt_xml_filename =
      ament_index_cpp::get_package_share_directory("plansys2_executor") +
      "/behavior_trees/plansys2_start_action_bt.xml";
  }

  std::ifstream start_action_bt_ifs(default_start_action_bt_xml_filename);
  if (!start_action_bt_ifs) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Error openning [" << default_start_action_bt_xml_filename << "]");
    return CallbackReturnT::FAILURE;
  }

  start_action_bt_xml_.assign(
    std::istreambuf_iterator<char>(start_action_bt_ifs), std::istreambuf_iterator<char>());

  auto default_end_action_bt_xml_filename =
    this->get_parameter("default_end_action_bt_xml_filename").as_string();
  if (default_end_action_bt_xml_filename.empty()) {
    default_end_action_bt_xml_filename =
      ament_index_cpp::get_package_share_directory("plansys2_executor") +
      "/behavior_trees/plansys2_end_action_bt.xml";
  }

  std::ifstream end_action_bt_ifs(default_end_action_bt_xml_filename);
  if (!end_action_bt_ifs) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Error openning [" << default_end_action_bt_xml_filename << "]");
    return CallbackReturnT::FAILURE;
  }

  end_action_bt_xml_.assign(
    std::istreambuf_iterator<char>(end_action_bt_ifs), std::istreambuf_iterator<char>());

  dotgraph_pub_ = this->create_publisher<std_msgs::msg::String>("dot_graph", 1);
  execution_info_pub_ = create_publisher<plansys2_msgs::msg::ActionExecutionInfo>(
    "action_execution_info", 100);
  executing_plan_pub_ = create_publisher<plansys2_msgs::msg::Plan>(
    "executing_plan", rclcpp::QoS(100).transient_local());

  domain_client_ = std::make_shared<plansys2::DomainExpertClient>();
  problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();
  planner_client_ = std::make_shared<plansys2::PlannerClient>();

  RCLCPP_INFO(get_logger(), "[%s] Configured", get_name());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ExecutorNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Activating...", get_name());
  dotgraph_pub_->on_activate();
  execution_info_pub_->on_activate();
  executing_plan_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "[%s] Activated", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ExecutorNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Deactivating...", get_name());
  dotgraph_pub_->on_deactivate();
  executing_plan_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "[%s] Deactivated", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ExecutorNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Cleaning up...", get_name());
  dotgraph_pub_.reset();
  executing_plan_pub_.reset();
  RCLCPP_INFO(get_logger(), "[%s] Cleaned up", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ExecutorNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Shutting down...", get_name());
  dotgraph_pub_.reset();
  executing_plan_pub_.reset();
  RCLCPP_INFO(get_logger(), "[%s] Shutted down", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ExecutorNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_ERROR(get_logger(), "[%s] Error transition", get_name());

  return CallbackReturnT::SUCCESS;
}

void
ExecutorNode::get_ordered_sub_goals_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetOrderedSubGoals::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetOrderedSubGoals::Response> response)
{
  if (ordered_sub_goals_.has_value()) {
    response->sub_goals = ordered_sub_goals_.value();
    response->success = true;
  } else {
    response->success = false;
    response->error_info = "No current plan.";
  }
}

std::optional<std::vector<plansys2_msgs::msg::Tree>>
ExecutorNode::getOrderedSubGoals()
{
  if (!current_plan_.has_value()) {
    return {};
  }

  auto goal = problem_client_->getGoal();
  auto local_state = problem_client_->getState();

  std::vector<plansys2_msgs::msg::Tree> ordered_goals;
  std::vector<uint32_t> unordered_subgoals = parser::pddl::getSubtreeIds(goal);

  // just in case some goals are already satisfied
  for (auto it = unordered_subgoals.begin(); it != unordered_subgoals.end(); ) {
    if (plansys2::check(goal, local_state, *it)) {
      plansys2_msgs::msg::Tree new_goal;
      parser::pddl::fromString(new_goal, "(and " + parser::pddl::toString(goal, (*it)) + ")");
      ordered_goals.push_back(new_goal);
      it = unordered_subgoals.erase(it);
    } else {
      ++it;
    }
  }

  for (const auto & plan_item : current_plan_.value().items) {
    auto actions = domain_client_->getActions();
    std::string action_name = get_action_name(plan_item.action);
    if (std::find(actions.begin(), actions.end(), action_name) != actions.end()) {
      std::shared_ptr<plansys2_msgs::msg::Action> action =
        domain_client_->getAction(
        action_name, get_action_params(plan_item.action));
      plansys2::apply(action->effects, local_state);
    } else {
      std::shared_ptr<plansys2_msgs::msg::DurativeAction> action =
        domain_client_->getDurativeAction(
        action_name, get_action_params(plan_item.action));
      plansys2::apply(action->at_start_effects, local_state);
      plansys2::apply(action->at_end_effects, local_state);
    }


    for (auto it = unordered_subgoals.begin(); it != unordered_subgoals.end(); ) {
      if (plansys2::check(goal, local_state, *it)) {
        plansys2_msgs::msg::Tree new_goal;
        parser::pddl::fromString(new_goal, "(and " + parser::pddl::toString(goal, (*it)) + ")");
        ordered_goals.push_back(new_goal);
        it = unordered_subgoals.erase(it);
      } else {
        ++it;
      }
    }
  }

  return ordered_goals;
}

void
ExecutorNode::get_plan_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetPlan::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetPlan::Response> response)
{
  if (current_plan_) {
    response->success = true;
    response->plan = current_plan_.value();
  } else {
    response->success = false;
    response->error_info = "Plan not available";
  }
}

rclcpp_action::GoalResponse
ExecutorNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ExecutePlan::Goal> goal)
{
  RCLCPP_DEBUG(this->get_logger(), "Received goal request with order");

  current_plan_ = {};
  ordered_sub_goals_ = {};

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ExecutorNode::handle_cancel(
  const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  RCLCPP_DEBUG(this->get_logger(), "Received request to cancel goal");

  cancel_plan_requested_ = true;

  return rclcpp_action::CancelResponse::ACCEPT;
}

void
ExecutorNode::execute(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  auto feedback = std::make_shared<ExecutePlan::Feedback>();
  auto result = std::make_shared<ExecutePlan::Result>();

  cancel_plan_requested_ = false;

  current_plan_ = goal_handle->get_goal()->plan;

  if (!current_plan_.has_value()) {
    RCLCPP_ERROR(get_logger(), "No plan found");
    result->success = false;
    goal_handle->succeed(result);

    // Publish void plan
    executing_plan_pub_->publish(plansys2_msgs::msg::Plan());
    return;
  }

  executing_plan_pub_->publish(current_plan_.value());

  auto action_map = std::make_shared<std::map<std::string, ActionExecutionInfo>>();
  auto action_timeout_actions = this->get_parameter("action_timeouts.actions").as_string_array();

  (*action_map)[":0"] = ActionExecutionInfo();
  (*action_map)[":0"].action_executor = ActionExecutor::make_shared("(INIT)", shared_from_this());
  (*action_map)[":0"].action_executor->set_internal_status(ActionExecutor::Status::SUCCESS);
  (*action_map)[":0"].at_start_effects_applied = true;
  (*action_map)[":0"].at_end_effects_applied = true;

  for (const auto & plan_item : current_plan_.value().items) {
    auto index = plansys2::bt_builder::BTBuilder::to_action_id(plan_item, 3);


    (*action_map)[index] = ActionExecutionInfo();
    (*action_map)[index].action_executor =
      ActionExecutor::make_shared(plan_item.action, shared_from_this());

    auto actions = domain_client_->getActions();
    std::string action_name_ = get_action_name(plan_item.action);
    if (std::find(actions.begin(), actions.end(), action_name_) != actions.end()) {
      (*action_map)[index].action_info = domain_client_->getAction(
        action_name_, get_action_params(plan_item.action));
    } else {
      (*action_map)[index].action_info = domain_client_->getDurativeAction(
        action_name_, get_action_params(plan_item.action));
    }

    std::string action_name = (*action_map)[index].action_info.get_action_name();
    (*action_map)[index].duration = plan_item.duration;
    if (std::find(
        action_timeout_actions.begin(), action_timeout_actions.end(),
        action_name) != action_timeout_actions.end() &&
      this->has_parameter("action_timeouts." + action_name + ".duration_overrun_percentage"))
    {
      (*action_map)[index].duration_overrun_percentage = this->get_parameter(
        "action_timeouts." + action_name + ".duration_overrun_percentage").as_double();
    }
    RCLCPP_INFO(
      get_logger(), "Action %s timeout percentage %f", action_name.c_str(),
      (*action_map)[index].duration_overrun_percentage);
  }

  ordered_sub_goals_ = getOrderedSubGoals();

  auto bt_builder_plugin = this->get_parameter("bt_builder_plugin").as_string();
  if (bt_builder_plugin.empty()) {
    bt_builder_plugin = "SimpleBTBuilder";
  }

  std::shared_ptr<plansys2::bt_builder::BTBuilder> bt_builder;
  try {
    bt_builder = bt_builder_loader_.createSharedInstance("plansys2::bt_builder::" + bt_builder_plugin);
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(get_logger(), "pluginlib error: %s", ex.what());
  }

  if (bt_builder_plugin == "SimpleBTBuilder") {
    bt_builder->initialize(action_bt_xml_);
  } else if (bt_builder_plugin == "STNBTBuilder") {
    bt_builder_plugin = "SimpleBTBuilder";
    bt_builder->initialize(action_bt_xml_);
    RCLCPP_WARN(get_logger(), "STN disabled until fixed. Using SimpleBTBuilder instead");
    // auto precision = this->get_parameter("action_time_precision").as_int();
    // bt_builder->initialize(start_action_bt_xml_, end_action_bt_xml_, precision);
  }

  auto bt_xml_tree = bt_builder->get_tree(current_plan_.value());
  if (bt_xml_tree.empty()) {
    RCLCPP_ERROR(get_logger(), "Error computing behavior tree!");

    result->success = false;
    goal_handle->succeed(result);

    // Publish void plan
    executing_plan_pub_->publish(plansys2_msgs::msg::Plan());
    return;
  }

  auto action_graph = bt_builder->get_graph();
  std_msgs::msg::String dotgraph_msg;
  dotgraph_msg.data = bt_builder->get_dotgraph(
    action_map, this->get_parameter("enable_dotgraph_legend").as_bool(),
    this->get_parameter("print_graph").as_bool());
  dotgraph_pub_->publish(dotgraph_msg);

  std::filesystem::path tp = std::filesystem::temp_directory_path();
  std::ofstream out(std::string("/tmp/") + get_namespace() + "/bt.xml");
  out << bt_xml_tree;
  out.close();

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ExecuteAction>("ExecuteAction");
  factory.registerNodeType<WaitAction>("WaitAction");
  factory.registerNodeType<CheckAction>("CheckAction");
  factory.registerNodeType<CheckOverAllReq>("CheckOverAllReq");
  factory.registerNodeType<WaitAtStartReq>("WaitAtStartReq");
  factory.registerNodeType<CheckAtEndReq>("CheckAtEndReq");
  factory.registerNodeType<ApplyAtStartEffect>("ApplyAtStartEffect");
  factory.registerNodeType<ApplyAtEndEffect>("ApplyAtEndEffect");
  factory.registerNodeType<CheckTimeout>("CheckTimeout");

  (*action_map)[":0"].at_start_effects_applied_time = now();
  (*action_map)[":0"].at_end_effects_applied_time = now();

  auto blackboard = BT::Blackboard::create();
  blackboard->set("action_map", action_map);
  blackboard->set("action_graph", action_graph);
  blackboard->set("node", shared_from_this());
  blackboard->set("domain_client", domain_client_);
  blackboard->set("problem_client", problem_client_);
  blackboard->set("bt_builder", bt_builder);

  auto tree = factory.createTreeFromText(bt_xml_tree, blackboard);

  auto info_pub = create_wall_timer(
    1s, [this, &action_map]() {
      auto msgs = get_feedback_info(action_map);
      for (const auto & msg : msgs) {
        execution_info_pub_->publish(msg);
      }
    });

  rclcpp::Rate rate(10);
  auto status = BT::NodeStatus::RUNNING;

  while (status == BT::NodeStatus::RUNNING && !cancel_plan_requested_) {
    try {
      status = tree.tickOnce();
    } catch (std::exception & e) {
      std::cerr << e.what() << std::endl;
      status = BT::NodeStatus::FAILURE;
    }

    feedback->action_execution_status = get_feedback_info(action_map);
    goal_handle->publish_feedback(feedback);

    dotgraph_msg.data = bt_builder->get_dotgraph(
      action_map, this->get_parameter("enable_dotgraph_legend").as_bool());
    dotgraph_pub_->publish(dotgraph_msg);

    rate.sleep();
  }

  if (cancel_plan_requested_) {
    tree.haltTree();
  }

  if (status == BT::NodeStatus::FAILURE) {
    tree.haltTree();
    RCLCPP_ERROR(get_logger(), "Executor BT finished with FAILURE state");
  }

  dotgraph_msg.data = bt_builder->get_dotgraph(
    action_map, this->get_parameter("enable_dotgraph_legend").as_bool());
  dotgraph_pub_->publish(dotgraph_msg);

  result->success = status == BT::NodeStatus::SUCCESS;
  result->action_execution_status = get_feedback_info(action_map);

  size_t i = 0;
  while (i < result->action_execution_status.size() && result->success) {
    if (result->action_execution_status[i].status !=
      plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED)
    {
      result->success = false;
    }
    i++;
  }

  if (rclcpp::ok()) {
    if (cancel_plan_requested_) {
      goal_handle->canceled(result);
    } else {
      goal_handle->succeed(result);
    }
    if (result->success) {
      RCLCPP_INFO(this->get_logger(), "Plan Succeeded");
    } else {
      RCLCPP_INFO(this->get_logger(), "Plan Failed");
    }
  }
}

void
ExecutorNode::handle_accepted(const std::shared_ptr<GoalHandleExecutePlan> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&ExecutorNode::execute, this, _1), goal_handle}.detach();
}

std::vector<plansys2_msgs::msg::ActionExecutionInfo>
ExecutorNode::get_feedback_info(
  std::shared_ptr<std::map<std::string,
  ActionExecutionInfo>> action_map)
{
  std::vector<plansys2_msgs::msg::ActionExecutionInfo> ret;

  if (!action_map) {
    return ret;
  }

  for (const auto & action : *action_map) {
    if (!action.second.action_executor) {
      RCLCPP_WARN(
        get_logger(), "Action executor does not exist for %s. Skipping", action.first.c_str());
      continue;
    }

    plansys2_msgs::msg::ActionExecutionInfo info;
    switch (action.second.action_executor->get_internal_status()) {
      case ActionExecutor::IDLE:
      case ActionExecutor::DEALING:
        info.status = plansys2_msgs::msg::ActionExecutionInfo::NOT_EXECUTED;
        break;
      case ActionExecutor::RUNNING:
        info.status = plansys2_msgs::msg::ActionExecutionInfo::EXECUTING;
        break;
      case ActionExecutor::SUCCESS:
        info.status = plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED;
        break;
      case ActionExecutor::FAILURE:
        info.status = plansys2_msgs::msg::ActionExecutionInfo::FAILED;
        break;
      case ActionExecutor::CANCELLED:
        info.status = plansys2_msgs::msg::ActionExecutionInfo::CANCELLED;
        break;
    }

    info.action_full_name = action.first;

    info.start_stamp = action.second.action_executor->get_start_time();
    info.status_stamp = action.second.action_executor->get_status_time();
    info.action = action.second.action_executor->get_action_name();

    info.arguments = action.second.action_executor->get_action_params();
    info.duration = rclcpp::Duration::from_seconds(action.second.duration);
    info.completion = action.second.action_executor->get_completion();
    info.message_status = action.second.action_executor->get_feedback();

    ret.push_back(info);
  }

  return ret;
}

void
ExecutorNode::print_execution_info(
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> exec_info)
{
  fprintf(stderr, "Execution info =====================\n");

  for (const auto & action_info : *exec_info) {
    fprintf(stderr, "[%s]", action_info.first.c_str());
    switch (action_info.second.action_executor->get_internal_status()) {
      case ActionExecutor::IDLE:
        fprintf(stderr, "\tIDLE\n");
        break;
      case ActionExecutor::DEALING:
        fprintf(stderr, "\tDEALING\n");
        break;
      case ActionExecutor::RUNNING:
        fprintf(stderr, "\tRUNNING\n");
        break;
      case ActionExecutor::SUCCESS:
        fprintf(stderr, "\tSUCCESS\n");
        break;
      case ActionExecutor::FAILURE:
        fprintf(stderr, "\tFAILURE\n");
        break;
    }
    if (action_info.second.action_info.is_empty()) {
      fprintf(stderr, "\tWith no action info\n");
    }

    if (action_info.second.at_start_effects_applied) {
      fprintf(stderr, "\tAt start effects applied\n");
    } else {
      fprintf(stderr, "\tAt start effects NOT applied\n");
    }

    if (action_info.second.at_end_effects_applied) {
      fprintf(stderr, "\tAt end effects applied\n");
    } else {
      fprintf(stderr, "\tAt end effects NOT applied\n");
    }
  }
}

}  // namespace plansys2
