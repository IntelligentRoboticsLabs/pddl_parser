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

#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "plansys2_executor/ComputeBT.hpp"

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

ComputeBT::ComputeBT()
: rclcpp_lifecycle::LifecycleNode("compute_bt"),
  bt_builder_loader_("plansys2_executor", "plansys2::BTBuilder")
{
  using namespace std::placeholders;

  this->declare_parameter<std::string>("action_bt_xml_filename", "");
  this->declare_parameter<std::string>("start_action_bt_xml_filename", "");
  this->declare_parameter<std::string>("end_action_bt_xml_filename", "");
  this->declare_parameter<std::string>("bt_builder_plugin", "");
  this->declare_parameter<std::string>("domain", "");
  this->declare_parameter<std::string>("problem", "");
  this->declare_parameter<int>("action_time_precision", 3);
  this->declare_parameter<bool>("enable_dotgraph_legend", true);
  this->declare_parameter<bool>("print_graph", true);
  this->declare_parameter("action_timeouts.actions", std::vector<std::string>{});
  // Declaring individual action parameters so they can be queried on the command line
  auto action_timeouts_actions = this->get_parameter("action_timeouts.actions").as_string_array();
  for (auto action : action_timeouts_actions) {
    this->declare_parameter<double>(
      "action_timeouts." + action + ".duration_overrun_percentage",
      0.0);
  }

  compute_bt_srv_ = create_service<std_srvs::srv::Trigger>(
    "compute_bt",
    std::bind(
      &ComputeBT::computeBTCallback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
ComputeBT::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Configuring...", get_name());

  auto action_bt_xml_filename =
    this->get_parameter("action_bt_xml_filename").as_string();
  if (action_bt_xml_filename.empty()) {
    action_bt_xml_filename =
      ament_index_cpp::get_package_share_directory("plansys2_executor") +
      "/behavior_trees/plansys2_action_bt.xml";
  }

  std::ifstream action_bt_ifs(action_bt_xml_filename);
  if (!action_bt_ifs) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error openning [" << action_bt_xml_filename << "]");
    return CallbackReturnT::FAILURE;
  }

  action_bt_xml_.assign(
    std::istreambuf_iterator<char>(action_bt_ifs), std::istreambuf_iterator<char>());

  auto start_action_bt_xml_filename =
    this->get_parameter("start_action_bt_xml_filename").as_string();
  if (start_action_bt_xml_filename.empty()) {
    start_action_bt_xml_filename =
      ament_index_cpp::get_package_share_directory("plansys2_executor") +
      "/behavior_trees/plansys2_start_action_bt.xml";
  }

  std::ifstream start_action_bt_ifs(start_action_bt_xml_filename);
  if (!start_action_bt_ifs) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Error openning [" << start_action_bt_xml_filename << "]");
    return CallbackReturnT::FAILURE;
  }

  start_action_bt_xml_.assign(
    std::istreambuf_iterator<char>(start_action_bt_ifs), std::istreambuf_iterator<char>());

  auto end_action_bt_xml_filename =
    this->get_parameter("end_action_bt_xml_filename").as_string();
  if (end_action_bt_xml_filename.empty()) {
    end_action_bt_xml_filename =
      ament_index_cpp::get_package_share_directory("plansys2_executor") +
      "/behavior_trees/plansys2_end_action_bt.xml";
  }

  std::ifstream end_action_bt_ifs(end_action_bt_xml_filename);
  if (!end_action_bt_ifs) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Error openning [" << end_action_bt_xml_filename << "]");
    return CallbackReturnT::FAILURE;
  }

  end_action_bt_xml_.assign(
    std::istreambuf_iterator<char>(end_action_bt_ifs), std::istreambuf_iterator<char>());

  dotgraph_pub_ = this->create_publisher<std_msgs::msg::String>("plan_dotgraph", 1);

  domain_node_ = std::make_shared<plansys2::DomainExpertNode>();
  planner_node_ = std::make_shared<plansys2::PlannerNode>();
  problem_node_ = std::make_shared<plansys2::ProblemExpertNode>();

  domain_client_ = std::make_shared<plansys2::DomainExpertClient>();
  planner_client_ = std::make_shared<plansys2::PlannerClient>();
  problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();

  RCLCPP_INFO(get_logger(), "[%s] Configured", get_name());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ComputeBT::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Activating...", get_name());
  dotgraph_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "[%s] Activated", get_name());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ComputeBT::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Deactivating...", get_name());
  dotgraph_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "[%s] Deactivated", get_name());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ComputeBT::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Cleaning up...", get_name());
  dotgraph_pub_.reset();
  RCLCPP_INFO(get_logger(), "[%s] Cleaned up", get_name());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ComputeBT::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "[%s] Shutting down...", get_name());
  dotgraph_pub_.reset();
  RCLCPP_INFO(get_logger(), "[%s] Shutted down", get_name());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ComputeBT::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_ERROR(get_logger(), "[%s] Error transition", get_name());
  return CallbackReturnT::SUCCESS;
}

void
ComputeBT::computeBTCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  auto domain_filename = this->get_parameter("domain").as_string();
  const std::filesystem::path domain_path{domain_filename};
  if (!std::filesystem::exists(domain_path)) {
    RCLCPP_ERROR(get_logger(), "%s not found!", domain_filename.c_str());
    return;
  }

  auto problem_filename = this->get_parameter("problem").as_string();
  const std::filesystem::path problem_path{problem_filename};
  if (!std::filesystem::exists(problem_path)) {
    RCLCPP_ERROR(get_logger(), "%s not found!", problem_filename.c_str());
    return;
  }

  auto problem_string = getProblem(problem_filename);

  domain_node_->set_parameter({"model_file", domain_filename});
  problem_node_->set_parameter({"model_file", domain_filename});

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node_->get_node_base_interface());
  exe.add_node(problem_node_->get_node_base_interface());
  exe.add_node(planner_node_->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });


  domain_node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  planner_node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  domain_node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  problem_client_->addProblem(problem_string);

  auto domain = domain_client_->getDomain();
  auto problem = problem_client_->getProblem();
  auto plan = planner_client_->getPlan(domain, problem);

  savePlan(plan.value(), problem_path.stem().u8string());

  auto action_map = std::make_shared<std::map<std::string, ActionExecutionInfo>>();
  auto action_timeout_actions = this->get_parameter("action_timeouts.actions").as_string_array();

  for (const auto & plan_item : plan.value().items) {
    auto index = BTBuilder::to_action_id(plan_item, 3);


    (*action_map)[index] = ActionExecutionInfo();
    (*action_map)[index].action_executor =
      ActionExecutor::make_shared(plan_item.action, shared_from_this());

    auto actions = domain_client_->getActions();
    std::string action_name_ = get_action_name(plan_item.action);
    if (std::find(actions.begin(), actions.end(), action_name_) != actions.end()) {
      (*action_map)[index].action_info.action = domain_client_->getAction(
        action_name_, get_action_params(plan_item.action));
    } else {
      (*action_map)[index].action_info.action = domain_client_->getDurativeAction(
        action_name_, get_action_params(plan_item.action));
    }

    (*action_map)[index].duration = plan_item.duration;
    std::string action_name = (*action_map)[index].action_info.get_action_name();
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

  auto bt_builder_plugin = this->get_parameter("bt_builder_plugin").as_string();
  if (bt_builder_plugin.empty()) {
    bt_builder_plugin = "SimpleBTBuilder";
  }
  RCLCPP_INFO(get_logger(), "bt_builder_plugin: %s", bt_builder_plugin.c_str());

  std::shared_ptr<plansys2::BTBuilder> bt_builder;
  try {
    bt_builder = bt_builder_loader_.createSharedInstance("plansys2::" + bt_builder_plugin);
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(get_logger(), "pluginlib error: %s", ex.what());
  }

  if (bt_builder_plugin == "SimpleBTBuilder") {
    bt_builder->initialize(action_bt_xml_);
  } else if (bt_builder_plugin == "STNBTBuilder") {
    auto precision = this->get_parameter("action_time_precision").as_int();
    bt_builder->initialize(start_action_bt_xml_, end_action_bt_xml_, precision);
  }

  auto bt_xml_tree = bt_builder->get_tree(plan.value());
  if (bt_xml_tree.empty()) {
    RCLCPP_ERROR(get_logger(), "Error computing behavior tree!");

    finish = true;
    t.join();
    response->success = false;
    return;
  }

  saveBT(bt_xml_tree, problem_path.stem().u8string());

  auto action_graph = bt_builder->get_graph();
  std_msgs::msg::String dotgraph_msg;
  dotgraph_msg.data = bt_builder->get_dotgraph(
    action_map, this->get_parameter("enable_dotgraph_legend").as_bool(),
    this->get_parameter("print_graph").as_bool());
  dotgraph_pub_->publish(dotgraph_msg);
  saveDotGraph(dotgraph_msg.data, problem_path.stem().u8string());

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

  finish = true;
  t.join();

  response->success = true;
}

std::string
ComputeBT::getProblem(const std::string & filename) const
{
  std::string ret;
  std::ifstream file(filename);
  if (file) {
    std::ostringstream ss;
    ss << file.rdbuf();
    ret = ss.str();
  }

  return ret;
}

void
ComputeBT::savePlan(const plansys2_msgs::msg::Plan & plan, const std::string & filename) const
{
  std::ofstream file(filename + "_plan.pddl");
  if (file.is_open()) {
    for (const auto & item : plan.items) {
      file << item.time << ": " << item.action << "  [" << item.duration << "]\n";
    }
    file.close();
  } else {
    std::cerr << "Unable to open " << filename << "_plan.pddl" << std::endl;
  }
}

void
ComputeBT::saveBT(const std::string & bt_xml, const std::string & filename) const
{
  std::ofstream file(filename + "_bt.xml");
  if (file.is_open()) {
    file << bt_xml;
    file.close();
  } else {
    std::cerr << "Unable to open " << filename << "_bt.xml" << std::endl;
  }
}

void
ComputeBT::saveDotGraph(const std::string & dotgraph, const std::string & filename) const
{
  std::ofstream file(filename + "_graph.dot");
  if (file.is_open()) {
    file << dotgraph << "\n";
    file.close();
  } else {
    std::cerr << "Unable to open " << filename << "_graph.dot" << std::endl;
  }
}

}  // namespace plansys2
