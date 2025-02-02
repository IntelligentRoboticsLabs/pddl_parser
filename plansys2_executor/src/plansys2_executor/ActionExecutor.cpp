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

#include <string>
#include <memory>
#include <vector>

#include "plansys2_pddl_parser/Utils.hpp"

#include "plansys2_executor/ActionExecutor.hpp"

namespace plansys2
{

using std::placeholders::_1;
using namespace std::chrono_literals;

ActionExecutor::ActionExecutor(
  const std::string & action,
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
: node_(node), state_(IDLE), completion_(0.0)
{
  action_hub_pub_ = node_->create_publisher<plansys2_msgs::msg::ActionExecution>(
    "actions_hub", rclcpp::QoS(100).reliable());
  action_hub_sub_ = node_->create_subscription<plansys2_msgs::msg::ActionExecution>(
    "actions_hub", rclcpp::QoS(100).reliable(),
    std::bind(&ActionExecutor::action_hub_callback, this, _1));

  state_time_ = node_->now();

  action_ = action;
  action_name_ = get_name(action);
  action_params_ = get_params(action);
  start_execution_ = node_->now();
  state_time_ = start_execution_;
}

ActionExecutor::~ActionExecutor()
{
  clean_up();
}

void
ActionExecutor::clean_up()
{
  if (action_hub_sub_ != nullptr) {
    action_hub_sub_->clear_on_new_message_callback();
  }
}

void
ActionExecutor::action_hub_callback(plansys2_msgs::msg::ActionExecution::SharedPtr msg)
{
  last_msg_ = msg;

  switch (last_msg_->type) {
    case plansys2_msgs::msg::ActionExecution::REQUEST:
    case plansys2_msgs::msg::ActionExecution::CONFIRM:
    case plansys2_msgs::msg::ActionExecution::REJECT:
    case plansys2_msgs::msg::ActionExecution::CANCEL:
      // These cases have no meaning requester
      break;
    case plansys2_msgs::msg::ActionExecution::RESPONSE:
      if (last_msg_->arguments == action_params_ && last_msg_->action == action_name_) {
        if (state_ == DEALING) {
          confirm_performer(last_msg_->node_id);
          current_performer_id_ = last_msg_->node_id;
          state_ = RUNNING;
          waiting_timer_ = nullptr;
          start_execution_ = node_->now();
          state_time_ = node_->now();
        } else {
          reject_performer(last_msg_->node_id);
        }
      }
      break;
    case plansys2_msgs::msg::ActionExecution::FEEDBACK:
      if (state_ != RUNNING || last_msg_->arguments != action_params_ ||
        last_msg_->action != action_name_ || last_msg_->node_id != current_performer_id_)
      {
        return;
      }
      feedback_ = last_msg_->status;
      completion_ = last_msg_->completion;
      state_time_ = node_->now();

      break;
    case plansys2_msgs::msg::ActionExecution::FINISH:
      if (last_msg_->arguments == action_params_ &&
        last_msg_->action == action_name_ && last_msg_->node_id == current_performer_id_)
      {
        if (last_msg_->success) {
          state_ = SUCCESS;
        } else {
          state_ = FAILURE;
        }

        feedback_ = last_msg_->status;
        completion_ = last_msg_->completion;

        state_time_ = node_->now();

        action_hub_pub_->on_deactivate();
      }
      break;
    default:
      RCLCPP_ERROR(
        node_->get_logger(), "Msg %d type not recognized in %s executor requester",
        last_msg_->type, action_.c_str());
      break;
  }
}

void
ActionExecutor::confirm_performer(const std::string & node_id)
{
  plansys2_msgs::msg::ActionExecution msg;
  msg.type = plansys2_msgs::msg::ActionExecution::CONFIRM;
  msg.node_id = node_id;
  msg.action = action_name_;
  msg.arguments = action_params_;

  action_hub_pub_->publish(msg);
}

void
ActionExecutor::reject_performer(const std::string & node_id)
{
  plansys2_msgs::msg::ActionExecution msg;
  msg.type = plansys2_msgs::msg::ActionExecution::REJECT;
  msg.node_id = node_id;
  msg.action = action_name_;
  msg.arguments = action_params_;

  action_hub_pub_->publish(msg);
}

void
ActionExecutor::request_for_performers()
{
  plansys2_msgs::msg::ActionExecution msg;
  msg.type = plansys2_msgs::msg::ActionExecution::REQUEST;
  msg.node_id = node_->get_name();
  msg.action = action_name_;
  msg.arguments = action_params_;

  action_hub_pub_->publish(msg);
}

BT::NodeStatus
ActionExecutor::get_status()
{
  switch (state_) {
    case IDLE:
      return BT::NodeStatus::IDLE;
      break;
    case DEALING:
    case RUNNING:
      return BT::NodeStatus::RUNNING;
      break;
    case SUCCESS:
      return BT::NodeStatus::SUCCESS;
      break;
    case FAILURE:
      return BT::NodeStatus::FAILURE;
      break;
    default:
      return BT::NodeStatus::IDLE;
      break;
  }
}

bool
ActionExecutor::is_finished()
{
  return state_ == SUCCESS || state_ == FAILURE;
}

BT::NodeStatus
ActionExecutor::tick(const rclcpp::Time & now)
{
  switch (state_) {
    case IDLE:
      state_ = DEALING;
      state_time_ = node_->now();

      action_hub_pub_->on_activate();

      completion_ = 0.0;
      feedback_ = "";

      request_for_performers();
      waiting_timer_ = node_->create_wall_timer(
        1s, std::bind(&ActionExecutor::wait_timeout, this));
      break;
    case DEALING:
      {
        auto time_since_dealing = (node_->now() - state_time_).seconds();
        if (time_since_dealing > 30.0) {
          RCLCPP_ERROR(
            node_->get_logger(),
            "Aborting %s. Timeout after requesting for 30 seconds", action_.c_str());
          state_ = FAILURE;
        }
      }
      break;

    case RUNNING:
      break;
    case SUCCESS:
    case FAILURE:
    case CANCELLED:
      break;
    default:
      break;
  }

  return get_status();
}

void
ActionExecutor::cancel()
{
  state_ = CANCELLED;
  plansys2_msgs::msg::ActionExecution msg;
  msg.type = plansys2_msgs::msg::ActionExecution::CANCEL;
  msg.node_id = current_performer_id_;
  msg.action = action_name_;
  msg.arguments = action_params_;

  action_hub_pub_->publish(msg);
}

std::string
ActionExecutor::get_name(const std::string & action_expr)
{
  std::string working_action_expr = parser::pddl::getReducedString(action_expr);
  working_action_expr.erase(0, 1);  // remove initial (
  working_action_expr.pop_back();  // remove last )

  size_t delim = working_action_expr.find(" ");

  return working_action_expr.substr(0, delim);
}

std::vector<std::string>
ActionExecutor::get_params(const std::string & action_expr)
{
  std::vector<std::string> ret;

  std::string working_action_expr = parser::pddl::getReducedString(action_expr);
  working_action_expr.erase(0, 1);  // remove initial (
  working_action_expr.pop_back();  // remove last )

  size_t delim = working_action_expr.find(" ");

  working_action_expr = working_action_expr.substr(delim + 1);

  size_t start = 0, end = 0;
  while (end != std::string::npos) {
    end = working_action_expr.find(" ", start);
    auto param = working_action_expr.substr(
      start, (end == std::string::npos) ? std::string::npos : end - start);
    ret.push_back(param);
    start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
  }

  return ret;
}

void
ActionExecutor::wait_timeout()
{
  RCLCPP_WARN(node_->get_logger(), "No action performer for %s. retrying", action_.c_str());
  request_for_performers();
}

}  // namespace plansys2
