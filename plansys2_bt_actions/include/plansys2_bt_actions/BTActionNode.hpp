// Copyright (c) 2018 Intel Corporation
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

#ifndef PLANSYS2_BT_ACTIONS__BTACTIONNODE_HPP_
#define PLANSYS2_BT_ACTIONS__BTACTIONNODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace plansys2
{

using namespace std::chrono_literals;  // NOLINT

template<class ActionT>
class BtActionNode : public BT::ActionNodeBase
{
public:
  BtActionNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfig & conf)
  : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name)
  {
    if (!config().blackboard->get("node", node_)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get 'node' from the blackboard");
    }

    // Get the required items from the blackboard
    server_timeout_ = 5s;

    // Initialize the input and output messages
    goal_ = typename ActionT::Goal();
    result_ = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult();

    std::string remapped_action_name;
    if (getInput("server_name", remapped_action_name)) {
      action_name_ = remapped_action_name;
    }

    // Give the derive class a chance to do any initialization
    RCLCPP_INFO(node_->get_logger(), "\"%s\" BtActionNode initialized", xml_tag_name.c_str());
  }

  BtActionNode() = delete;

  virtual ~BtActionNode()
  {
  }

  // Create instance of an action server
  bool createActionClient(const std::string & action_name)
  {
    // Now that we have the ROS node to use, create the action client for this BT action
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name);

    // Make sure the server is actually there before continuing
    RCLCPP_INFO(node_->get_logger(), "Waiting for \"%s\" action server", action_name.c_str());

    bool success_waiting = action_client_->wait_for_action_server(server_timeout_);

    if (!success_waiting) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Timeout (%ld secs) waiting for \"%s\" action server",
        server_timeout_.count() * 1000,
        action_name.c_str());
    }

    return success_waiting;
  }

  // Any subclass of BtActionNode that accepts parameters must provide a providedPorts method
  // and call providedBasicPorts in it.
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("server_name", "Action server name"),
      BT::InputPort<double>(
        "server_timeout",
        5.0,
        "The amount of time to wait for a response from the action server, in seconds")
    };
    // The user defined ports are added to the basic ports
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  // Derived classes can override any of the following methods to hook into the
  // processing for the action: on_tick, and on_success

  // Could do dynamic checks, such as getting updates to values on the blackboard
  // Can also update variable goal_updated_ to request a new goal
  virtual BT::NodeStatus on_tick()
  {
    return BT::NodeStatus::RUNNING;
  }

  // Provides the opportunity for derived classes to log feedback, update the
  // goal, or cancel the goal
  virtual void on_feedback(
    const std::shared_ptr<const typename ActionT::Feedback> feedback)
  {
    (void)feedback;
  }

  // Called upon successful completion of the action. A derived class can override this
  // method to put a value on the blackboard, for example.
  virtual BT::NodeStatus on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

  // Called when a the action is aborted. By default, the node will return FAILURE.
  // The user may override it to return another value, instead.
  virtual BT::NodeStatus on_aborted()
  {
    return BT::NodeStatus::FAILURE;
  }

  // Called when a the action is cancelled. By default, the node will return SUCCESS.
  // The user may override it to return another value, instead.
  virtual BT::NodeStatus on_cancelled()
  {
    return BT::NodeStatus::SUCCESS;
  }

  // The main override required by a BT action
  BT::NodeStatus tick() override
  {
    switch (state_) {
      case IDLE:
        {
          RCLCPP_DEBUG(node_->get_logger(), "%s IDLE", node_->get_name());
          assert((status() == BT::NodeStatus::IDLE));

          double server_timeout = 5.0;
          if (!getInput("server_timeout", server_timeout)) {
            RCLCPP_INFO(
              node_->get_logger(),
              "Missing input port [server_timeout], "
              "using default value of 5s");
          }
          server_timeout_ = std::chrono::milliseconds(static_cast<int>(server_timeout * 1000.0));

          if (!createActionClient(action_name_)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create action client");
            return BT::NodeStatus::FAILURE;
          }

          // User defined tick
          auto user_status = on_tick();
          if (user_status != BT::NodeStatus::RUNNING) {
            return user_status;
          }

          on_new_goal_received();

          state_ = GOAL_SENT;

          return BT::NodeStatus::RUNNING;
        }
        break;

      case GOAL_SENT:
        {
          RCLCPP_DEBUG(node_->get_logger(), "%s GOAL_SENT", node_->get_name());
          if (future_goal_handle_.valid()) {
            goal_handle_ = future_goal_handle_.get();

            if (!goal_handle_) {
              RCLCPP_ERROR(
                node_->get_logger(),
                "Goal was rejected by action server %s",
                action_name_.c_str());
              state_ = GOAL_FAILURE;
              return BT::NodeStatus::FAILURE;
            } else {
              state_ = GOAL_EXECUTING;
              return BT::NodeStatus::RUNNING;
            }
          } else {
            if ((node_->now() - goal_sent_ts_) > server_timeout_) {
              RCLCPP_ERROR(
                node_->get_logger(),
                "Failed to send goal to action server %s",
                action_name_.c_str());
              state_ = GOAL_FAILURE;
              return BT::NodeStatus::FAILURE;
            } else {
              return BT::NodeStatus::RUNNING;
            }
          }
        }
        break;

      case GOAL_EXECUTING:
        {
          RCLCPP_DEBUG(node_->get_logger(), "%s GOAL_EXECUTING", node_->get_name());
          auto goal_status = goal_handle_->get_status();
          auto user_status = on_tick();

          if (user_status != BT::NodeStatus::RUNNING) {
            cancel_goal();
            state_ = GOAL_CANCELLING;
            return user_status;
          }

          if (goal_updated_ && (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
            goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED))
          {
            on_new_goal_received();
            state_ = GOAL_SENT;
          }

          if (goal_result_available_) {
            state_ = GOAL_FINISHING;
          }

          return BT::NodeStatus::RUNNING;
        }
        break;

      case GOAL_FINISHING:
        {
          RCLCPP_DEBUG(node_->get_logger(), "%s GOAL_FINISHING", node_->get_name());
          switch (result_.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              state_ = GOAL_FINISHED;
              return on_success();

            case rclcpp_action::ResultCode::ABORTED:
              state_ = GOAL_FINISHED;
              return on_aborted();

            case rclcpp_action::ResultCode::CANCELED:
              state_ = GOAL_FINISHED;
              return on_cancelled();

            default:
              throw std::logic_error("BtActionNode::Tick: invalid status value");
          }
        }
        break;

      case GOAL_CANCELLING:
        {
          RCLCPP_DEBUG(node_->get_logger(), "%s GOAL_CANCELLING", node_->get_name());
          if (future_cancer_handle_.valid()) {
            state_ = GOAL_FINISHED;
            return BT::NodeStatus::SUCCESS;
          } else {
            RCLCPP_ERROR(
              node_->get_logger(),
              "Failed to cancel action server for %s", action_name_.c_str());
            state_ = GOAL_FAILURE;
            return BT::NodeStatus::FAILURE;
          }
        }
        break;

      case GOAL_FINISHED:
        {
          RCLCPP_DEBUG(node_->get_logger(), "%s GOAL_FINISHED", node_->get_name());
          return BT::NodeStatus::SUCCESS;
        }
        break;

      case GOAL_FAILURE:
        {
          RCLCPP_DEBUG(node_->get_logger(), "%s GOAL_FAILURE", node_->get_name());
          return BT::NodeStatus::FAILURE;
        }
        break;

      default:
        break;
    }

    return BT::NodeStatus::RUNNING;
  }

  // The other (optional) override required by a BT action. In this case, we
  // make sure to cancel the ROS2 action if it is still running.
  void halt() override
  {
    if (should_cancel_goal()) {
      cancel_goal();
    }

    resetStatus();
  }

protected:
  void cancel_goal()
  {
    future_cancer_handle_ = action_client_->async_cancel_goal(goal_handle_);
  }

  bool should_cancel_goal()
  {
    // Shut the node down if it is currently running
    if (status() != BT::NodeStatus::RUNNING) {
      return false;
    }

    auto status = goal_handle_->get_status();

    // Check if the goal is still executing
    return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
           status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  }


  void on_new_goal_received()
  {
    goal_result_available_ = false;
    auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    send_goal_options.result_callback =
      [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result) {
        // TODO(#1652): a work around until rcl_action interface is updated
        // if goal ids are not matched, the older goal call this callback so ignore the result
        // if matched, it must be processed (including aborted)
        if (this->goal_handle_->get_goal_id() == result.goal_id) {
          goal_result_available_ = true;
          result_ = result;
        }
      };
    send_goal_options.feedback_callback =
      [this](typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
      const std::shared_ptr<const typename ActionT::Feedback> feedback) {
        on_feedback(feedback);
      };

    RCLCPP_INFO(
      node_->get_logger(),
      "Sending goal to action server %s",
      action_name_.c_str());

    future_goal_handle_ = action_client_->async_send_goal(goal_, send_goal_options);
    goal_sent_ts_ = node_->now();
  }

  void increment_recovery_count()
  {
    int recovery_count = 0;
    config().blackboard->get<int>("number_recoveries", recovery_count);  // NOLINT
    recovery_count += 1;
    config().blackboard->set<int>("number_recoveries", recovery_count);  // NOLINT
  }

  std::string action_name_;
  typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;

  // All ROS2 actions have a goal and a result
  typename ActionT::Goal goal_;
  bool goal_updated_{false};
  bool goal_result_available_{false};
  std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>
  future_goal_handle_;
  std::shared_future<typename ActionT::Impl::CancelGoalService::Response::SharedPtr>
  future_cancer_handle_;
  rclcpp::Time goal_sent_ts_;
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;

  // The node that will be used for any ROS operations
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  // The timeout value while waiting for response from a server when a
  // new action goal is sent or canceled
  std::chrono::milliseconds server_timeout_;

  static const int IDLE = 0;
  static const int GOAL_SENT = 1;
  static const int GOAL_EXECUTING = 2;
  static const int GOAL_FINISHING = 3;
  static const int GOAL_CANCELLING = 4;
  static const int GOAL_FINISHED = 5;
  static const int GOAL_FAILURE = 6;

  int state_ {IDLE};
};


}  // namespace plansys2

#endif  // PLANSYS2_BT_ACTIONS__BTACTIONNODE_HPP_
