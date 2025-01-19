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

#ifndef PLANSYS2_PLANNER__PLANNERCLIENT_HPP_
#define PLANSYS2_PLANNER__PLANNERCLIENT_HPP_

#include <optional>
#include <string>
#include <vector>

#include "plansys2_planner/PlannerInterface.hpp"

#include "plansys2_msgs/srv/get_plan.hpp"
#include "plansys2_msgs/srv/get_plan_array.hpp"

#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

class PlannerClient : public PlannerInterface
{
public:
  PlannerClient();

  std::optional<plansys2_msgs::msg::Plan> getPlan(
    const std::string & domain, const std::string & problem,
    const std::string & node_namespace = "");

  plansys2_msgs::msg::PlanArray getPlanArray(
    const std::string & domain, const std::string & problem,
    const std::string & node_namespace = "");

private:
  rclcpp::Client<plansys2_msgs::srv::GetPlan>::SharedPtr
    get_plan_client_;
  rclcpp::Client<plansys2_msgs::srv::GetPlanArray>::SharedPtr
    get_plan_array_client_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Duration solver_timeout_ = rclcpp::Duration(15, 0);
};

}  // namespace plansys2

#endif  // PLANSYS2_PLANNER__PLANNERCLIENT_HPP_
