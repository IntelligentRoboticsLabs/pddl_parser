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

#ifndef PLANSYS2_POPF_PLAN_SOLVER__POPF_PLAN_SOLVER_HPP_
#define PLANSYS2_POPF_PLAN_SOLVER__POPF_PLAN_SOLVER_HPP_

#include <filesystem>
#include <optional>
#include <memory>
#include <string>

#include "plansys2_core/PlanSolverBase.hpp"

// using namespace std::chrono_literals;
using std::chrono_literals::operator""s;

namespace plansys2
{

class POPFPlanSolver : public PlanSolverBase
{
private:
  std::string arguments_parameter_name_;
  std::string output_dir_parameter_name_;
  bool cancel_requested_;

public:
  POPFPlanSolver();

  std::optional<std::filesystem::path> create_folders(const std::string & node_namespace);

  void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr, const std::string &) override;

  std::optional<plansys2_msgs::msg::Plan> getPlan(
    const std::string & domain, const std::string & problem,
    const std::string & node_namespace = "",
    const rclcpp::Duration solver_timeout = 15s);

  bool isDomainValid(
    const std::string & domain,
    const std::string & node_namespace = "");

protected:
  std::optional<plansys2_msgs::msg::Plan> parse_plan_result(const std::string & plan_path);
};

}  // namespace plansys2

#endif  // PLANSYS2_POPF_PLAN_SOLVER__POPF_PLAN_SOLVER_HPP_
