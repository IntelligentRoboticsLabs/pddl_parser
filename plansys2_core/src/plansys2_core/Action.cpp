// Copyright 2024 Intelligent Robotics Lab
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

#include "plansys2_core/Action.hpp"

namespace plansys2 {

  std::string ActionVariant::get_action_string() const
  {
    std::string action_string;
    if (std::holds_alternative<plansys2::Action>(*action_)) {
      action_string = parser::pddl::nameActionsToString(
        std::make_shared<plansys2::Action>(std::get<plansys2::Action>(*action_)));
    } else if (std::holds_alternative<plansys2::DurativeAction>(*action_)) {
      action_string = parser::pddl::nameActionsToString(
        std::make_shared<plansys2::DurativeAction>(std::get<plansys2::DurativeAction>(*action_)));
    }
    return action_string;
  }

  std::string ActionVariant::get_action_name() const
  {
    std::string action_name;
    if (std::holds_alternative<plansys2::Action>(*action_)) {
      action_name = std::get<plansys2::Action>(*action_).name;
    } else if (std::holds_alternative<plansys2::DurativeAction>(*action_)) {
      action_name = std::get<plansys2::DurativeAction>(*action_).name;
    }
    return action_name;
  }

  std::vector<plansys2_msgs::msg::Param> ActionVariant::get_action_params() const
  {
    std::vector<plansys2_msgs::msg::Param> params;
    if (std::holds_alternative<plansys2::Action>(*action_)) {
      params = std::get<plansys2::Action>(*action_).parameters;
    } else if (std::holds_alternative<plansys2::DurativeAction>(*action_)) {
      params = std::get<plansys2::DurativeAction>(*action_).parameters;
    }
    return params;
  }

  plansys2_msgs::msg::Tree ActionVariant::get_overall_requirements() const
  {
    plansys2_msgs::msg::Tree reqs;
    if (std::holds_alternative<plansys2::Action>(*action_)) {
      reqs = std::get<plansys2::Action>(*action_).preconditions;
    } else if (std::holds_alternative<plansys2::DurativeAction>(*action_)) {
      reqs = std::get<plansys2::DurativeAction>(*action_).over_all_requirements;
    }
    return reqs;
  }

  plansys2_msgs::msg::Tree ActionVariant::get_at_start_requirements() const
  {
    plansys2_msgs::msg::Tree reqs;
    if (std::holds_alternative<plansys2::DurativeAction>(*action_)) {
      reqs = std::get<plansys2::DurativeAction>(*action_).at_start_requirements;
    }
    return reqs;
  }

  plansys2_msgs::msg::Tree ActionVariant::get_at_end_requirements() const
  {
    plansys2_msgs::msg::Tree reqs;
    if (std::holds_alternative<plansys2::DurativeAction>(*action_)) {
      reqs = std::get<plansys2::DurativeAction>(*action_).at_end_requirements;
    }
    return reqs;
  }

  plansys2_msgs::msg::Tree ActionVariant::get_at_start_effects() const
  {
    plansys2_msgs::msg::Tree effects;
    if (std::holds_alternative<plansys2::DurativeAction>(*action_)) {
      effects = std::get<plansys2::DurativeAction>(*action_).at_start_effects;
    }
    return effects;
  }

  plansys2_msgs::msg::Tree ActionVariant::get_at_end_effects() const
  {
    plansys2_msgs::msg::Tree effects;
    if (std::holds_alternative<plansys2::Action>(*action_)) {
      effects = std::get<plansys2::Action>(*action_).effects;
    } else if (std::holds_alternative<plansys2::DurativeAction>(*action_)) {
      effects = std::get<plansys2::DurativeAction>(*action_).at_end_effects;
    }
    return effects;
  }

}  // namespace plansys2
