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

#ifndef PLANSYS2_CORE__ACTION_HPP_
#define PLANSYS2_CORE__ACTION_HPP_

#include "plansys2_core/Types.hpp"

#include "plansys2_msgs/msg/action.hpp"
#include "plansys2_msgs/msg/durative_action.hpp"

namespace plansys2 {

class Action : public plansys2_msgs::msg::Action
{
public:
  Action()
  : plansys2_msgs::msg::Action() {}
  // explicit Action(const std::string & action)
  // : plansys2_msgs::msg::Action(parser::pddl::fromStringAction(action)) {}
  Action(const plansys2_msgs::msg::Action & action)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Action(action) {}

  bool operator==(const Action & action) const
  {
    return parser::pddl::checkActionEquality(*this, action);
  }
};

class DurativeAction : public plansys2_msgs::msg::DurativeAction
{
public:
  DurativeAction()
  : plansys2_msgs::msg::DurativeAction() {}
  // explicit DurativeAction(const std::string & action)
  // : plansys2_msgs::msg::DurativeAction(parser::pddl::fromStringDurativeAction(action)) {}
  DurativeAction(const plansys2_msgs::msg::DurativeAction & action)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::DurativeAction(action) {}

  bool operator==(const DurativeAction & action) const
  {
    return parser::pddl::checkDurativeActionEquality(*this, action);
  }
};

class ActionVariant
{
public:
  using shared_ptr_action = std::shared_ptr<plansys2_msgs::msg::Action>;
  using shared_ptr_durative = std::shared_ptr<plansys2_msgs::msg::DurativeAction>;

  std::variant<
    std::shared_ptr<plansys2_msgs::msg::Action>,
    std::shared_ptr<plansys2_msgs::msg::DurativeAction>> action;

  ActionVariant & operator=(shared_ptr_action ptr)
  {
    action = ptr;
    return *this;
  }

  ActionVariant & operator=(shared_ptr_durative ptr)
  {
    action = ptr;
    return *this;
  }

  std::string get_action_string() const
  {
    std::string action_string;
    if (std::holds_alternative<shared_ptr_action>(action)) {
      action_string = parser::pddl::nameActionsToString(
        std::get<shared_ptr_action>(action));
    } else if (std::holds_alternative<shared_ptr_durative>(action)) {
      action_string = parser::pddl::nameActionsToString(
        std::get<shared_ptr_durative>(action));
    }
    return action_string;
  }

  std::string get_action_name() const
  {
    std::string action_name;
    if (std::holds_alternative<shared_ptr_action>(action)) {
      action_name = std::get<shared_ptr_action>(action)->name;
    } else if (std::holds_alternative<shared_ptr_durative>(action)) {
      action_name = std::get<shared_ptr_durative>(action)->name;
    }
    return action_name;
  }

  std::vector<plansys2_msgs::msg::Param> get_action_params() const
  {
    std::vector<plansys2_msgs::msg::Param> params;
    if (std::holds_alternative<shared_ptr_action>(action)) {
      params = std::get<shared_ptr_action>(action)->parameters;
    } else if (std::holds_alternative<shared_ptr_durative>(action)) {
      params = std::get<shared_ptr_durative>(action)->parameters;
    }
    return params;
  }

  plansys2_msgs::msg::Tree get_overall_requirements() const
  {
    plansys2_msgs::msg::Tree reqs;
    if (std::holds_alternative<shared_ptr_action>(action)) {
      reqs = std::get<shared_ptr_action>(action)->preconditions;
    } else if (std::holds_alternative<shared_ptr_durative>(action)) {
      reqs = std::get<shared_ptr_durative>(action)->over_all_requirements;
    }
    return reqs;
  }

  plansys2_msgs::msg::Tree get_at_start_requirements() const
  {
    plansys2_msgs::msg::Tree reqs;
    if (std::holds_alternative<shared_ptr_durative>(action)) {
      reqs = std::get<shared_ptr_durative>(action)->at_start_requirements;
    }
    return reqs;
  }

  plansys2_msgs::msg::Tree get_at_end_requirements() const
  {
    plansys2_msgs::msg::Tree reqs;
    if (std::holds_alternative<shared_ptr_durative>(action)) {
      reqs = std::get<shared_ptr_durative>(action)->at_end_requirements;
    }
    return reqs;
  }

  plansys2_msgs::msg::Tree get_at_start_effects() const
  {
    plansys2_msgs::msg::Tree effects;
    if (std::holds_alternative<shared_ptr_durative>(action)) {
      effects = std::get<shared_ptr_durative>(action)->at_start_effects;
    }
    return effects;
  }

  plansys2_msgs::msg::Tree get_at_end_effects() const
  {
    plansys2_msgs::msg::Tree effects;
    if (std::holds_alternative<shared_ptr_action>(action)) {
      effects = std::get<shared_ptr_action>(action)->effects;
    } else if (std::holds_alternative<shared_ptr_durative>(action)) {
      effects = std::get<shared_ptr_durative>(action)->at_end_effects;
    }
    return effects;
  }

  bool is_action() const
  {
    return std::holds_alternative<shared_ptr_action>(action);
  }

  bool is_durative_action() const
  {
    return std::holds_alternative<shared_ptr_durative>(action);
  }
};

}  // namespace plansys2

namespace std
{
  template<>
  struct hash<plansys2::Action>
  {
    std::size_t operator()(const plansys2::Action & action) const noexcept
    {
      std::size_t seed = 0;
      hash_combine(seed, action.name);
      hash_combine(seed, action.parameters.size());

      for (const auto & param : action.parameters) {
        hash_combine(seed, param.name);
      }

      for (auto node : action.preconditions.nodes)
      {
        hash_combine(seed, node);
      }
      for (auto node : action.effects.nodes)
      {
        hash_combine(seed, node);
      }

      return seed;
    }
  };

  template<>
  struct hash<plansys2::DurativeAction>
  {
    std::size_t operator()(const plansys2::DurativeAction & action) const noexcept
    {
      std::size_t seed = 0;
      hash_combine(seed, action.name);
      hash_combine(seed, action.parameters.size());

      for (const auto & param : action.parameters) {
        hash_combine(seed, param.name);
      }

      for (auto node : action.at_start_requirements.nodes)
      {
        hash_combine(seed, node);
      }
      for (auto node : action.over_all_requirements.nodes)
      {
        hash_combine(seed, node);
      }
      for (auto node : action.at_end_requirements.nodes)
      {
        hash_combine(seed, node);
      }
      for (auto node : action.at_start_effects.nodes)
      {
        hash_combine(seed, node);
      }
      for (auto node : action.at_end_effects.nodes)
      {
        hash_combine(seed, node);
      }

      return seed;
    }
  };
}  // namespace std

#endif  // PLANSYS2_CORE__ACTION_HPP_
