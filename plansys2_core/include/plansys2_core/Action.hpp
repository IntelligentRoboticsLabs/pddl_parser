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
  DurativeAction(const plansys2_msgs::msg::DurativeAction & action)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::DurativeAction(action) {}

  bool operator==(const DurativeAction & action) const
  {
    return parser::pddl::checkDurativeActionEquality(*this, action);
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

namespace plansys2 {

class ActionVariant
{
public:
  using ActionVariantType = std::variant<plansys2::Action, plansys2::DurativeAction>;

  ActionVariant() {}

  template <typename ActionT>
  ActionVariant(ActionT action) : action_(std::make_shared<ActionVariantType>(action)) {}

  template <typename ActionT>
  ActionVariant & operator=(ActionT ptr)
  {
    action_ = std::make_shared<ActionVariantType>(ptr);
    return *this;
  }

  template <typename ActionT>
  ActionVariant & operator=(std::shared_ptr<ActionT> ptr)
  {
    action_ = std::make_shared<ActionVariantType>(*ptr);
    return *this;
  }

  bool operator==(const ActionVariant& other) const {
    return *action_ == *other.action_;
  }

  size_t hash() const {
    return std::visit([](auto&& arg) { return std::hash<std::decay_t<decltype(arg)>>{}(arg); }, *action_);
  }

  std::string get_action_string() const
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

  std::string get_action_name() const
  {
    std::string action_name;
    if (std::holds_alternative<plansys2::Action>(*action_)) {
      action_name = std::get<plansys2::Action>(*action_).name;
    } else if (std::holds_alternative<plansys2::DurativeAction>(*action_)) {
      action_name = std::get<plansys2::DurativeAction>(*action_).name;
    }
    return action_name;
  }

  std::vector<plansys2_msgs::msg::Param> get_action_params() const
  {
    std::vector<plansys2_msgs::msg::Param> params;
    if (std::holds_alternative<plansys2::Action>(*action_)) {
      params = std::get<plansys2::Action>(*action_).parameters;
    } else if (std::holds_alternative<plansys2::DurativeAction>(*action_)) {
      params = std::get<plansys2::DurativeAction>(*action_).parameters;
    }
    return params;
  }

  plansys2_msgs::msg::Tree get_overall_requirements() const
  {
    plansys2_msgs::msg::Tree reqs;
    if (std::holds_alternative<plansys2::Action>(*action_)) {
      reqs = std::get<plansys2::Action>(*action_).preconditions;
    } else if (std::holds_alternative<plansys2::DurativeAction>(*action_)) {
      reqs = std::get<plansys2::DurativeAction>(*action_).over_all_requirements;
    }
    return reqs;
  }

  plansys2_msgs::msg::Tree get_at_start_requirements() const
  {
    plansys2_msgs::msg::Tree reqs;
    if (std::holds_alternative<plansys2::DurativeAction>(*action_)) {
      reqs = std::get<plansys2::DurativeAction>(*action_).at_start_requirements;
    }
    return reqs;
  }

  plansys2_msgs::msg::Tree get_at_end_requirements() const
  {
    plansys2_msgs::msg::Tree reqs;
    if (std::holds_alternative<plansys2::DurativeAction>(*action_)) {
      reqs = std::get<plansys2::DurativeAction>(*action_).at_end_requirements;
    }
    return reqs;
  }

  plansys2_msgs::msg::Tree get_at_start_effects() const
  {
    plansys2_msgs::msg::Tree effects;
    if (std::holds_alternative<plansys2::DurativeAction>(*action_)) {
      effects = std::get<plansys2::DurativeAction>(*action_).at_start_effects;
    }
    return effects;
  }

  plansys2_msgs::msg::Tree get_at_end_effects() const
  {
    plansys2_msgs::msg::Tree effects;
    if (std::holds_alternative<plansys2::Action>(*action_)) {
      effects = std::get<plansys2::Action>(*action_).effects;
    } else if (std::holds_alternative<plansys2::DurativeAction>(*action_)) {
      effects = std::get<plansys2::DurativeAction>(*action_).at_end_effects;
    }
    return effects;
  }

  bool is_action() const
  {
    return std::holds_alternative<plansys2::Action>(*action_);
  }

  bool is_durative_action() const
  {
    return std::holds_alternative<plansys2::DurativeAction>(*action_);
  }

  bool is_empty() const { return action_->index() == std::variant_npos;}

private:
  std::shared_ptr<ActionVariantType> action_;
};
}  // namespace plansys2

namespace std
{
  template<>
  struct hash<plansys2::ActionVariant> {
    std::size_t operator()(const plansys2::ActionVariant& av) const noexcept {
      return av.hash();
    }
  };
}  // namespace std

#endif  // PLANSYS2_CORE__ACTION_HPP_
