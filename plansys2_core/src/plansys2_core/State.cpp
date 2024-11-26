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

#include "plansys2_core/State.hpp"

namespace plansys2
{
State::State(
  const std::unordered_set<plansys2::Instance> & instances,
  const std::unordered_set<plansys2::Function> & functions,
  const std::unordered_set<plansys2::Predicate> & predicates,
  const std::unordered_set<plansys2::Predicate> & inferred_predicates)
: instances_(instances),
  functions_(functions),
  predicates_(predicates),
  inferred_predicates_(inferred_predicates)
{
  for (const auto & p : inferred_predicates_) {
    inferred_predicates_map_[p.name].insert(p);
  }
  inferred_predicates_.insert(predicates_.begin(), predicates_.end());
}

State::State(
  const std::unordered_set<plansys2::Instance> & instances,
  const std::unordered_set<plansys2::Function> & functions,
  const std::unordered_set<plansys2::Predicate> & predicates,
  const std::unordered_set<plansys2::Predicate> & inferred_predicates,
  const plansys2::DerivedGraph & derived_predicates)
: instances_(instances),
  functions_(functions),
  predicates_(predicates),
  inferred_predicates_(inferred_predicates),
  derived_predicates_(derived_predicates)
{
  for (const auto & p : inferred_predicates_) {
    inferred_predicates_map_[p.name].insert(p);
  }
  inferred_predicates_.insert(predicates_.begin(), predicates_.end());
}

State::State(const plansys2_msgs::msg::State & state)
{
  instances_ = plansys2::convertVectorToUnorderedSet<plansys2::Instance, plansys2_msgs::msg::Param>(
    state.instances);
  functions_ = plansys2::convertVectorToUnorderedSet<plansys2::Function, plansys2_msgs::msg::Node>(
    state.functions);
  predicates_ =
    plansys2::convertVectorToUnorderedSet<plansys2::Predicate, plansys2_msgs::msg::Node>(
    state.predicates);
  inferred_predicates_ =
    plansys2::convertVectorToUnorderedSet<plansys2::Predicate, plansys2_msgs::msg::Node>(
    state.inferred_predicates);
  derived_predicates_ = state.derived_predicates;
}

bool State::operator==(const State & state) const
{
  return this->instances_ == state.instances_ && this->functions_ == state.functions_ &&
         this->predicates_ == state.predicates_ &&
         this->inferred_predicates_ == state.inferred_predicates_ &&
         this->derived_predicates_ == state.derived_predicates_;
}

size_t State::getNumberInferredFromDerived(const plansys2::Derived & derived) const
{
  auto it = inferred_predicates_map_.find(derived.predicate.name);
  if (it == inferred_predicates_map_.end()) {
    return 0;
  }
  return it->second.size();
}

bool State::addInferredPredicate(const plansys2::Predicate & predicate)
{
  auto res = inferred_predicates_.emplace(predicate);
  if (res.second) {
    inferred_predicates_map_[predicate.name].insert(predicate);
  }
  return res.second;
}

bool State::addInferredPredicate(plansys2::Predicate && predicate)
{
  auto moved_predicate = std::move(predicate);
  auto res = inferred_predicates_.emplace(moved_predicate);
  if (res.second) {
    inferred_predicates_map_[moved_predicate.name].insert(moved_predicate);
  }
  return res.second;
}

void State::clearPredicates()
{
  predicates_.clear();
  inferred_predicates_.clear();
  inferred_predicates_map_.clear();
}

void State::clearState()
{
  instances_.clear();
  functions_.clear();
  predicates_.clear();
  inferred_predicates_.clear();
  derived_predicates_.clear();
  inferred_predicates_map_.clear();
}

void State::resetInferredPredicates()
{
  inferred_predicates_ = predicates_;
  inferred_predicates_map_.clear();
}

void State::initInferredPredicates()
{
  inferred_predicates_ = predicates_;
  inferred_predicates_map_.clear();
}

void State::ungroundSingleDerivedPredicate(const plansys2::Derived & derived)
{
  auto it_map = inferred_predicates_map_.find(derived.predicate.name);
  if (it_map == inferred_predicates_map_.end()) {
    return;
  }
  for (const auto & predicate : it_map->second) {
    inferred_predicates_.erase(predicate);
  }
  it_map->second.clear();
}

void State::ungroundDerivedPredicate(const plansys2::Derived & derived)
{
  derived_predicates_.depthFirstTraverse(
    derived, [&](const plansys2::NodeVariant & node) {
      if (node.isDerived()) {
        ungroundSingleDerivedPredicate(node.getDerivedNode());
      }
    });
}

plansys2_msgs::msg::State State::getAsMsg()
{
  plansys2_msgs::msg::State state;
  state.instances =
    plansys2::convertUnorderedSetToVector<plansys2_msgs::msg::Param, plansys2::Instance>(
    instances_);
  state.functions =
    plansys2::convertUnorderedSetToVector<plansys2_msgs::msg::Node, plansys2::Function>(functions_);
  state.predicates =
    plansys2::convertUnorderedSetToVector<plansys2_msgs::msg::Node, plansys2::Predicate>(
    predicates_);
  state.inferred_predicates =
    plansys2::convertUnorderedSetToVector<plansys2_msgs::msg::Node, plansys2::Predicate>(
    inferred_predicates_);
  state.derived_predicates =
    plansys2::convertUnorderedSetToVector<plansys2_msgs::msg::Derived, plansys2::Derived>(
    derived_predicates_.getDerivedPredicates());
  return state;
}

void State::addActionsAndPruneDerived(const std::vector<plansys2::ActionVariant> & actions)
{
  derived_predicates_.appendActions(actions);
  derived_predicates_ = derived_predicates_.pruneGraphToActions(actions);
}

void State::pruneDerivedPredicatesToActions(const std::vector<plansys2::ActionVariant> & actions)
{
  derived_predicates_ = derived_predicates_.pruneGraphToActions(actions);
}

}  // namespace plansys2
