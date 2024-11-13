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

#ifndef PLANSYS2_CORE__STATE_HPP_
#define PLANSYS2_CORE__STATE_HPP_

#include "plansys2_core/Types.hpp"
#include "plansys2_core/Graph.hpp"

namespace plansys2
{

class State
{
public:
  State() {}

  State(const std::unordered_set<plansys2::Instance>& instances,
    std::unordered_set<plansys2::Function>& functions,
    const std::unordered_set<plansys2::Predicate>& predicates
  ) : instances_(instances), functions_(functions), predicates_(predicates) {}

  State(const std::unordered_set<plansys2::Instance>& instances,
    const std::unordered_set<plansys2::Function>& functions,
    const std::unordered_set<plansys2::Predicate>& predicates,
    const std::unordered_set<plansys2::Predicate>& inferred_predicates
  ) : instances_(instances), functions_(functions), predicates_(predicates), inferred_predicates_(inferred_predicates) {}

  State(const std::unordered_set<plansys2::Instance>& instances,
    const std::unordered_set<plansys2::Function>& functions,
    const std::unordered_set<plansys2::Predicate>& predicates,
    const std::unordered_set<plansys2::Predicate>& inferred_predicates,
    const plansys2::DerivedGraph& derived_predicates
  ) : instances_(instances), functions_(functions), predicates_(predicates), inferred_predicates_(inferred_predicates), derived_predicates_(derived_predicates) {}

  State(const plansys2_msgs::msg::State &state)
  {
    instances_ = plansys2::convertVectorToUnorderedSet<plansys2::Instance, plansys2_msgs::msg::Param>(
      state.instances);
    functions_ = plansys2::convertVectorToUnorderedSet<plansys2::Function, plansys2_msgs::msg::Node>(
      state.functions);
    predicates_ = plansys2::convertVectorToUnorderedSet<plansys2::Predicate, plansys2_msgs::msg::Node>(
      state.predicates);
    inferred_predicates_ = plansys2::convertVectorToUnorderedSet<plansys2::Predicate, plansys2_msgs::msg::Node>(
      state.inferred_predicates);
    derived_predicates_ = state.derived_predicates;
  }

  bool operator==(const State &state) const
  {
    return this->instances_ == state.instances_ &&
      this->functions_ == state.functions_ &&
      this->predicates_ == state.predicates_ &&
      this->inferred_predicates_ == state.inferred_predicates_ &&
      this->derived_predicates_ == state.derived_predicates_;
  }

  auto &getInstances() const { return instances_; }
  auto &getFunctions() const { return functions_; }
  auto &getPredicates() const { return predicates_; }
  auto &getInferredPredicates() const { return inferred_predicates_.empty() ? predicates_ : inferred_predicates_; }
  auto &getDerivedPredicates() const { return derived_predicates_; }
  auto getDerivedPredicatesDepthFirst() const { return derived_predicates_.getDerivedPredicatesDepthFirst(); }

  auto getInstancesSize() const { return instances_.size(); }
  auto getFunctionsSize() const { return functions_.size(); }
  auto getPredicatesSize() const { return predicates_.size(); }
  auto getInferredPredicatesSize() const { return inferred_predicates_.size(); }
  auto getDerivedPredicatesSize() const { return derived_predicates_.getEdgeNumber(); }

  bool addInstance(const plansys2::Instance& instance) { return instances_.insert(instance).second;}
  bool addInstance(plansys2::Instance&& instance) { return instances_.insert(std::move(instance)).second;}

  bool addPredicate(const plansys2::Predicate& predicate) { return predicates_.insert(predicate).second;}
  bool addPredicate(plansys2::Predicate&& predicate) { return predicates_.insert(std::move(predicate)).second;}

  bool addInferredPredicate(const plansys2::Predicate& predicate) { return inferred_predicates_.emplace(predicate).second;}
  bool addInferredPredicate(plansys2::Predicate&& predicate) { return inferred_predicates_.emplace(std::move(predicate)).second;}
  void addInferredPredicates(std::unordered_set<plansys2::Predicate>&& predicates) { inferred_predicates_.insert(predicates.begin(), predicates.end());}

  bool addFunction(const plansys2::Function& function) { return functions_.insert(function).second;}
  bool addFunction(plansys2::Function&& function) { return functions_.insert(std::move(function)).second;}

  void removeInstance(const std::unordered_set<plansys2::Instance>::const_iterator it) {instances_.erase(it);}
  bool removeInstance(plansys2::Instance instance) {return instances_.erase(instance) == 1;}

  auto removePredicate(const std::unordered_set<plansys2::Predicate>::const_iterator it) {return predicates_.erase(it);}
  bool removePredicate(const plansys2::Predicate& predicate) {return predicates_.erase(predicate) == 1;}
  bool removePredicate(plansys2::Predicate&& predicate) {return predicates_.erase(std::move(predicate)) == 1;}

  auto removeFunction(const std::unordered_set<plansys2::Function>::const_iterator it) {return functions_.erase(it);}
  bool removeFunction(const plansys2::Function& function) {return functions_.erase(function) == 1;}
  bool removeFunction(plansys2::Function&& function) {return functions_.erase(std::move(function)) == 1;}

  bool hasInstance(const plansys2::Instance& instance) {return instances_.find(instance) != instances_.end();}
  bool hasInstance(plansys2::Instance&& instance) {return instances_.find(std::move(instance)) != instances_.end();}

  bool hasPredicate(const plansys2::Predicate& predicate) {return predicates_.find(predicate) != predicates_.end();}
  bool hasPredicate(plansys2::Predicate&& predicate) {return predicates_.find(std::move(predicate)) != predicates_.end();}

  bool hasInferredPredicate(const plansys2::Predicate& predicate) {return inferred_predicates_.find(predicate) != inferred_predicates_.end();}
  bool hasInferredPredicate(plansys2::Predicate&& predicate) {return inferred_predicates_.find(std::move(predicate)) != inferred_predicates_.end();}

  bool hasFunction(const plansys2::Function& function) {return functions_.find(function) != functions_.end();}
  bool hasFunction(plansys2::Function&& function) {return functions_.find(std::move(function)) != functions_.end();}

  auto getInstance(const plansys2::Instance& instance) const {return instances_.find(instance);}
  auto getInstance(plansys2::Instance&& instance) const {return instances_.find(std::move(instance));}

  auto getPredicate(const plansys2::Predicate& predicate) const {return predicates_.find(predicate);}
  auto getPredicate(plansys2::Predicate&& predicate) const {return predicates_.find(std::move(predicate));}

  auto getFunction(const plansys2::Function& function) const {return functions_.find(function);}
  auto getFunction(plansys2::Function&& function) const {return functions_.find(std::move(function));}

  void setInferredPredicates(const std::unordered_set<plansys2::Predicate> & inferred_predicates) {inferred_predicates_ = std::move(inferred_predicates);}
  void setInferredPredicates(std::unordered_set<plansys2::Predicate> && inferred_predicates) {inferred_predicates_ = std::move(inferred_predicates);}
  void setDerivedPredicates(const plansys2::DerivedGraph derived_predicates) {derived_predicates_ = derived_predicates;}

  bool hasDerivedPredicates() {return derived_predicates_.getEdgeNumber() == 0;}

  void reserveInferredPredicates(size_t size) {inferred_predicates_.reserve(size);}

  void clearPredicates() {predicates_.clear(); inferred_predicates_.clear();}
  void clearState() { instances_.clear(); functions_.clear(); predicates_.clear(); inferred_predicates_.clear(); derived_predicates_.clear();}

  void resetInferredPredicates() {inferred_predicates_ = predicates_;}

  plansys2_msgs::msg::State getAsMsg()
  {
    plansys2_msgs::msg::State state;
    state.instances = plansys2::convertUnorderedSetToVector<plansys2_msgs::msg::Param, plansys2::Instance>(
        instances_);
    state.functions = plansys2::convertUnorderedSetToVector<plansys2_msgs::msg::Node, plansys2::Function>(
        functions_);
    state.predicates = plansys2::convertUnorderedSetToVector<plansys2_msgs::msg::Node, plansys2::Predicate>(
        predicates_);
    state.inferred_predicates = plansys2::convertUnorderedSetToVector<plansys2_msgs::msg::Node, plansys2::Predicate>(
        inferred_predicates_);
    state.derived_predicates = derived_predicates_.getDerivedPredicates();
    return state;
  }

private:
  std::unordered_set<plansys2::Instance> instances_;
  std::unordered_set<plansys2::Function> functions_;
  std::unordered_set<plansys2::Predicate> predicates_;
  std::unordered_set<plansys2::Predicate> inferred_predicates_;
  plansys2::DerivedGraph derived_predicates_;

  friend struct std::hash<State>;
};
}  // namespace plansys2

namespace std {
  template <>
  struct hash<plansys2::State> {
    std::size_t operator()(const plansys2::State & state) const {
      std::size_t seed = 0;

      for (const auto & instance : state.instances_) {
        hash_combine(seed, instance);
      }
      for (const auto & function : state.functions_) {
        hash_combine(seed, function);
      }
      for (const auto & predicate : state.predicates_) {
        hash_combine(seed, predicate);
      }
      for (const auto & inferred_predicate : state.inferred_predicates_) {
        hash_combine(seed, inferred_predicate);
      }
      hash_combine(seed, state.derived_predicates_);

      return seed;
    }
  };
}  // namespace std

#endif  // PLANSYS2_CORE__STATE_HPP_
