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

#ifndef PLANSYS2_CORE__TYPES_HPP_
#define PLANSYS2_CORE__TYPES_HPP_

#include <string>
#include <vector>
#include <unordered_set>

#include "plansys2_msgs/msg/derived.hpp"
#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/state.hpp"
#include "plansys2_msgs/msg/tree.hpp"

#include "plansys2_pddl_parser/Utils.hpp"

namespace plansys2
{

template<class toT, class fromT>
std::vector<toT>
convertVector(const std::vector<fromT> & in_vector)
{
  std::vector<toT> ret (in_vector.begin(), in_vector.end());
  return ret;
}

template<class toT, class fromT>
std::unordered_set<toT>
convertVectorToUnorderedSet(const std::vector<fromT> & in_vector)
{
  std::unordered_set<toT> ret (in_vector.begin(), in_vector.end());
  return ret;
}

template<class toT, class fromT>
std::vector<toT>
convertUnorderedSetToVector(const std::unordered_set<fromT> & in_unordered_set)
{
  std::vector<toT> ret (in_unordered_set.begin(), in_unordered_set.end());
  return ret;
}

class Instance : public plansys2_msgs::msg::Param
{
public:
  Instance()
  : plansys2_msgs::msg::Param() {}
  explicit Instance(const std::string & name, const std::string & type = {})
  : plansys2_msgs::msg::Param(parser::pddl::fromStringParam(name, type)) {}
  Instance(const plansys2_msgs::msg::Param & instance)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Param(instance) {}

  bool operator==(const Instance & i2) const
  {
    return parser::pddl::checkParamEquality(*this, i2);
  }
};

class Predicate : public plansys2_msgs::msg::Node
{
public:
  Predicate()
  : plansys2_msgs::msg::Node() {}
  explicit Predicate(const std::string & pred)
  : plansys2_msgs::msg::Node(parser::pddl::fromStringPredicate(pred)) {}
  Predicate(const plansys2_msgs::msg::Node & pred)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Node(pred) {}

  bool operator==(const Predicate & p2) const
  {
    return parser::pddl::checkNodeEquality(*this, p2);
  }
};

class Function : public plansys2_msgs::msg::Node
{
public:
  Function()
  : plansys2_msgs::msg::Node() {}
  explicit Function(const std::string & func)
  : plansys2_msgs::msg::Node(parser::pddl::fromStringFunction(func)) {}
  Function(const plansys2_msgs::msg::Node & func)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Node(func) {}

  bool operator==(const Function & f2) const
  {
    return parser::pddl::checkNodeEquality(*this, f2);
  }
};

class Goal : public plansys2_msgs::msg::Tree
{
public:
  Goal()
  : plansys2_msgs::msg::Tree() {}
  explicit Goal(const std::string & goal)
  : plansys2_msgs::msg::Tree(parser::pddl::fromString(goal)) {}
  Goal(const plansys2_msgs::msg::Tree & goal)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Tree(goal) {}
};

}  // namespace plansys2

namespace std
{
template<>
struct hash<plansys2::Instance>
{
  std::size_t operator()(const plansys2::Instance & inst) const noexcept
  {
    return std::hash<std::string>{}(inst.name);
  }
};

template<>
struct hash<plansys2::Predicate>
{
  std::size_t operator()(const plansys2::Function & func) const noexcept
  {
    std::size_t h1 = std::hash<std::string>{}(func.name);
    std::size_t h2 = 0;

    for (const auto & param : func.parameters) {
      h2 ^= std::hash<std::string>{}(param.name) + 0x9e3779b9 + (h2 << 6) + (h2 >> 2);
    }

    return h1 ^ h2;
  }
};

template<>
struct hash<plansys2::Function>
{
  std::size_t operator()(const plansys2::Predicate & pred) const noexcept
  {
    std::size_t h1 = std::hash<std::string>{}(pred.name);
    std::size_t h2 = 0;

    for (const auto & param : pred.parameters) {
      h2 ^= std::hash<std::string>{}(param.name) + 0x9e3779b9 + (h2 << 6) + (h2 >> 2);
    }

    return h1 ^ h2;
  }
};
}  // namespace std

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
    const std::vector<plansys2_msgs::msg::Derived>& derived_predicates
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

  auto &getInstances() const { return instances_; }
  auto &getFunctions() const { return functions_; }
  auto &getPredicates() const { return predicates_; }
  auto &getInferredPredicates() const { return inferred_predicates_.empty() ? predicates_ : inferred_predicates_; }
  auto &getDerivedPredicates() const { return derived_predicates_; }

  auto getInstancesSize() const { return instances_.size(); }
  auto getFunctionsSize() const { return functions_.size(); }
  auto getPredicatesSize() const { return predicates_.size(); }
  auto getInferredPredicatesSize() const { return inferred_predicates_.size(); }
  auto getDerivedPredicatesSize() const { return derived_predicates_.size(); }

  bool addInstance(const plansys2::Instance& instance) { return instances_.insert(instance).second;}
  bool addInstance(plansys2::Instance&& instance) { return instances_.insert(std::move(instance)).second;}

  bool addPredicate(const plansys2::Predicate& predicate) { return predicates_.insert(predicate).second;}
  bool addPredicate(plansys2::Predicate&& predicate) { return predicates_.insert(std::move(predicate)).second;}

  bool addInferredPredicate(const plansys2::Predicate& predicate) { return inferred_predicates_.insert(predicate).second;}
  bool addInferredPredicate(plansys2::Predicate&& predicate) { return inferred_predicates_.insert(std::move(predicate)).second;}

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
  void setDerivedPredicates(const std::vector<plansys2_msgs::msg::Derived> derived_predicates) {derived_predicates_ = derived_predicates;}

  bool hasDerivedPredicates() {return !derived_predicates_.empty();}

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
    state.derived_predicates = derived_predicates_;
    return state;
  }

private:
  std::unordered_set<plansys2::Instance> instances_;
  std::unordered_set<plansys2::Function> functions_;
  std::unordered_set<plansys2::Predicate> predicates_;
  std::unordered_set<plansys2::Predicate> inferred_predicates_;
  std::vector<plansys2_msgs::msg::Derived> derived_predicates_;

};
}  // namespace plansys2

#endif  // PLANSYS2_CORE__TYPES_HPP_
