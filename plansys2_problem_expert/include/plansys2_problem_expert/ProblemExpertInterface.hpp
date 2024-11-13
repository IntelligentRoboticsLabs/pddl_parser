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

#ifndef PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTINTERFACE_HPP_
#define PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTINTERFACE_HPP_

#include <string>
#include <vector>
#include <unordered_set>

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/tree.hpp"

#include "plansys2_core/Types.hpp"
#include "plansys2_core/State.hpp"

namespace plansys2
{

class ProblemExpertInterface
{
public:
  ProblemExpertInterface() {}

  virtual std::unordered_set<plansys2::Instance> getInstances() = 0;
  virtual bool addInstance(const plansys2::Instance & instance) = 0;
  virtual bool removeInstance(const plansys2::Instance & instance) = 0;
  virtual std::optional<plansys2::Instance> getInstance(const std::string & name) = 0;

  virtual std::unordered_set<plansys2::Predicate> getPredicates() = 0;
  virtual std::unordered_set<plansys2::Predicate> getInferredPredicates() = 0;
  virtual bool addPredicate(const plansys2::Predicate & predicate) = 0;
  virtual bool removePredicate(const plansys2::Predicate & predicate) = 0;
  virtual bool existPredicate(const plansys2::Predicate & predicate) = 0;
  virtual std::optional<plansys2::Predicate> getPredicate(const std::string & expr) = 0;

  virtual std::unordered_set<plansys2::Function> getFunctions() = 0;
  virtual bool addFunction(const plansys2::Function & function) = 0;
  virtual bool removeFunction(const plansys2::Function & function) = 0;
  virtual bool existFunction(const plansys2::Function & function) = 0;
  virtual bool updateFunction(const plansys2::Function & function) = 0;
  virtual std::optional<plansys2::Function> getFunction(const std::string & expr) = 0;

  virtual plansys2::State getState() = 0;

  virtual plansys2::Goal getGoal() = 0;
  virtual bool setGoal(const plansys2::Goal & goal) = 0;
  virtual bool isGoalSatisfied(const plansys2::Goal & goal) = 0;

  virtual bool clearGoal() = 0;
  virtual bool clearKnowledge() = 0;

  virtual std::string getProblem() = 0;
  virtual bool addProblem(const std::string & problem_str) = 0;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTINTERFACE_HPP_
