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

#ifndef PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERT_HPP_
#define PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERT_HPP_

#include <optional>
#include <string>
#include <vector>
#include <memory>
#include <map>
#include <unordered_set>

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/tree.hpp"

#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_problem_expert/ProblemExpertInterface.hpp"
#include "plansys2_domain_expert/DomainExpert.hpp"

namespace plansys2
{

class ProblemExpert : public ProblemExpertInterface
{
public:
  explicit ProblemExpert(std::shared_ptr<DomainExpert> & domain_expert);

  std::unordered_set<plansys2::Instance> getInstances();
  bool addInstance(const plansys2::Instance & instance);
  bool removeInstance(const plansys2::Instance & instance);
  std::optional<plansys2::Instance> getInstance(const std::string & name);

  void groundPredicate(
    std::unordered_set<plansys2::Predicate> & current_predicates,
    const plansys2::Predicate & predicate,
    const std::vector<std::map<std::string, std::string>> & params_values_vector);
  std::unordered_set<plansys2::Predicate> solveDerivedPredicates(
    std::unordered_set<plansys2::Predicate> & predicates);
  std::unordered_set<plansys2::Predicate> solveAllDerivedPredicates(
    const std::unordered_set<plansys2::Predicate> & predicates);

  std::unordered_set<plansys2::Predicate> getPredicates();
  bool addPredicate(const plansys2::Predicate & predicate);
  bool removePredicate(const plansys2::Predicate & predicate);
  bool existPredicate(const plansys2::Predicate & predicate);
  std::optional<plansys2::Predicate> getPredicate(const std::string & expr);

  std::vector<plansys2::Function> getFunctions();
  bool addFunction(const plansys2::Function & function);
  bool removeFunction(const plansys2::Function & function);
  bool existFunction(const plansys2::Function & function);
  bool updateFunction(const plansys2::Function & function);
  std::optional<plansys2::Function> getFunction(const std::string & expr);

  plansys2::Goal getGoal();
  bool setGoal(const plansys2::Goal & goal);
  bool isGoalSatisfied(const plansys2::Goal & goal);

  bool clearGoal();
  bool clearKnowledge();

  std::string getProblem();
  bool addProblem(const std::string & problem_str);

  bool existInstance(const std::string & name);
  bool isValidType(const std::string & type);
  bool isValidPredicate(const plansys2::Predicate & predicate);
  bool isValidFunction(const plansys2::Function & function);
  bool isValidGoal(const plansys2::Goal & goal);

private:
  bool checkPredicateTreeTypes(
    const plansys2_msgs::msg::Tree & tree,
    std::shared_ptr<DomainExpert> & domain_expert_,
    uint8_t node_id = 0);

  void removeInvalidPredicates(
    std::unordered_set<plansys2::Predicate> & predicates,
    const plansys2::Instance & instance);
  void removeInvalidFunctions(
    std::vector<plansys2::Function> & functions,
    const plansys2::Instance & instance);
  void removeInvalidGoals(const plansys2::Instance & instance);

  std::unordered_set<plansys2::Instance> instances_;
  std::unordered_set<plansys2::Predicate> predicates_;
  std::vector<plansys2::Function> functions_;
  plansys2::Goal goal_;

  std::shared_ptr<DomainExpert> domain_expert_;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERT_HPP_
