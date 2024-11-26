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

#include "plansys2_problem_expert/ProblemExpert.hpp"

#include <omp.h>  // OpenMP for parallelization

#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "plansys2_core/Utils.hpp"
#include "plansys2_pddl_parser/Domain.hpp"
#include "plansys2_pddl_parser/Instance.hpp"
#include "plansys2_problem_expert/Utils.hpp"

namespace plansys2
{

ProblemExpert::ProblemExpert(std::shared_ptr<DomainExpert> & domain_expert)
: domain_expert_(domain_expert)
{
  state_.setDerivedPredicates(domain_expert_->getDerivedPredicatesGraph());
}

void ProblemExpert::updateInferredPredicates() {
  solveDerivedPredicates(state_);
}

plansys2::State ProblemExpert::getState()
{
  return state_;
}

bool ProblemExpert::addInstance(const plansys2::Instance & instance)
{
  plansys2::Instance lowercase_instance = instance;
  std::transform(
    lowercase_instance.type.begin(), lowercase_instance.type.end(), lowercase_instance.type.begin(),
    [](unsigned char c) {return std::tolower(c);});
  for (auto i = 0; i < lowercase_instance.sub_types.size(); ++i) {
    std::transform(
      lowercase_instance.sub_types[i].begin(), lowercase_instance.sub_types[i].end(),
      lowercase_instance.sub_types[i].begin(), [](unsigned char c) {return std::tolower(c);});
  }

  if (!isValidType(lowercase_instance.type)) {
    return false;
  }

  std::optional<plansys2::Instance> existing_instance = getInstance(lowercase_instance.name);
  bool exist_instance = existing_instance.has_value();

  if (exist_instance && existing_instance.value().type != lowercase_instance.type) {
    return false;
  }

  if (!exist_instance) {
    state_.addInstance(lowercase_instance);
  }

  return true;
}

std::unordered_set<plansys2::Instance> ProblemExpert::getInstances() {return state_.getInstances();}

bool ProblemExpert::removeInstance(const plansys2::Instance & instance)
{
  auto res = state_.removeInstance(instance);
  removeInvalidPredicates(instance);
  removeInvalidFunctions(instance);
  removeInvalidGoals(instance);

  return res;
}

std::optional<plansys2::Instance> ProblemExpert::getInstance(const std::string & instance_name)
{
  auto it = state_.getInstance(parser::pddl::fromStringParam(instance_name));
  if (it != state_.getInstances().end()) {
    return *it;
  }
  return {};
}

std::unordered_set<plansys2::Predicate> ProblemExpert::getPredicates()
{
  return state_.getPredicates();
}

std::unordered_set<plansys2::Predicate> ProblemExpert::getInferredPredicates()
{
  updateInferredPredicates();
  return state_.getInferredPredicates();
}

bool ProblemExpert::addPredicate(const plansys2::Predicate & predicate)
{
  if (state_.hasPredicate(predicate))
  {
    return true;
  }
  if (isValidPredicate(predicate)) {
    return state_.addPredicate(predicate);
  }
  return false;
}

bool ProblemExpert::removePredicate(const plansys2::Predicate & predicate)
{
  if (!isValidPredicate(predicate)) {  // if predicate is not valid, error
    return false;
  }
  return state_.removePredicate(predicate);
}

std::optional<plansys2::Predicate> ProblemExpert::getPredicate(const std::string & expr)
{
  auto it = state_.getPredicate(parser::pddl::fromStringPredicate(expr));
  if (it != state_.getPredicates().end()) {
    return *it;
  }
  return {};
}

std::unordered_set<plansys2::Function> ProblemExpert::getFunctions() {return state_.getFunctions();}

bool ProblemExpert::addFunction(const plansys2::Function & function)
{
  if (!existFunction(function)) {
    if (isValidFunction(function)) {
      state_.addFunction(function);
      return true;
    } else {
      return false;
    }
  } else {
    return updateFunction(function);
  }
}

bool ProblemExpert::removeFunction(const plansys2::Function & function)
{
  return state_.removeFunction(function);
}

bool ProblemExpert::updateFunction(const plansys2::Function & function)
{
  if (existFunction(function)) {
    if (isValidFunction(function)) {
      removeFunction(function);
      state_.addFunction(function);
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

std::optional<plansys2::Function> ProblemExpert::getFunction(const std::string & expr)
{
  auto it = state_.getFunction(parser::pddl::fromStringFunction(expr));
  if (it != state_.getFunctions().end()) {
    return *it;
  }
  return {};
}

void ProblemExpert::removeInvalidPredicates(const plansys2::Instance & instance)
{
  for (auto it = state_.getPredicates().begin(); it != state_.getPredicates().end(); ) {
    if (
      std::find_if(
        it->parameters.begin(), it->parameters.end(), [&](const plansys2_msgs::msg::Param & param) {
          return param.name == instance.name;
        }) != it->parameters.end())
    {
      it = state_.removePredicate(it);
    } else {
      ++it;
    }
  }
}

void ProblemExpert::removeInvalidFunctions(const plansys2::Instance & instance)
{
  for (auto it = state_.getFunctions().begin(); it != state_.getFunctions().end(); ) {
    if (
      std::find_if(
        it->parameters.begin(), it->parameters.end(), [&](const plansys2_msgs::msg::Param & param) {
          return param.name == instance.name;
        }) != it->parameters.end())
    {
      it = state_.removeFunction(it);
    } else {
      ++it;
    }
  }
}

void ProblemExpert::removeInvalidGoals(const plansys2::Instance & instance)
{
  // Get subgoals.
  auto subgoals = parser::pddl::getSubtrees(goal_);

  // Check for subgoals before continuing.
  if (subgoals.empty()) {
    return;
  }

  // Remove invalid subgoals.
  for (auto rit = subgoals.rbegin(); rit != subgoals.rend(); ++rit) {
    // Get predicates.
    std::vector<plansys2_msgs::msg::Node> predicates;
    parser::pddl::getPredicates(predicates, *rit);

    // Check predicates for removed instance.
    bool params_valid = true;
    for (const auto & predicate : predicates) {
      if (
        std::find_if(
          predicate.parameters.begin(), predicate.parameters.end(),
          [&](const plansys2_msgs::msg::Param & param) {return param.name == instance.name;}) !=
        predicate.parameters.end())
      {
        params_valid = false;
        break;
      }
    }

    // Remove invalid subgoal.
    if (!params_valid) {
      subgoals.erase(std::next(rit).base());
      continue;
    }

    // Get functions.
    std::vector<plansys2_msgs::msg::Node> functions;
    parser::pddl::getFunctions(functions, *rit);

    // Check functions for removed instance.
    params_valid = true;
    for (const auto & function : functions) {
      if (
        std::find_if(
          function.parameters.begin(), function.parameters.end(),
          [&](const plansys2_msgs::msg::Param & param) {return param.name == instance.name;}) !=
        function.parameters.end())
      {
        params_valid = false;
        break;
      }
    }

    // Remove invalid subgoal.
    if (!params_valid) {
      subgoals.erase(std::next(rit).base());
    }
  }

  // Create a new goal from the remaining subgoals.
  auto tree = parser::pddl::fromSubtrees(subgoals, goal_.nodes[0].node_type);
  if (tree) {
    goal_ = plansys2::Goal(*tree);
  } else {
    goal_.nodes.clear();
  }
}

plansys2::Goal ProblemExpert::getGoal() {return goal_;}

bool ProblemExpert::setGoal(const plansys2::Goal & goal)
{
  if (isValidGoal(goal)) {
    goal_ = goal;
    return true;
  } else {
    return false;
  }
}

bool ProblemExpert::isGoalSatisfied(const plansys2::Goal & goal)
{
  updateInferredPredicates();
  return check(goal, state_);
}

bool ProblemExpert::clearGoal()
{
  goal_.nodes.clear();
  return true;
}

bool ProblemExpert::clearKnowledge()
{
  state_.clearState();
  clearGoal();

  return true;
}

bool ProblemExpert::isValidType(const std::string & type)
{
  std::string lowercase_type = type;
  std::transform(
    lowercase_type.begin(), lowercase_type.end(), lowercase_type.begin(),
    [](unsigned char c) {return std::tolower(c);});

  auto valid_types = domain_expert_->getTypes();
  auto it = std::find(valid_types.begin(), valid_types.end(), lowercase_type);

  return it != valid_types.end();
}

bool ProblemExpert::existInstance(const std::string & name)
{
  return state_.hasInstance(parser::pddl::fromStringParam(name));
}

bool ProblemExpert::existPredicate(const plansys2::Predicate & predicate)
{
  return state_.hasPredicate(predicate);
}

bool ProblemExpert::existInferredPredicate(const plansys2::Predicate & predicate)
{
  return state_.hasInferredPredicate(predicate);
}

bool ProblemExpert::existFunction(const plansys2::Function & function)
{
  return state_.hasFunction(function);
}

bool ProblemExpert::isValidPredicate(const plansys2::Predicate & predicate)
{
  bool valid = false;

  const std::optional<plansys2::Predicate> & model_predicate =
    domain_expert_->getPredicate(predicate.name);
  if (model_predicate) {
    if (model_predicate.value().parameters.size() == predicate.parameters.size()) {
      bool same_types = true;
      int i = 0;
      while (same_types && i < predicate.parameters.size()) {
        auto arg_type = getInstance(predicate.parameters[i].name);

        if (!arg_type.has_value()) {
          // It might be a constant, so we check if the type is correct
          same_types = false;
          auto constants = domain_expert_->getConstants(model_predicate.value().parameters[i].type);
          for (auto constant : constants) {
            if (constant == predicate.parameters[i].name) {
              same_types = true;
              break;
            }
          }
        } else if (arg_type.value().type != model_predicate.value().parameters[i].type) {
          bool isSubtype = false;
          for (std::string subType : model_predicate.value().parameters[i].sub_types) {
            if (arg_type.value().type == subType) {
              isSubtype = true;
              break;
            }
          }
          if (!isSubtype) {
            same_types = false;
          }
        }
        i++;
      }
      valid = same_types;
    }
  }

  return valid;
}

bool ProblemExpert::isValidFunction(const plansys2::Function & function)
{
  bool valid = false;

  const std::optional<plansys2::Function> & model_function =
    domain_expert_->getFunction(function.name);
  if (model_function) {
    if (model_function.value().parameters.size() == function.parameters.size()) {
      bool same_types = true;
      int i = 0;
      while (same_types && i < function.parameters.size()) {
        auto arg_type = getInstance(function.parameters[i].name);

        if (!arg_type.has_value()) {
          // It might be a constant, so we check if the type is correct
          same_types = false;
          auto constants = domain_expert_->getConstants(model_function.value().parameters[i].type);
          for (auto constant : constants) {
            if (constant == function.parameters[i].name) {
              same_types = true;
              break;
            }
          }
        } else if (arg_type.value().type != model_function.value().parameters[i].type) {
          bool isSubtype = false;
          for (std::string subType : model_function.value().parameters[i].sub_types) {
            if (arg_type.value().type == subType) {
              isSubtype = true;
              break;
            }
          }
          if (!isSubtype) {
            same_types = false;
          }
        }
        i++;
      }
      valid = same_types;
    }
  }

  return valid;
}

bool ProblemExpert::isValidGoal(const plansys2::Goal & goal)
{
  return checkPredicateTreeTypes(goal, domain_expert_);
}

bool ProblemExpert::checkPredicateTreeTypes(
  const plansys2_msgs::msg::Tree & tree, std::shared_ptr<DomainExpert> & domain_expert,
  uint8_t node_id)
{
  if (node_id >= tree.nodes.size()) {
    return false;
  }

  switch (tree.nodes[node_id].node_type) {
    case plansys2_msgs::msg::Node::AND: {
        bool ret = true;

        for (auto & child_id : tree.nodes[node_id].children) {
          ret = ret && checkPredicateTreeTypes(tree, domain_expert, child_id);
        }
        return ret;
      }

    case plansys2_msgs::msg::Node::OR: {
        bool ret = true;

        for (auto & child_id : tree.nodes[node_id].children) {
          ret = ret && checkPredicateTreeTypes(tree, domain_expert, child_id);
        }
        return ret;
      }

    case plansys2_msgs::msg::Node::NOT: {
        return checkPredicateTreeTypes(tree, domain_expert, tree.nodes[node_id].children[0]);
      }

    case plansys2_msgs::msg::Node::PREDICATE: {
        return isValidPredicate(tree.nodes[node_id]);
      }

    case plansys2_msgs::msg::Node::FUNCTION: {
        return isValidFunction(tree.nodes[node_id]);
      }

    case plansys2_msgs::msg::Node::EXPRESSION: {
        bool ret = true;

        for (auto & child_id : tree.nodes[node_id].children) {
          ret = ret && checkPredicateTreeTypes(tree, domain_expert, child_id);
        }
        return ret;
      }

    case plansys2_msgs::msg::Node::FUNCTION_MODIFIER: {
        bool ret = true;

        for (auto & child_id : tree.nodes[node_id].children) {
          ret = ret && checkPredicateTreeTypes(tree, domain_expert, child_id);
        }
        return ret;
      }

    case plansys2_msgs::msg::Node::NUMBER: {
        return true;
      }

    default:
      // LCOV_EXCL_START
      std::cerr << "checkPredicateTreeTypes: Error parsing expresion ["
                << parser::pddl::toString(tree, node_id) << "]" << std::endl;
      // LCOV_EXCL_STOP
  }

  return false;
}

std::string ProblemExpert::getProblem()
{
  parser::pddl::Domain domain(domain_expert_->getDomain());
  parser::pddl::Instance problem(domain);

  problem.name = "problem_1";

  for (const auto & instance : state_.getInstances()) {
    bool is_constant = domain.getType(instance.type)->parseConstant(instance.name).first;
    if (is_constant) {
      std::cout << "Skipping adding constant to problem :object: " << instance.name << " "
                << instance.type << std::endl;
    } else {
      problem.addObject(instance.name, instance.type);
    }
  }

  for (plansys2_msgs::msg::Node predicate : state_.getPredicates()) {
    StringVec v;

    for (size_t i = 0; i < predicate.parameters.size(); i++) {
      v.push_back(predicate.parameters[i].name);
    }

    std::transform(predicate.name.begin(), predicate.name.end(), predicate.name.begin(), ::tolower);

    problem.addInit(predicate.name, v);
  }

  for (plansys2_msgs::msg::Node function : state_.getFunctions()) {
    StringVec v;

    for (size_t i = 0; i < function.parameters.size(); i++) {
      v.push_back(function.parameters[i].name);
    }

    std::transform(function.name.begin(), function.name.end(), function.name.begin(), ::tolower);

    problem.addInit(function.name, function.value, v);
  }

  const std::string gs = parser::pddl::toString(goal_);
  problem.addGoal(gs);

  std::ostringstream stream;
  stream << problem;
  return stream.str();
}

bool ProblemExpert::addProblem(const std::string & problem_str)
{
  if (problem_str.empty()) {
    std::cerr << "Empty problem." << std::endl;
    return false;
  }
  parser::pddl::Domain domain(domain_expert_->getDomain());

  std::string lc_problem = problem_str;
  std::transform(
    problem_str.begin(), problem_str.end(), lc_problem.begin(), [](unsigned char c) {
      return std::tolower(c);
    });

  lc_problem = remove_comments(lc_problem);

  std::cout << "Domain:\n" << domain << std::endl;
  std::cout << "Problem:\n" << lc_problem << std::endl;

  parser::pddl::Instance problem(domain);

  std::string domain_name = problem.getDomainName(lc_problem);
  if (domain_name.empty()) {
    std::cerr << "Domain name is empty" << std::endl;
    return false;
  } else if (!domain_expert_->existDomain(domain_name)) {
    std::cerr << "Domain name does not exist: " << domain_name << std::endl;
    return false;
  }

  domain.name = domain_name;
  try {
    problem.parse(lc_problem);
  } catch (std::runtime_error ex) {
    // all errors thrown by the Stringreader object extend std::runtime_error
    std::cerr << ex.what() << std::endl;
    return false;
  }

  std::cout << "Parsed problem: " << problem << std::endl;

  for (unsigned i = 0; i < domain.types.size(); ++i) {
    if (domain.types[i]->constants.size()) {
      for (unsigned j = 0; j < domain.types[i]->constants.size(); ++j) {
        plansys2::Instance instance;
        instance.name = domain.types[i]->constants[j];
        instance.type = domain.types[i]->name;
        std::cout << "Adding constant: " << instance.name << " " << instance.type << std::endl;
        addInstance(instance);
      }
    }
  }

  for (unsigned i = 0; i < domain.types.size(); ++i) {
    if (domain.types[i]->objects.size()) {
      for (unsigned j = 0; j < domain.types[i]->objects.size(); ++j) {
        plansys2::Instance instance;
        instance.name = domain.types[i]->objects[j];
        instance.type = domain.types[i]->name;
        std::cout << "Adding instance: " << instance.name << " " << instance.type << std::endl;
        addInstance(instance);
      }
    }
  }

  plansys2_msgs::msg::Tree tree;
  for (auto ground : problem.init) {
    auto tree_node = ground->getTree(tree, domain);
    switch (tree_node->node_type) {
      case plansys2_msgs::msg::Node::PREDICATE: {
          plansys2::Predicate pred_node(*tree_node);
          std::cout << "Adding predicate: " << parser::pddl::toString(tree, tree_node->node_id)
                    << std::endl;
          if (!addPredicate(pred_node)) {
            std::cerr << "Failed to add predicate: "
                      << parser::pddl::toString(tree, tree_node->node_id) << std::endl;
          }
        } break;
      case plansys2_msgs::msg::Node::FUNCTION: {
          plansys2::Function func_node(*tree_node);
          std::cout << "Adding function: " << parser::pddl::toString(tree, tree_node->node_id)
                    << std::endl;
          if (!addFunction(func_node)) {
            std::cerr << "Failed to add function: "
                      << parser::pddl::toString(tree, tree_node->node_id) << std::endl;
          }
        } break;
      default:
        break;
    }
  }

  plansys2_msgs::msg::Tree goal;
  auto node = problem.goal->getTree(goal, domain);
  std::cout << "Adding Goal: " << parser::pddl::toString(goal) << std::endl;
  if (setGoal(goal)) {
    std::cout << "Goal insertion ok" << std::endl;
  } else {
    std::cout << "Goal insertion failed" << std::endl;
  }

  return true;
}

}  // namespace plansys2
