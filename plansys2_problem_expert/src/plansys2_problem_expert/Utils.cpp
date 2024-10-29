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

#include "plansys2_problem_expert/Utils.hpp"

#include <omp.h>  // OpenMP for parallelization

#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include "plansys2_pddl_parser/Utils.hpp"

namespace plansys2
{

std::tuple<bool, bool, double, std::vector<std::map<std::string, std::string>>> evaluate(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  std::unordered_set<plansys2::Instance> & instances,
  std::unordered_set<plansys2::Predicate> & predicates, std::vector<plansys2::Function> & functions,
  bool apply, bool use_state, uint8_t node_id, bool negate)
{
  if (tree.nodes.empty()) {
    return {true, true, 0, {}};
  }

  const auto & current_node = tree.nodes[node_id];
  switch (current_node.node_type) {
    case plansys2_msgs::msg::Node::AND: {
        bool success = true;
        bool truth_value = true;
        std::vector<std::map<std::string, std::string>> param_values;

        for (const auto & child_id : current_node.children) {
          auto [child_success, child_value, _, child_param_values] = evaluate(
            tree, problem_client, instances, predicates, functions, apply, use_state, child_id,
            negate);

          success &= child_success;
          truth_value &= child_value;
          if (!truth_value) {
            return {success, false, 0, {}};
            break;
          }

          if (param_values.empty()) {
            param_values = std::move(child_param_values);
          } else if (!child_param_values.empty()) {
            param_values = mergeParamsValuesVector(param_values, std::move(child_param_values));
            if (param_values.empty()) {
              return {success, false, 0, {}};
            }
          }
        }
        return {success, truth_value, 0, param_values};
      }

    case plansys2_msgs::msg::Node::OR: {
        bool success = true;
        bool truth_value = false;
        std::vector<std::map<std::string, std::string>> param_values;

        for (auto & child_id : current_node.children) {
          auto [child_success, child_value, _, child_param_values] = evaluate(
            tree, problem_client, instances, predicates, functions, apply, use_state, child_id,
            negate);
          success = success && child_success;
          truth_value = truth_value || child_value;
          param_values.insert(
            param_values.end(), child_param_values.begin(), child_param_values.end());
        }
        return {success, truth_value, 0, param_values};
      }

    case plansys2_msgs::msg::Node::NOT: {
        auto result = evaluate(
          tree, problem_client, instances, predicates, functions, apply, use_state,
          current_node.children[0], !negate);

        return result;
      }

    case plansys2_msgs::msg::Node::PREDICATE: {
        bool success = true;
        bool value = true;
        std::vector<std::map<std::string, std::string>> param_values;

        if (apply) {
          if (use_state) {
            auto it = predicates.find(current_node);
            if (negate && it != predicates.end()) {
              predicates.erase(it);
            } else if (!negate && it == predicates.end()) {
              auto insert_res = predicates.insert(current_node);
            }
          } else {
            success &= negate ? problem_client->removePredicate(current_node) :
              problem_client->addPredicate(current_node);
          }
        } else {
          if (use_state) {
            std::tie(value, param_values) = unifyPredicate(current_node, predicates);
            if (negate) {
              std::tie(value, param_values) =
                negateResult(current_node, value, param_values, instances);
            }
          } else {
            value = negate ^ problem_client->existPredicate(current_node);
          }
        }
        return {success, value, 0, param_values};
      }

    case plansys2_msgs::msg::Node::FUNCTION: {
        bool success = true;
        double value = 0;
        std::vector<std::map<std::string, std::string>> param_values;

        if (use_state) {
          auto it = std::find_if(
            functions.begin(), functions.end(),
            std::bind(&parser::pddl::checkNodeEquality, std::placeholders::_1, current_node));
          if (it != functions.end()) {
            value = it->value;
          } else {
            success = false;
          }
        } else {
          std::optional<plansys2_msgs::msg::Node> func =
            problem_client->getFunction(parser::pddl::toString(tree, node_id));

          if (func.has_value()) {
            value = func.value().value;
          } else {
            success = false;
          }
        }

        return {success, false, value, param_values};
      }

    case plansys2_msgs::msg::Node::EXPRESSION: {
        auto [left_success, left_value, left_double, left_param_values] = evaluate(
          tree, problem_client, instances, predicates, functions, apply, use_state,
          current_node.children[0], negate);
        auto [right_success, right_value, right_double, right_param_values] = evaluate(
          tree, problem_client, instances, predicates, functions, apply, use_state,
          current_node.children[1], negate);

        std::vector<std::map<std::string, std::string>> param_values;

        if (!left_success || !right_success) {
          return {false, false, 0, {}};
        }

        switch (current_node.expression_type) {
          case plansys2_msgs::msg::Node::COMP_GE:
            if (left_double >= right_double) {
              return {true, static_cast<bool>(negate ^ true), 0, {}};
            } else {
              return {true, static_cast<bool>(negate ^ false), 0, {}};
            }
            break;
          case plansys2_msgs::msg::Node::COMP_GT:
            if (left_double > right_double) {
              return {true, static_cast<bool>(negate ^ true), 0, {}};
            } else {
              return {true, static_cast<bool>(negate ^ false), 0, {}};
            }
            break;
          case plansys2_msgs::msg::Node::COMP_LE:
            if (left_double <= right_double) {
              return {true, static_cast<bool>(negate ^ true), 0, {}};
            } else {
              return {true, static_cast<bool>(negate ^ false), 0, {}};
            }
            break;
          case plansys2_msgs::msg::Node::COMP_LT:
            if (left_double < right_double) {
              return {true, static_cast<bool>(negate ^ true), 0, {}};
            } else {
              return {true, static_cast<bool>(negate ^ false), 0, {}};
            }
            break;
          case plansys2_msgs::msg::Node::COMP_EQ: {
              auto c_t = plansys2_msgs::msg::Node::CONSTANT;
              auto p_t = plansys2_msgs::msg::Node::PARAMETER;
              auto n_t = plansys2_msgs::msg::Node::NUMBER;

              const auto & c0 = tree.nodes[current_node.children[0]];
              const auto & c1 = tree.nodes[current_node.children[1]];

              const auto c0_type = c0.node_type;
              const auto c1_type = c1.node_type;

              if ((c0_type == c_t && c1_type == p_t) || (c0_type == p_t && c1_type == c_t)) {
                param_values = (c0_type == c_t) ?
                  mergeParamsValuesVector({{{c1.name, c0.name}}}, right_param_values) :
                  mergeParamsValuesVector(left_param_values, {{{c0.name, c1.name}}});

                bool result = !param_values.empty();
                if (negate) {
                  plansys2_msgs::msg::Node aux_node;
                  aux_node.parameters.push_back(
                    c0_type ==
                    p_t ? c0.parameters[0] : c1.parameters[0]);
                  std::tie(result, param_values) =
                    negateResult(aux_node, result, param_values, instances);
                }
                return {true, result, 0, param_values};
              }

              if (c0_type == p_t && c1_type == p_t) {
                std::vector<std::map<std::string, std::string>> new_param_values;
                new_param_values.reserve(right_param_values.size());
                for (const auto & right_param_value : right_param_values) {
                  new_param_values.push_back({{c0.name, right_param_value.at(c1.name)}});
                }
                param_values = mergeParamsValuesVector(left_param_values, new_param_values);
                for (auto & param_value : param_values) {
                  param_value[c1.name] = param_value[c0.name];
                }
                bool result = !param_values.empty();
                if (negate) {
                  plansys2_msgs::msg::Node aux_node;
                  aux_node.parameters.push_back(c0.parameters[0]);
                  aux_node.parameters.push_back(c1.parameters[0]);
                  std::tie(result, param_values) =
                    negateResult(aux_node, result, param_values, instances);
                }
                return {true, result, 0, param_values};
              }

              if (c0_type == c_t && c1_type == c_t) {
                return {true, static_cast<bool>(negate ^ (c0.name == c1.name)), 0, {}};
              }

              if (c0_type == n_t && c1_type == n_t) {
                return {true, static_cast<bool>(negate ^ (left_double == right_double)), 0, {}};
              }
              break;
            }
          case plansys2_msgs::msg::Node::ARITH_MULT:
            return {true, false, left_double * right_double, {}};
            break;
          case plansys2_msgs::msg::Node::ARITH_DIV:
            if (std::abs(right_double) > 1e-5) {
              return {true, false, left_double / right_double, {}};
            } else {
              // Division by zero not allowed.
              return {false, false, 0, {}};
            }
            break;
          case plansys2_msgs::msg::Node::ARITH_ADD:
            return {true, false, left_double + right_double, {}};
            break;
          case plansys2_msgs::msg::Node::ARITH_SUB:
            return {true, false, left_double - right_double, {}};
            break;
          default:
            break;
        }

        return {false, false, 0., {}};
      }

    case plansys2_msgs::msg::Node::FUNCTION_MODIFIER: {
        auto [left_success, left_value, left_double, left_param_values] = evaluate(
          tree, problem_client, instances, predicates, functions, apply, use_state,
          current_node.children[0], negate);
        auto [right_success, right_value, right_double, right_param_values] = evaluate(
          tree, problem_client, instances, predicates, functions, apply, use_state,
          current_node.children[1], negate);

        if (!left_success || !right_success) {
          return {false, false, 0, {}};
        }

        bool success = true;
        double value = 0;

        switch (current_node.modifier_type) {
          case plansys2_msgs::msg::Node::ASSIGN:
            value = right_double;
            break;
          case plansys2_msgs::msg::Node::INCREASE:
            value = left_double + right_double;
            break;
          case plansys2_msgs::msg::Node::DECREASE:
            value = left_double - right_double;
            break;
          case plansys2_msgs::msg::Node::SCALE_UP:
            value = left_double * right_double;
            break;
          case plansys2_msgs::msg::Node::SCALE_DOWN:
            // Division by zero not allowed.
            if (std::abs(right_double) > 1e-5) {
              value = left_double / right_double;
            } else {
              success = false;
            }
            break;
          default:
            success = false;
            break;
        }

        if (success && apply) {
          uint8_t left_id = current_node.children[0];
          if (use_state) {
            auto it = std::find_if(
              functions.begin(), functions.end(),
              std::bind(
                &parser::pddl::checkNodeEquality, std::placeholders::_1, tree.nodes[left_id]));
            if (it != functions.end()) {
              it->value = value;
            } else {
              success = false;
            }
          } else {
            std::stringstream ss;
            ss << "(= " << parser::pddl::toString(tree, left_id) << " " << value << ")";
            problem_client->updateFunction(parser::pddl::fromStringFunction(ss.str()));
          }
        }

        return {success, false, value, {}};
      }

    case plansys2_msgs::msg::Node::NUMBER: {
        return {true, true, current_node.value, {}};
      }

    case plansys2_msgs::msg::Node::CONSTANT: {
        if (current_node.name.size() > 0) {
          return {true, true, 0, {}};
        }
        return {true, false, 0, {}};
      }

    case plansys2_msgs::msg::Node::OBJECT: {
        if (current_node.name.size() > 0) {
          return {true, true, 0, {}};
        }
        return {true, false, 0, {}};
      }

    case plansys2_msgs::msg::Node::PARAMETER: {
        std::vector<std::map<std::string, std::string>> param_values;
        auto current_parameter = current_node.parameters[0];
        if (current_parameter.name.front() != '?') {
          std::map<std::string, std::string> param_value = {
            {current_node.name, current_parameter.name}};
          param_values.emplace_back(param_value);
          return {true, true, 0, param_values};
        }
        for (const auto & instance : instances) {
          if (parser::pddl::checkParamTypeEquivalence(current_parameter, instance)) {
            std::map<std::string, std::string> param_value = {
              {current_parameter.name, instance.name}};
            param_values.emplace_back(param_value);
          }
        }
        return {true, false, 0, param_values};
      }

    case plansys2_msgs::msg::Node::EXISTS: {
        std::vector<std::map<std::string, std::string>> param_values;

        auto [child_success, child_value, _, child_param_values] = evaluate(
          tree, problem_client, instances, predicates, functions, apply, use_state,
          current_node.children[0], negate);
        return {child_success, child_value, 0, child_param_values};
      }

    default:
      std::cerr << "evaluate: Error parsing expresion [" << parser::pddl::toString(tree, node_id)
                << "]" << std::endl;
  }
  return {false, false, 0, {}};
}

std::tuple<bool, std::vector<std::map<std::string, std::string>>> unifyPredicate(
  const plansys2::Predicate & predicate, const std::unordered_set<plansys2::Predicate> & predicates)
{
  std::vector<std::map<std::string, std::string>> param_dict_vector;
  const size_t param_count = predicate.parameters.size();
  std::map<std::string, int> variable_parameters;
  for (size_t i = 0; i < param_count; ++i) {
    // If the parameter name starts with '?', store the mapping
    if (predicate.parameters[i].name.front() == '?') {
      variable_parameters[predicate.parameters[i].name] = i;
    }
  }

  if (variable_parameters.empty()) {
    return std::make_tuple(predicates.find(predicate) != predicates.end(), param_dict_vector);
  }

  param_dict_vector.reserve(predicates.size());
  bool result = false;
  for (const plansys2::Predicate & p : predicates) {
    if (parser::pddl::checkNodeEquality(p, predicate)) {
      std::map<std::string, std::string> params_dict;

      for (const auto & variable : variable_parameters) {
        params_dict.emplace(variable.first, p.parameters[variable.second].name);
      }
      result = true;
      if (params_dict.empty()) {
        return std::make_tuple(result, std::move(param_dict_vector));
      }
      param_dict_vector.emplace_back(std::move(params_dict));
    }
  }

  return std::make_tuple(result, std::move(param_dict_vector));
}

std::tuple<bool, std::vector<std::map<std::string, std::string>>> unifyFunction(
  const plansys2::Function & function, const std::vector<plansys2::Function> & functions)
{
  std::vector<std::map<std::string, std::string>> param_dict_vector;
  param_dict_vector.reserve(functions.size());

  bool result = false;
  const size_t param_count = function.parameters.size();

  for (const plansys2::Function & p : functions) {
    if (parser::pddl::checkNodeEquality(p, function)) {
      std::map<std::string, std::string> params_dict;

      for (size_t i = 0; i < param_count; ++i) {
        // If the parameter name starts with '?', store the mapping
        if (function.parameters[i].name.front() == '?') {
          params_dict.emplace(function.parameters[i].name, p.parameters[i].name);
        }
      }
      result = true;
      if (params_dict.empty()) {
        return std::make_tuple(result, std::move(param_dict_vector));
      }
      param_dict_vector.emplace_back(std::move(params_dict));
    }
  }

  return std::make_tuple(result, std::move(param_dict_vector));
}

std::vector<std::map<std::string, std::string>> complementParamsValuesVector(
  const std::vector<plansys2_msgs::msg::Param> & params,
  const std::vector<std::map<std::string, std::string>> & param_dict_vector,
  const std::unordered_set<plansys2::Instance> & instances)
{
  std::vector<std::vector<std::string>> parameters_vector;
  parameters_vector.reserve(params.size());

  for (size_t i = 0; i < params.size(); i++) {
    std::vector<std::string> p_vector;
    for (const auto & instance : instances) {
      if (parser::pddl::checkParamTypeEquivalence(params[i], instance)) {
        p_vector.emplace_back(instance.name);
      }
    }
    parameters_vector.emplace_back(std::move(p_vector));
  }

  std::vector<std::map<std::string, std::string>> complement_set;
  if (parameters_vector.empty()) {
    return complement_set;
  }

  complement_set.emplace_back();

  for (size_t i = 0; i < parameters_vector.size(); i++) {
    std::vector<std::map<std::string, std::string>> temp_result;
    temp_result.reserve(complement_set.size() * parameters_vector[i].size());

    for (const auto & combination : complement_set) {
      for (const auto & element : parameters_vector[i]) {
        std::map<std::string, std::string> new_combination = combination;
        new_combination[params[i].name] = element;

        if (
          i == parameters_vector.size() - 1 &&
          std::find(param_dict_vector.begin(), param_dict_vector.end(), new_combination) !=
          param_dict_vector.end())
        {
          continue;
        }
        temp_result.emplace_back(new_combination);
      }
    }
    complement_set = std::move(temp_result);
  }
  return std::move(complement_set);
}

std::tuple<bool, std::vector<std::map<std::string, std::string>>> negateResult(
  const plansys2_msgs::msg::Node & node, const bool & result,
  const std::vector<std::map<std::string, std::string>> & param_dict_vector,
  const std::unordered_set<plansys2::Instance> & instances)
{
  std::vector<plansys2_msgs::msg::Param> params;
  for (size_t i = 0; i < node.parameters.size(); i++) {
    if (node.parameters[i].name.front() == '?') {
      params.push_back(node.parameters[i]);
    }
  }

  if (params.empty()) {
    return {static_cast<bool>(true ^ result), {}};
  }

  auto complement_param_dict_vector =
    complementParamsValuesVector(params, param_dict_vector, instances);
  return {
    !result || (result && !complement_param_dict_vector.empty()),
    std::move(complement_param_dict_vector)};
}

void mergeParamsValuesDicts(
  const std::map<std::string, std::string> & dict1,
  const std::map<std::string, std::string> & dict2, std::map<std::string, std::string> & dict3)
{
  dict3.clear();

  auto it1 = dict1.begin();
  auto it2 = dict2.begin();

  // Iterate through both maps simultaneously
  while (it1 != dict1.end() && it2 != dict2.end()) {
    if (it1->first < it2->first) {
      dict3.emplace(it1->first, it1->second);  // Insert from dict1
      ++it1;
    } else if (it1->first > it2->first) {
      dict3.emplace(it2->first, it2->second);  // Insert from dict2
      ++it2;
    } else {
      // Keys are equal, check if values are the same
      if (it1->second != it2->second) {
        dict3.clear();
        return;  // Different values for same parameter, return empty dict
      }
      dict3.emplace(it1->first, it1->second);  // Insert the common element
      ++it1;
      ++it2;
    }
  }

  // Insert remaining elements from dict1
  while (it1 != dict1.end()) {
    dict3.emplace(it1->first, it1->second);
    ++it1;
  }

  // Insert remaining elements from dict2
  while (it2 != dict2.end()) {
    dict3.emplace(it2->first, it2->second);
    ++it2;
  }
}

std::vector<std::map<std::string, std::string>> mergeParamsValuesVector(
  const std::vector<std::map<std::string, std::string>> & vector1,
  const std::vector<std::map<std::string, std::string>> & vector2)
{
  std::vector<std::map<std::string, std::string>> vector3;
  vector3.reserve(vector1.size() * vector2.size());  // Reserve space to avoid reallocations

  // Store results in thread-local vectors
  std::vector<std::vector<std::map<std::string, std::string>>> local_results(omp_get_max_threads());

#pragma omp parallel
  {
    int thread_id = omp_get_thread_num();
    std::vector<std::map<std::string, std::string>> & local_vector = local_results[thread_id];

#pragma omp for schedule(dynamic)
    for (size_t i = 0; i < vector1.size(); ++i) {
      for (const auto & dict2 : vector2) {
        std::map<std::string, std::string> dict3;

        // Perform the merge of dict1 and dict2 into dict3
        mergeParamsValuesDicts(vector1[i], dict2, dict3);

        if (!dict3.empty()) {
          local_vector.emplace_back(std::move(dict3));  // Safely push to local vector
        }
      }
    }
  }

  // Combine results from all threads
  for (const auto & local_vector : local_results) {
    vector3.insert(vector3.end(), local_vector.begin(), local_vector.end());
  }
  return std::move(vector3);  // Return the merged vector
}

std::tuple<bool, bool, double, std::vector<std::map<std::string, std::string>>> evaluate(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client, bool apply, uint32_t node_id)
{
  std::unordered_set<plansys2::Instance> instances;
  std::unordered_set<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;
  return evaluate(tree, problem_client, instances, predicates, functions, apply, false, node_id);
}

std::tuple<bool, bool, double, std::vector<std::map<std::string, std::string>>> evaluate(
  const plansys2_msgs::msg::Tree & tree, std::unordered_set<plansys2::Instance> & instances,
  std::unordered_set<plansys2::Predicate> & predicates, std::vector<plansys2::Function> & functions,
  bool apply, uint32_t node_id)
{
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client;
  return evaluate(tree, problem_client, instances, predicates, functions, apply, true, node_id);
}

bool check(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client, uint32_t node_id)
{
  std::tuple<bool, bool, double, std::vector<std::map<std::string, std::string>>> ret =
    evaluate(tree, problem_client, false, node_id);

  return std::get<1>(ret);
}

bool check(
  const plansys2_msgs::msg::Tree & tree, std::unordered_set<plansys2::Instance> & instances,
  std::unordered_set<plansys2::Predicate> & predicates, std::vector<plansys2::Function> & functions,
  uint32_t node_id)
{
  std::tuple<bool, bool, double, std::vector<std::map<std::string, std::string>>> ret =
    evaluate(tree, instances, predicates, functions, false, node_id);

  return std::get<1>(ret);
}

bool apply(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client, uint32_t node_id)
{
  std::tuple<bool, bool, double, std::vector<std::map<std::string, std::string>>> ret =
    evaluate(tree, problem_client, true, node_id);

  return std::get<0>(ret);
}

bool apply(
  const plansys2_msgs::msg::Tree & tree, std::unordered_set<plansys2::Instance> & instances,
  std::unordered_set<plansys2::Predicate> & predicates, std::vector<plansys2::Function> & functions,
  uint32_t node_id)
{
  std::tuple<bool, bool, double, std::vector<std::map<std::string, std::string>>> ret =
    evaluate(tree, instances, predicates, functions, true, node_id);

  return std::get<0>(ret);
}

std::pair<std::string, int> parse_action(const std::string & input)
{
  std::string action = parser::pddl::getReducedString(input);
  int time = -1;

  size_t delim = action.find(":");
  if (delim != std::string::npos) {
    time = std::stoi(action.substr(delim + 1, action.length() - delim - 1));
    action.erase(action.begin() + delim, action.end());
  }

  action.erase(0, 1);  // remove initial (
  action.pop_back();   // remove last )

  return std::make_pair(action, time);
}

std::string get_action_expression(const std::string & input)
{
  auto action = parse_action(input);
  return action.first;
}

int get_action_time(const std::string & input)
{
  auto action = parse_action(input);
  return action.second;
}

std::string get_action_name(const std::string & input)
{
  auto expr = get_action_expression(input);
  size_t delim = expr.find(" ");
  return expr.substr(0, delim);
}

std::vector<std::string> get_action_params(const std::string & input)
{
  std::vector<std::string> ret;

  auto expr = get_action_expression(input);

  size_t delim = expr.find(" ");
  if (delim != std::string::npos) {
    expr.erase(expr.begin(), expr.begin() + delim + 1);
  }

  size_t start = 0, end = 0;
  while (end != std::string::npos) {
    end = expr.find(" ", start);
    auto param = expr.substr(start, (end == std::string::npos) ? std::string::npos : end - start);
    ret.push_back(param);
    start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
  }

  return ret;
}

}  // namespace plansys2
