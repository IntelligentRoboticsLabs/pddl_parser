// Copyright 2020 Intelligent Robotics Lab
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

#ifndef PLANSYS2_PROBLEM_EXPERT__UTILS_HPP_
#define PLANSYS2_PROBLEM_EXPERT__UTILS_HPP_

#include <tuple>
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <set>
#include <utility>
#include <unordered_set>

#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_msgs/msg/tree.hpp"

namespace plansys2
{

std::tuple<bool, std::vector<std::map<std::string, std::string>>> unifyPredicate(
  const plansys2::Predicate & predicate,
  const std::unordered_set<plansys2::Predicate> & predicates);

std::tuple<bool, std::vector<std::map<std::string, std::string>>> unifyFunction(
  const plansys2::Function & function,
  const std::vector<plansys2::Function> & functions);

void mergeParamsValuesDicts(
  const std::map<std::string, std::string> & dict1,
  const std::map<std::string, std::string> & dict2,
  std::map<std::string, std::string> & dict3);
std::vector<std::map<std::string, std::string>> mergeParamsValuesVector(
  const std::vector<std::map<std::string, std::string>> & vector1,
  const std::vector<std::map<std::string, std::string>> & vector2);
std::vector<std::map<std::string, std::string>> complementParamsValuesVector(
  const std::vector<plansys2_msgs::msg::Param> & params,
  const std::vector<std::map<std::string, std::string>> & param_dict_vector,
  const std::unordered_set<plansys2::Instance> & instances);

std::tuple<bool, std::vector<std::map<std::string, std::string>>> negateResult(
  const plansys2_msgs::msg::Node & node,
  const bool & result,
  const std::vector<std::map<std::string, std::string>> & param_dict_vector,
  const std::unordered_set<plansys2::Instance> & instances
);

std::unordered_set<plansys2::Predicate> solveAllDerivedPredicates(
  std::unordered_set<plansys2::Instance> & instances,
  std::unordered_set<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  const std::vector<plansys2_msgs::msg::Derived> & derived_predicates
);

std::unordered_set<plansys2::Predicate> solveDerivedPredicates(
  std::unordered_set<plansys2::Instance> & instances,
  std::unordered_set<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  const std::vector<plansys2_msgs::msg::Derived> & derived_predicates
);

void groundPredicate(
  std::unordered_set<plansys2::Instance> & instances,
  std::unordered_set<plansys2::Predicate> & current_predicates,
  const plansys2::Predicate & predicate,
  const std::vector<std::map<std::string, std::string>> & params_values_vector
);

/// Evaluate a PDDL expression represented as a tree.
/**
 * \param[in] node The root node of the PDDL expression.
 * \param[in] problem_client The problem expert client.
 * \param[in] instances Current instances state.
 * \param[in] predicates Current predicates state.
 * \param[in] functions Current functions state.
 * \param[in] apply Apply result to problem expert or state.
 * \param[in] use_state Use state representation or problem client.
 * \param[in] negate Invert the truth value.
 * \return result <- tuple(bool, bool, double)
 *         result(0) true if success
 *         result(1) truth value of boolean expression
 *         result(2) value of numeric expression
 *         result(3) vector with the set of possible  values for the expression parameters
 */
std::tuple<bool, bool, double, std::vector<std::map<std::string, std::string>>> evaluate(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  std::unordered_set<plansys2::Instance> & instances,
  std::unordered_set<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  bool apply = false,
  bool use_state = false,
  uint8_t node_id = 0,
  bool negate = false);

std::tuple<bool, bool, double, std::vector<std::map<std::string, std::string>>> evaluate(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  bool apply = false,
  uint32_t node_id = 0);

std::tuple<bool, bool, double, std::vector<std::map<std::string, std::string>>> evaluate(
  const plansys2_msgs::msg::Tree & tree,
  std::unordered_set<plansys2::Instance> & instances,
  std::unordered_set<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  bool apply = false,
  uint32_t node_id = 0);

/// Check a PDDL expression represented as a tree.
/**
* \param[in] node The root node of the PDDL expression.
* \param[in] problem_client The problem expert client.
* \return ret Truth value of the PDDL expression.
*
* This function calls the evaluate function.
*/
bool check(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  uint32_t node_id = 0);

bool check(
  const plansys2_msgs::msg::Tree & tree,
  std::unordered_set<plansys2::Instance> & instances,
  std::unordered_set<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  uint32_t node_id = 0);

/// Apply a PDDL expression represented as a tree.
/**
 * \param[in] node The root node of the PDDL expression.
 * \param[in] problem_client The problem expert client.
 * \return success Indicates whether the execution was successful.
 *
 * This function calls the evaluate function.
 */
bool apply(
  const plansys2_msgs::msg::Tree & tree,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client,
  uint32_t node_id = 0);

bool apply(
  const plansys2_msgs::msg::Tree & tree,
  std::unordered_set<plansys2::Instance> & instances,
  std::unordered_set<plansys2::Predicate> & predicates,
  std::vector<plansys2::Function> & functions,
  uint32_t node_id = 0);

/// Parse the action expression and time (optional) from an input string.
/**
* \param[in] input The input string.
* \return result <- pair(string, int)
*         result(0) The action expression.
*         result(1) The action start time.
*
* The input string can have either of the following formats.
*   "(<name> <param_1> ... <param_n>)"
*   "(<name> <param_1> ... <param_n>):<time>"
 * The output action expression will have the following format.
 *   "<name> <param_1> ... <param_n>"
*/
std::pair<std::string, int> parse_action(const std::string & input);

/// Parse the action expression from an input string.
/**
 * \param[in] input The input string.
 * \return The action expression.
 *
 * The input string can have either of the following formats.
 *   "(<name> <param_1> ... <param_n>)"
 *   "(<name> <param_1> ... <param_n>):<time>"
 */
std::string get_action_expression(const std::string & input);

/// Parse the action time from an input string.
/**
 * \param[in] input The input string.
 * \return The action start time.
 *
 * The input string can have either of the following formats.
 *   "(<name> <param_1> ... <param_n>)"
 *   "(<name> <param_1> ... <param_n>):<time>"
 */
int get_action_time(const std::string & input);

/// Parse the action name from an input string.
/**
 * \param[in] input The input string.
 * \return The name of the action.
 *
 * The input string can have either of the following formats.
 *   "(<name> <param_1> ... <param_n>)"
 *   "(<name> <param_1> ... <param_n>):<time>"
 */
std::string get_action_name(const std::string & input);

/// Parse the action parameter names from an input string.
/**
 * \param[in] input The input string.
 * \return A list of the action parameter names.
 *
 * The input string can have either of the following formats.
 *   "(<name> <param_1> ... <param_n>)"
 *   "(<name> <param_1> ... <param_n>):<time>"
 */
std::vector<std::string> get_action_params(const std::string & action_expr);

}  // namespace plansys2


#endif  // PLANSYS2_PROBLEM_EXPERT__UTILS_HPP_
