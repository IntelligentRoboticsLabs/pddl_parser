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

#ifndef PLANSYS2_PDDL_PARSER__UTILS_HPP_
#define PLANSYS2_PDDL_PARSER__UTILS_HPP_

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "plansys2_msgs/msg/action.hpp"
#include "plansys2_msgs/msg/durative_action.hpp"
#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/tree.hpp"

namespace parser
{
namespace pddl
{

/// Removes newlines, duplicated spaces, tabs and spaces from parenthesis
/**
 * \param[in] expr The expression to be reduced
 * \return The reduced expression
 */
std::string getReducedString(const std::string & expr);

/// Returns node type corresponding to the string input
/**
 * \param[in] input The input string
 * \return The node type
 */
uint8_t getNodeType(const std::string & expr, uint8_t def = plansys2_msgs::msg::Node::UNKNOWN);

/// Returns expression type and start position of an expression in a string
/**
 * \param[in] input The input string
 * \return result <- tuple(uint8_t, int)
 *         result(0) The expression type
 *         result(1) The start position of the first occurrence
 */
std::tuple<uint8_t, int> getExpr(const std::string & input);

/// Returns expression type corresponding to the string input
/**
 * \param[in] input The input string
 * \return The expression type
 */
uint8_t getExprType(const std::string & input);

/// Returns function modifier type and start position of a function modifier in a string
/**
 * \param[in] input The input string
 * \return result <- tuple(uint8_t, int)
 *         result(0) The function modifier type
 *         result(1) The start position of the first occurrence
 */
std::tuple<uint8_t, int> getFunMod(const std::string & input);

/// Returns function modifier type corresponding to the string input
/**
 * \param[in] input The input string
 * \return The function modifier type
 */
uint8_t getFunModType(const std::string & input);

int getParenthesis(const std::string & wexpr, int start);

std::vector<std::string> getSubExpr(const std::string & expr);

std::string nameActionsToString(const std::shared_ptr<plansys2_msgs::msg::Action> action);

std::string nameActionsToString(const std::shared_ptr<plansys2_msgs::msg::DurativeAction> action);

std::string toString(
  const plansys2_msgs::msg::Tree & tree, uint32_t node_id = 0, bool negate = false);

std::string toString(const plansys2_msgs::msg::Node & node);

std::string toStringPredicate(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringFunction(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringNumber(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringAnd(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringOr(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringNot(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringExpression(
  const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringFunctionModifier(
  const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringConstant(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringParameter(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

std::string toStringExists(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate);

/// This function creates a complete tree.
/**
 * This function recursivelly extracts the logic expressions and predicates from the expression.
 *
 * \param[in] tree The tree object to be created
 * \param[in] expr A string containing predicates and logic operators
 * \param[in] construct A string containing the associated PDDL constructs
 * \return A smart pointer to the node created
*/
plansys2_msgs::msg::Node::SharedPtr fromString(
  plansys2_msgs::msg::Tree & tree, const std::string & expr, bool negate = false,
  uint8_t parent = plansys2_msgs::msg::Node::UNKNOWN);

plansys2_msgs::msg::Tree fromString(
  const std::string & expr, bool negate = false,
  uint8_t parent = plansys2_msgs::msg::Node::UNKNOWN);

plansys2_msgs::msg::Node fromStringPredicate(const std::string & predicate);

plansys2_msgs::msg::Node fromStringExists(const std::string & exists);

plansys2_msgs::msg::Node fromStringFunction(const std::string & function);

plansys2_msgs::msg::Param fromStringParam(const std::string & name, const std::string & type = {});

plansys2_msgs::msg::Tree fromPredicates(const std::vector<std::string> & preds);

plansys2_msgs::msg::Tree::SharedPtr fromSubtree(
  const plansys2_msgs::msg::Tree & subtree, uint8_t node_type);

plansys2_msgs::msg::Tree::SharedPtr fromSubtrees(
  const std::vector<plansys2_msgs::msg::Tree> & subtrees, uint8_t node_type);

std::vector<uint32_t> getSubtreeIds(const plansys2_msgs::msg::Tree & tree);

std::vector<plansys2_msgs::msg::Tree> getSubtrees(const plansys2_msgs::msg::Tree & tree);

void getSubtreeChildren(
  plansys2_msgs::msg::Tree & subtree, const plansys2_msgs::msg::Tree & tree, uint32_t tree_parent,
  uint32_t subtree_parent);

void getPredicates(
  std::vector<plansys2_msgs::msg::Node> & predicates, const plansys2_msgs::msg::Tree & tree,
  uint32_t node_id = 0, bool negate = false);

void getFunctions(
  std::vector<plansys2_msgs::msg::Node> & functions, const plansys2_msgs::msg::Tree & tree,
  uint32_t node_id = 0, bool negate = false);

bool checkTreeEquality(
  const plansys2_msgs::msg::Tree & first, const plansys2_msgs::msg::Tree & second);

bool checkNodeEquality(
  const plansys2_msgs::msg::Node & first, const plansys2_msgs::msg::Node & second);

bool checkActionEquality(
  const plansys2_msgs::msg::Action & first, const plansys2_msgs::msg::Action & second);

bool checkDurativeActionEquality(
  const plansys2_msgs::msg::DurativeAction & first, const plansys2_msgs::msg::DurativeAction & second);

// bool checkNodeEqualityFreeParams(
//   const plansys2_msgs::msg::Node & first, const plansys2_msgs::msg::Node & second, std::vector<int> free_params);

bool checkParamEquality(
  const plansys2_msgs::msg::Param & first, const plansys2_msgs::msg::Param & second);

bool empty(const plansys2_msgs::msg::Tree & tree);

// Check if type of second is equal or a subtype of first
bool checkParamTypeEquivalence(
  const plansys2_msgs::msg::Param & first, const plansys2_msgs::msg::Param & second);

}  // namespace pddl
}  // namespace parser

#endif  // PLANSYS2_PDDL_PARSER__UTILS_HPP_
