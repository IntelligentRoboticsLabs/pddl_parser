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

#include "plansys2_pddl_parser/Utils.hpp"

#include <functional>
#include <iostream>
#include <limits>
#include <regex>

namespace parser
{
namespace pddl
{

static void printN(std::ostream & s, uint8_t node_type)
{
  switch (node_type) {
    case plansys2_msgs::msg::Node::UNKNOWN:
      s << "NODE TYPE UNKNOWN" << std::endl;
      break;
    case plansys2_msgs::msg::Node::AND:
      s << "NODE TYPE AND" << std::endl;
      break;
    case plansys2_msgs::msg::Node::OR:
      s << "NODE TYPE OR" << std::endl;
      break;
    case plansys2_msgs::msg::Node::NOT:
      s << "NODE TYPE NOT" << std::endl;
      break;
    case plansys2_msgs::msg::Node::PREDICATE:
      s << "NODE TYPE PRED" << std::endl;
      break;
    case plansys2_msgs::msg::Node::FUNCTION:
      s << "NODE TYPE FUNC" << std::endl;
      break;
    case plansys2_msgs::msg::Node::EXPRESSION:
      s << "NODE TYPE EXPR" << std::endl;
      break;
    case plansys2_msgs::msg::Node::NUMBER:
      s << "NODE TYPE NUM" << std::endl;
      break;
    case plansys2_msgs::msg::Node::EXISTS:
      s << "NODE TYPE EXISTS" << std::endl;
      break;
    case plansys2_msgs::msg::Node::COMP_GE:
      s << "NODE TYPE COMP GE" << std::endl;
      break;
    case plansys2_msgs::msg::Node::COMP_GT:
      s << "NODE TYPE COMP GT" << std::endl;
      break;
    case plansys2_msgs::msg::Node::COMP_LE:
      s << "NODE TYPE COMP LE" << std::endl;
      break;
    case plansys2_msgs::msg::Node::COMP_LT:
      s << "NODE TYPE COMP LT" << std::endl;
      break;
    case plansys2_msgs::msg::Node::COMP_EQ:
      s << "NODE TYPE COMP EQ" << std::endl;
      break;
    case plansys2_msgs::msg::Node::ARITH_ADD:
      s << "NODE TYPE ARITH ADD" << std::endl;
      break;
    case plansys2_msgs::msg::Node::ARITH_SUB:
      s << "NODE TYPE ARITH SUB" << std::endl;
      break;
    case plansys2_msgs::msg::Node::ARITH_MULT:
      s << "NODE TYPE ARITH MULT" << std::endl;
      break;
    case plansys2_msgs::msg::Node::ARITH_DIV:
      s << "NODE TYPE ARITH DIV" << std::endl;
      break;
    case plansys2_msgs::msg::Node::ASSIGN:
      s << "NODE TYPE ASSIGN" << std::endl;
      break;
    case plansys2_msgs::msg::Node::INCREASE:
      s << "NODE TYPE INCREASE" << std::endl;
      break;
    case plansys2_msgs::msg::Node::DECREASE:
      s << "NODE TYPE DECREASE" << std::endl;
      break;
    case plansys2_msgs::msg::Node::SCALE_UP:
      s << "NODE TYPE SCALE UP" << std::endl;
      break;
    case plansys2_msgs::msg::Node::SCALE_DOWN:
      s << "NODE TYPE SCALE DOWN" << std::endl;
      break;
    case plansys2_msgs::msg::Node::FUNCTION_MODIFIER:
      s << "NODE TYPE FUNCTION MODIFIER" << std::endl;
      break;
    default:
      s << "NODE TYPE UNDEFINED" << std::endl;
      break;
  }
}

std::string getReducedString(const std::string & expr)
{
  std::regex nts_chars("[\n\t]*", std::regex_constants::ECMAScript);
  std::string ret = std::regex_replace(expr, nts_chars, "");
  std::regex open_paren("\\( ", std::regex_constants::ECMAScript);
  ret = std::regex_replace(ret, open_paren, "(");
  std::regex close_paren(" \\)", std::regex_constants::ECMAScript);
  ret = std::regex_replace(ret, close_paren, ")");
  return ret;
}

uint8_t getNodeType(const std::string & expr, uint8_t default_node_type)
{
  auto node_type = default_node_type;

  std::smatch match;
  int first = std::numeric_limits<int>::max();

  if (std::regex_search(expr, match, std::regex("\\(\\s*and[ (]"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      node_type = plansys2_msgs::msg::Node::AND;
    }
  }

  if (std::regex_search(expr, match, std::regex("\\(\\s*or[ (]"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      node_type = plansys2_msgs::msg::Node::OR;
    }
  }

  if (std::regex_search(expr, match, std::regex("\\(\\s*not[ (]"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      node_type = plansys2_msgs::msg::Node::NOT;
    }
  }

  if (std::regex_search(expr, match, std::regex("\\(\\s*exists[ (]"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      node_type = plansys2_msgs::msg::Node::EXISTS;
    }
  }

  std::tuple<uint8_t, int> modifier_search_result = getFunMod(expr);
  if (std::get<0>(modifier_search_result) != plansys2_msgs::msg::Node::UNKNOWN) {
    if (std::get<1>(modifier_search_result) < first) {
      first = std::get<1>(modifier_search_result);
      node_type = plansys2_msgs::msg::Node::FUNCTION_MODIFIER;
    }
  }

  std::string wexpr = expr;
  while (wexpr.size() > 0) {
    std::regex num_regexp("[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)");

    if (std::regex_search(wexpr, match, num_regexp)) {
      bool valid_number = true;

      // Ignore integer characters within a parameter or construct name.
      // A valid number can only be preceded by a space or a left parenthesis.
      if (match.position() > 0) {
        valid_number = false;
        if (isspace(wexpr.at(match.position() - 1))) {
          valid_number = true;
        } else if (wexpr.at(match.position() - 1) == '(') {
          valid_number = true;
        }
      }

      if (valid_number) {
        if (static_cast<int>(match.position()) < first) {
          first = static_cast<int>(match.position());
          node_type = plansys2_msgs::msg::Node::NUMBER;
        }
        break;
      } else {
        wexpr.erase(wexpr.begin(), wexpr.begin() + match.position() + match.length());
      }
    } else {
      break;
    }
  }

  // The number search must precede the expression search in order to differentiate between an
  // addition or subtraction expression and a number with a "+" or "-" prefix.
  std::tuple<uint8_t, int> expression_search_result = getExpr(expr);
  if (std::get<0>(expression_search_result) != plansys2_msgs::msg::Node::UNKNOWN) {
    if (std::get<1>(expression_search_result) < first) {
      first = std::get<1>(expression_search_result);
      node_type = plansys2_msgs::msg::Node::EXPRESSION;
    }
  }

  return node_type;
}

std::tuple<uint8_t, int> getExpr(const std::string & input)
{
  auto expr_type = plansys2_msgs::msg::Node::UNKNOWN;

  std::smatch match;
  int first = std::numeric_limits<int>::max();

  if (std::regex_search(input, match, std::regex("\\s*>="))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      expr_type = plansys2_msgs::msg::Node::COMP_GE;
    }
  }

  if (std::regex_search(input, match, std::regex("\\s*>"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      expr_type = plansys2_msgs::msg::Node::COMP_GT;
    }
  }

  if (std::regex_search(input, match, std::regex("\\s*<="))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      expr_type = plansys2_msgs::msg::Node::COMP_LE;
    }
  }

  if (std::regex_search(input, match, std::regex("\\s*<"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      expr_type = plansys2_msgs::msg::Node::COMP_LT;
    }
  }

  if (std::regex_search(input, match, std::regex("\\s*="))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      expr_type = plansys2_msgs::msg::Node::COMP_EQ;
    }
  }

  if (std::regex_search(input, match, std::regex("\\s*\\*"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      expr_type = plansys2_msgs::msg::Node::ARITH_MULT;
    }
  }

  if (std::regex_search(input, match, std::regex("\\s*\\/"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      expr_type = plansys2_msgs::msg::Node::ARITH_DIV;
    }
  }

  if (std::regex_search(input, match, std::regex("\\s*\\+"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      expr_type = plansys2_msgs::msg::Node::ARITH_ADD;
    }
  }

  if (std::regex_search(input, match, std::regex("\\s*\\-"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      expr_type = plansys2_msgs::msg::Node::ARITH_SUB;
    }
  }

  return std::make_tuple(expr_type, first);
}

uint8_t getExprType(const std::string & input)
{
  std::tuple<uint8_t, int> result = getExpr(input);
  return std::get<0>(result);
}

std::tuple<uint8_t, int> getFunMod(const std::string & input)
{
  auto fun_mod_type = plansys2_msgs::msg::Node::UNKNOWN;

  std::smatch match;
  int first = std::numeric_limits<int>::max();

  if (std::regex_search(input, match, std::regex("assign"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      fun_mod_type = plansys2_msgs::msg::Node::ASSIGN;
    }
  }

  if (std::regex_search(input, match, std::regex("increase"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      fun_mod_type = plansys2_msgs::msg::Node::INCREASE;
    }
  }

  if (std::regex_search(input, match, std::regex("decrease"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      fun_mod_type = plansys2_msgs::msg::Node::DECREASE;
    }
  }

  if (std::regex_search(input, match, std::regex("scale-up"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      fun_mod_type = plansys2_msgs::msg::Node::SCALE_UP;
    }
  }

  if (std::regex_search(input, match, std::regex("scale-down"))) {
    if (static_cast<int>(match.position()) < first) {
      first = static_cast<int>(match.position());
      fun_mod_type = plansys2_msgs::msg::Node::SCALE_DOWN;
    }
  }

  return std::make_tuple(fun_mod_type, first);
}

uint8_t getFunModType(const std::string & input)
{
  std::tuple<uint8_t, int> result = getFunMod(input);
  return std::get<0>(result);
}

int getParenthesis(const std::string & wexpr, int start)
{
  int it = start + 1;
  int balance = 1;

  while (it < wexpr.size()) {
    if (wexpr[it] == '(') {
      balance++;
    }
    if (wexpr[it] == ')') {
      balance--;
    }

    if (balance == 0) {
      return it;
    }

    it++;
  }

  return it;
}

std::vector<std::string> getSubExpr(const std::string & expr)
{
  std::vector<std::string> ret;

  std::string wexpr(expr);

  // Remove first ( and last )
  while (wexpr.back() == ' ') {
    wexpr.pop_back();
  }
  wexpr.pop_back();
  while (wexpr.front() == ' ') {
    wexpr.erase(0, 1);
  }
  wexpr.erase(0, 1);

  while (wexpr.size() > 0) {
    int first = wexpr.find("(");

    std::smatch match;
    std::regex num_regexp("[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)");
    bool found_num = std::regex_search(wexpr, match, num_regexp);

    if (found_num && first != std::string::npos) {
      if (match.position() < first) {
        ret.push_back(wexpr.substr(match.position(), match.length()));
        wexpr.erase(wexpr.begin(), wexpr.begin() + match.position() + match.length());
      } else {
        int last = getParenthesis(wexpr, first);
        ret.push_back(wexpr.substr(first, last - first + 1));
        wexpr.erase(wexpr.begin(), wexpr.begin() + last + 1);
      }
    } else if (found_num) {
      ret.push_back(wexpr.substr(match.position(), match.length()));
      wexpr.erase(wexpr.begin(), wexpr.begin() + match.position() + match.length());
    } else if (first != std::string::npos) {
      int last = getParenthesis(wexpr, first);
      ret.push_back(wexpr.substr(first, last - first + 1));
      wexpr.erase(wexpr.begin(), wexpr.begin() + last + 1);
    } else {
      break;
    }
  }

  return ret;
}

std::string nameActionsToString(const std::shared_ptr<plansys2_msgs::msg::Action> action)
{
  std::string ret = action->name;
  for (const auto & param : action->parameters) {
    ret = ret + " " + param.name;
  }
  return ret;
}

std::string nameActionsToString(const std::shared_ptr<plansys2_msgs::msg::DurativeAction> action)
{
  std::string ret = action->name;
  for (const auto & param : action->parameters) {
    ret = ret + " " + param.name;
  }
  return ret;
}

std::string toString(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  std::string ret;
  switch (tree.nodes[node_id].node_type) {
    case plansys2_msgs::msg::Node::PREDICATE:
      ret = toStringPredicate(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::FUNCTION:
      ret = toStringFunction(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::NUMBER:
      ret = toStringNumber(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::AND:
      ret = toStringAnd(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::OR:
      ret = toStringOr(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::NOT:
      ret = toStringNot(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::EXPRESSION:
      ret = toStringExpression(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::FUNCTION_MODIFIER:
      ret = toStringFunctionModifier(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::CONSTANT:
      ret = toStringConstant(tree, node_id, negate);
      break;
    case plansys2_msgs::msg::Node::PARAMETER:
      ret = toStringParameter(tree, node_id, negate);
    case plansys2_msgs::msg::Node::EXISTS:
      ret = toStringExists(tree, node_id, negate);
      break;
    default:
      std::cerr << "Unsupported node to string conversion" << std::endl;
      break;
  }

  return ret;
}

std::string toString(const plansys2_msgs::msg::Node & node)
{
  if (
    node.node_type != plansys2_msgs::msg::Node::PREDICATE &&
    node.node_type != plansys2_msgs::msg::Node::FUNCTION)
  {
    std::cerr << "Unsupported node to string conversion" << std::endl;
    return {};
  }

  plansys2_msgs::msg::Tree tree;
  tree.nodes.push_back(node);

  return toString(tree);
}

std::string toStringPredicate(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  std::string ret;

  if (negate) {
    ret = "(not (" + tree.nodes[node_id].name;
  } else {
    ret = "(" + tree.nodes[node_id].name;
  }

  for (const auto & param : tree.nodes[node_id].parameters) {
    ret += " " + param.name;
  }

  if (negate) {
    ret += "))";
  } else {
    ret += ")";
  }

  return ret;
}

std::string toStringFunction(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  std::string ret;

  ret = "(" + tree.nodes[node_id].name;
  for (const auto & param : tree.nodes[node_id].parameters) {
    ret += " " + param.name;
  }

  ret += ")";

  return ret;
}

std::string toStringNumber(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  return std::to_string(tree.nodes[node_id].value);
}

std::string toStringAnd(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  if (tree.nodes[node_id].children.empty()) {
    return {};
  }

  std::string ret;

  if (!negate) {
    ret = "(and ";
  } else {
    ret = "(or ";
  }

  for (auto child_id : tree.nodes[node_id].children) {
    ret += toString(tree, child_id, negate);
  }
  ret += ")";

  return ret;
}

std::string toStringOr(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  if (tree.nodes[node_id].children.empty()) {
    return {};
  }

  std::string ret;

  if (!negate) {
    ret = "(or ";
  } else {
    ret = "(and ";
  }

  for (auto child_id : tree.nodes[node_id].children) {
    ret += toString(tree, child_id, negate);
  }
  ret += ")";

  return ret;
}

std::string toStringNot(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  if (tree.nodes[node_id].children.empty()) {
    return {};
  }
  return toString(tree, tree.nodes[node_id].children[0], !negate);
}

std::string toStringExpression(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  if (tree.nodes[node_id].children.size() < 2) {
    return {};
  }

  std::string ret;
  if (negate) {
    ret = "(not ";
  }

  switch (tree.nodes[node_id].expression_type) {
    case plansys2_msgs::msg::Node::COMP_GE:
      ret += "(>= ";
      break;
    case plansys2_msgs::msg::Node::COMP_GT:
      ret += "(> ";
      break;
    case plansys2_msgs::msg::Node::COMP_LE:
      ret += "(<= ";
      break;
    case plansys2_msgs::msg::Node::COMP_LT:
      ret += "(< ";
      break;
    case plansys2_msgs::msg::Node::COMP_EQ:
      ret += "(= ";
      break;
    case plansys2_msgs::msg::Node::ARITH_MULT:
      ret = "(* ";
      break;
    case plansys2_msgs::msg::Node::ARITH_DIV:
      ret = "(/ ";
      break;
    case plansys2_msgs::msg::Node::ARITH_ADD:
      ret = "(+ ";
      break;
    case plansys2_msgs::msg::Node::ARITH_SUB:
      ret = "(- ";
      break;
    default:
      ret = "(UNKNOWN ";
      std::cerr << "toStringExpression: Unknown expression type: ";
      printN(std::cerr, tree.nodes[node_id].expression_type);
      std::cerr << std::endl;
      break;
  }

  for (auto child_id : tree.nodes[node_id].children) {
    ret += " " + toString(tree, child_id, negate);
  }
  ret += ")";
  if (negate) {
    ret += ")";
  }

  return ret;
}

std::string toStringFunctionModifier(
  const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  if (tree.nodes[node_id].children.size() < 2) {
    return {};
  }

  std::string ret;

  switch (tree.nodes[node_id].modifier_type) {
    case plansys2_msgs::msg::Node::ASSIGN:
      ret = "(assign ";
      break;
    case plansys2_msgs::msg::Node::INCREASE:
      ret = "(increase ";
      break;
    case plansys2_msgs::msg::Node::DECREASE:
      ret = "(decrease ";
      break;
    case plansys2_msgs::msg::Node::SCALE_UP:
      ret = "(scale-up ";
      break;
    case plansys2_msgs::msg::Node::SCALE_DOWN:
      ret = "(scale-down ";
      break;
    default:
      break;
  }

  for (auto child_id : tree.nodes[node_id].children) {
    ret += toString(tree, child_id, negate);
  }
  ret += ")";

  return ret;
}

std::string toStringConstant(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  return tree.nodes[node_id].name;
}

std::string toStringParameter(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  return tree.nodes[node_id].parameters[0].name;
}

std::string toStringExists(const plansys2_msgs::msg::Tree & tree, uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return {};
  }

  if (tree.nodes[node_id].children.empty()) {
    return {};
  }

  std::string ret = "(exists (";

  bool first_param = true;
  for (const auto & param : tree.nodes[node_id].parameters) {
    ret += first_param ? param.name : " " + param.name;
    first_param = false;
  }
  ret += ") ";

  for (auto child_id : tree.nodes[node_id].children) {
    ret += toString(tree, child_id, negate);
  }
  ret += ")";

  return ret;
}

plansys2_msgs::msg::Node::SharedPtr fromString(
  plansys2_msgs::msg::Tree & tree, const std::string & expr, bool negate, uint8_t parent)
{
  std::string wexpr = getReducedString(expr);

  if (wexpr == "(and)" || wexpr == "(and )") {
    return nullptr;
  }

  auto default_node_type = plansys2_msgs::msg::Node::UNKNOWN;
  switch (parent) {
    case plansys2_msgs::msg::Node::AND:
      default_node_type = plansys2_msgs::msg::Node::PREDICATE;
      break;
    case plansys2_msgs::msg::Node::OR:
      default_node_type = plansys2_msgs::msg::Node::PREDICATE;
      break;
    case plansys2_msgs::msg::Node::NOT:
      default_node_type = plansys2_msgs::msg::Node::PREDICATE;
      break;
    case plansys2_msgs::msg::Node::EXPRESSION:
      default_node_type = plansys2_msgs::msg::Node::FUNCTION;
      break;
    case plansys2_msgs::msg::Node::FUNCTION_MODIFIER:
      default_node_type = plansys2_msgs::msg::Node::FUNCTION;
      break;
  }

  auto node_type = getNodeType(wexpr, default_node_type);

  switch (node_type) {
    case plansys2_msgs::msg::Node::AND: {
        auto node = std::make_shared<plansys2_msgs::msg::Node>();
        node->node_type = node_type;
        node->node_id = tree.nodes.size();
        node->negate = negate;
        tree.nodes.push_back(*node);

        std::vector<std::string> subexprs = getSubExpr(wexpr);

        for (const auto & e : subexprs) {
          auto child = fromString(tree, e, negate, node_type);
          tree.nodes[node->node_id].children.push_back(child->node_id);
        }

        return node;
      }
    case plansys2_msgs::msg::Node::OR: {
        auto node = std::make_shared<plansys2_msgs::msg::Node>();
        node->node_type = node_type;
        node->node_id = tree.nodes.size();
        node->negate = negate;
        tree.nodes.push_back(*node);

        std::vector<std::string> subexprs = getSubExpr(wexpr);

        for (const auto & e : subexprs) {
          auto child = fromString(tree, e, negate, node_type);
          tree.nodes[node->node_id].children.push_back(child->node_id);
        }

        return node;
      }
    case plansys2_msgs::msg::Node::NOT: {
        auto node = std::make_shared<plansys2_msgs::msg::Node>();
        node->node_type = node_type;
        node->node_id = tree.nodes.size();
        node->negate = negate;
        tree.nodes.push_back(*node);

        std::vector<std::string> subexprs = getSubExpr(wexpr);

        auto child = fromString(tree, subexprs[0], !negate, node_type);
        tree.nodes[node->node_id].children.push_back(child->node_id);

        return node;
      }
    case plansys2_msgs::msg::Node::PREDICATE: {
        auto node = std::make_shared<plansys2_msgs::msg::Node>(fromStringPredicate(wexpr));
        node->node_id = tree.nodes.size();
        node->negate = negate;
        tree.nodes.push_back(*node);

        return node;
      }
    case plansys2_msgs::msg::Node::FUNCTION: {
        auto node = std::make_shared<plansys2_msgs::msg::Node>(fromStringFunction(wexpr));
        node->node_id = tree.nodes.size();
        node->negate = negate;
        tree.nodes.push_back(*node);

        return node;
      }
    case plansys2_msgs::msg::Node::EXPRESSION: {
        auto node = std::make_shared<plansys2_msgs::msg::Node>();
        node->node_type = node_type;
        node->expression_type = getExprType(wexpr);
        node->node_id = tree.nodes.size();
        node->negate = negate;
        tree.nodes.push_back(*node);

        std::vector<std::string> subexprs = getSubExpr(wexpr);

        for (const auto & e : subexprs) {
          auto child = fromString(tree, e, false, node_type);
          tree.nodes[node->node_id].children.push_back(child->node_id);
        }

        return node;
      }
    case plansys2_msgs::msg::Node::FUNCTION_MODIFIER: {
        auto node = std::make_shared<plansys2_msgs::msg::Node>();
        node->node_type = node_type;
        node->modifier_type = getFunModType(wexpr);
        node->node_id = tree.nodes.size();
        node->negate = negate;
        tree.nodes.push_back(*node);

        std::vector<std::string> subexprs = getSubExpr(wexpr);

        for (const auto & e : subexprs) {
          auto child = fromString(tree, e, false, node_type);
          tree.nodes[node->node_id].children.push_back(child->node_id);
        }

        return node;
      }
    case plansys2_msgs::msg::Node::NUMBER: {
        auto node = std::make_shared<plansys2_msgs::msg::Node>();
        node->node_type = node_type;
        node->node_id = tree.nodes.size();
        node->value = std::stod(wexpr);
        node->negate = negate;
        tree.nodes.push_back(*node);

        return node;
      }

    case plansys2_msgs::msg::Node::EXISTS: {
        auto node = std::make_shared<plansys2_msgs::msg::Node>(fromStringExists(wexpr));
        node->node_id = tree.nodes.size();
        node->negate = negate;
        tree.nodes.push_back(*node);

        size_t begin_exists = wexpr.find("exists", 0) + 6;
        size_t end_exists = wexpr.find(")", begin_exists);
        wexpr = wexpr.substr(end_exists + 1, std::string::npos);

        auto child = fromString(tree, wexpr, negate, node_type);
        tree.nodes[node->node_id].children.push_back(child->node_id);

        return node;
      }
    // LCOV_EXCL_START
    default:
      std::cerr << "fromString: Error parsing expresion [" << wexpr << "]" << std::endl;
      // LCOV_EXCL_STOP
  }

  return nullptr;
}

plansys2_msgs::msg::Tree fromString(const std::string & expr, bool negate, uint8_t parent)
{
  plansys2_msgs::msg::Tree tree;
  fromString(tree, expr, negate, parent);
  return tree;
}

plansys2_msgs::msg::Node fromStringPredicate(const std::string & predicate)
{
  plansys2_msgs::msg::Node ret;
  ret.node_type = plansys2_msgs::msg::Node::PREDICATE;

  std::vector<std::string> tokens;
  size_t start = 0, end = 0;

  while (end != std::string::npos) {
    end = predicate.find(" ", start);
    tokens.push_back(
      predicate.substr(start, (end == std::string::npos) ? std::string::npos : end - start));
    start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
  }

  tokens[0].erase(0, 1);
  ret.name = tokens[0];

  tokens.back().pop_back();

  for (size_t i = 1; i < tokens.size(); i++) {
    plansys2_msgs::msg::Param param;
    param.name = tokens[i];
    ret.parameters.push_back(param);
  }

  ret.value = 0;

  return ret;
}

plansys2_msgs::msg::Node fromStringExists(const std::string & exists)
{
  plansys2_msgs::msg::Node ret;
  ret.node_type = plansys2_msgs::msg::Node::EXISTS;

  std::vector<std::string> tokens;
  size_t start = 0;
  size_t end_exists = exists.find(")", start);
  size_t end = 0;

  while (end < end_exists) {
    end = exists.find(" ", start);
    tokens.push_back(
      exists.substr(start, (end == std::string::npos) ? std::string::npos : end - start));
    start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
  }

  tokens.back().pop_back();
  tokens[1].erase(0, 1);

  for (size_t i = 1; i < tokens.size(); i++) {
    plansys2_msgs::msg::Param param;
    param.name = tokens[i];
    ret.parameters.push_back(param);
  }

  ret.value = 0;

  return ret;
}

plansys2_msgs::msg::Node fromStringFunction(const std::string & function)
{
  plansys2_msgs::msg::Node ret;
  ret.node_type = plansys2_msgs::msg::Node::FUNCTION;

  std::regex name_regexp("[a-zA-Z][a-zA-Z0-9_\\-]*");
  std::regex number_regexp("[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)");

  std::smatch match;
  std::string temp = function;

  if (std::regex_search(temp, match, name_regexp)) {
    ret.name = match.str(0);
    temp = match.suffix().str();
  }

  while (std::regex_search(temp, match, name_regexp)) {
    plansys2_msgs::msg::Param param;
    param.name = match.str(0);
    ret.parameters.push_back(param);
    temp = match.suffix().str();
  }

  if (std::regex_search(temp, match, number_regexp)) {
    ret.value = std::stod(match.str(0));
  }

  return ret;
}

plansys2_msgs::msg::Param fromStringParam(const std::string & name, const std::string & type)
{
  plansys2_msgs::msg::Param ret;
  ret.name = name;
  ret.type = type;
  return ret;
}

plansys2_msgs::msg::Tree fromPredicates(const std::vector<std::string> & preds)
{
  plansys2_msgs::msg::Node node;
  node.node_type = plansys2_msgs::msg::Node::AND;
  node.node_id = 0;
  node.negate = false;

  plansys2_msgs::msg::Tree tree;
  tree.nodes.push_back(node);

  for (const auto & pred : preds) {
    auto child = fromStringPredicate(pred);
    child.node_id = tree.nodes.size();
    child.negate = false;
    tree.nodes.push_back(child);
    tree.nodes[0].children.push_back(child.node_id);
  }

  return tree;
}

plansys2_msgs::msg::Tree::SharedPtr fromSubtree(
  const plansys2_msgs::msg::Tree & subtree, uint8_t node_type)
{
  std::vector<plansys2_msgs::msg::Tree> temp;
  temp.push_back(subtree);
  return fromSubtrees(temp, node_type);
}

plansys2_msgs::msg::Tree::SharedPtr fromSubtrees(
  const std::vector<plansys2_msgs::msg::Tree> & subtrees, uint8_t node_type)
{
  if (
    node_type != plansys2_msgs::msg::Node::AND && node_type != plansys2_msgs::msg::Node::OR &&
    node_type != plansys2_msgs::msg::Node::NOT)
  {
    std::cerr << "fromSubtrees: Unsupported root type." << std::endl;
    return nullptr;
  }

  if (node_type == plansys2_msgs::msg::Node::NOT && subtrees.size() > 1) {
    std::cerr << "fromSubtree: A NOT node can only operate on single subtree." << std::endl;
    return nullptr;
  }

  if (subtrees.empty()) {
    std::cerr << "fromSubtree: The subtrees vector is empty." << std::endl;
    return nullptr;
  }

  plansys2_msgs::msg::Node node;
  node.node_type = node_type;
  node.node_id = 0;
  node.negate = false;

  auto tree = std::make_shared<plansys2_msgs::msg::Tree>();
  tree->nodes.push_back(node);

  for (unsigned i = 0; i < subtrees.size(); ++i) {
    auto tree_size = tree->nodes.size();
    tree->nodes[0].children.push_back(tree_size);
    tree->nodes.insert(
      std::end(tree->nodes), std::begin(subtrees[i].nodes), std::end(subtrees[i].nodes));
    for (unsigned j = 0; j < subtrees[i].nodes.size(); ++j) {
      tree->nodes[tree_size + j].node_id += tree_size;
      for (unsigned k = 0; k < subtrees[i].nodes[j].children.size(); ++k) {
        tree->nodes[tree_size + j].children[k] += tree_size;
      }
    }
    if (node_type == plansys2_msgs::msg::Node::NOT) {
      tree->nodes[1].negate = true;
      break;
    }
  }

  return tree;
}

std::vector<uint32_t> getSubtreeIds(const plansys2_msgs::msg::Tree & tree)
{
  if (tree.nodes.empty()) {  // No expression
    return {};
  }

  switch (tree.nodes.front().node_type) {
    case plansys2_msgs::msg::Node::AND: {
        return tree.nodes.front().children;
      }
    default:
      std::cerr << "getSubtreeIds: Error parsing expresion [" << toString(tree) << "]" << std::endl;
  }

  return {};
}

std::vector<plansys2_msgs::msg::Tree> getSubtrees(const plansys2_msgs::msg::Tree & tree)
{
  std::vector<uint32_t> node_ids = parser::pddl::getSubtreeIds(tree);
  std::vector<plansys2_msgs::msg::Tree> subtrees;
  for (auto node_id : node_ids) {
    plansys2_msgs::msg::Tree subtree;
    subtree.nodes.push_back(tree.nodes[node_id]);
    subtree.nodes[0].node_id = 0;
    subtree.nodes[0].children.clear();
    getSubtreeChildren(subtree, tree, node_id, 0);
    subtrees.push_back(subtree);
  }
  return subtrees;
}

void getSubtreeChildren(
  plansys2_msgs::msg::Tree & subtree, const plansys2_msgs::msg::Tree & tree, uint32_t tree_parent,
  uint32_t subtree_parent)
{
  for (auto child_id : tree.nodes[tree_parent].children) {
    auto subtree_size = subtree.nodes.size();
    subtree.nodes[subtree_parent].children.push_back(subtree_size);
    subtree.nodes.push_back(tree.nodes[child_id]);
    subtree.nodes.back().node_id = subtree_size;
    subtree.nodes.back().children.clear();
    getSubtreeChildren(subtree, tree, child_id, subtree_size);
  }
}

void getPredicates(
  std::vector<plansys2_msgs::msg::Node> & predicates, const plansys2_msgs::msg::Tree & tree,
  uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return;
  }

  switch (tree.nodes[node_id].node_type) {
    case plansys2_msgs::msg::Node::FUNCTION:
    case plansys2_msgs::msg::Node::EXPRESSION:
    case plansys2_msgs::msg::Node::FUNCTION_MODIFIER:
    case plansys2_msgs::msg::Node::NUMBER:
      // These cases have no meaning
      break;
    case plansys2_msgs::msg::Node::AND:
      for (auto child_id : tree.nodes[node_id].children) {
        getPredicates(predicates, tree, child_id, negate);
      }
      break;
    case plansys2_msgs::msg::Node::OR:
      for (auto child_id : tree.nodes[node_id].children) {
        getPredicates(predicates, tree, child_id, negate);
      }
      break;
    case plansys2_msgs::msg::Node::NOT:
      getPredicates(predicates, tree, tree.nodes[node_id].children[0], !negate);
      break;
    case plansys2_msgs::msg::Node::ACTION:
      for (auto child_id : tree.nodes[node_id].children) {
        getPredicates(predicates, tree, child_id, negate);
      }
      break;
    case plansys2_msgs::msg::Node::PREDICATE:
      plansys2_msgs::msg::Node pred = tree.nodes[node_id];
      if (
        std::find_if(
          predicates.begin(), predicates.end(),
          std::bind(&checkNodeEquality, std::placeholders::_1, pred)) == predicates.end())
      {
        pred.negate = negate;
        predicates.push_back(pred);
      }
      break;
  }
}

void getFunctions(
  std::vector<plansys2_msgs::msg::Node> & functions, const plansys2_msgs::msg::Tree & tree,
  uint32_t node_id, bool negate)
{
  if (node_id >= tree.nodes.size()) {
    return;
  }

  switch (tree.nodes[node_id].node_type) {
    case plansys2_msgs::msg::Node::AND:
    case plansys2_msgs::msg::Node::OR:
    case plansys2_msgs::msg::Node::NOT:
    case plansys2_msgs::msg::Node::NUMBER:
      // These cases have no meaning
      break;
    case plansys2_msgs::msg::Node::ACTION:
      for (auto child_id : tree.nodes[node_id].children) {
        getFunctions(functions, tree, child_id, negate);
      }
      break;
    case plansys2_msgs::msg::Node::EXPRESSION:
      for (auto child_id : tree.nodes[node_id].children) {
        getFunctions(functions, tree, child_id, negate);
      }
      break;
    case plansys2_msgs::msg::Node::FUNCTION_MODIFIER:
      for (auto child_id : tree.nodes[node_id].children) {
        getFunctions(functions, tree, child_id, negate);
      }
      break;
    case plansys2_msgs::msg::Node::FUNCTION:
      plansys2_msgs::msg::Node func = tree.nodes[node_id];
      if (
        std::find_if(
          functions.begin(), functions.end(),
          std::bind(&checkNodeEquality, std::placeholders::_1, func)) == functions.end())
      {
        func.value = 0.0;
        functions.push_back(func);
      }
      break;
  }
}

bool checkTreeEquality(
  const plansys2_msgs::msg::Tree & first, const plansys2_msgs::msg::Tree & second)
{
  if (first.nodes.size() != second.nodes.size()) {
    return false;
  }

  for (unsigned i = 0; i < first.nodes.size(); i++) {
    if (!checkNodeEquality(first.nodes[i], second.nodes[i])) {
      return false;
    }
  }

  return true;
}

bool checkNodeEquality(
  const plansys2_msgs::msg::Node & first, const plansys2_msgs::msg::Node & second)
{
  if (first.node_type != second.node_type) {
    return false;
  }

  if (
    first.node_type == plansys2_msgs::msg::Node::ACTION ||
    first.node_type == plansys2_msgs::msg::Node::PREDICATE ||
    first.node_type == plansys2_msgs::msg::Node::FUNCTION)
  {
    if (first.name != second.name) {
      return false;
    }
  }

  if (first.node_type == plansys2_msgs::msg::Node::EXPRESSION) {
    if (first.expression_type != second.expression_type) {
      return false;
    }
  }

  if (first.node_type == plansys2_msgs::msg::Node::FUNCTION_MODIFIER) {
    if (first.modifier_type != second.modifier_type) {
      return false;
    }
  }

  if (first.node_type == plansys2_msgs::msg::Node::NUMBER) {
    if (abs(first.value - second.value) > 1e-9) {
      return false;
    }
  }

  if (first.children.size() != second.children.size()) {
    return false;
  }

  if (first.parameters.size() != second.parameters.size()) {
    return false;
  }

  for (unsigned i = 0; i < first.parameters.size(); i++) {
    if (!checkParamEquality(first.parameters[i], second.parameters[i])) {
      return false;
    }
  }

  return true;
}

bool checkParamEquality(
  const plansys2_msgs::msg::Param & first, const plansys2_msgs::msg::Param & second)
{
  if (first.name != second.name) {
    return false;
  }

  return true;
}

bool empty(const plansys2_msgs::msg::Tree & tree)
{
  if (tree.nodes.empty()) {
    return true;
  }

  if (
    (tree.nodes[0].node_type == plansys2_msgs::msg::Node::AND ||
    tree.nodes[0].node_type == plansys2_msgs::msg::Node::OR ||
    tree.nodes[0].node_type == plansys2_msgs::msg::Node::NOT ||
    tree.nodes[0].node_type == plansys2_msgs::msg::Node::EXPRESSION ||
    tree.nodes[0].node_type == plansys2_msgs::msg::Node::FUNCTION_MODIFIER) &&
    tree.nodes[0].children.empty())
  {
    return true;
  }

  return false;
}

}  // namespace pddl
}  // namespace parser
