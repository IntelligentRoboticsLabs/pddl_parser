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
#include <iomanip>

#include "plansys2_pddl_parser/Domain.hpp"

namespace parser
{
namespace pddl
{

template<>
void GroundFunc<double>::PDDLPrint(
  std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts, const Domain & d) const
{
  tabindent(s, indent);
  s << "( = ";
  TypeGround::PDDLPrint(s, 0, ts, d);
  s << " " << std::fixed << std::setprecision(10) << static_cast<double>(value) << " )";
}

template<>
void GroundFunc<int>::PDDLPrint(
  std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts, const Domain & d) const
{
  tabindent(s, indent);
  s << "( = ";
  TypeGround::PDDLPrint(s, 0, ts, d);
  s << " " << d.types[(reinterpret_cast<Function *>(lifted))->returnType]->object(value) << " )";
}

template<>
plansys2_msgs::msg::Node::SharedPtr GroundFunc<double>::getTree(
  plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace) const
{
  auto node = TypeGround::getTree(tree, d, replace);
  node->value = value;
  return node;
}

template<>
plansys2_msgs::msg::Node::SharedPtr GroundFunc<int>::getTree(
  plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace) const
{
  auto node = TypeGround::getTree(tree, d, replace);
  node->value = value;
  return node;
}

template<>
void GroundFunc<double>::parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
{
  TypeGround::parse(f, ts, d);

  f.next();
  std::string s = f.getToken();
  std::istringstream i(s);
  if (!(i >> value)) {
    f.tokenExit(s);
  }

  f.next();
  f.assert_token(")");
}

template<>
void GroundFunc<int>::parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
{
  TypeGround::parse(f, ts, d);

  f.next();
  std::string s = f.getToken();
  std::pair<bool, unsigned> p =
    d.types[(reinterpret_cast<Function *>(lifted))->returnType]->parseObject(s);
  if (p.first) {
    value = p.second;
  } else {
    std::pair<bool, int> q =
      d.types[(reinterpret_cast<Function *>(lifted))->returnType]->parseConstant(s);
    if (q.first) {
      value = q.second;
    } else {
      f.tokenExit(s);
    }
  }

  f.next();
  f.assert_token(")");
}

}  // namespace pddl
}  // namespace parser
