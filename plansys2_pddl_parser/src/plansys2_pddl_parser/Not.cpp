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
#include "plansys2_pddl_parser/Domain.hpp"

namespace parser
{
namespace pddl
{

void Not::PDDLPrint(
  std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts, const Domain & d) const
{
  tabindent(s, indent);
  s << "( not ";
  if (cond) {
    cond->PDDLPrint(s, 0, ts, d);
  }
  s << " )";
}

plansys2_msgs::msg::Node::SharedPtr Not::getTree(
  plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace) const
{
  plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
  node->node_type = plansys2_msgs::msg::Node::NOT;
  node->node_id = tree.nodes.size();
  tree.nodes.push_back(*node);

  if (cond) {
    plansys2_msgs::msg::Node::SharedPtr child = cond->getTree(tree, d, replace);
    tree.nodes[node->node_id].children.push_back(child->node_id);
  }

  return node;
}

void Not::parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
{
  f.next();
  f.assert_token("(");

  cond = d.createCondition(f);

  if (!cond) {
    f.tokenExit(f.getToken());
  }

  cond->parse(f, ts, d);

  f.next();
  f.assert_token(")");
}

}  // namespace pddl
}  // namespace parser
