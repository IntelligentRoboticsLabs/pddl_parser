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
#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_pddl_parser/Condition.hpp"

namespace parser
{
namespace pddl
{

class Oneof : public Condition
{
public:
  CondVec conds;

  Oneof() {}

  Oneof(const Oneof * o, Domain & d)
  {
    for (unsigned i = 0; i < o->conds.size(); ++i) {
      conds.push_back(o->conds[i]->copy(d));
    }
  }

  ~Oneof()
  {
    for (unsigned i = 0; i < conds.size(); ++i) {
      delete conds[i];
    }
  }

  void print(std::ostream & s) const
  {
    for (unsigned i = 0; i < conds.size(); ++i) {
      conds[i]->print(s);
    }
  }

  void PDDLPrint(
    std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts,
    const Domain & d) const override;

  plansys2_msgs::msg::Node::SharedPtr getTree(
    plansys2_msgs::msg::Tree & tree, const Domain & d,
    const std::vector<std::string> & replace = {}) const override;

  void parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d);

  void add(Condition * cond) {conds.push_back(cond);}

  void addParams(int m, unsigned n)
  {
    for (unsigned i = 0; i < conds.size(); ++i) {
      conds[i]->addParams(m, n);
    }
  }

  Condition * copy(Domain & d) {return new Oneof(this, d);}
};

}  // namespace pddl
}  // namespace parser
