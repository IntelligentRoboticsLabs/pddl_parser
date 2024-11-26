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
#include "plansys2_pddl_parser/ParamCond.hpp"

namespace parser
{
namespace pddl
{

class Exists : public ParamCond
{
public:
  Condition * cond;

  Exists()
  : cond(0) {}

  Exists(const Exists * e, Domain & d)
  : ParamCond(e), cond(0)
  {
    if (e->cond) {
      cond = e->cond->copy(d);
    }
  }

  ~Exists()
  {
    if (cond) {
      delete cond;
    }
  }

  void print(std::ostream & s) const
  {
    s << "Exists" << params << ":\n";
    cond->print(s);
  }

  void PDDLPrint(
    std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts,
    const Domain & d) const override;

  plansys2_msgs::msg::Node::SharedPtr getTree(
    plansys2_msgs::msg::Tree & tree, const Domain & d,
    const std::vector<std::string> & replace = {}) const override;

  void parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d);

  void addParams(int m, unsigned n) {cond->addParams(m, n);}

  Condition * copy(Domain & d) {return new Exists(this, d);}
};

}  // namespace pddl
}  // namespace parser
