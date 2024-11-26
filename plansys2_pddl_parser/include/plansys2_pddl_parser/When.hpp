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

class When : public Condition
{
public:
  Condition * pars, * cond;

  When()
  : pars(0), cond(0) {}

  When(const When * w, Domain & d)
  : pars(0), cond(0)
  {
    if (w->pars) {
      pars = w->pars->copy(d);
    }
    if (w->cond) {
      cond = w->cond->copy(d);
    }
  }

  ~When()
  {
    if (pars) {
      delete pars;
    }
    if (cond) {
      delete cond;
    }
  }

  void print(std::ostream & s) const
  {
    s << "when:\n";
    if (pars) {
      pars->print(s);
    }
    if (cond) {
      cond->print(s);
    }
  }

  void PDDLPrint(
    std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts,
    const Domain & d) const override;

  plansys2_msgs::msg::Node::SharedPtr getTree(
    plansys2_msgs::msg::Tree & tree, const Domain & d,
    const std::vector<std::string> & replace = {}) const override;

  void parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d);

  void addParams(int m, unsigned n)
  {
    pars->addParams(m, n);
    cond->addParams(m, n);
  }

  Condition * copy(Domain & d) {return new When(this, d);}
};

}  // namespace pddl
}  // namespace parser
