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

#include <string>
#include <vector>

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_pddl_parser/Action.hpp"
#include "plansys2_pddl_parser/And.hpp"
#include "plansys2_pddl_parser/Expression.hpp"

namespace parser
{
namespace pddl
{

class Instance;

class TemporalAction : public Action
{
public:
  Expression * durationExpr;
  // pre_s and eff_s inherited from Action
  And * pre_o, * pre_e, * eff_e;

  explicit TemporalAction(const std::string & s)
  : Action(s), durationExpr(0), pre_o(0), pre_e(0), eff_e(0)
  {
  }

  ~TemporalAction()
  {
    if (durationExpr) {
      delete durationExpr;
    }
    if (pre_o) {
      delete pre_o;
    }
    if (pre_e) {
      delete pre_e;
    }
    if (eff_e) {
      delete eff_e;
    }
  }

  void print(std::ostream & s) const
  {
    s << name << params << "\n";
    s << "Duration: " << durationExpr->info() << "\n";
    s << "Pre_s: " << pre;
    s << "Eff_s: " << eff;
    s << "Pre_o: " << pre_o;
    s << "Pre_e: " << pre_e;
    s << "Eff_e: " << eff_e;
  }

  double duration()
  {
    if (durationExpr) {
      return durationExpr->evaluate();
    } else {
      return 0;
    }
  }

  Expression * parseDuration(Stringreader & f, TokenStruct<std::string> & ts, Domain & d);

  void printCondition(
    std::ostream & s, const TokenStruct<std::string> & ts, const Domain & d, const std::string & t,
    And * a) const;

  void PDDLPrint(
    std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts,
    const Domain & d) const override;

  plansys2_msgs::msg::Node::SharedPtr getTree(
    plansys2_msgs::msg::Tree & tree, const Domain & d,
    const std::vector<std::string> & replace = {}) const override;

  void parseCondition(Stringreader & f, TokenStruct<std::string> & ts, Domain & d, And * a);

  void parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d);

  GroundVec preconsStart();

  GroundVec preconsOverall();

  GroundVec preconsEnd();

  CondVec endEffects();

  GroundVec addEndEffects();

  GroundVec deleteEndEffects();
};

}  // namespace pddl
}  // namespace parser
