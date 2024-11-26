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
#include "plansys2_pddl_parser/Instance.hpp"

namespace parser
{
namespace pddl
{

Expression * TemporalAction::parseDuration(
  Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
{
  return createExpression(f, ts, d);
}

void TemporalAction::printCondition(
  std::ostream & s, const TokenStruct<std::string> & ts, const Domain & d, const std::string & t,
  And * a) const
{
  for (unsigned i = 0; a && i < a->conds.size(); ++i) {
    s << "\t\t( " << t << " ";
    a->conds[i]->PDDLPrint(s, 0, ts, d);
    s << " )\n";
  }
}

void TemporalAction::PDDLPrint(
  std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts, const Domain & d) const
{
  s << "( :durative-action " << name << "\n";

  s << "  :parameters ";

  TokenStruct<std::string> astruct;

  printParams(0, s, astruct, d);

  s << "  :duration ( = ?duration ";
  if (durationExpr) {
    durationExpr->PDDLPrint(s, 0, astruct, d);
  } else {
    s << "1";
  }
  s << " )\n";

  s << "  :condition\n";
  s << "\t( and\n";
  printCondition(s, astruct, d, "at start", reinterpret_cast<And *>(pre));
  printCondition(s, astruct, d, "over all", pre_o);
  printCondition(s, astruct, d, "at end", pre_e);
  s << "\t)\n";

  s << "  :effect\n";
  s << "\t( and\n";
  printCondition(s, astruct, d, "at start", reinterpret_cast<And *>(eff));
  printCondition(s, astruct, d, "at end", eff_e);
  s << "\t)\n";

  s << ")\n";
}

plansys2_msgs::msg::Node::SharedPtr TemporalAction::getTree(
  plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace) const
{
  throw UnsupportedConstruct("TemporalAction");
}

void TemporalAction::parseCondition(
  Stringreader & f, TokenStruct<std::string> & ts, Domain & d, And * a)
{
  f.next();
  f.assert_token("(");
  Condition * c = d.createCondition(f);
  c->parse(f, ts, d);
  a->conds.push_back(c);
}

void TemporalAction::parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
{
  f.next();
  f.assert_token(":parameters");
  f.assert_token("(");

  TokenStruct<std::string> astruct = f.parseTypedList(true, d.types);
  params = d.convertTypes(astruct.types);

  f.next();
  f.assert_token(":duration");
  f.assert_token("(");
  f.assert_token("=");
  f.assert_token("?duration");
  durationExpr = parseDuration(f, astruct, d);
  f.next();
  f.assert_token(")");

  f.next();
  f.assert_token(":");
  std::string s = f.getToken();
  if (s == "condition") {
    pre = new And;
    pre_o = new And;
    pre_e = new And;
    f.next();
    f.assert_token("(");
    if (f.getChar() != ')') {
      s = f.getToken();
      if (s == "and") {
        for (f.next(); f.getChar() != ')'; f.next()) {
          f.assert_token("(");
          s = f.getToken();
          f.next();
          std::string t = f.getToken();

          if (s == "at" && t == "start") {
            parseCondition(f, astruct, d, reinterpret_cast<And *>(pre));
          } else if (s == "over" && t == "all") {
            parseCondition(f, astruct, d, pre_o);
          } else if (s == "at" && t == "end") {
            parseCondition(f, astruct, d, pre_e);
          } else {
            f.tokenExit(s + " " + t);
          }

          f.next();
          f.assert_token(")");
        }
        ++f.c;
      } else {
        f.next();
        std::string t = f.getToken();

        if (s == "at" && t == "start") {
          parseCondition(f, astruct, d, reinterpret_cast<And *>(pre));
        } else if (s == "over" && t == "all") {
          parseCondition(f, astruct, d, pre_o);
        } else if (s == "at" && t == "end") {
          parseCondition(f, astruct, d, pre_e);
        } else {
          f.tokenExit(s + " " + t);
        }

        f.next();
        f.assert_token(")");
      }
    } else {
      ++f.c;
    }

    f.next();
    f.assert_token(":");
    s = f.getToken();
  }
  if (s != "effect") {
    f.tokenExit(s);
  }

  f.next();
  f.assert_token("(");
  if (f.getChar() != ')') {
    eff = new And;
    eff_e = new And;

    s = f.getToken();
    if (s == "and") {
      for (f.next(); f.getChar() != ')'; f.next()) {
        f.assert_token("(");
        s = f.getToken();
        f.next();
        std::string t = f.getToken();

        if (s == "at" && t == "start") {
          parseCondition(f, astruct, d, reinterpret_cast<And *>(eff));
        } else if (s == "at" && t == "end") {
          parseCondition(f, astruct, d, eff_e);
        } else {
          f.tokenExit(s + " " + t);
        }

        f.next();
        f.assert_token(")");
      }
      ++f.c;
    } else {
      f.next();
      std::string t = f.getToken();

      if (s == "at" && t == "start") {
        parseCondition(f, astruct, d, reinterpret_cast<And *>(eff));
      } else if (s == "at" && t == "end") {
        parseCondition(f, astruct, d, eff_e);
      } else {
        f.tokenExit(s + " " + t);
      }

      f.next();
      f.assert_token(")");
    }
  } else {
    ++f.c;
  }

  f.next();
  f.assert_token(")");
}

GroundVec TemporalAction::preconsStart() {return getGroundsFromCondition(pre, false);}

GroundVec TemporalAction::preconsOverall() {return getGroundsFromCondition(pre_o, false);}

GroundVec TemporalAction::preconsEnd() {return getGroundsFromCondition(pre_e, false);}

CondVec TemporalAction::endEffects() {return getSubconditionsFromCondition(eff_e);}

GroundVec TemporalAction::addEndEffects() {return getGroundsFromCondition(eff_e, false);}

GroundVec TemporalAction::deleteEndEffects() {return getGroundsFromCondition(eff_e, true);}

}  // namespace pddl
}  // namespace parser
