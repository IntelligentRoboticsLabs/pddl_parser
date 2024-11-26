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

Derived::Derived(const Derived * z, Domain & d)
: Lifted(z), cond(0), lifted(d.preds.get(z->name))
{
  if (z->cond) {
    cond = z->cond->copy(d);
  }
}

void Derived::PDDLPrint(
  std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts, const Domain & d) const
{
  s << "( :derived ( " << name;

  TokenStruct<std::string> dstruct(ts);

  for (unsigned i = 0; i < params.size(); ++i) {
    std::stringstream ss;
    ss << "?" << d.types[params[i]]->getName() << dstruct.size();
    dstruct.insert(ss.str());
    s << " " << ss.str();
    if (d.typed) {
      s << " - " << d.types[params[i]]->name;
    }
  }
  s << " )\n";

  if (cond) {
    cond->PDDLPrint(s, 1, dstruct, d);
  }

  s << "\n)\n";
}

plansys2_msgs::msg::Node::SharedPtr Derived::getTree(
  plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace) const
{
  throw UnsupportedConstruct("Derived");
}

void Derived::parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
{
  f.next();
  f.assert_token("(");
  name = f.getToken(d.preds);
  TokenStruct<std::string> dstruct = f.parseTypedList(true, d.types);
  params = d.convertTypes(dstruct.types);

  f.next();
  f.assert_token("(");
  cond = d.createCondition(f);
  cond->parse(f, dstruct, d);

  f.next();
  f.assert_token(")");
}

}  // namespace pddl
}  // namespace parser
