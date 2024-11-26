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

void Lifted::PDDLPrint(
  std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts, const Domain & d) const
{
  tabindent(s, indent);
  s << "( " << name;
  for (unsigned i = 0; i < params.size(); ++i) {
    if (ts.size()) {
      s << ts[i];
    } else {
      s << " ?" << d.types[params[i]]->getName() << i;
    }
    if (d.typed) {
      s << " - " << d.types[params[i]]->name;
    }
  }
  s << " )";
}

plansys2_msgs::msg::Node::SharedPtr Lifted::getTree(
  plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace) const
{
  throw UnsupportedConstruct("Lifted");
}

void Lifted::parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
{
  TokenStruct<std::string> lstruct = f.parseTypedList(true, d.types);
  params = d.convertTypes(lstruct.types);
}

}  // namespace pddl
}  // namespace parser
