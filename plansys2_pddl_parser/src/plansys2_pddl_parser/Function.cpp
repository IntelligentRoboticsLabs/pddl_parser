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

void Function::PDDLPrint(
  std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts, const Domain & d) const
{
  Lifted::PDDLPrint(s, indent, ts, d);
  if (returnType >= 0) {
    s << " - " << d.types[returnType]->name;
  }
}

plansys2_msgs::msg::Node::SharedPtr Function::getTree(
  plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace) const
{
  throw UnsupportedConstruct("Function");
}

void Function::parse(Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
{
  Lifted::parse(f, ts, d);

  f.next();
  if (f.getChar() == '-') {
    f.assert_token("-");
    std::string s = f.getToken();
    if (s != "number") {
      f.c -= s.size();
      returnType = d.types.index(f.getToken(d.types));
    }
  }
}

}  // namespace pddl
}  // namespace parser
