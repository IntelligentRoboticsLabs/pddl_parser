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

void ParamCond::printParams(
  unsigned first, std::ostream & s, TokenStruct<std::string> & ts, const Domain & d) const
{
  s << "(";
  for (unsigned i = first; i < params.size(); ++i) {
    std::stringstream ss;
    ss << "?" << d.types[params[i]]->getName() << ts.size();
    ts.insert(ss.str());
    s << " " << ss.str();
    if (d.typed) {
      s << " - " << d.types[params[i]]->name;
    }
  }
  s << " )\n";
}

}  // namespace pddl
}  // namespace parser
