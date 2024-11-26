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

#include "plansys2_pddl_parser/Type.hpp"

namespace parser
{
namespace pddl
{

class EitherType : public Type
{
public:
  explicit EitherType(const std::string & s)
  : Type(s) {}

  explicit EitherType(const EitherType * t)
  : Type(t) {}

  std::string getName() const
  {
    std::string out = "either";
    for (unsigned i = 0; i < subtypes.size(); ++i) {
      out += "_" + subtypes[i]->getName();
    }
    return out;
  }

  void PDDLPrint(std::ostream & s) const override {}

  Type * copy() {return new EitherType(this);}
};

}  // namespace pddl
}  // namespace parser
