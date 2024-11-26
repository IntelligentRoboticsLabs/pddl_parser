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

#include <list>
#include <utility>

#include "plansys2_pddl_parser/Condition.hpp"

namespace parser
{
namespace pddl
{

typedef std::list<std::pair<Condition *, unsigned>> CondList;

class CondIter : public std::iterator<std::input_iterator_tag, Condition *>
{
public:
  CondList condStack;

  explicit CondIter(Condition * c)
  {
    condStack.push_back(std::make_pair(c, 0));
    (*this)++;
  }

  explicit CondIter(const CondIter & ci)
  : condStack(ci.condStack) {}

  MyIterator & operator++()
  {
    while (condStack().size() && condStack.last().done()) {
      condStack.pop_back();
    }

    return *this;
  }

  bool hasNext() {return condStack.size();}

  Condition * operator*() {return condStack.last();}
};

}  // namespace pddl
}  // namespace parser
