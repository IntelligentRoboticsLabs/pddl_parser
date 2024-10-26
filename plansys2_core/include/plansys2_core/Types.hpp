// Copyright 2019 Intelligent Robotics Lab
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

#ifndef PLANSYS2_CORE__TYPES_HPP_
#define PLANSYS2_CORE__TYPES_HPP_

#include <string>
#include <vector>
#include <unordered_set>

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/tree.hpp"

#include "plansys2_pddl_parser/Utils.hpp"

namespace plansys2
{

class Instance : public plansys2_msgs::msg::Param
{
public:
  Instance()
  : plansys2_msgs::msg::Param() {}
  explicit Instance(const std::string & name, const std::string & type = {})
  : plansys2_msgs::msg::Param(parser::pddl::fromStringParam(name, type)) {}
  Instance(const plansys2_msgs::msg::Param & instance)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Param(instance) {}

  bool operator==(const Instance & i2) const
  {
    return parser::pddl::checkParamEquality(*this, i2);
  }
};

class Predicate : public plansys2_msgs::msg::Node
{
public:
  Predicate()
  : plansys2_msgs::msg::Node() {}
  explicit Predicate(const std::string & pred)
  : plansys2_msgs::msg::Node(parser::pddl::fromStringPredicate(pred)) {}
  Predicate(const plansys2_msgs::msg::Node & pred)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Node(pred) {}

  bool operator==(const Predicate & p2) const
  {
    return parser::pddl::checkNodeEquality(*this, p2);
  }
};

class Function : public plansys2_msgs::msg::Node
{
public:
  Function()
  : plansys2_msgs::msg::Node() {}
  explicit Function(const std::string & func)
  : plansys2_msgs::msg::Node(parser::pddl::fromStringFunction(func)) {}
  Function(const plansys2_msgs::msg::Node & func)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Node(func) {}
};

class Goal : public plansys2_msgs::msg::Tree
{
public:
  Goal()
  : plansys2_msgs::msg::Tree() {}
  explicit Goal(const std::string & goal)
  : plansys2_msgs::msg::Tree(parser::pddl::fromString(goal)) {}
  Goal(const plansys2_msgs::msg::Tree & goal)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Tree(goal) {}
};

template<class toT, class fromT>
std::vector<toT>
convertVector(const std::vector<fromT> & in_vector)
{
  std::vector<toT> ret;
  for (const auto & item : in_vector) {
    ret.push_back(item);
  }
  return ret;
}

template<class toT, class fromT>
std::unordered_set<toT>
convertVectorToUnorderedSet(const std::vector<fromT> & in_vector)
{
  std::unordered_set<toT> ret;
  for (const auto & item : in_vector) {
    ret.insert(item);
  }
  return ret;
}

template<class toT, class fromT>
std::vector<toT>
convertUnorderedSetToVector(const std::unordered_set<fromT> & in_unordered_set)
{
  std::vector<toT> ret;
  for (const auto & item : in_unordered_set) {
    ret.push_back(item);
  }
  return ret;
}


}  // namespace plansys2

namespace std
{
template<>
struct hash<plansys2::Predicate>
{
  std::size_t operator()(const plansys2::Predicate & pred) const noexcept
  {
    std::size_t h1 = std::hash<std::string>{}(pred.name);
    std::size_t h2 = 0;

    for (const auto & param : pred.parameters) {
      h2 ^= std::hash<std::string>{}(param.name) + 0x9e3779b9 + (h2 << 6) + (h2 >> 2);
    }

    return h1 ^ h2;
  }
};

template<>
struct hash<plansys2::Instance>
{
  std::size_t operator()(const plansys2::Instance & inst) const noexcept
  {
    return std::hash<std::string>{}(inst.name);
  }
};
}  // namespace std

#endif  // PLANSYS2_CORE__TYPES_HPP_
