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
#include <unordered_set>
#include <variant>
#include <vector>

#include "plansys2_msgs/msg/derived.hpp"
#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/state.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

namespace plansys2
{

template<class toT, class fromT>
std::vector<toT> convertVector(const std::vector<fromT> & in_vector)
{
  std::vector<toT> ret(in_vector.begin(), in_vector.end());
  return ret;
}

template<class toT, class fromT>
std::unordered_set<toT> convertVectorToUnorderedSet(const std::vector<fromT> & in_vector)
{
  std::unordered_set<toT> ret(in_vector.begin(), in_vector.end());
  return ret;
}

template<class toT, class fromT>
std::vector<toT> convertUnorderedSetToVector(const std::unordered_set<fromT> & in_unordered_set)
{
  std::vector<toT> ret(in_unordered_set.begin(), in_unordered_set.end());
  return ret;
}

class Instance : public plansys2_msgs::msg::Param
{
public:
  Instance()
  : plansys2_msgs::msg::Param() {}
  explicit Instance(const std::string & name, const std::string & type = {})
  : plansys2_msgs::msg::Param(parser::pddl::fromStringParam(name, type))
  {
  }
  Instance(const plansys2_msgs::msg::Param & instance)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Param(instance)
  {
  }

  bool operator==(const Instance & i2) const {return parser::pddl::checkParamEquality(*this, i2);}
};

class Predicate : public plansys2_msgs::msg::Node
{
public:
  Predicate()
  : plansys2_msgs::msg::Node() {}
  explicit Predicate(const std::string & pred)
  : plansys2_msgs::msg::Node(parser::pddl::fromStringPredicate(pred))
  {
  }
  Predicate(const plansys2_msgs::msg::Node & pred)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Node(pred)
  {
  }

  bool operator==(const Predicate & p2) const {return parser::pddl::checkNodeEquality(*this, p2);}
};

class Derived : public plansys2_msgs::msg::Derived
{
public:
  Derived()
  : plansys2_msgs::msg::Derived() {}
  Derived(const plansys2_msgs::msg::Derived & derived)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Derived(derived)
  {
  }

  bool operator==(const Derived & d) const
  {
    return parser::pddl::checkNodeEquality(this->predicate, d.predicate) &&
           parser::pddl::checkTreeEquality(this->preconditions, d.preconditions);
  }
};

class Function : public plansys2_msgs::msg::Node
{
public:
  Function()
  : plansys2_msgs::msg::Node() {}
  explicit Function(const std::string & func)
  : plansys2_msgs::msg::Node(parser::pddl::fromStringFunction(func))
  {
  }
  Function(const plansys2_msgs::msg::Node & func)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Node(func)
  {
  }

  bool operator==(const Function & f2) const {return parser::pddl::checkNodeEquality(*this, f2);}
};

class Goal : public plansys2_msgs::msg::Tree
{
public:
  Goal()
  : plansys2_msgs::msg::Tree() {}
  explicit Goal(const std::string & goal)
  : plansys2_msgs::msg::Tree(parser::pddl::fromString(goal))
  {
  }
  Goal(const plansys2_msgs::msg::Tree & goal)  // NOLINT(runtime/explicit)
  : plansys2_msgs::msg::Tree(goal)
  {
  }
};

}  // namespace plansys2

namespace std
{

template<typename T>
inline void hash_combine(std::size_t & seed, const T & value)
{
  seed ^= std::hash<T>{}(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

inline std::size_t hash_node(const plansys2_msgs::msg::Node & node)
{
  std::size_t seed = 0;

  hash_combine(seed, node.name);
  hash_combine(seed, node.node_type);
  hash_combine(seed, node.children.size());
  hash_combine(seed, node.parameters.size());

  for (const auto & param : node.parameters) {
    hash_combine(seed, param.name);
  }
  return seed;
}

template<>
struct hash<plansys2_msgs::msg::Node>
{
  std::size_t operator()(const plansys2_msgs::msg::Node & node) const noexcept
  {
    return hash_node(node);
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

template<>
struct hash<plansys2::Predicate>
{
  std::size_t operator()(const plansys2::Predicate & pred) const noexcept
  {
    return hash_node(pred);
  }
};

template<>
struct hash<plansys2::Derived>
{
  std::size_t operator()(const plansys2::Derived & derived) const noexcept
  {
    std::size_t seed = 0;
    hash_combine(seed, derived.predicate);

    for (auto node : derived.preconditions.nodes) {
      hash_combine(seed, node);
    }

    return seed;
  }
};

template<>
struct hash<plansys2::Function>
{
  std::size_t operator()(const plansys2::Function & func) const noexcept {return hash_node(func);}
};
}  // namespace std

#endif  // PLANSYS2_CORE__TYPES_HPP_
