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

#ifndef PLANSYS2_CORE__UTILS_HPP_
#define PLANSYS2_CORE__UTILS_HPP_

#include <string>
#include <vector>

namespace plansys2
{

std::vector<std::string> tokenize(const std::string & string, const std::string & delim);

/**
 * @brief get a substring without empty lines
 *
 * @param string original string
 * @param init_pos first character in the original string
 * @param end_pos last character in the original string
 * @return a substring without empty lines
 */
std::string substr_without_empty_lines(
  std::string string, std::size_t init_pos, std::size_t end_pos);

/**
 * @brief remove the comments on a pddl string
 *
 * @param pddl a pddl string
 * @return a pddl string without comments
 */
std::string remove_comments(const std::string & pddl);

}  // namespace plansys2

#endif  // PLANSYS2_CORE__UTILS_HPP_
