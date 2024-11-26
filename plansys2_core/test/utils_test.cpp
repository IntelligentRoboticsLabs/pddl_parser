// Copyright 2029 Intelligent Robotics Lab
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

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "plansys2_core/Utils.hpp"

TEST(domain_expert, functions)
{
  std::string st1("this is a message");
  std::vector<std::string> est1 = {"this", "is", "a", "message"};
  std::vector<std::string> est1b = {"this", "a", "message"};
  auto res1 = plansys2::tokenize(st1, " ");

  ASSERT_EQ(res1, est1);
  ASSERT_NE(res1, est1b);

  std::string st2("this:is:a:message");
  std::vector<std::string> est2 = {"this", "is", "a", "message"};
  auto res2 = plansys2::tokenize(st2, ":");

  ASSERT_EQ(res2, est2);

  std::string st3("this       is a     message");
  std::vector<std::string> est3 = {"this", "is", "a", "message"};
  auto res3 = plansys2::tokenize(st3, " ");

  ASSERT_EQ(res3, est3);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
