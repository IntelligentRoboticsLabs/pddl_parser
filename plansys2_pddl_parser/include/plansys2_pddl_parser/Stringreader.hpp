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

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "plansys2_pddl_parser/TokenStruct.hpp"
#include "plansys2_pddl_parser/Type.hpp"

namespace parser
{
namespace pddl
{

class Domain;

class ExpectedToken : public std::runtime_error
{
public:
  explicit ExpectedToken(const std::string & token)
  : std::runtime_error(token + " expected") {}
};

class UnknownToken : public std::runtime_error
{
public:
  explicit UnknownToken(const std::string & token)
  : std::runtime_error(token + " does not name a known token")
  {
  }
};

class UnexpectedEOF : public std::runtime_error
{
public:
  UnexpectedEOF()
  : std::runtime_error("Unexpected EOF found") {}
};

class Stringreader
{
public:
  std::vector<std::string> lines;
  int current_line;  // current line of file
  std::string s;
  unsigned r, c;  // current row and column of file

  explicit Stringreader(const std::string & domain)
  : current_line(0), r(1), c(0)
  {
    lines = getLines(domain);

    s = lines[current_line++];
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    next();
  }

  ~Stringreader() {}

  std::vector<std::string> getLines(const std::string & text)
  {
    std::vector<std::string> ret;
    size_t start = 0, end = 0;

    while (end != std::string::npos) {
      end = text.find("\n", start);
      ret.push_back(
        text.substr(start, (end == std::string::npos) ? std::string::npos : end - start));
      start = ((end > (std::string::npos - 1)) ? std::string::npos : end + 1);
    }

    return ret;
  }

  // characters to be ignored
  bool ignore(char c) {return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\f';}

  // parenthesis
  bool paren(char c) {return c == '(' || c == ')' || c == '{' || c == '}';}

  // current character
  char getChar() {return s[c];}

  // print line and column
  void printLine() {std::cout << "Line " << r << ", column " << c + 1 << ": ";}

  void tokenExit(const std::string & t)
  {
    c -= t.size();
    printLine();
    throw UnknownToken(t);
  }

  // get next non-ignored character
  void next()
  {
    for (; c < s.size() && ignore(s[c]); ++c) {
    }
    while (c == s.size() || s[c] == ';') {
      ++r;
      c = 0;

      s = lines[current_line++];
      std::transform(s.begin(), s.end(), s.begin(), ::tolower);

      for (; c < s.size() && ignore(s[c]); ++c) {
      }
    }
  }

  // get token converted to lowercase
  std::string getToken()
  {
    std::ostringstream os;
    while (c < s.size() && !ignore(s[c]) && !paren(s[c]) && s[c] != ',') {
      os << (65 <= s[c] && s[c] <= 90 ? static_cast<char>(s[c++] + 32) : s[c++]);
    }

    return os.str();
  }

  // get token converted to lowercase
  // check that the token exists
  template<typename T>
  std::string getToken(const TokenStruct<T> & ts)
  {
    std::string t = getToken();
    if (ts.index(t) < 0) {
      tokenExit(t);
    }
    return t;
  }

  // assert syntax
  void assert_token(const std::string & t)
  {
    unsigned b = 0;
    for (unsigned k = 0; c + k < s.size() && k < t.size(); ++k) {
      b += s[c + k] == t[k] || (65 <= s[c + k] && s[c + k] <= 90 && s[c + k] == t[k] - 32);
    }

    if (b < t.size()) {
      printLine();
      throw ExpectedToken(t);
    }
    c += t.size();
    next();
  }

  // parse the name of a domain or instance
  std::string parseName(const std::string & u)
  {
    std::string out;
    std::string t[5] = {"(", "define", "(", u, ")"};
    for (unsigned i = 0; i < 5; ++i) {
      assert_token(t[i]);
      if (i == 3) {
        out = getToken();
        next();
      }
    }
    return out;
  }

  // parse a typed list
  // if check is true, checks that types exist
  TokenStruct<std::string> parseTypedList(
    bool check, const TokenStruct<Type *> & ts = TokenStruct<Type *>(), const std::string & lt = "")
  {
    unsigned k = 0;
    TokenStruct<std::string> out;
    for (next(); getChar() != ')' && lt.find(getChar()) == std::string::npos; next()) {
      if (getChar() == '-') {
        assert_token("-");

        std::string t;
        // check if the type is "either"
        if (getChar() == '(') {
          assert_token("(");
          assert_token("either");

          t = "( either";
          for (; getChar() != ')'; next()) {
            if (check) {
              t += " " + getToken(ts);
            } else {
              t += " " + getToken();
            }
          }
          t += " )";
          ++c;
        } else if (check) {
          t = getToken(ts);
        } else {
          t = getToken();
        }

        out.types.insert(out.types.end(), out.size() - k, t);
        k = out.size();
      } else if (getChar() == '(') {
        assert_token("(");
        assert_token(":private");
        getToken();
        out.append(parseTypedList(check, ts));
      } else {
        out.insert(getToken());
      }
    }
    if (k < out.size()) {
      out.types.insert(out.types.end(), out.size() - k, check ? "object" : "");
    }
    ++c;

    return out;
  }
};

}  // namespace pddl
}  // namespace parser
