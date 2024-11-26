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
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) < (b) ? (b) : (a))
#define SQR(a) ((a) * (a))

#define PI 3.1415926535897932384

typedef std::set<int> IntSet;
typedef std::vector<int> IntVec;
typedef std::set<double> DoubleSet;
typedef std::vector<IntSet> SetVec;
typedef std::pair<int, int> IntPair;
typedef std::vector<IntPair> PairVec;
typedef std::map<int, IntSet> SetMap;
typedef std::vector<double> DoubleVec;
typedef std::vector<unsigned> UnsignedVec;
typedef std::vector<std::string> StringVec;
typedef std::pair<double, double> DoublePair;

// create a vector of integers from lo to (hi-1)
inline IntVec incvec(unsigned lo, unsigned hi)
{
  IntVec out;
  for (unsigned i = lo; i < hi; ++i) {
    out.push_back(i);
  }
  return out;
}

// insert all elements of a collection into a stream
template<typename T>
std::ostream & insertAll(std::ostream & stream, const T & begin, const T & end)
{
  T i = begin;
  stream << "[";
  if (i != end) {
    stream << *(i++);
  }
  while (i != end) {
    stream << "," << *(i++);
  }
  return stream << "]";
}

// insert a pair into a stream
template<typename T, typename U>
std::ostream & operator<<(std::ostream & stream, const std::pair<T, U> & p)
{
  return stream << "(" << p.first << "," << p.second << ")";
}

// insert a list into a stream
template<typename T>
std::ostream & operator<<(std::ostream & stream, const std::list<T> & l)
{
  return insertAll(stream, l.begin(), l.end());
}

// insert a map into a stream
template<typename T, typename U>
std::ostream & operator<<(std::ostream & stream, const std::map<T, U> & m)
{
  return insertAll(stream, m.begin(), m.end());
}

// insert a multiset into a stream
template<typename T>
std::ostream & operator<<(std::ostream & stream, const std::multiset<T> & s)
{
  return insertAll(stream, s.begin(), s.end());
}

// insert a set into a stream
template<typename T>
std::ostream & operator<<(std::ostream & stream, const std::set<T> & s)
{
  return insertAll(stream, s.begin(), s.end());
}

// insert a vector into a stream
template<typename T>
std::ostream & operator<<(std::ostream & stream, const std::vector<T> & v)
{
  return insertAll(stream, v.begin(), v.end());
}

inline void tabindent(std::ostream & stream, unsigned indent)
{
  for (unsigned i = 0; i < indent; ++i) {
    stream << "\t";
  }
}
