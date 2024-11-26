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
#include "plansys2_pddl_parser/Instance.hpp"

namespace parser
{
namespace pddl
{

void FunctionExpression::PDDLPrint(
  std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts, const Domain & d) const
{
  ParamCond * c = d.funcs[d.funcs.index(fun->name)];

  s << "( " << fun->name;
  unsigned int psize = c->params.size();
  for (unsigned i = 0; i < psize; ++i) {
    if (ts.size() && fun->params[i] >= 0 && (unsigned)fun->params[i] < ts.size()) {
      s << " " << ts[fun->params[i]];
    } else if (fun->params[i] >= 0 && (unsigned)fun->params[i] >= ts.size()) {
      s << " ?" << fun->params[i];
    } else {
      // It is a constant parameter for the function
      s << " ";
      constants[i]->PDDLPrint(s, indent, ts, d);
    }
  }
  s << " )";
}

plansys2_msgs::msg::Node::SharedPtr FunctionExpression::getTree(
  plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace) const
{
  plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
  node->node_type = plansys2_msgs::msg::Node::FUNCTION;
  node->node_id = tree.nodes.size();
  node->name = fun->name;
  for (unsigned i = 0; i < fun->params.size(); ++i) {
    plansys2_msgs::msg::Param param;
    if (i < replace.size()) {
      param.name = replace[fun->params[i]];
    } else {
      param.name = "?" + std::to_string(fun->params[i]);
    }
    param.type = d.types[fun->params[i]]->name;
    node->parameters.push_back(param);
  }
  tree.nodes.push_back(*node);
  return node;
}

double FunctionExpression::evaluate(Instance & ins, const StringVec & par)
{
  ParamCond * c = ins.d.funcs[ins.d.funcs.index(fun->name)];

  IntVec v(c->params.size());
  for (unsigned i = 0; i < v.size(); ++i) {
    std::pair<bool, unsigned> p = ins.d.types[c->params[i]]->parseObject(par[fun->params[i]]);
    if (p.first) {
      v[i] = p.second;
    } else {
      std::pair<bool, int> q = ins.d.types[c->params[i]]->parseConstant(par[fun->params[i]]);
      if (q.first) {
        v[i] = q.second;
      } else {
        return 1;
      }
    }
  }

  for (unsigned i = 0; i < ins.init.size(); ++i) {
    if (ins.init[i]->name == c->name && ins.init[i]->params == v) {
      return ((GroundFunc<double> *)ins.init[i])->value;
    }
  }
  return 1;
}

void ConstExpression::PDDLPrint(
  std::ostream & s, unsigned indent, const TokenStruct<std::string> & ts, const Domain & d) const
{
  s << d.types[tid]->constants[constant];
}

plansys2_msgs::msg::Node::SharedPtr ConstExpression::getTree(
  plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace) const
{
  plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
  node->node_type = plansys2_msgs::msg::Node::CONSTANT;
  node->node_id = tree.nodes.size();
  node->name = d.types[tid]->constants[constant];
  tree.nodes.push_back(*node);
  return node;
}

Expression * createExpression(Stringreader & f, TokenStruct<std::string> & ts, Domain & d)
{
  f.next();

  if (f.getChar() == '(') {
    ++f.c;
    f.next();
    std::string s = f.getToken();
    if (s == "+" || s == "-" || s == "*" || s == "/") {
      // It is a composite expression
      CompositeExpression * ce = new CompositeExpression(s);
      ce->parse(f, ts, d);
      return ce;
    } else {
      // It is a function expression
      f.c -= s.size();
      Function * fun = d.funcs.get(f.getToken(d.funcs));
      ParamCond * p = new Lifted(fun);
      std::vector<Expression *> c(fun->params.size());
      std::fill(c.begin(), c.end(), nullptr);
      for (unsigned i = 0; i < fun->params.size(); ++i) {
        f.next();
        std::string s = f.getToken();
        if (d.isConstant(s)) {
          p->params[i] = -1;
          IntPair ct = d.constantTypeIdConstId(s);
          c[i] = new ConstExpression(ct.first, ct.second);
        } else if (ts.index(s) >= 0) {
          p->params[i] = ts.index(s);
        } else {
          f.tokenExit(s);
        }
      }
      f.next();
      f.assert_token(")");
      return new FunctionExpression(p, c);
    }
  } else if (f.getChar() == '?') {
    // It is a parameter variable
    std::string s = f.getToken();

    int k = ts.index(s);
    if (k >= 0) {
      // It is a parameter variable
      return new ParamExpression(k);
    } else if (s == "?duration") {
      // It is a ?duration expression
      return new DurationExpression();
    } else {
      // Neither a known parameter variable nor ?duration
      f.tokenExit(s);
      return nullptr;
    }
  } else {
    std::string s = f.getToken();

    if (d.isConstant(s)) {
      // It is a constant!
      IntPair ct = d.constantTypeIdConstId(s);
      return new ConstExpression(ct.first, ct.second);
    } else if (s == "#t") {
      // This construct is used in PDDL+ to model continuous evolution
      // of some function within processes
      std::ostringstream os;
      os << "\"" << s << "\" is supported only in PDDL+,"
         << "and not supported by Plansys2" << std::endl;
      f.printLine();
      std::cout << os.str();
      throw new std::runtime_error(os.str());
      return nullptr;
    } else {
      // It is expected to be a number!
      double d;
      try {
        d = std::stod(s);
      } catch (const std::invalid_argument &) {
        std::ostringstream os;
        os << "Expected a number, found \"" << s << "\"" << std::endl;
        f.printLine();
        std::cout << os.str();
        throw new std::runtime_error(os.str());
      } catch (const std::out_of_range &) {
        std::ostringstream os;
        os << "Expected a number, found \"" << s << "\"" << std::endl
           << "and it goes out of range" << std::endl;
        f.printLine();
        std::cout << os.str();
        throw new std::runtime_error(os.str());
      }
      return new ValueExpression(d);
    }
  }
}

}  // namespace pddl
}  // namespace parser
