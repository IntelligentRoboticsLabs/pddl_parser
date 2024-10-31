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

#ifndef PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTCLIENT_HPP_
#define PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTCLIENT_HPP_

#include <optional>
#include <string>
#include <vector>
#include <unordered_set>

#include "plansys2_problem_expert/ProblemExpertInterface.hpp"
#include "plansys2_core/Types.hpp"

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/param.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_msgs/srv/add_problem.hpp"
#include "plansys2_msgs/srv/add_problem_goal.hpp"
#include "plansys2_msgs/srv/affect_node.hpp"
#include "plansys2_msgs/srv/affect_param.hpp"
#include "plansys2_msgs/srv/exist_node.hpp"
#include "plansys2_msgs/srv/get_problem.hpp"
#include "plansys2_msgs/srv/get_problem_goal.hpp"
#include "plansys2_msgs/srv/get_problem_instance_details.hpp"
#include "plansys2_msgs/srv/get_problem_instances.hpp"
#include "plansys2_msgs/srv/get_node_details.hpp"
#include "plansys2_msgs/srv/get_states.hpp"
#include "plansys2_msgs/srv/is_problem_goal_satisfied.hpp"
#include "plansys2_msgs/srv/remove_problem_goal.hpp"
#include "plansys2_msgs/srv/clear_problem_knowledge.hpp"

#include "rclcpp/rclcpp.hpp"

namespace plansys2
{

class ProblemExpertClient : public ProblemExpertInterface
{
public:
  ProblemExpertClient();

  std::unordered_set<plansys2::Instance> getInstances();
  bool addInstance(const plansys2::Instance & instance);
  bool removeInstance(const plansys2::Instance & instance);
  std::optional<plansys2::Instance> getInstance(const std::string & name);

  std::unordered_set<plansys2::Predicate> getInferredPredicates();
  std::unordered_set<plansys2::Predicate> getPredicates();
  bool addPredicate(const plansys2::Predicate & predicate);
  bool removePredicate(const plansys2::Predicate & predicate);
  bool existPredicate(const plansys2::Predicate & predicate);
  std::optional<plansys2::Predicate> getPredicate(const std::string & predicate);

  std::unordered_set<plansys2::Function> getFunctions();
  bool addFunction(const plansys2::Function & function);
  bool removeFunction(const plansys2::Function & function);
  bool existFunction(const plansys2::Function & function);
  bool updateFunction(const plansys2::Function & function);
  std::optional<plansys2::Function> getFunction(const std::string & function);

  plansys2::State getState();

  plansys2::Goal getGoal();
  bool setGoal(const plansys2::Goal & goal);
  bool isGoalSatisfied(const plansys2::Goal & goal);

  bool clearGoal();
  bool clearKnowledge();

  std::string getProblem();
  bool addProblem(const std::string & problem_str);

  rclcpp::Time getUpdateTime() const {return update_time_;}

private:
  rclcpp::Client<plansys2_msgs::srv::AddProblem>::SharedPtr
    add_problem_client_;
  rclcpp::Client<plansys2_msgs::srv::AddProblemGoal>::SharedPtr
    add_problem_goal_client_;
  rclcpp::Client<plansys2_msgs::srv::AffectParam>::SharedPtr
    add_problem_instance_client_;
  rclcpp::Client<plansys2_msgs::srv::AffectNode>::SharedPtr
    add_problem_predicate_client_;
  rclcpp::Client<plansys2_msgs::srv::AffectNode>::SharedPtr
    add_problem_function_client_;
  rclcpp::Client<plansys2_msgs::srv::GetProblemGoal>::SharedPtr
    get_problem_goal_client_;
  rclcpp::Client<plansys2_msgs::srv::GetProblemInstanceDetails>::SharedPtr
    get_problem_instance_details_client_;
  rclcpp::Client<plansys2_msgs::srv::GetProblemInstances>::SharedPtr
    get_problem_instances_client_;
  rclcpp::Client<plansys2_msgs::srv::GetNodeDetails>::SharedPtr
    get_problem_predicate_details_client_;
  rclcpp::Client<plansys2_msgs::srv::GetStates>::SharedPtr
    get_problem_predicates_client_;
  rclcpp::Client<plansys2_msgs::srv::GetStates>::SharedPtr
    get_problem_inferred_predicates_client_;
  rclcpp::Client<plansys2_msgs::srv::GetNodeDetails>::SharedPtr
    get_problem_function_details_client_;
  rclcpp::Client<plansys2_msgs::srv::GetStates>::SharedPtr
    get_problem_functions_client_;
  rclcpp::Client<plansys2_msgs::srv::GetProblem>::SharedPtr
    get_problem_client_;
  rclcpp::Client<plansys2_msgs::srv::RemoveProblemGoal>::SharedPtr
    remove_problem_goal_client_;
  rclcpp::Client<plansys2_msgs::srv::ClearProblemKnowledge>::SharedPtr
    clear_problem_knowledge_client_;
  rclcpp::Client<plansys2_msgs::srv::AffectParam>::SharedPtr
    remove_problem_instance_client_;
  rclcpp::Client<plansys2_msgs::srv::AffectNode>::SharedPtr
    remove_problem_predicate_client_;
  rclcpp::Client<plansys2_msgs::srv::AffectNode>::SharedPtr
    remove_problem_function_client_;
  rclcpp::Client<plansys2_msgs::srv::ExistNode>::SharedPtr
    exist_problem_predicate_client_;
  rclcpp::Client<plansys2_msgs::srv::ExistNode>::SharedPtr
    exist_problem_function_client_;
  rclcpp::Client<plansys2_msgs::srv::AffectNode>::SharedPtr
    update_problem_function_client_;
  rclcpp::Client<plansys2_msgs::srv::IsProblemGoalSatisfied>::SharedPtr
    is_problem_goal_satisfied_client_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time update_time_;
};

}  // namespace plansys2

#endif  // PLANSYS2_PROBLEM_EXPERT__PROBLEMEXPERTCLIENT_HPP_
