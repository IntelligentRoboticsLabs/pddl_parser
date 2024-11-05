// Copyright 2021 Intelligent Robotics Lab
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

#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "gtest/gtest.h"
#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

#include "plansys2_tests/test_action_node.hpp"
#include "plansys2_tests/execution_logger.hpp"

TEST(test_3, test_3)
{
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto executor_node = std::make_shared<plansys2::ExecutorNode>();

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  auto executor_client = std::make_shared<plansys2::ExecutorClient>();

  auto move_1 = plansys2_tests::TestAction::make_shared("move");
  auto move_2 = plansys2_tests::TestAction::make_shared("move");
  auto move_3 = plansys2_tests::TestAction::make_shared("move");
  auto transport_1 = plansys2_tests::TestAction::make_shared("transport");
  auto transport_2 = plansys2_tests::TestAction::make_shared("transport");
  auto transport_3 = plansys2_tests::TestAction::make_shared("transport");
  auto assemble_1 = plansys2_tests::TestAction::make_shared("assemble");
  auto assemble_2 = plansys2_tests::TestAction::make_shared("assemble");
  auto assemble_3 = plansys2_tests::TestAction::make_shared("assemble");

  auto execution_logger = plansys2_tests::ExecutionLogger::make_shared();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_tests");

  domain_node->set_parameter({"model_file", pkgpath + "/test_3/pddl/test_3.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/test_3/pddl/test_3.pddl"});

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(executor_node->get_node_base_interface());
  exe.add_node(execution_logger->get_node_base_interface());

  exe.add_node(move_1->get_node_base_interface());
  exe.add_node(move_2->get_node_base_interface());
  exe.add_node(move_3->get_node_base_interface());
  exe.add_node(transport_1->get_node_base_interface());
  exe.add_node(transport_2->get_node_base_interface());
  exe.add_node(transport_3->get_node_base_interface());
  exe.add_node(assemble_1->get_node_base_interface());
  exe.add_node(assemble_2->get_node_base_interface());
  exe.add_node(assemble_3->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  problem_client->addInstance(plansys2::Instance("robot1", "robot"));
  problem_client->addInstance(plansys2::Instance("robot2", "robot"));
  problem_client->addInstance(plansys2::Instance("robot3", "robot"));
  problem_client->addInstance(plansys2::Instance("wheels_zone", "zone"));
  problem_client->addInstance(plansys2::Instance("sterwheel_zone", "zone"));
  problem_client->addInstance(plansys2::Instance("body_car_zone", "zone"));
  problem_client->addInstance(plansys2::Instance("assembly_zone", "zone"));
  problem_client->addInstance(plansys2::Instance("recharge_zone", "zone"));
  problem_client->addInstance(plansys2::Instance("wheel_1", "piece"));
  problem_client->addInstance(plansys2::Instance("wheel_2", "piece"));
  problem_client->addInstance(plansys2::Instance("wheel_3", "piece"));
  problem_client->addInstance(plansys2::Instance("body_car_1", "piece"));
  problem_client->addInstance(plansys2::Instance("body_car_2", "piece"));
  problem_client->addInstance(plansys2::Instance("body_car_3", "piece"));
  problem_client->addInstance(plansys2::Instance("sterwheel_1", "piece"));
  problem_client->addInstance(plansys2::Instance("sterwheel_2", "piece"));
  problem_client->addInstance(plansys2::Instance("sterwheel_3", "piece"));
  problem_client->addInstance(plansys2::Instance("car1", "car"));
  problem_client->addInstance(plansys2::Instance("car2", "car"));
  problem_client->addInstance(plansys2::Instance("car3", "car"));
  problem_client->addPredicate(plansys2::Predicate("(robot_at robot1 assembly_zone)"));
  problem_client->addPredicate(plansys2::Predicate("(robot_at robot2 assembly_zone)"));
  problem_client->addPredicate(plansys2::Predicate("(robot_at robot3 assembly_zone)"));
  problem_client->addPredicate(plansys2::Predicate("(is_assembly_zone assembly_zone)"));
  problem_client->addPredicate(plansys2::Predicate("(robot_available robot1)"));
  problem_client->addPredicate(plansys2::Predicate("(robot_available robot2)"));
  problem_client->addPredicate(plansys2::Predicate("(robot_available robot3)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_at wheel_1 wheels_zone)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_at body_car_1 body_car_zone)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_at sterwheel_1 sterwheel_zone)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_is_wheel wheel_1)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_is_body_car body_car_1)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_is_steering_wheel sterwheel_1)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_at wheel_2 wheels_zone)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_at body_car_2 body_car_zone)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_at sterwheel_2 sterwheel_zone)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_is_wheel wheel_2)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_is_body_car body_car_2)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_is_steering_wheel sterwheel_2)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_at wheel_3 wheels_zone)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_at body_car_3 body_car_zone)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_at sterwheel_3 sterwheel_zone)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_is_wheel wheel_3)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_is_body_car body_car_3)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_is_steering_wheel sterwheel_3)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_not_used wheel_1)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_not_used wheel_2)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_not_used wheel_3)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_not_used body_car_1)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_not_used body_car_2)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_not_used body_car_3)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_not_used sterwheel_1)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_not_used sterwheel_2)"));
  problem_client->addPredicate(plansys2::Predicate("(piece_not_used sterwheel_3)"));

  problem_client->setGoal(
    plansys2::Goal(
      "(and(car_assembled car1) (car_assembled car2) (car_assembled car3))"));

  auto domain = domain_client->getDomain();
  auto problem = problem_client->getProblem();
  auto plan = planner_client->getPlan(domain, problem);

  ASSERT_FALSE(domain.empty());
  ASSERT_FALSE(problem.empty());
  ASSERT_TRUE(plan.has_value());

  ASSERT_TRUE(executor_client->start_plan_execution(plan.value()));

  rclcpp::Rate rate(5);
  while (executor_client->execute_and_check_plan()) {
    rate.sleep();
  }

  auto result = executor_client->getResult();

  ASSERT_TRUE(result.value().success);

  ASSERT_TRUE(
    execution_logger->sorted(
  {
    "(move robot1 assembly_zone body_car_zone):0",
    "(transport robot1 body_car_1 body_car_zone assembly_zone):2000",
    "(assemble robot1 assembly_zone wheel_1 body_car_1 sterwheel_1 car1):5002",
    "(move robot1 assembly_zone wheels_zone):10003",
    "(transport robot1 wheel_2 wheels_zone assembly_zone):12004",
    "(move robot1 assembly_zone sterwheel_zone):15005",
    "(transport robot1 sterwheel_3 sterwheel_zone assembly_zone):17006",
    "(assemble robot1 assembly_zone wheel_3 body_car_3 sterwheel_3 car3):20008"
  }));

  ASSERT_TRUE(
    execution_logger->sorted(
  {
    "(move robot2 assembly_zone sterwheel_zone):0",
    "(transport robot2 sterwheel_1 sterwheel_zone assembly_zone):2000",
    "(move robot2 assembly_zone body_car_zone):5002",
    "(transport robot2 body_car_2 body_car_zone assembly_zone):7003",
    "(assemble robot2 assembly_zone wheel_2 body_car_2 sterwheel_2 car2):15005"
  }));

  ASSERT_TRUE(
    execution_logger->sorted(
  {
    "(move robot3 assembly_zone wheels_zone):0",
    "(transport robot3 wheel_1 wheels_zone assembly_zone):2000",
    "(move robot3 assembly_zone sterwheel_zone):5002",
    "(transport robot3 sterwheel_2 sterwheel_zone assembly_zone):7003",
    "(move robot3 assembly_zone body_car_zone):10004",
    "(transport robot3 body_car_3 body_car_zone assembly_zone):12005",
    "(move robot3 assembly_zone wheels_zone):15006",
    "(transport robot3 wheel_3 wheels_zone assembly_zone):17007"
  }));

  ASSERT_TRUE(
    execution_logger->before(
      "(transport robot1 body_car_1 body_car_zone assembly_zone):2000",
      "(assemble robot1 assembly_zone wheel_1 body_car_1 sterwheel_1 car1):5002"
  ));

  ASSERT_TRUE(
    execution_logger->before(
      "(transport robot2 sterwheel_1 sterwheel_zone assembly_zone):2000",
      "(assemble robot1 assembly_zone wheel_1 body_car_1 sterwheel_1 car1):5002"
  ));

  ASSERT_TRUE(
    execution_logger->before(
      "(transport robot3 wheel_1 wheels_zone assembly_zone):2000",
      "(assemble robot1 assembly_zone wheel_1 body_car_1 sterwheel_1 car1):5002"
  ));

  ASSERT_TRUE(
    execution_logger->before(
      "(transport robot1 wheel_2 wheels_zone assembly_zone):12004",
      "(assemble robot2 assembly_zone wheel_2 body_car_2 sterwheel_2 car2):15005"
  ));

  ASSERT_TRUE(
    execution_logger->before(
      "(transport robot2 body_car_2 body_car_zone assembly_zone):7003",
      "(assemble robot2 assembly_zone wheel_2 body_car_2 sterwheel_2 car2):15005"
  ));

  ASSERT_TRUE(
    execution_logger->before(
      "(transport robot3 sterwheel_2 sterwheel_zone assembly_zone):7003",
      "(assemble robot2 assembly_zone wheel_2 body_car_2 sterwheel_2 car2):15005"
  ));

  ASSERT_TRUE(
    execution_logger->before(
      "(transport robot1 sterwheel_3 sterwheel_zone assembly_zone):17006",
      "(assemble robot1 assembly_zone wheel_3 body_car_3 sterwheel_3 car3):20008"
  ));

  ASSERT_TRUE(
    execution_logger->before(
      "(transport robot3 body_car_3 body_car_zone assembly_zone):12005",
      "(assemble robot1 assembly_zone wheel_3 body_car_3 sterwheel_3 car3):20008"
  ));

  ASSERT_TRUE(
    execution_logger->before(
      "(transport robot3 wheel_3 wheels_zone assembly_zone):17007",
      "(assemble robot1 assembly_zone wheel_3 body_car_3 sterwheel_3 car3):20008"
  ));
  finish = true;
  t.join();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
