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

#include <string>
#include <vector>
#include <regex>
#include <iostream>
#include <memory>
#include <fstream>
#include <map>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "plansys2_domain_expert/DomainExpertNode.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_problem_expert/ProblemExpertNode.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_planner/PlannerNode.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_executor/BTBuilder.hpp"

#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_executor/ExecutorNode.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_problem_expert/Utils.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/blackboard.h"

#include "plansys2_executor/behavior_tree/execute_action_node.hpp"
#include "plansys2_executor/behavior_tree/wait_action_node.hpp"
#include "plansys2_executor/behavior_tree/wait_atstart_req_node.hpp"
#include "plansys2_executor/behavior_tree/check_overall_req_node.hpp"
#include "plansys2_executor/behavior_tree/check_atend_req_node.hpp"
#include "plansys2_executor/behavior_tree/apply_atstart_effect_node.hpp"
#include "plansys2_executor/behavior_tree/apply_atend_effect_node.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "gtest/gtest.h"


TEST(executor, action_executor_api)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  ASSERT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto action_executor_1 = plansys2::ActionExecutor::make_shared("action_1", node);

  ASSERT_EQ(action_executor_1->get_status(), BT::NodeStatus::IDLE);
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  using Ptr = std::shared_ptr<MoveAction>;
  static Ptr make_shared(const std::string & node_name, const std::chrono::nanoseconds & rate)
  {
    return std::make_shared<MoveAction>(node_name, rate);
  }


  MoveAction(const std::string & id, const std::chrono::nanoseconds & rate)
  : ActionExecutorClient(id, rate),
    executions_(0),
    cycles_(0),
    runtime_(0)
  {
  }

  void set_runtime(double runtime)
  {
    runtime_ = runtime;
  }

  CallbackReturnT
  on_activate(const rclcpp_lifecycle::State & state)
  {
    std::cerr << "MoveAction::on_activate" << std::endl;
    counter_ = 0;
    start_ = now();

    return ActionExecutorClient::on_activate(state);
  }

  void do_work() override
  {
    RCLCPP_INFO_STREAM(get_logger(), "Executing [" << action_managed_ << "]");
    for (const auto & arg : current_arguments_) {
      RCLCPP_INFO_STREAM(get_logger(), "\t[" << arg << "]");
    }

    cycles_++;
    auto current_time = now();
    auto elapsed_time = (current_time - start_).seconds();

    rclcpp::Rate rate(100);

    if (runtime_ > 1e-5) {
      if (elapsed_time > runtime_) {
        finish(true, 1.0, "completed");
        executions_++;
      } else {
        send_feedback(elapsed_time / runtime_, "running");
        rate.sleep();
      }
    } else {
      if (counter_++ > 3) {
        finish(true, 1.0, "completed");
        executions_++;
      } else {
        send_feedback(counter_ * 0.0, "running");
      }
    }
  }

  int counter_;
  int executions_;
  int cycles_;
  double runtime_;
  rclcpp::Time start_;
};

class TransportAction : public plansys2::ActionExecutorClient
{
public:
  using Ptr = std::shared_ptr<MoveAction>;
  static Ptr make_shared(const std::string & node_name, const std::chrono::nanoseconds & rate)
  {
    return std::make_shared<MoveAction>(node_name, rate);
  }


  TransportAction(const std::string & id, const std::chrono::nanoseconds & rate)
  : ActionExecutorClient(id, rate)
  {
    executions_ = 0;
    cycles_ = 0;
  }

  CallbackReturnT
  on_activate(const rclcpp_lifecycle::State & state)
  {
    std::cerr << "TransportAction::on_activate" << std::endl;
    counter_ = 0;

    return ActionExecutorClient::on_activate(state);
  }

  void do_work() override
  {
    RCLCPP_INFO_STREAM(get_logger(), "Executing [" << action_managed_ << "]");
    for (const auto & arg : current_arguments_) {
      RCLCPP_INFO_STREAM(get_logger(), "\t[" << arg << "]");
    }

    cycles_++;

    if (counter_++ > 3) {
      finish(true, 1.0, "completed");
      executions_++;
    } else {
      send_feedback(counter_ * 0.0, "running");
    }
  }

  int counter_;
  int executions_;
  int cycles_;
};

TEST(executor, action_executor_client)
{
  auto test_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
  auto aux_node = rclcpp_lifecycle::LifecycleNode::make_shared("aux_node");

  auto move_action_1_node = MoveAction::make_shared("move_node_1", 100ms);
  auto move_action_2_node = MoveAction::make_shared("move_node_2", 100ms);
  auto transport_action_node = TransportAction::make_shared("transport_node", 50ms);

  move_action_1_node->set_parameter({"action_name", "move"});
  move_action_2_node->set_parameter({"action_name", "move"});
  transport_action_node->set_parameter({"action_name", "transport"});

  move_action_1_node->set_parameter(
    {"specialized_arguments", std::vector<std::string>({"robot1"})});
  move_action_2_node->set_parameter(
    {"specialized_arguments", std::vector<std::string>({"robot2"})});

  test_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  aux_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  move_action_1_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  move_action_2_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  transport_action_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  test_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  aux_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(test_node->get_node_base_interface());
  exe.add_node(aux_node->get_node_base_interface());
  exe.add_node(move_action_1_node->get_node_base_interface());
  exe.add_node(move_action_2_node->get_node_base_interface());
  exe.add_node(transport_action_node->get_node_base_interface());

  std::vector<plansys2_msgs::msg::ActionExecution> history_msgs;
  bool confirmed = false;
  auto actions_sub = aux_node->create_subscription<plansys2_msgs::msg::ActionExecution>(
    "actions_hub",
    rclcpp::QoS(100).reliable(), [&](plansys2_msgs::msg::ActionExecution::UniquePtr msg) {
      history_msgs.push_back(*msg);
    });

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });


  std::string bt_xml_tree =
    R"(
    <root BTCPP_format="4" main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
          <Parallel success_count="2" failure_count="1">
            <ExecuteAction    action="(move robot1 wheels_zone assembly_zone):5"/>
            <ExecuteAction    action="(move robot1 steering_wheels_zone assembly_zone):5"/>
          </Parallel>
          <ExecuteAction    action="(transport robot2 steering_wheels_zone assembly_zone):10"/>
          <ExecuteAction    action="(move robot2 wheels_zone assembly_zone):15"/>
          <ExecuteAction    action="(transport robot2 steering_wheels_zone assembly_zone):20"/>
          <ExecuteAction    action="(move robot1 wheels_zone assembly_zone):25"/>
          <ExecuteAction    action="(transport robot2 steering_wheels_zone assembly_zone):30"/>
          <ExecuteAction    action="(move robot1 steering_wheels_zone assembly_zone):35"/>
       </Sequence>
      </BehaviorTree>
    </root>
  )";

  auto blackboard = BT::Blackboard::create();

  auto action_map = std::make_shared<std::map<std::string, plansys2::ActionExecutionInfo>>();
  blackboard->set("action_map", action_map);
  blackboard->set("node", test_node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<plansys2::ExecuteAction>("ExecuteAction");
  factory.registerNodeType<plansys2::WaitAction>("WaitAction");

  auto tree = factory.createTreeFromText(bt_xml_tree, blackboard);


  auto status = BT::NodeStatus::RUNNING;
  while (status != BT::NodeStatus::SUCCESS) {
    status = tree.tickOnce();
  }

  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(move_action_1_node->executions_, 4);
  ASSERT_EQ(move_action_1_node->cycles_, 20);
  ASSERT_EQ(move_action_2_node->executions_, 1);
  ASSERT_EQ(move_action_2_node->cycles_, 5);
  ASSERT_EQ(transport_action_node->executions_, 3);
  ASSERT_EQ(transport_action_node->cycles_, 15);
  // ASSERT_EQ(history_msgs.size(), 64);


  finish = true;
  t.join();
}

TEST(executor, action_executor)
{
  auto test_node = rclcpp::Node::make_shared("get_action_from_string");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/factory.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/factory.pddl"});

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });


  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

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

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("robot1", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("robot2", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("robot3", "robot")));

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wheels_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("steering_wheels_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("body_car_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("assembly_zone", "zone")));

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wheel_1", "piece")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wheel_2", "piece")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wheel_3", "piece")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("body_car_1", "piece")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("body_car_2", "piece")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("body_car_3", "piece")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("steering_wheel_1", "piece")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("steering_wheel_2", "piece")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("steering_wheel_3", "piece")));

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("car_1", "car")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("car_2", "car")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("car_3", "car")));

  std::vector<std::string> predicates = {
    "(robot_at robot1 assembly_zone)",
    "(robot_at robot2 assembly_zone)",
    "(robot_at robot3 assembly_zone)",
    "(is_assembly_zone assembly_zone)",
    "(robot_available robot1)",
    "(robot_available robot2)",
    "(robot_available robot3)",
    "(piece_at wheel_1 wheels_zone)",
    "(piece_at body_car_1 body_car_zone)",
    "(piece_at steering_wheel_1 steering_wheels_zone)",
    "(piece_is_wheel wheel_1)",
    "(piece_is_body_car body_car_1)",
    "(piece_is_steering_wheel steering_wheel_1)",
    "(piece_at wheel_2 wheels_zone)",
    "(piece_at body_car_2 body_car_zone)",
    "(piece_at steering_wheel_2 steering_wheels_zone)",
    "(piece_is_wheel wheel_2)",
    "(piece_is_body_car body_car_2)",
    "(piece_is_steering_wheel steering_wheel_2)",
    "(piece_at wheel_3 wheels_zone)",
    "(piece_at body_car_3 body_car_zone)",
    "(piece_at steering_wheel_3 steering_wheels_zone)",
    "(piece_is_wheel wheel_3)",
    "(piece_is_body_car body_car_3)",
    "(piece_is_steering_wheel steering_wheel_3)",
    "(piece_not_used wheel_1)",
    "(piece_not_used wheel_2)",
    "(piece_not_used wheel_3)",
    "(piece_not_used body_car_1)",
    "(piece_not_used body_car_2)",
    "(piece_not_used body_car_3)",
    "(piece_not_used steering_wheel_1)",
    "(piece_not_used steering_wheel_2)",
    "(piece_not_used steering_wheel_3)"};

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }

  ASSERT_TRUE(problem_client->setGoal(plansys2::Goal("(and (car_assembled car_1))")));

  auto plan = planner_client->getPlan(domain_client->getDomain(), problem_client->getProblem());
  ASSERT_TRUE(plan);


  std::shared_ptr<plansys2::BTBuilder> bt_builder;
  pluginlib::ClassLoader<plansys2::BTBuilder> bt_builder_loader("plansys2_executor",
    "plansys2::BTBuilder");
  try {
    bt_builder = bt_builder_loader.createSharedInstance("plansys2::SimpleBTBuilder");
  } catch (pluginlib::PluginlibException & ex) {
    std::cerr << "pluginlib error: " << std::string(ex.what()) << std::endl;
  }

  bt_builder->initialize();
  auto tree_str = bt_builder->get_tree(plan.value());

  finish = true;
  t.join();
}

class ExecuteActionTest : public plansys2::ExecuteAction
{
public:
  ExecuteActionTest(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf)
  : ExecuteAction(xml_tag_name, conf) {}

  void halt() override
  {
    halted_ = true;
    ExecuteAction::halt();
  }

  BT::NodeStatus tick() override
  {
    halted_ = false;
    test_status = ExecuteAction::tick();
    return test_status;
  }

  static bool halted_;
  static BT::NodeStatus test_status;
};

class WaitActionTest : public plansys2::WaitAction
{
public:
  WaitActionTest(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf)
  : WaitAction(xml_tag_name, conf) {}

  void halt() override
  {
    halted_ = true;
    WaitAction::halt();
  }

  BT::NodeStatus tick() override
  {
    test_status = WaitAction::tick();
    return test_status;
  }

  static bool halted_;
  static BT::NodeStatus test_status;
};

class CheckOverAllReqTest : public plansys2::CheckOverAllReq
{
public:
  CheckOverAllReqTest(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf)
  : CheckOverAllReq(xml_tag_name, conf) {}

  void halt() override
  {
    halted_ = true;
    CheckOverAllReq::halt();
  }

  BT::NodeStatus tick() override
  {
    test_status = CheckOverAllReq::tick();
    return test_status;
  }

  static bool halted_;
  static BT::NodeStatus test_status;
};

class WaitAtStartReqTest : public plansys2::WaitAtStartReq
{
public:
  WaitAtStartReqTest(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf)
  : WaitAtStartReq(xml_tag_name, conf) {}

  void halt() override
  {
    halted_ = true;
    WaitAtStartReq::halt();
  }

  BT::NodeStatus tick() override
  {
    test_status = WaitAtStartReq::tick();
    return test_status;
  }

  static bool halted_;
  static BT::NodeStatus test_status;
};

class CheckAtEndReqTest : public plansys2::CheckAtEndReq
{
public:
  CheckAtEndReqTest(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf)
  : CheckAtEndReq(xml_tag_name, conf) {}

  void halt() override
  {
    halted_ = true;
    CheckAtEndReq::halt();
  }

  BT::NodeStatus tick() override
  {
    test_status = CheckAtEndReq::tick();
    return test_status;
  }

  static bool halted_;
  static BT::NodeStatus test_status;
};

class ApplyAtStartEffectTest : public plansys2::ApplyAtStartEffect
{
public:
  ApplyAtStartEffectTest(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf)
  : ApplyAtStartEffect(xml_tag_name, conf) {}

  void halt() override
  {
    halted_ = true;
    ApplyAtStartEffect::halt();
  }

  BT::NodeStatus tick() override
  {
    test_status = ApplyAtStartEffect::tick();
    return test_status;
  }

  static bool halted_;
  static BT::NodeStatus test_status;
};

class ApplyAtEndEffectTest : public plansys2::ApplyAtEndEffect
{
public:
  ApplyAtEndEffectTest(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf)
  : ApplyAtEndEffect(xml_tag_name, conf) {}

  void halt() override
  {
    halted_ = true;
    ApplyAtEndEffect::halt();
  }

  BT::NodeStatus tick() override
  {
    test_status = ApplyAtEndEffect::tick();
    return test_status;
  }

  static bool halted_;
  static BT::NodeStatus test_status;
};

BT::NodeStatus ExecuteActionTest::test_status = BT::NodeStatus::IDLE;
BT::NodeStatus WaitActionTest::test_status = BT::NodeStatus::IDLE;
BT::NodeStatus CheckOverAllReqTest::test_status = BT::NodeStatus::IDLE;
BT::NodeStatus WaitAtStartReqTest::test_status = BT::NodeStatus::IDLE;
BT::NodeStatus CheckAtEndReqTest::test_status = BT::NodeStatus::IDLE;
BT::NodeStatus ApplyAtStartEffectTest::test_status = BT::NodeStatus::IDLE;
BT::NodeStatus ApplyAtEndEffectTest::test_status = BT::NodeStatus::IDLE;

bool ExecuteActionTest::halted_ = false;
bool WaitActionTest::halted_ = false;
bool CheckOverAllReqTest::halted_ = false;
bool WaitAtStartReqTest::halted_ = false;
bool CheckAtEndReqTest::halted_ = false;
bool ApplyAtStartEffectTest::halted_ = false;
bool ApplyAtEndEffectTest::halted_ = false;

TEST(executor, action_real_action_1)
{
  auto test_node = rclcpp::Node::make_shared("action_real_action_1");
  auto test_lf_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_lf_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();

  auto move_action_node = MoveAction::make_shared("move_action_performer", 100ms);
  move_action_node->set_parameter({"action_name", "move"});

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/factory3.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/factory3.pddl"});

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(move_action_node->get_node_base_interface());
  exe.add_node(test_lf_node->get_node_base_interface());


  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });


  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  move_action_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  test_lf_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

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
  test_lf_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("r2d2", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wheels_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("steering_wheels_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("body_car_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("assembly_zone", "zone")));

  std::string bt_xml_tree =
    R"(
    <root BTCPP_format="4" main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <Sequence name="(move r2d2 steering_wheels_zone assembly_zone):0">
          <WaitAction action="other"/>
          <WaitAtStartReq action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
          <ApplyAtStartEffect action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
          <ReactiveSequence name="(move r2d2 steering_wheels_zone assembly_zone):0">
            <CheckOverAllReq action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
            <ExecuteAction action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
          </ReactiveSequence>
          <CheckAtEndReq action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
          <ApplyAtEndEffect action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
        </Sequence>
      </BehaviorTree>
    </root>
  )";

  auto action_map = std::make_shared<std::map<std::string, plansys2::ActionExecutionInfo>>();
  (*action_map)["(move r2d2 steering_wheels_zone assembly_zone):0"] =
    plansys2::ActionExecutionInfo();
  (*action_map)["(move r2d2 steering_wheels_zone assembly_zone):0"].action_executor =
    plansys2::ActionExecutor::make_shared(
    "(move r2d2 steering_wheels_zone assembly_zone)", test_lf_node);
  (*action_map)["(move r2d2 steering_wheels_zone assembly_zone):0"].action_info =
    domain_client->getDurativeAction(
    plansys2::get_action_name("(move r2d2 steering_wheels_zone assembly_zone):0"),
    plansys2::get_action_params("(move r2d2 steering_wheels_zone assembly_zone):0"));

  auto blackboard = BT::Blackboard::create();

  blackboard->set("action_map", action_map);
  blackboard->set("node", test_lf_node);
  blackboard->set("domain_client", domain_client);
  blackboard->set("problem_client", problem_client);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ExecuteActionTest>("ExecuteAction");
  factory.registerNodeType<WaitActionTest>("WaitAction");
  factory.registerNodeType<CheckOverAllReqTest>("CheckOverAllReq");
  factory.registerNodeType<WaitAtStartReqTest>("WaitAtStartReq");
  factory.registerNodeType<CheckAtEndReqTest>("CheckAtEndReq");
  factory.registerNodeType<ApplyAtStartEffectTest>("ApplyAtStartEffect");
  factory.registerNodeType<ApplyAtEndEffectTest>("ApplyAtEndEffect");

  // Test WaitAtStartReq
  try {
    auto tree = factory.createTreeFromText(bt_xml_tree, blackboard);

    auto status = BT::NodeStatus::RUNNING;

    for (int i = 0; i < 10; i++) {
      status = tree.tickOnce();
      ASSERT_EQ(status, BT::NodeStatus::RUNNING);
      ASSERT_EQ(WaitActionTest::test_status, BT::NodeStatus::RUNNING);
    }
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
  }

  // Test ApplyAtStartEffect and CheckOverAllReq
  bt_xml_tree =
    R"(
    <root BTCPP_format="4" main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <Sequence name="(move r2d2 steering_wheels_zone assembly_zone):0">
          <WaitAtStartReq action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
          <ApplyAtStartEffect action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
          <ReactiveSequence name="(move r2d2 steering_wheels_zone assembly_zone):0">
            <CheckOverAllReq action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
            <ExecuteAction action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
          </ReactiveSequence>
          <CheckAtEndReq action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
          <ApplyAtEndEffect action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
        </Sequence>
      </BehaviorTree>
    </root>
  )";


  std::vector<std::string> predicates = {
    "(robot_at r2d2 steering_wheels_zone)",
    "(robot_available r2d2)",
    "(battery_full r2d2)",
  };

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }

  try {
    auto tree = factory.createTreeFromText(bt_xml_tree, blackboard);

    auto status = BT::NodeStatus::RUNNING;

    ASSERT_TRUE(problem_client->existPredicate(plansys2::Predicate("(robot_available r2d2)")));
    status = tree.tickOnce();

    ASSERT_EQ(ApplyAtStartEffectTest::test_status, BT::NodeStatus::SUCCESS);
    ASSERT_FALSE(problem_client->existPredicate(plansys2::Predicate("(robot_available r2d2)")));
    ASSERT_FALSE(
      problem_client->existPredicate(
        plansys2::Predicate(
          "(robot_at r2d2 steering_wheels_zone)")));

    status = tree.tickOnce();
    ASSERT_EQ(CheckOverAllReqTest::test_status, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(ExecuteActionTest::test_status, BT::NodeStatus::RUNNING);
    status = tree.tickOnce();
    ASSERT_EQ(CheckOverAllReqTest::test_status, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(ExecuteActionTest::test_status, BT::NodeStatus::RUNNING);

    ASSERT_TRUE(problem_client->removePredicate(plansys2::Predicate("(battery_full r2d2)")));

    status = tree.tickOnce();
    ASSERT_EQ(CheckOverAllReqTest::test_status, BT::NodeStatus::FAILURE);
    ASSERT_EQ(status, BT::NodeStatus::FAILURE);
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
  }

  // Test CheckAtEndReq and ApplyAtEndEffect
  (*action_map)["(move r2d2 steering_wheels_zone assembly_zone):0"].at_start_effects_applied =
    false;

  ApplyAtStartEffectTest::test_status = BT::NodeStatus::IDLE;

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }

  try {
    auto tree = factory.createTreeFromText(bt_xml_tree, blackboard);

    auto status = BT::NodeStatus::RUNNING;

    ASSERT_TRUE(problem_client->existPredicate(plansys2::Predicate("(robot_available r2d2)")));

    while (ApplyAtStartEffectTest::test_status != BT::NodeStatus::SUCCESS) {
      status = tree.tickOnce();
    }

    ASSERT_FALSE(
      problem_client->existPredicate(
        plansys2::Predicate(
          "(robot_at r2d2 assembly_zone)")));
    ASSERT_FALSE(problem_client->existPredicate(plansys2::Predicate("(robot_available r2d2)")));

    while (ExecuteActionTest::test_status != BT::NodeStatus::SUCCESS) {
      status = tree.tickOnce();
      ASSERT_EQ(CheckOverAllReqTest::test_status, BT::NodeStatus::SUCCESS);
    }

    while (ApplyAtEndEffectTest::test_status != BT::NodeStatus::SUCCESS) {
      status = tree.tickOnce();
    }

    ASSERT_TRUE(
      problem_client->existPredicate(plansys2::Predicate("(robot_at r2d2 assembly_zone)")));
    ASSERT_TRUE(problem_client->existPredicate(plansys2::Predicate("(robot_available r2d2)")));
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
  }


  ExecuteActionTest::test_status = BT::NodeStatus::IDLE;
  WaitActionTest::test_status = BT::NodeStatus::IDLE;
  CheckOverAllReqTest::test_status = BT::NodeStatus::IDLE;
  WaitAtStartReqTest::test_status = BT::NodeStatus::IDLE;
  CheckAtEndReqTest::test_status = BT::NodeStatus::IDLE;
  ApplyAtStartEffectTest::test_status = BT::NodeStatus::IDLE;
  ApplyAtEndEffectTest::test_status = BT::NodeStatus::IDLE;

  ExecuteActionTest::halted_ = false;
  WaitActionTest::halted_ = false;
  CheckOverAllReqTest::halted_ = false;
  WaitAtStartReqTest::halted_ = false;
  CheckAtEndReqTest::halted_ = false;
  ApplyAtStartEffectTest::halted_ = false;
  ApplyAtEndEffectTest::halted_ = false;

  finish = true;
  t.join();
}

TEST(executor, action_real_action_2)
{
  auto test_node = rclcpp::Node::make_shared("action_real_action_2");
  auto test_lf_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_lf_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();

  auto move_action_node = MoveAction::make_shared("move_action_performer", 100ms);
  move_action_node->set_parameter({"action_name", "move"});

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/factory4.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/factory4.pddl"});

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(move_action_node->get_node_base_interface());
  exe.add_node(test_lf_node->get_node_base_interface());


  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });


  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  move_action_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  test_lf_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

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
  test_lf_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("r2d2", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wheels_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("steering_wheels_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("body_car_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("assembly_zone", "zone")));

  std::string bt_xml_tree =
    R"(
    <root BTCPP_format="4" main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <Sequence name="(move r2d2 steering_wheels_zone assembly_zone):0">
          <WaitAction action="other"/>
          <WaitAtStartReq action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
          <ApplyAtStartEffect action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
          <ReactiveSequence name="(move r2d2 steering_wheels_zone assembly_zone):0">
            <CheckOverAllReq action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
            <ExecuteAction action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
          </ReactiveSequence>
          <CheckAtEndReq action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
          <ApplyAtEndEffect action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
        </Sequence>
      </BehaviorTree>
    </root>
  )";

  auto action_map = std::make_shared<std::map<std::string, plansys2::ActionExecutionInfo>>();
  (*action_map)["(move r2d2 steering_wheels_zone assembly_zone):0"] =
    plansys2::ActionExecutionInfo();
  (*action_map)["(move r2d2 steering_wheels_zone assembly_zone):0"].action_executor =
    plansys2::ActionExecutor::make_shared(
    "(move r2d2 steering_wheels_zone assembly_zone)", test_lf_node);
  (*action_map)["(move r2d2 steering_wheels_zone assembly_zone):0"].action_info =
    domain_client->getAction(
    plansys2::get_action_name("(move r2d2 steering_wheels_zone assembly_zone):0"),
    plansys2::get_action_params("(move r2d2 steering_wheels_zone assembly_zone):0"));

  auto blackboard = BT::Blackboard::create();

  blackboard->set("action_map", action_map);
  blackboard->set("node", test_lf_node);
  blackboard->set("domain_client", domain_client);
  blackboard->set("problem_client", problem_client);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ExecuteActionTest>("ExecuteAction");
  factory.registerNodeType<WaitActionTest>("WaitAction");
  factory.registerNodeType<CheckOverAllReqTest>("CheckOverAllReq");
  factory.registerNodeType<WaitAtStartReqTest>("WaitAtStartReq");
  factory.registerNodeType<CheckAtEndReqTest>("CheckAtEndReq");
  factory.registerNodeType<ApplyAtStartEffectTest>("ApplyAtStartEffect");
  factory.registerNodeType<ApplyAtEndEffectTest>("ApplyAtEndEffect");

  // Test WaitAtStartReq
  try {
    auto tree = factory.createTreeFromText(bt_xml_tree, blackboard);

    auto status = BT::NodeStatus::RUNNING;

    for (int i = 0; i < 10; i++) {
      status = tree.tickOnce();
      ASSERT_EQ(status, BT::NodeStatus::RUNNING);
      ASSERT_EQ(WaitActionTest::test_status, BT::NodeStatus::RUNNING);
    }
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
  }

  // Test ApplyAtStartEffect and CheckOverAllReq
  bt_xml_tree =
    R"(
    <root BTCPP_format="4" main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <Sequence name="(move r2d2 steering_wheels_zone assembly_zone):0">
          <WaitAtStartReq action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
          <ApplyAtStartEffect action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
          <ReactiveSequence name="(move r2d2 steering_wheels_zone assembly_zone):0">
            <CheckOverAllReq action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
            <ExecuteAction action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
          </ReactiveSequence>
          <CheckAtEndReq action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
          <ApplyAtEndEffect action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
        </Sequence>
      </BehaviorTree>
    </root>
  )";


  std::vector<std::string> predicates = {
    "(robot_at r2d2 steering_wheels_zone)",
    "(robot_available r2d2)",
    "(battery_full r2d2)",
  };

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }

  try {
    auto tree = factory.createTreeFromText(bt_xml_tree, blackboard);

    auto status = BT::NodeStatus::RUNNING;

    ASSERT_TRUE(problem_client->existPredicate(plansys2::Predicate("(robot_available r2d2)")));
    status = tree.tickOnce();

    ASSERT_EQ(ApplyAtStartEffectTest::test_status, BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(problem_client->existPredicate(plansys2::Predicate("(robot_available r2d2)")));
    ASSERT_TRUE(
      problem_client->existPredicate(
        plansys2::Predicate(
          "(robot_at r2d2 steering_wheels_zone)")));

    status = tree.tickOnce();
    ASSERT_EQ(CheckOverAllReqTest::test_status, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(ExecuteActionTest::test_status, BT::NodeStatus::RUNNING);
    status = tree.tickOnce();
    ASSERT_EQ(CheckOverAllReqTest::test_status, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(ExecuteActionTest::test_status, BT::NodeStatus::RUNNING);

    ASSERT_TRUE(problem_client->removePredicate(plansys2::Predicate("(battery_full r2d2)")));

    status = tree.tickOnce();
    ASSERT_EQ(CheckOverAllReqTest::test_status, BT::NodeStatus::FAILURE);
    ASSERT_EQ(status, BT::NodeStatus::FAILURE);
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
  }

  // Test CheckAtEndReq and ApplyAtEndEffect
  (*action_map)["(move r2d2 steering_wheels_zone assembly_zone):0"].at_start_effects_applied =
    false;

  ApplyAtStartEffectTest::test_status = BT::NodeStatus::IDLE;

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }

  try {
    auto tree = factory.createTreeFromText(bt_xml_tree, blackboard);

    auto status = BT::NodeStatus::RUNNING;

    ASSERT_TRUE(problem_client->existPredicate(plansys2::Predicate("(robot_available r2d2)")));

    while (ApplyAtStartEffectTest::test_status != BT::NodeStatus::SUCCESS) {
      status = tree.tickOnce();
    }

    ASSERT_FALSE(
      problem_client->existPredicate(
        plansys2::Predicate(
          "(robot_at r2d2 assembly_zone)")));
    ASSERT_FALSE(problem_client->existPredicate(plansys2::Predicate("(robot_available r2d2)")));

    while (ExecuteActionTest::test_status != BT::NodeStatus::SUCCESS) {
      status = tree.tickOnce();
      ASSERT_EQ(CheckOverAllReqTest::test_status, BT::NodeStatus::SUCCESS);
    }

    while (ApplyAtEndEffectTest::test_status != BT::NodeStatus::SUCCESS) {
      status = tree.tickOnce();
    }

    ASSERT_TRUE(
      problem_client->existPredicate(plansys2::Predicate("(robot_at r2d2 assembly_zone)")));
    ASSERT_TRUE(problem_client->existPredicate(plansys2::Predicate("(robot_available r2d2)")));
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
  }


  ExecuteActionTest::test_status = BT::NodeStatus::IDLE;
  WaitActionTest::test_status = BT::NodeStatus::IDLE;
  CheckOverAllReqTest::test_status = BT::NodeStatus::IDLE;
  WaitAtStartReqTest::test_status = BT::NodeStatus::IDLE;
  CheckAtEndReqTest::test_status = BT::NodeStatus::IDLE;
  ApplyAtStartEffectTest::test_status = BT::NodeStatus::IDLE;
  ApplyAtEndEffectTest::test_status = BT::NodeStatus::IDLE;

  ExecuteActionTest::halted_ = false;
  WaitActionTest::halted_ = false;
  CheckOverAllReqTest::halted_ = false;
  WaitAtStartReqTest::halted_ = false;
  CheckAtEndReqTest::halted_ = false;
  ApplyAtStartEffectTest::halted_ = false;
  ApplyAtEndEffectTest::halted_ = false;

  finish = true;
  t.join();
}

TEST(executor, cancel_bt_execution)
{
  auto test_node = rclcpp::Node::make_shared("action_real_action_1");
  auto test_lf_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_lf_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();

  auto move_action_node = MoveAction::make_shared("move_action_performer", 100ms);
  move_action_node->set_parameter({"action_name", "move"});

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/factory3.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/factory3.pddl"});

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(move_action_node->get_node_base_interface());
  exe.add_node(test_lf_node->get_node_base_interface());


  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });


  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  move_action_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  test_lf_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

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
  test_lf_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("r2d2", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wheels_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("steering_wheels_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("body_car_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("assembly_zone", "zone")));

  std::string bt_xml_tree =
    R"(
    <root BTCPP_format="4" main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <WaitAtStartReq action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
        <Sequence name="(move r2d2 steering_wheels_zone assembly_zone):0">
          <ApplyAtStartEffect action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
          <ReactiveSequence name="(move r2d2 steering_wheels_zone assembly_zone):0">
            <CheckOverAllReq action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
            <ExecuteAction action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
          </ReactiveSequence>
          <CheckAtEndReq action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
          <ApplyAtEndEffect action="(move r2d2 steering_wheels_zone assembly_zone):0"/>
        </Sequence>
      </BehaviorTree>
    </root>
  )";

  auto action_map = std::make_shared<std::map<std::string, plansys2::ActionExecutionInfo>>();
  (*action_map)["(move r2d2 steering_wheels_zone assembly_zone):0"] =
    plansys2::ActionExecutionInfo();
  (*action_map)["(move r2d2 steering_wheels_zone assembly_zone):0"].action_executor =
    plansys2::ActionExecutor::make_shared(
    "(move r2d2 steering_wheels_zone assembly_zone)", test_lf_node);
  (*action_map)["(move r2d2 steering_wheels_zone assembly_zone):0"].action_info =
    domain_client->getDurativeAction(
    plansys2::get_action_name("(move r2d2 steering_wheels_zone assembly_zone):0"),
    plansys2::get_action_params("(move r2d2 steering_wheels_zone assembly_zone):0"));

  auto blackboard = BT::Blackboard::create();

  blackboard->set("action_map", action_map);
  blackboard->set("node", test_lf_node);
  blackboard->set("domain_client", domain_client);
  blackboard->set("problem_client", problem_client);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ExecuteActionTest>("ExecuteAction");
  factory.registerNodeType<WaitActionTest>("WaitAction");
  factory.registerNodeType<CheckOverAllReqTest>("CheckOverAllReq");
  factory.registerNodeType<WaitAtStartReqTest>("WaitAtStartReq");
  factory.registerNodeType<CheckAtEndReqTest>("CheckAtEndReq");
  factory.registerNodeType<ApplyAtStartEffectTest>("ApplyAtStartEffect");
  factory.registerNodeType<ApplyAtEndEffectTest>("ApplyAtEndEffect");

  std::vector<std::string> predicates = {
    "(robot_at r2d2 steering_wheels_zone)",
    "(robot_available r2d2)",
    "(battery_full r2d2)",
  };

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }

  try {
    auto tree = factory.createTreeFromText(bt_xml_tree, blackboard);

    auto status = BT::NodeStatus::RUNNING;

    ASSERT_TRUE(problem_client->existPredicate(plansys2::Predicate("(robot_available r2d2)")));
    status = tree.tickOnce();

    ASSERT_EQ(ApplyAtStartEffectTest::test_status, BT::NodeStatus::SUCCESS);
    ASSERT_FALSE(problem_client->existPredicate(plansys2::Predicate("(robot_available r2d2)")));
    ASSERT_FALSE(
      problem_client->existPredicate(
        plansys2::Predicate(
          "(robot_at r2d2 steering_wheels_zone)")));

    status = tree.tickOnce();
    ASSERT_EQ(CheckOverAllReqTest::test_status, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(ExecuteActionTest::test_status, BT::NodeStatus::RUNNING);
    status = tree.tickOnce();
    ASSERT_EQ(CheckOverAllReqTest::test_status, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(ExecuteActionTest::test_status, BT::NodeStatus::RUNNING);

    tree.haltTree();

    ASSERT_EQ(ExecuteActionTest::test_status, BT::NodeStatus::RUNNING);
    ASSERT_EQ(CheckOverAllReqTest::test_status, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(CheckAtEndReqTest::test_status, BT::NodeStatus::IDLE);
    ASSERT_EQ(ApplyAtStartEffectTest::test_status, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(ApplyAtEndEffectTest::test_status, BT::NodeStatus::IDLE);

    ASSERT_TRUE(ExecuteActionTest::halted_);
    ASSERT_TRUE(CheckOverAllReqTest::halted_);
    ASSERT_TRUE(CheckAtEndReqTest::halted_);
    ASSERT_TRUE(ApplyAtStartEffectTest::halted_);
    ASSERT_TRUE(ApplyAtEndEffectTest::halted_);

    tree = factory.createTreeFromText(bt_xml_tree, blackboard);

    status = tree.tickOnce();
    status = tree.tickOnce();
    status = tree.tickOnce();
    ASSERT_EQ(ApplyAtStartEffectTest::test_status, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(CheckOverAllReqTest::test_status, BT::NodeStatus::SUCCESS);
    ASSERT_EQ(ExecuteActionTest::test_status, BT::NodeStatus::RUNNING);
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
  }

  ExecuteActionTest::test_status = BT::NodeStatus::IDLE;
  WaitActionTest::test_status = BT::NodeStatus::IDLE;
  CheckOverAllReqTest::test_status = BT::NodeStatus::IDLE;
  WaitAtStartReqTest::test_status = BT::NodeStatus::IDLE;
  CheckAtEndReqTest::test_status = BT::NodeStatus::IDLE;
  ApplyAtStartEffectTest::test_status = BT::NodeStatus::IDLE;
  ApplyAtEndEffectTest::test_status = BT::NodeStatus::IDLE;

  ExecuteActionTest::halted_ = false;
  WaitActionTest::halted_ = false;
  CheckOverAllReqTest::halted_ = false;
  WaitAtStartReqTest::halted_ = false;
  CheckAtEndReqTest::halted_ = false;
  ApplyAtStartEffectTest::halted_ = false;
  ApplyAtEndEffectTest::halted_ = false;

  finish = true;
  t.join();
}

class ExecutorNodeTest : public plansys2::ExecutorNode
{
public:
  bool is_cancelled() {return cancel_plan_requested_;}
};

TEST(executor, executor_client_execute_plan)
{
  auto test_node_1 = rclcpp::Node::make_shared("test_node_1");
  auto test_node_2 = rclcpp::Node::make_shared("test_node_2");
  auto test_node_3 = rclcpp::Node::make_shared("test_node_3");
  auto test_lf_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_lf_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto executor_node = std::make_shared<ExecutorNodeTest>();

  auto move_action_node = MoveAction::make_shared("move_action_performer", 100ms);
  move_action_node->set_parameter({"action_name", "move"});

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  auto executor_client = std::make_shared<plansys2::ExecutorClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/factory3.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/factory3.pddl"});

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(executor_node->get_node_base_interface());
  exe.add_node(move_action_node->get_node_base_interface());
  exe.add_node(test_lf_node->get_node_base_interface());


  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });


  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  move_action_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  test_lf_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node_1->now();
    while ((test_node_1->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  test_lf_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node_1->now();
    while ((test_node_1->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("r2d2", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wheels_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("steering_wheels_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("body_car_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("assembly_zone", "zone")));

  std::vector<std::string> predicates = {
    "(robot_at r2d2 steering_wheels_zone)",
    "(robot_available r2d2)",
    "(battery_full r2d2)",
  };

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }
  problem_client->setGoal(plansys2::Goal("(and(robot_at r2d2 assembly_zone))"));

  auto domain = domain_client->getDomain();
  auto problem = problem_client->getProblem();
  auto plan = planner_client->getPlan(domain, problem);
  ASSERT_FALSE(domain.empty());
  ASSERT_FALSE(problem.empty());
  ASSERT_TRUE(plan.has_value());

  {
    rclcpp::Rate rate(10);
    auto start = test_node_1->now();

    ASSERT_TRUE(executor_client->start_plan_execution(plan.value()));

    while (rclcpp::ok() && executor_client->execute_and_check_plan()) {
      auto feedback = executor_client->getFeedBack();
      ASSERT_LT(feedback.action_execution_status.size(), 3);
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->existPredicate(plansys2::Predicate("(robot_at r2d2 assembly_zone)")));

  ASSERT_TRUE(executor_client->getResult().has_value());
  auto result = executor_client->getResult().value();

  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.action_execution_status.size(), 2u);
  for (const auto & action_status : result.action_execution_status) {
    ASSERT_EQ(action_status.status, plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED);
  }

  problem_client->setGoal(plansys2::Goal("(and(robot_at r2d2 body_car_zone))"));

  problem = problem_client->getProblem();
  plan = planner_client->getPlan(domain, problem);
  ASSERT_FALSE(problem.empty());
  ASSERT_TRUE(plan.has_value());

  {
    rclcpp::Rate rate(10);
    auto start = test_node_1->now();

    ASSERT_TRUE(executor_client->start_plan_execution(plan.value()));

    while (rclcpp::ok() && executor_client->execute_and_check_plan()) {
      auto feedback = executor_client->getFeedBack();
      ASSERT_LT(feedback.action_execution_status.size(), 3);
      rate.sleep();
    }
  }

  ASSERT_TRUE(
    problem_client->existPredicate(plansys2::Predicate("(robot_at r2d2 body_car_zone)")));

  ASSERT_TRUE(executor_client->getResult().has_value());
  result = executor_client->getResult().value();
  for (const auto & action_status : result.action_execution_status) {
    ASSERT_EQ(action_status.status, plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED);
  }

  finish = true;
  t.join();
}

TEST(executor, executor_client_execute_plan_2)
{
  auto test_node_1 = rclcpp::Node::make_shared("test_node_1");
  auto test_node_2 = rclcpp::Node::make_shared("test_node_2");
  auto test_node_3 = rclcpp::Node::make_shared("test_node_3");
  auto test_lf_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_lf_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto executor_node = std::make_shared<ExecutorNodeTest>();

  auto move_action_node = MoveAction::make_shared("move_action_performer", 100ms);
  move_action_node->set_parameter({"action_name", "move"});

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  auto executor_client = std::make_shared<plansys2::ExecutorClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/factory4.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/factory4.pddl"});

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(executor_node->get_node_base_interface());
  exe.add_node(move_action_node->get_node_base_interface());
  exe.add_node(test_lf_node->get_node_base_interface());


  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });


  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  move_action_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  test_lf_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node_1->now();
    while ((test_node_1->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  test_lf_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node_1->now();
    while ((test_node_1->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("r2d2", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wheels_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("steering_wheels_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("body_car_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("assembly_zone", "zone")));

  std::vector<std::string> predicates = {
    "(robot_at r2d2 steering_wheels_zone)",
    "(robot_available r2d2)",
    "(battery_full r2d2)",
  };

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }
  problem_client->setGoal(plansys2::Goal("(and(robot_at r2d2 assembly_zone))"));

  auto domain = domain_client->getDomain();
  auto problem = problem_client->getProblem();
  auto plan = planner_client->getPlan(domain, problem);
  ASSERT_FALSE(domain.empty());
  ASSERT_FALSE(problem.empty());
  ASSERT_TRUE(plan.has_value());

  {
    rclcpp::Rate rate(10);
    auto start = test_node_1->now();

    ASSERT_TRUE(executor_client->start_plan_execution(plan.value()));
    while (rclcpp::ok() && executor_client->execute_and_check_plan()) {
      auto feedback = executor_client->getFeedBack();
      ASSERT_LT(feedback.action_execution_status.size(), 3);
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->existPredicate(plansys2::Predicate("(robot_at r2d2 assembly_zone)")));

  ASSERT_TRUE(executor_client->getResult().has_value());
  auto result = executor_client->getResult().value();

  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.action_execution_status.size(), 2u);
  for (const auto & action_status : result.action_execution_status) {
    ASSERT_EQ(action_status.status, plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED);
  }
  problem_client->setGoal(plansys2::Goal("(and(robot_at r2d2 body_car_zone))"));

  problem = problem_client->getProblem();
  plan = planner_client->getPlan(domain, problem);
  ASSERT_FALSE(problem.empty());
  ASSERT_TRUE(plan.has_value());
  {
    rclcpp::Rate rate(10);
    auto start = test_node_1->now();

    ASSERT_TRUE(executor_client->start_plan_execution(plan.value()));
    while (rclcpp::ok() && executor_client->execute_and_check_plan()) {
      auto feedback = executor_client->getFeedBack();
      ASSERT_LT(feedback.action_execution_status.size(), 3);
      rate.sleep();
    }
  }

  ASSERT_TRUE(
    problem_client->existPredicate(plansys2::Predicate("(robot_at r2d2 body_car_zone)")));

  ASSERT_TRUE(executor_client->getResult().has_value());
  result = executor_client->getResult().value();
  for (const auto & action_status : result.action_execution_status) {
    ASSERT_EQ(action_status.status, plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED);
  }

  finish = true;
  t.join();
}

class PatrolAction : public plansys2::ActionExecutorClient
{
public:
  using Ptr = std::shared_ptr<PatrolAction>;
  static Ptr make_shared(const std::string & node_name, const std::chrono::nanoseconds & rate)
  {
    return std::make_shared<PatrolAction>(node_name, rate);
  }


  PatrolAction(const std::string & id, const std::chrono::nanoseconds & rate)
  : ActionExecutorClient(id, rate)
  {
    executions_ = 0;
    cycles_ = 0;
  }

  CallbackReturnT
  on_activate(const rclcpp_lifecycle::State & state)
  {
    std::cerr << "PatrolAction::on_activate" << std::endl;
    counter_ = 0;

    return ActionExecutorClient::on_activate(state);
  }

  void do_work() override
  {
    RCLCPP_INFO_STREAM(get_logger(), "Executing [" << action_managed_ << "]");
    for (const auto & arg : current_arguments_) {
      RCLCPP_INFO_STREAM(get_logger(), "\t[" << arg << "]");
    }

    cycles_++;

    if (counter_++ > 1) {
      finish(true, 1.0, "completed");
      executions_++;
    } else {
      send_feedback((counter_ / 1.0) * 100.0, "running");
    }
  }

  int counter_;
  int executions_;
  int cycles_;
};


TEST(executor, executor_client_ordered_sub_goals)
{
  auto test_node_1 = rclcpp::Node::make_shared("test_node_1");
  auto test_node_2 = rclcpp::Node::make_shared("test_node_2");
  auto test_node_3 = rclcpp::Node::make_shared("test_node_3");
  auto test_lf_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_lf_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto executor_node = std::make_shared<ExecutorNodeTest>();

  auto move_action_node = MoveAction::make_shared("move_action_performer", 100ms);
  move_action_node->set_parameter({"action_name", "move"});

  auto patrol_action_node = PatrolAction::make_shared("patrol_action_performer", 100ms);
  patrol_action_node->set_parameter({"action_name", "patrol"});

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  auto executor_client = std::make_shared<plansys2::ExecutorClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/domain_charging.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/domain_charging.pddl"});

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(executor_node->get_node_base_interface());
  exe.add_node(move_action_node->get_node_base_interface());
  exe.add_node(patrol_action_node->get_node_base_interface());
  exe.add_node(test_lf_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  move_action_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  patrol_action_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  test_lf_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node_1->now();
    while ((test_node_1->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  test_lf_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node_1->now();
    while ((test_node_1->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("r2d2", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wp0", "waypoint")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wp1", "waypoint")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wp2", "waypoint")));

  std::vector<std::string> predicates = {
    "(robot_at r2d2 wp0)",
    "(connected wp0 wp1)",
    "(connected wp1 wp0)",
    "(connected wp0 wp2)",
    "(connected wp2 wp0)",
    "(connected wp1 wp2)",
    "(connected wp2 wp1)",
  };
  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }

  std::vector<std::string> functions = {
    "(= (speed r2d2) 1.0)",
    "(= (max_range r2d2) 100.0)",
    "(= (state_of_charge r2d2) 100.0)",
    "(= (distance wp0 wp1) 5.0)",
    "(= (distance wp1 wp0) 5.0)",
    "(= (distance wp0 wp2) 15.0)",
    "(= (distance wp2 wp0) 15.0)",
    "(= (distance wp1 wp2) 5.0)",
    "(= (distance wp2 wp1) 5.0)",
  };
  for (const auto & func : functions) {
    ASSERT_TRUE(problem_client->addFunction(plansys2::Function(func)));
  }

  problem_client->setGoal(plansys2::Goal("(and(patrolled wp1) (patrolled wp2))"));

  plansys2_msgs::msg::Tree sub_goal_1;
  plansys2_msgs::msg::Tree sub_goal_2;
  parser::pddl::fromString(sub_goal_1, "(and(patrolled wp1))");
  parser::pddl::fromString(sub_goal_2, "(and(patrolled wp2))");
  std::vector<plansys2_msgs::msg::Tree> expected_sub_goals = {sub_goal_1, sub_goal_2};

  auto domain = domain_client->getDomain();
  auto problem = problem_client->getProblem();
  auto plan = planner_client->getPlan(domain, problem);
  ASSERT_FALSE(domain.empty());
  ASSERT_FALSE(problem.empty());
  ASSERT_TRUE(plan.has_value());

  {
    rclcpp::Rate rate(10);
    auto start = test_node_1->now();

    ASSERT_TRUE(executor_client->start_plan_execution(plan.value()));

    while (rclcpp::ok() && executor_client->execute_and_check_plan()) {
      auto feedback = executor_client->getFeedBack();

      ASSERT_LE(feedback.action_execution_status.size(), 5);
      rate.sleep();
    }
  }
  std::vector<plansys2_msgs::msg::Tree> actual_sub_goals = executor_client->getOrderedSubGoals();

  ASSERT_EQ(actual_sub_goals.size(), expected_sub_goals.size());
  for (size_t i = 0; i < actual_sub_goals.size(); i++) {
    ASSERT_EQ(
      parser::pddl::toString(actual_sub_goals[i]),
      parser::pddl::toString(expected_sub_goals[i]));
  }

  finish = true;
  t.join();
}

TEST(executor, executor_client_cancel_plan)
{
  auto test_node_1 = rclcpp::Node::make_shared("test_node_1");
  auto test_node_2 = rclcpp::Node::make_shared("test_node_2");
  auto test_node_3 = rclcpp::Node::make_shared("test_node_3");
  auto test_lf_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_lf_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto executor_node = std::make_shared<ExecutorNodeTest>();

  auto move_action_node = MoveAction::make_shared("move_action_performer", 1s);
  move_action_node->set_parameter({"action_name", "move"});

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  auto executor_client = std::make_shared<plansys2::ExecutorClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/factory3.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/factory3.pddl"});

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(executor_node->get_node_base_interface());
  exe.add_node(move_action_node->get_node_base_interface());
  exe.add_node(test_lf_node->get_node_base_interface());


  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });


  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  move_action_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  test_lf_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node_1->now();
    while ((test_node_1->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  test_lf_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node_1->now();
    while ((test_node_1->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("r2d2", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wheels_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("steering_wheels_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("body_car_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("assembly_zone", "zone")));

  std::vector<std::string> predicates = {
    "(robot_at r2d2 steering_wheels_zone)",
    "(robot_available r2d2)",
    "(battery_full r2d2)",
  };

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }

  problem_client->setGoal(plansys2::Goal("(and(robot_at r2d2 assembly_zone))"));

  auto domain = domain_client->getDomain();
  auto problem = problem_client->getProblem();
  auto plan = planner_client->getPlan(domain, problem);
  ASSERT_FALSE(domain.empty());
  ASSERT_FALSE(problem.empty());
  ASSERT_TRUE(plan.has_value());

  {
    rclcpp::Rate rate(10);
    auto start = test_node_1->now();

    ASSERT_TRUE(executor_client->start_plan_execution(plan.value()));

    while (rclcpp::ok() && executor_client->execute_and_check_plan()) {
      auto feedback = executor_client->getFeedBack();
      if (!feedback.action_execution_status.empty() &&
        feedback.action_execution_status[0].status ==
        plansys2_msgs::msg::ActionExecutionInfo::EXECUTING &&
        (test_node_1->now() - start).seconds() > 3)
      {
        executor_client->cancel_plan_execution();
        break;
      }
      rate.sleep();
    }
  }

  ASSERT_FALSE(executor_client->getResult().has_value());

  {
    rclcpp::Rate rate(10);
    auto start = test_node_1->now();
    while ((test_node_1->now() - start).seconds() < 2.0) {
      rate.sleep();
    }
  }

  ASSERT_EQ(
    move_action_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  finish = true;
  t.join();
}

TEST(executor, action_timeout)
{
  auto test_node_1 = rclcpp::Node::make_shared("test_node_1");
  auto test_node_2 = rclcpp::Node::make_shared("test_node_2");
  auto test_node_3 = rclcpp::Node::make_shared("test_node_3");
  auto test_node_4 = rclcpp::Node::make_shared("test_node_4");
  auto test_lf_node = rclcpp_lifecycle::LifecycleNode::make_shared("test_lf_node");
  auto domain_node = std::make_shared<plansys2::DomainExpertNode>();
  auto problem_node = std::make_shared<plansys2::ProblemExpertNode>();
  auto planner_node = std::make_shared<plansys2::PlannerNode>();
  auto executor_node = std::make_shared<ExecutorNodeTest>();

  auto move_action_node = MoveAction::make_shared("move_action_performer", 100ms);
  move_action_node->set_parameter({"action_name", "move"});
  move_action_node->set_runtime(10.0);

  auto domain_client = std::make_shared<plansys2::DomainExpertClient>();
  auto problem_client = std::make_shared<plansys2::ProblemExpertClient>();
  auto planner_client = std::make_shared<plansys2::PlannerClient>();
  auto executor_client = std::make_shared<plansys2::ExecutorClient>();

  std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_executor");

  domain_node->set_parameter({"model_file", pkgpath + "/pddl/factory3.pddl"});
  problem_node->set_parameter({"model_file", pkgpath + "/pddl/factory3.pddl"});
  executor_node->set_parameter(
    {"default_action_bt_xml_filename",
      pkgpath + "/test_behavior_trees/test_action_timeout_bt.xml"});
  executor_node->set_parameter({"bt_builder_plugin", "SimpleBTBuilder"});
  executor_node->set_parameter({"action_timeouts.actions", std::vector<std::string>({"move"})});
  // have to declare because the actions vector above was not available at node creation
  executor_node->declare_parameter<double>(
    "action_timeouts.move.duration_overrun_percentage", 1.0);
  executor_node->set_parameter({"action_timeouts.move.duration_overrun_percentage", 1.0});

  rclcpp::experimental::executors::EventsExecutor exe;

  exe.add_node(domain_node->get_node_base_interface());
  exe.add_node(problem_node->get_node_base_interface());
  exe.add_node(planner_node->get_node_base_interface());
  exe.add_node(executor_node->get_node_base_interface());
  exe.add_node(move_action_node->get_node_base_interface());
  exe.add_node(test_lf_node->get_node_base_interface());

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  move_action_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  test_lf_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node_1->now();
    while ((test_node_1->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  domain_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  problem_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  planner_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  executor_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  test_lf_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = test_node_1->now();
    while ((test_node_1->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("r2d2", "robot")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("wheels_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("steering_wheels_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("body_car_zone", "zone")));
  ASSERT_TRUE(problem_client->addInstance(plansys2::Instance("assembly_zone", "zone")));

  std::vector<std::string> predicates = {
    "(robot_at r2d2 steering_wheels_zone)",
    "(robot_available r2d2)",
    "(battery_full r2d2)",
  };

  for (const auto & pred : predicates) {
    ASSERT_TRUE(problem_client->addPredicate(plansys2::Predicate(pred)));
  }

  problem_client->setGoal(plansys2::Goal("(and(robot_at r2d2 assembly_zone))"));

  auto domain = domain_client->getDomain();
  auto problem = problem_client->getProblem();
  auto plan = planner_client->getPlan(domain, problem);

  ASSERT_FALSE(domain.empty());
  ASSERT_FALSE(problem.empty());
  ASSERT_TRUE(plan.has_value());

  {
    rclcpp::Rate rate(10);

    ASSERT_TRUE(executor_client->start_plan_execution(plan.value()));

    while (rclcpp::ok() && executor_client->execute_and_check_plan()) {
      auto feedback = executor_client->getFeedBack();
      std::stringstream ss;
      ss.setf(std::ios_base::fixed, std::ios_base::floatfield);
      for (const auto & action_feedback : feedback.action_execution_status) {
        ss << "[" << action_feedback.action << " " << std::setprecision(1) <<
          action_feedback.completion * 100.0 <<
          "%]";
      }
      auto & clk = *executor_node->get_clock();
      RCLCPP_WARN_THROTTLE(executor_node->get_logger(), clk, 500, "%s", ss.str().c_str());
      rate.sleep();
    }
  }

  ASSERT_TRUE(executor_client->getResult().has_value());
  auto result = executor_client->getResult().value();
  ASSERT_FALSE(result.success);
  ASSERT_EQ(
    result.action_execution_status[0].status,
    plansys2_msgs::msg::ActionExecutionInfo::CANCELLED);

  {
    rclcpp::Rate rate(10);
    auto start = test_node_1->now();
    while ((test_node_1->now() - start).seconds() < 2.0) {
      rate.sleep();
    }
  }

  ASSERT_EQ(
    move_action_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  finish = true;
  t.join();
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
