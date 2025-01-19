cmake_minimum_required(VERSION 3.5)

project(plansys2_tests)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(behaviortree_cpp REQUIRED)



set(dependencies
    rclcpp
    rclcpp_lifecycle
    lifecycle_msgs
    rclcpp_cascade_lifecycle
    rclcpp_action
    plansys2_pddl_parser
    ament_index_cpp
    plansys2_msgs
    plansys2_domain_expert
    plansys2_problem_expert
    plansys2_planner
    plansys2_executor
    behaviortree_cpp
)

include_directories(include)

set(${PROJECT_NAME}_SOURCES
  src/plansys2_tests/test_action_node.cpp
  src/plansys2_tests/execution_logger.cpp
)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  add_subdirectory(test_1)
  add_subdirectory(test_2)
  add_subdirectory(test_3)
endif()

ament_package()
