cmake_minimum_required(VERSION 3.5)
project(plansys2_executor)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_core REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(pluginlib REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    Eigen3
    rclcpp
    rclcpp_lifecycle
    lifecycle_msgs
    rclcpp_cascade_lifecycle
    rclcpp_action
    plansys2_pddl_parser
    ament_index_cpp
    plansys2_core
    plansys2_msgs
    plansys2_domain_expert
    plansys2_problem_expert
    plansys2_planner
    pluginlib
    behaviortree_cpp
    std_msgs
    std_srvs
)

include_directories(include)

set(EXECUTOR_SOURCES
  src/plansys2_executor/ExecutorClient.cpp
  src/plansys2_executor/ActionExecutor.cpp
  src/plansys2_executor/ActionExecutorClient.cpp
  src/plansys2_executor/ExecutorNode.cpp
  src/plansys2_executor/ComputeBT.cpp
  src/plansys2_executor/behavior_tree/execute_action_node.cpp
  src/plansys2_executor/behavior_tree/wait_action_node.cpp
  src/plansys2_executor/behavior_tree/check_action_node.cpp
  src/plansys2_executor/behavior_tree/wait_atstart_req_node.cpp
  src/plansys2_executor/behavior_tree/check_overall_req_node.cpp
  src/plansys2_executor/behavior_tree/check_atend_req_node.cpp
  src/plansys2_executor/behavior_tree/check_timeout_node.cpp
  src/plansys2_executor/behavior_tree/apply_atstart_effect_node.cpp
  src/plansys2_executor/behavior_tree/apply_atend_effect_node.cpp
)


add_library(${PROJECT_NAME} SHARED ${EXECUTOR_SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME})

add_executable(executor_node
  src/executor_node.cpp
)
ament_target_dependencies(executor_node ${dependencies})
target_link_libraries(executor_node ${PROJECT_NAME})

add_executable(compute_bt
  src/compute_bt.cpp
)
ament_target_dependencies(compute_bt ${dependencies})
target_link_libraries(compute_bt ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

add_library(bt_builder_plugins SHARED
  src/plansys2_executor/bt_builder_plugins/simple_bt_builder.cpp
  src/plansys2_executor/bt_builder_plugins/stn_bt_builder.cpp
)
ament_target_dependencies(bt_builder_plugins ${dependencies})

pluginlib_export_plugin_description_file(plansys2_executor plugins.xml)

install(DIRECTORY launch behavior_trees DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  bt_builder_plugins
  executor_node
  compute_bt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} bt_builder_plugins)
ament_export_dependencies(${dependencies})

ament_package()
